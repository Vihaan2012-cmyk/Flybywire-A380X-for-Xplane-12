use super::message::{DataStatus, RateAccumulator};
use std::collections::VecDeque;

pub const LOW_SPEED_BPS: f64 = 12_500.0;
pub const HIGH_SPEED_BPS: f64 = 100_000.0;
pub const WORD_BITS: f64 = 32.0;
pub const MIN_GAP_BITS: f64 = 4.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ssm {
    FailureWarning,
    NoComputedData,
    FunctionalTest,
    NormalOperation,
}
impl Ssm {
    fn bits(self) -> u8 {
        match self {
            Ssm::FailureWarning => 0b00,
            Ssm::NoComputedData => 0b01,
            Ssm::FunctionalTest => 0b10,
            Ssm::NormalOperation => 0b11,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Word {
    pub label: u8,
    pub sdi: u8,
    pub data: u32,
    pub ssm: Ssm,
}
impl Word {
    pub fn new(label: u8, sdi: u8, data: u32, ssm: Ssm) -> Self {
        Self { label, sdi: sdi & 0b11, data: data & 0x7_FFFF, ssm }
    }

    pub fn odd_parity_bit(&self) -> bool {
        let ones = self.label.count_ones() + self.sdi.count_ones() + self.data.count_ones() + (self.ssm.bits() as u32).count_ones();
        ones % 2 == 0
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BusFaults {
    pub open: f64,
    pub short: f64,
}

pub fn min_word_period_s(speed_bps: f64) -> f64 {
    (WORD_BITS + MIN_GAP_BITS) / speed_bps
}

struct PendingWord {
    arrival_s: f64,
    word: Word,
    parity_ok: bool,
}

pub struct Arinc429Channel {
    period_s: f64,
    speed_bps: f64,
    accumulated_s: f64,
    loss_acc: RateAccumulator,
    corrupt_acc: RateAccumulator,
    in_flight: VecDeque<PendingWord>,
    last_arrival: Option<(f64, Option<Word>)>,
}
impl Arinc429Channel {
    pub fn new(period_s: f64, speed_bps: f64) -> Self {
        Self {
            period_s: period_s.max(min_word_period_s(speed_bps)),
            speed_bps,
            accumulated_s: 0.0,
            loss_acc: RateAccumulator::default(),
            corrupt_acc: RateAccumulator::default(),
            in_flight: VecDeque::new(),
            last_arrival: None,
        }
    }

    pub fn step(&mut self, dt_s: f64, now_s: f64, word: Word, faults: &BusFaults) {
        self.accumulated_s += dt_s.max(0.0);
        let mut emitted = 0;
        while self.accumulated_s >= self.period_s && emitted < 256 {
            self.accumulated_s -= self.period_s;
            emitted += 1;
            if self.loss_acc.fires(faults.open) {
                continue;
            }
            let parity_ok = !self.corrupt_acc.fires(faults.short);
            let arrival_s = now_s + WORD_BITS / self.speed_bps;
            self.in_flight.push_back(PendingWord { arrival_s, word, parity_ok });
        }
        while let Some(front) = self.in_flight.front() {
            if front.arrival_s > now_s {
                break;
            }
            let w = self.in_flight.pop_front().unwrap();
            self.last_arrival = Some((w.arrival_s, w.parity_ok.then_some(w.word)));
        }
    }

    pub fn status_at(&self, now_s: f64, staleness_s: f64) -> (DataStatus, f64, Option<Word>) {
        match self.last_arrival {
            None => (DataStatus::NoData, f64::INFINITY, None),
            Some((t, word)) => {
                let age = (now_s - t).max(0.0);
                if age > staleness_s {
                    (DataStatus::NoData, age, None)
                } else if let Some(w) = word {
                    (DataStatus::NormalOperation, age, Some(w))
                } else {
                    (DataStatus::NoComputedData, age, None)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odd_parity_makes_the_total_one_bit_count_odd() {
        let w = Word::new(0, 0, 0, Ssm::FailureWarning);
        assert!(w.odd_parity_bit());
        let w2 = Word::new(1, 0, 0, Ssm::FailureWarning);
        assert!(!w2.odd_parity_bit());
    }

    #[test]
    fn healthy_bus_delivers_normal_operation() {
        let mut ch = Arinc429Channel::new(0.05, HIGH_SPEED_BPS);
        let faults = BusFaults::default();
        let w = Word::new(0o203, 0, 12345, Ssm::NormalOperation);
        let mut now = 0.0;
        for _ in 0..200 {
            ch.step(0.01, now, w, &faults);
            now += 0.01;
        }
        let (status, age, word) = ch.status_at(now, 0.2);
        assert_eq!(status, DataStatus::NormalOperation);
        assert!(age < 0.2);
        assert_eq!(word, Some(w));
    }

    #[test]
    fn a_fully_open_bus_goes_stale() {
        let mut ch = Arinc429Channel::new(0.05, HIGH_SPEED_BPS);
        let faults = BusFaults { open: 1.0, short: 0.0 };
        let w = Word::new(0o203, 0, 12345, Ssm::NormalOperation);
        let mut now = 0.0;
        for _ in 0..200 {
            ch.step(0.01, now, w, &faults);
            now += 0.01;
        }
        let (status, _, word) = ch.status_at(now, 0.2);
        assert_eq!(status, DataStatus::NoData);
        assert_eq!(word, None);
    }

    #[test]
    fn a_fully_shorted_bus_delivers_words_that_fail_parity() {
        let mut ch = Arinc429Channel::new(0.05, HIGH_SPEED_BPS);
        let faults = BusFaults { open: 0.0, short: 1.0 };
        let w = Word::new(0o203, 0, 12345, Ssm::NormalOperation);
        let mut now = 0.0;
        for _ in 0..200 {
            ch.step(0.01, now, w, &faults);
            now += 0.01;
        }
        let (status, _, word) = ch.status_at(now, 0.2);
        assert_eq!(status, DataStatus::NoComputedData);
        assert_eq!(word, None);
    }

    #[test]
    fn the_ssm_field_is_the_sources_claim_independent_of_link_health() {
        let mut ch = Arinc429Channel::new(0.05, HIGH_SPEED_BPS);
        let faults = BusFaults::default();
        let w = Word::new(0o203, 0, 0, Ssm::FailureWarning);
        let mut now = 0.0;
        for _ in 0..200 {
            ch.step(0.01, now, w, &faults);
            now += 0.01;
        }
        let (status, _, word) = ch.status_at(now, 0.2);
        assert_eq!(status, DataStatus::NormalOperation, "the link delivered the word intact");
        assert_eq!(word.unwrap().ssm, Ssm::FailureWarning, "but the source flagged its own data bad");
    }

    #[test]
    fn period_is_clamped_to_what_the_speed_can_physically_carry() {
        let ch = Arinc429Channel::new(1e-6, LOW_SPEED_BPS);
        assert!(ch.period_s >= min_word_period_s(LOW_SPEED_BPS));
    }

    #[test]
    fn no_nan_at_zero_dt() {
        let mut ch = Arinc429Channel::new(0.05, HIGH_SPEED_BPS);
        ch.step(0.0, 0.0, Word::new(1, 0, 0, Ssm::NormalOperation), &BusFaults::default());
        let (_, age, _) = ch.status_at(0.0, 0.2);
        assert!(!age.is_nan());
    }
}

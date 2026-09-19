//! ARINC 429 point-to-point buses, the legacy links the AFDX network
//! gateways to for the LRUs that predate it (an IOM's ARINC 429 ports).
//! Public spec summaries (ARINC 429 tutorials from Condor
//! Engineering/Aeroflex, widely mirrored, and general avionics databus
//! textbooks) describe the wire format used here: a 32-bit word — an
//! 8-bit label, a 2-bit SDI, 19 data bits, a 2-bit Sign/Status Matrix
//! (SSM) and one odd-parity bit — sent one direction only, at 12.5 or 100
//! kbit/s, with a minimum 4-bit-time gap between words.
//!
//! Two things are modelled, each acting on a different element:
//!   * the word's own SSM, which the *source* sets to say whether its data
//!     is good (`Ssm::NormalOperation`), a self-test pattern, computed but
//!     not from a live sensor (`NoComputedData`), or the source itself has
//!     flagged failure/warning — this is content the receiver reads once a
//!     word arrives;
//!   * the bus itself failing open (broken wire/connector: nothing
//!     arrives) or shorted (stuck line: whatever arrives fails parity) —
//!     this is a property of the link, independent of what the source is
//!     trying to say, reusing `message::DataStatus` for "did a good word
//!     get here" since ARINC 429's SSM priority ordering
//!     (Failure/Warning < No Computed Data < Functional Test < Normal
//!     Operation) is the same four-level scheme, just named for the
//!     *sender's* claim rather than the *link's* delivery.

use super::message::{DataStatus, RateAccumulator};
use std::collections::VecDeque;

/// Low-speed ARINC 429, bit/s (public spec: 12.5 kbit/s +-1%).
pub const LOW_SPEED_BPS: f64 = 12_500.0;
/// High-speed ARINC 429, bit/s (public spec: 100 kbit/s +-1%).
pub const HIGH_SPEED_BPS: f64 = 100_000.0;
/// Bits per word: label(8) + SDI(2) + data(19) + SSM(2) + parity(1).
pub const WORD_BITS: f64 = 32.0;
/// Minimum inter-word gap, bit times (public spec).
pub const MIN_GAP_BITS: f64 = 4.0;

/// The word's Sign/Status Matrix for BNR (binary numeric) data, in its
/// real bit order 00..11 (public spec) — the source's own claim about its
/// data, independent of whether the word even arrives intact.
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

/// One ARINC 429 word. `label` is stored as the raw 8 bits (conventionally
/// written in octal in maintenance documentation, e.g. label 203 octal —
/// no conversion is needed here since nothing in this model cares about
/// the octal digit grouping, only equality/identification).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Word {
    pub label: u8,
    /// Source/Destination Identifier, 2 bits.
    pub sdi: u8,
    /// 19-bit data field.
    pub data: u32,
    pub ssm: Ssm,
}
impl Word {
    pub fn new(label: u8, sdi: u8, data: u32, ssm: Ssm) -> Self {
        Self { label, sdi: sdi & 0b11, data: data & 0x7_FFFF, ssm }
    }

    /// The parity bit an odd-parity transmitter appends: `true` (1) when
    /// the rest of the word has an even number of one-bits, so the total
    /// (including this bit) is odd.
    pub fn odd_parity_bit(&self) -> bool {
        let ones = self.label.count_ones() + self.sdi.count_ones() + self.data.count_ones() + (self.ssm.bits() as u32).count_ones();
        ones % 2 == 0
    }
}

/// One physical ARINC 429 wire pair: open (broken wire/pulled connector)
/// and short (stuck line) fractions, `0.0` healthy .. `1.0` fully failed.
#[derive(Clone, Copy, Debug, Default)]
pub struct BusFaults {
    /// Fraction of words that never reach the receiver at all.
    pub open: f64,
    /// Fraction of words that reach the receiver with the line held or
    /// glitching such that the received bit pattern's parity does not
    /// check out (a receiver never trusts a parity-failed word's fields).
    pub short: f64,
}

/// The transmission time of one word plus the minimum inter-word gap, s —
/// the fastest this label could legally be sent at `speed_bps`.
pub fn min_word_period_s(speed_bps: f64) -> f64 {
    (WORD_BITS + MIN_GAP_BITS) / speed_bps
}

struct PendingWord {
    arrival_s: f64,
    word: Word,
    parity_ok: bool,
}

/// One receiver's view of one ARINC 429 label on one wire: regulates
/// transmission to `period_s` (clamped to what `speed_bps` can physically
/// carry), carries words in flight for the (fixed, jitter-free — this is a
/// dedicated point-to-point line, not a switched network) transmission
/// latency, and reports staleness the same way `message::SideChannel` does.
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
    /// `period_s` is how often the source refreshes this label (real
    /// refresh rates vary hugely by parameter, tens of ms to a few
    /// seconds, so this is left to the caller rather than hard-coded).
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

    /// Advances by `dt_s`; `word` is whatever the source is currently
    /// driving onto the line (read fresh each time a transmission is due).
    pub fn step(&mut self, dt_s: f64, now_s: f64, word: Word, faults: &BusFaults) {
        self.accumulated_s += dt_s.max(0.0);
        let mut emitted = 0;
        while self.accumulated_s >= self.period_s && emitted < 256 {
            self.accumulated_s -= self.period_s;
            emitted += 1;
            if self.loss_acc.fires(faults.open) {
                continue; // open circuit: nothing reaches the receiver.
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

    /// The link's own delivery status (`NormalOperation` = a parity-good
    /// word within `staleness_s`, `NoComputedData` = the most recent word
    /// arrived but failed parity, `NoData` = nothing valid recently
    /// enough), its age, and the word itself when there is one to read —
    /// callers then read `word.ssm` for the *source's* claim about its
    /// data, a second, independent thing from whether the link delivered
    /// it at all.
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
        let w = Word::new(0, 0, 0, Ssm::FailureWarning); // all-zero payload: 0 ones.
        assert!(w.odd_parity_bit()); // needs the parity bit itself to be the 1.
        let w2 = Word::new(1, 0, 0, Ssm::FailureWarning); // label=1: 1 one already.
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
        let w = Word::new(0o203, 0, 0, Ssm::FailureWarning); // link healthy, source itself reports bad data.
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

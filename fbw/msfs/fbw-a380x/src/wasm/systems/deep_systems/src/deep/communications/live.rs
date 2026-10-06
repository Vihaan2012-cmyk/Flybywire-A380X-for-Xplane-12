use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::live::{Faults, Truth};

const ATA: u16 = 23;

fn f(n: u16) -> u64 {
    failure_id(RegArea::Communications, ATA, n)
}

#[derive(Default)]
pub struct CommunicationsLive {
    cids: [f64; 3],
    cids_channel: [f64; 4],
    capt_ptt_stuck: f64,
    fo_ptt_stuck: f64,
    third_ptt_stuck: f64,
    datalink_fault: f64,
    hf1_datalink_fault: f64,
    hf1_stuck_emitting: f64,
    hf2_datalink_fault: f64,
    hf2_stuck_emitting: f64,
    satcom_fault: f64,
    satcom_datalink_fault: f64,
    satcom_voice_fault: f64,
    vhf1_stuck_emitting: f64,
    vhf2_stuck_emitting: f64,
    vhf3_stuck_emitting: f64,
    vhf3_datalink_fault: f64,
}

impl CommunicationsLive {
    pub fn new() -> Self {
        Self::default()
    }
}

impl crate::deep::live::Area for CommunicationsLive {
    fn name(&self) -> &'static str {
        "communications"
    }

    fn tick(&mut self, _truth: &Truth, faults: &Faults) {
        self.cids = [faults.get(f(1)), faults.get(f(2)), faults.get(f(3))];
        self.cids_channel = [faults.get(f(4)), faults.get(f(5)), faults.get(f(6)), faults.get(f(7))];
        self.capt_ptt_stuck = faults.get(f(8));
        self.fo_ptt_stuck = faults.get(f(9));
        self.third_ptt_stuck = faults.get(f(10));
        self.datalink_fault = faults.get(f(11));
        self.hf1_datalink_fault = faults.get(f(12));
        self.hf1_stuck_emitting = faults.get(f(13));
        self.hf2_datalink_fault = faults.get(f(14));
        self.hf2_stuck_emitting = faults.get(f(15));
        self.satcom_fault = faults.get(f(16));
        self.satcom_datalink_fault = faults.get(f(17));
        self.satcom_voice_fault = faults.get(f(18));
        self.vhf1_stuck_emitting = faults.get(f(19));
        self.vhf2_stuck_emitting = faults.get(f(20));
        self.vhf3_stuck_emitting = faults.get(f(21));
        self.vhf3_datalink_fault = faults.get(f(22));
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        for (i, m) in self.cids.iter().enumerate() {
            out(&format!("DEEP_COM_CIDS_{}_FAULT", i + 1), *m);
        }
        for (name, m) in [("PA_UPPER", self.cids_channel[0]), ("PA_MAIN", self.cids_channel[1]), ("PA_LOWER", self.cids_channel[2]), ("INTERPHONE", self.cids_channel[3])] {
            out(&format!("DEEP_COM_CIDS_{name}_MAGNITUDE"), m);
        }
        out("DEEP_COM_CAPT_PTT_STUCK", self.capt_ptt_stuck);
        out("DEEP_COM_FO_PTT_STUCK", self.fo_ptt_stuck);
        out("DEEP_COM_THIRD_PTT_STUCK", self.third_ptt_stuck);
        out("DEEP_COM_DATALINK_FAULT", self.datalink_fault);
        out("DEEP_COM_HF1_DATALINK_FAULT", self.hf1_datalink_fault);
        out("DEEP_COM_HF1_EMITTING", self.hf1_stuck_emitting);
        out("DEEP_COM_HF2_DATALINK_FAULT", self.hf2_datalink_fault);
        out("DEEP_COM_HF2_EMITTING", self.hf2_stuck_emitting);
        out("DEEP_COM_SATCOM_FAULT", self.satcom_fault);
        out("DEEP_COM_SATCOM_DATALINK_FAULT", self.satcom_datalink_fault);
        out("DEEP_COM_SATCOM_VOICE_FAULT", self.satcom_voice_fault);
        out("DEEP_COM_VHF1_EMITTING", self.vhf1_stuck_emitting);
        out("DEEP_COM_VHF2_EMITTING", self.vhf2_stuck_emitting);
        out("DEEP_COM_VHF3_EMITTING", self.vhf3_stuck_emitting);
        out("DEEP_COM_VHF3_DATALINK_FAULT", self.vhf3_datalink_fault);
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(CommunicationsLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Registry;
    use crate::deep::live::Area;

    fn published(area: &mut CommunicationsLive, faults: &Faults) -> std::collections::BTreeMap<String, f64> {
        area.tick(&Truth::default(), faults);
        let mut out = std::collections::BTreeMap::new();
        area.publish(&mut |n, v| {
            out.insert(n.to_owned(), v);
        });
        out
    }

    #[test]
    fn healthy_is_silent_and_a_failure_is_reachable_and_zero_magnitude_matches_healthy() {
        let mut area = CommunicationsLive::new();
        let healthy = published(&mut area, &Faults::default());
        assert!(healthy.values().all(|&v| v == 0.0), "a healthy aircraft must publish nothing but zeros");

        let faulted = published(&mut area, &Faults::from_pairs([(f(8), 1.0)]));
        assert_eq!(faulted["DEEP_COM_CAPT_PTT_STUCK"], 1.0);

        let back = published(&mut area, &Faults::from_pairs([(f(8), 0.0)]));
        assert_eq!(back, healthy);
    }

    #[test]
    fn every_registered_failure_here_is_reachable_through_publish() {
        let mut r = Registry::default();
        super::super::registry::register(&mut r);
        let mut area = CommunicationsLive::new();
        for failure in r.failures.iter().filter(|f| f.area == RegArea::Communications) {
            let out = published(&mut area, &Faults::from_pairs([(failure.id, 1.0)]));
            assert!(out.values().any(|&v| v != 0.0), "{} (id {}) changed nothing this area publishes", failure.name, failure.id);
        }
    }

    #[test]
    fn cids_channels_are_independent_of_the_three_computers() {
        let mut area = CommunicationsLive::new();
        let channel_faulted = published(&mut area, &Faults::from_pairs([(f(4), 1.0)]));
        assert_eq!(channel_faulted["DEEP_COM_CIDS_PA_UPPER_MAGNITUDE"], 1.0);
        assert_eq!(channel_faulted["DEEP_COM_CIDS_1_FAULT"], 0.0);
    }
}

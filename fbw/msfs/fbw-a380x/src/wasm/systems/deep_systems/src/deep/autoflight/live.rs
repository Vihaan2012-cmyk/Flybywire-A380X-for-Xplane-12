use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::live::{Faults, Truth};

fn f(ata: u16, n: u16) -> u64 {
    failure_id(RegArea::AutoFlight, ata, n)
}

const ATA: u16 = 22;

pub struct AutoFlightLive {
    fcu_fault: f64,
    fcu_switched_off: f64,
    capt_bkup_fault: f64,
    fo_bkup_fault: f64,
    tcas_mode_fault: f64,
    approach_capability_downgraded: bool,
    dual_ap_engaged: bool,
    radio_height_ft: f64,
    athr_armed_or_active: bool,
    athr_eng_fault: [bool; 4],
}

impl Default for AutoFlightLive {
    fn default() -> Self {
        Self::new()
    }
}

impl AutoFlightLive {
    pub fn new() -> Self {
        Self {
            fcu_fault: 0.0,
            fcu_switched_off: 0.0,
            capt_bkup_fault: 0.0,
            fo_bkup_fault: 0.0,
            tcas_mode_fault: 0.0,
            approach_capability_downgraded: false,
            dual_ap_engaged: false,
            radio_height_ft: 0.0,
            athr_armed_or_active: false,
            athr_eng_fault: [false; 4],
        }
    }
}

impl crate::deep::live::Area for AutoFlightLive {
    fn name(&self) -> &'static str {
        "autoflight"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        self.fcu_fault = faults.get(f(ATA, 1));
        self.fcu_switched_off = faults.get(f(ATA, 2));
        self.capt_bkup_fault = faults.get(f(ATA, 3));
        self.fo_bkup_fault = faults.get(f(ATA, 4));
        self.tcas_mode_fault = faults.get(f(ATA, 5));
        self.approach_capability_downgraded = truth.prim_healthy.iter().any(|&h| !h);
        self.dual_ap_engaged = truth.ap1_active && truth.ap2_active;
        self.radio_height_ft = truth.radio_height_ft;
        self.athr_armed_or_active = truth.athr_status >= 1.0;
        self.athr_eng_fault = truth.athr_eng_fault;
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        out("DEEP_AUTOFLT_FCU_FAULT", self.fcu_fault);
        out("DEEP_AUTOFLT_FCU_SWITCHED_OFF", self.fcu_switched_off);
        out("DEEP_AUTOFLT_CAPT_FCU_BKUP_FAULT", self.capt_bkup_fault);
        out("DEEP_AUTOFLT_FO_FCU_BKUP_FAULT", self.fo_bkup_fault);
        out("DEEP_AUTOFLT_TCAS_MODE_FAULT", self.tcas_mode_fault);
        out("DEEP_AUTOFLT_APPROACH_CAPABILITY_DOWNGRADED", b(self.approach_capability_downgraded));
        out("DEEP_AUTOFLT_DUAL_AP_ENGAGED", b(self.dual_ap_engaged));
        out("DEEP_AUTOFLT_RADIO_HEIGHT_FT", self.radio_height_ft);
        out("DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE", b(self.athr_armed_or_active));
        for (i, &fault) in self.athr_eng_fault.iter().enumerate() {
            out(&format!("DEEP_AUTOFLT_ENG_{}_ATHR_FAULT", i + 1), b(fault));
        }
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(AutoFlightLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Registry;
    use crate::deep::live::{Area, Faults};

    fn published(area: &mut AutoFlightLive, truth: &Truth, faults: &Faults) -> std::collections::BTreeMap<String, f64> {
        area.tick(truth, faults);
        let mut out = std::collections::BTreeMap::new();
        area.publish(&mut |n, v| {
            out.insert(n.to_owned(), v);
        });
        out
    }

    #[test]
    fn fcu_fault_is_healthy_by_default_and_set_by_its_own_failure() {
        let mut area = AutoFlightLive::new();
        let healthy = published(&mut area, &Truth::default(), &Faults::default());
        assert_eq!(healthy["DEEP_AUTOFLT_FCU_FAULT"], 0.0);

        let faulted = published(&mut area, &Truth::default(), &Faults::from_pairs([(f(ATA, 1), 1.0)]));
        assert_eq!(faulted["DEEP_AUTOFLT_FCU_FAULT"], 1.0);
        let back = published(&mut area, &Truth::default(), &Faults::from_pairs([(f(ATA, 1), 0.0)]));
        assert_eq!(back["DEEP_AUTOFLT_FCU_FAULT"], 0.0);
    }

    #[test]
    fn fcu_switched_off_is_independent_of_fcu_fault() {
        let mut area = AutoFlightLive::new();
        let out = published(&mut area, &Truth::default(), &Faults::from_pairs([(f(ATA, 2), 1.0)]));
        assert_eq!(out["DEEP_AUTOFLT_FCU_SWITCHED_OFF"], 1.0);
        assert_eq!(out["DEEP_AUTOFLT_FCU_FAULT"], 0.0);
    }

    #[test]
    fn approach_capability_is_downgraded_when_any_prim_is_unhealthy() {
        let mut area = AutoFlightLive::new();
        let healthy = Truth { prim_healthy: [true; 3], ..Truth::default() };
        assert_eq!(published(&mut area, &healthy, &Faults::default())["DEEP_AUTOFLT_APPROACH_CAPABILITY_DOWNGRADED"], 0.0);

        let one_down = Truth { prim_healthy: [true, false, true], ..Truth::default() };
        assert_eq!(published(&mut area, &one_down, &Faults::default())["DEEP_AUTOFLT_APPROACH_CAPABILITY_DOWNGRADED"], 1.0);
    }

    #[test]
    fn every_registered_failure_here_is_reachable_through_publish() {
        let mut r = Registry::default();
        super::super::registry::register(&mut r);
        let mut area = AutoFlightLive::new();
        for failure in r.failures.iter().filter(|f| f.area == RegArea::AutoFlight) {
            let faults = Faults::from_pairs([(failure.id, 1.0)]);
            let out = published(&mut area, &Truth::default(), &faults);
            assert!(out.values().any(|&v| v != 0.0), "{} (id {}) changed nothing this area publishes", failure.name, failure.id);
        }
    }

    #[test]
    fn dual_ap_engaged_needs_both_autopilots() {
        let mut area = AutoFlightLive::new();
        let one = Truth { ap1_active: true, ap2_active: false, ..Truth::default() };
        assert_eq!(published(&mut area, &one, &Faults::default())["DEEP_AUTOFLT_DUAL_AP_ENGAGED"], 0.0);
        let both = Truth { ap1_active: true, ap2_active: true, ..Truth::default() };
        assert_eq!(published(&mut area, &both, &Faults::default())["DEEP_AUTOFLT_DUAL_AP_ENGAGED"], 1.0);
    }

    #[test]
    fn radio_height_is_a_plain_truth_passthrough() {
        let mut area = AutoFlightLive::new();
        let t = Truth { radio_height_ft: 150.0, ..Truth::default() };
        assert_eq!(published(&mut area, &t, &Faults::default())["DEEP_AUTOFLT_RADIO_HEIGHT_FT"], 150.0);
    }

    #[test]
    fn athr_status_and_per_engine_fault_are_independent_and_correctly_indexed() {
        let mut area = AutoFlightLive::new();
        let healthy = published(&mut area, &Truth::default(), &Faults::default());
        assert_eq!(healthy["DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE"], 0.0);
        for n in 1..=4 {
            assert_eq!(healthy[&format!("DEEP_AUTOFLT_ENG_{n}_ATHR_FAULT")], 0.0);
        }

        let armed = Truth { athr_status: 2.0, ..Truth::default() };
        assert_eq!(published(&mut area, &armed, &Faults::default())["DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE"], 1.0);

        let eng2_fault = Truth { athr_eng_fault: [false, true, false, false], ..Truth::default() };
        let out = published(&mut area, &eng2_fault, &Faults::default());
        assert_eq!(out["DEEP_AUTOFLT_ENG_2_ATHR_FAULT"], 1.0);
        assert_eq!(out["DEEP_AUTOFLT_ENG_1_ATHR_FAULT"], 0.0);
        assert_eq!(out["DEEP_AUTOFLT_ENG_3_ATHR_FAULT"], 0.0);
        assert_eq!(out["DEEP_AUTOFLT_ENG_4_ATHR_FAULT"], 0.0);
    }
}

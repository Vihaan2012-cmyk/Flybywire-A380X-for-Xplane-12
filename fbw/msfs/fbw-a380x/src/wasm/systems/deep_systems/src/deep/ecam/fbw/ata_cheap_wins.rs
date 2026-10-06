use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{var, Level};

pub fn wire(v: &mut Vec<FbwProc>) {
    for eng in 1..=4u32 {
        let i = u64::from(eng) - 1;
        v.push(
            proc(
                701_800_081 + i,
                "ENG n OIL FILTER CLOGGED",
                Level::Advisory,
                sd_page::ENG,
                crate::deep::api::all(vec![
                    var(&format!("A32NX_ENG_{eng}_OIL_FILTER_BYPASSED")).on(),
                    var(&format!("A32NX_ENG_{eng}_GASPATH_OIL_TEMP_C")).ge(50.0),
                ]),
                "the oil filter's own bypass valve has cracked, which physics::engine::oil only does once filter_clog has raised the element's viscous drop past FILTER_BYPASS_PSI -- a clogged filter, not an approximation of one",
            )
            .confirm(5.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10]),
        );
    }
}

#[cfg(test)]
mod tests {
    use crate::deep::api::Cond;
    use crate::deep::engine_accessories::live::live_system;
    use crate::deep::live::{Area, Faults, Truth};
    use std::collections::BTreeMap;

    #[test]
    fn a_clogging_oil_filter_reaches_the_published_bypass_bit_and_the_procedure_trigger() {
        let wirings = super::super::wirings();
        let proc = wirings.iter().find(|p| p.id == 701_800_081).expect("701800081 wired");

        let mut truth = Truth {
            dt_s: 1.0,
            engine_running: [true; 4],
            engine_n1_frac: [0.9; 4],
            engine_n2_frac: [0.9; 4],
            engine_n3_frac: [0.9; 4],
            engine_n2_healthy_frac: [0.9; 4],
            engine_n3_healthy_frac: [0.9; 4],
            ..Truth::default()
        };
        let mut area = live_system();
        for _ in 0..3_600 {
            tick(&mut *area, &truth);
        }
        let clean = tick(&mut *area, &truth);
        assert_eq!(clean.get("A32NX_ENG_1_OIL_FILTER_BYPASSED"), Some(&0.0));
        assert!(!eval(&proc.trigger, &clean), "must not be armed on a clean engine");

        truth.engine_oil_filter_bypassed[0] = true;
        truth.dt_s = 0.02;
        let clogged = tick(&mut *area, &truth);
        assert_eq!(clogged.get("A32NX_ENG_1_OIL_FILTER_BYPASSED"), Some(&1.0), "the bypass bit must reach the published var");
        assert!(eval(&proc.trigger, &clogged), "the procedure's own trigger must flip once the published bit is set");
        assert_eq!(clogged.get("A32NX_ENG_2_OIL_FILTER_BYPASSED"), Some(&0.0));
    }

    fn tick(area: &mut dyn Area, truth: &Truth) -> BTreeMap<String, f64> {
        area.tick(truth, &Faults::default());
        let mut out = BTreeMap::new();
        area.publish(&mut |k: &str, v: f64| {
            out.insert(k.to_string(), v);
        });
        out
    }

    fn eval(c: &Cond, vars: &BTreeMap<String, f64>) -> bool {
        c.eval(&|name| vars.get(name).copied().unwrap_or(0.0))
    }
}

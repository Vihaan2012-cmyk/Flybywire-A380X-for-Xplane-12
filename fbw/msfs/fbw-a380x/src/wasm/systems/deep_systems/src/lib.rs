#![allow(dead_code, unused_imports, unused_variables, clippy::all)]

pub mod deep;
pub mod msfs_excluded;
pub mod physics;

pub mod weight_balance;

pub mod fuel_network {
    pub const DEFAULT_LINE_FLOW_GAIN: f64 = 60.0;
}

pub mod mel;
pub mod mel_catalog;

pub mod extra_backend_fbw {
    pub fn time_of_day_from_sun(sun_pitch_deg: f64, sun_heading_deg: f64) -> u8 {
        if sun_pitch_deg > 0. {
            1
        } else if sun_pitch_deg < -6. {
            3
        } else if sun_heading_deg.rem_euclid(360.) < 180. {
            0
        } else {
            2
        }
    }
}

pub mod source_patch {
    #[derive(Clone, Debug)]
    pub struct SourcePatch {
        pub path: String,
        pub find: String,
        pub replace: String,
        pub reason: String,
    }
}

pub mod failures {
    pub mod extra {
        pub struct Owner;
        impl Owner {
            pub fn label(&self) -> &'static str {
                ""
            }
        }
        pub enum Effect {
            Hook { var: &'static str, owner: Owner },
        }
        pub struct Extra {
            pub id: u64,
            pub effect: Effect,
        }
        pub fn extra_failures() -> Vec<Extra> {
            Vec::new()
        }
    }
}

pub mod invariants {
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum Bound {
        None,
        NonNegative,
        TemperatureFloor(f64),
        Range(f64, f64),
    }

    impl Bound {
        fn violation(self, value: f64) -> Option<f64> {
            match self {
                Bound::None => None,
                Bound::NonNegative => (value < 0.).then_some(0.),
                Bound::TemperatureFloor(floor) => (value < floor).then_some(floor),
                Bound::Range(lo, hi) => {
                    if value < lo {
                        Some(lo)
                    } else if value > hi {
                        Some(hi)
                    } else {
                        None
                    }
                }
            }
        }
    }

    pub fn check(name: &str, value: f64, bound: Bound, hint: &str) -> f64 {
        if value.is_finite() {
            if let Some(clamped) = bound.violation(value) {
                crate::log(&format!("invariant: {name} = {value} out of bounds ({bound:?}), clamped to {clamped} [{hint}]"));
                return clamped;
            }
            return value;
        }
        let fallback = match bound {
            Bound::NonNegative => 0.0,
            Bound::TemperatureFloor(floor) => floor,
            Bound::Range(lo, _) => lo,
            Bound::None => 0.0,
        };
        crate::log(&format!("invariant: {name} = {value} not finite, set to {fallback} [{hint}]"));
        fallback
    }
}

pub const M_TO_FT: f64 = 3.280_84;

static LOG_QUIET: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub fn set_log_quiet(quiet: bool) {
    LOG_QUIET.store(quiet, std::sync::atomic::Ordering::Relaxed);
}

pub fn log(message: &str) {
    if LOG_QUIET.load(std::sync::atomic::Ordering::Relaxed) {
        return;
    }
    println!("deep: {message}");
}

mod facade;
mod gates;
mod gates_cpp;
mod gates_js;
mod gates_msfs;
mod gates_wave3;

pub mod random_failures;
pub mod scripted_failures;
pub mod wear;

pub use deep::frame::{DerivedFailure, Faults, PublishedFrame};
pub use deep::live::{CommandedSurfaces, Controls, IrOutputs, Truth, DOOR_NAMES};
pub use deep::integration::weather_model::EnvironmentTruth;
pub use facade::*;

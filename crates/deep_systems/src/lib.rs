//! Every A380 deep area, compiled from the X-Plane plugin's own `src/deep`
//! files, for FlyByWire's MSFS systems module.
//!
//! The areas were written against the plugin's crate, so they name a few of
//! its modules (`crate::physics::tyre`, `crate::weight_balance`, ...). The
//! modules of those names here carry only the items the areas use: the
//! plugin's own file where it is host-neutral, otherwise its constants and
//! functions copied with their source named. Nothing X-Plane-specific comes
//! along.
#![allow(dead_code, unused_imports, unused_variables, clippy::all)]

pub mod deep;
pub mod physics;

/// FlyByWire's stations and tanks and their moment arithmetic, the
/// plugin's `mass_balance.rs`, under the name the fuel area uses.
#[path = "../../../src/mass_balance.rs"]
pub mod weight_balance;

/// `fuel_network.rs`'s line gain, which the fuel area shares.
pub mod fuel_network {
    pub const DEFAULT_LINE_FLOW_GAIN: f64 = 60.0;
}

/// The plugin's MEL keeps deferrals; MSFS has none, so nothing is deferred.
pub mod mel {
    pub fn deferred_state(_id: u64) -> Option<f64> {
        None
    }
}

/// `extra_backend_fbw.rs`'s time of day from the sun, as FlyByWire's
/// lighting codes it (0 dawn, 1 day, 2 dusk, 3 night).
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

/// `source_patch.rs`'s patch record, which the ECAM area builds for the
/// X-Plane host's JS loader. MSFS applies the same patches at install time.
pub mod source_patch {
    #[derive(Clone, Debug)]
    pub struct SourcePatch {
        pub path: String,
        pub find: String,
        pub replace: String,
        pub reason: String,
    }
}

/// The X-Plane plugin's legacy failure hooks, which the failure audit's
/// `legacy_hook_families` groups; MSFS has none, so the list is empty.
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

/// The plugin's `invariants::check`: a physically bounded value outside its
/// bound is clamped to it (a non-finite one to the bound's floor) and
/// reported, never silently absorbed. The plugin records it in its
/// invariant log; here it goes to the module's console.
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

/// The plugin's `M_TO_FT` (`lib.rs`).
pub const M_TO_FT: f64 = 3.280_84;

/// The plugin logs to X-Plane's Log.txt; here, to the module's console.
pub fn log(message: &str) {
    println!("deep: {message}");
}

mod facade;
mod gates;
mod gates_cpp;
mod gates_js;
mod gates_msfs;

pub use deep::frame::{DerivedFailure, Faults, PublishedFrame};
pub use deep::live::{CommandedSurfaces, Controls, IrOutputs, Truth, DOOR_NAMES};
pub use deep::integration::weather_model::EnvironmentTruth;
pub use facade::*;

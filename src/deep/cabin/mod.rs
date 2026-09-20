//! ATA 25/38/44/52 cabin systems: potable water, waste, in-flight
//! entertainment and cabin power, galley equipment, doors/slides, and the
//! cabin crew call events a flight crew receives from all of the above.
//!
//! Every model here is a plain, self-contained struct (`new`, a `step`
//! taking an `...Inputs` and an `...Faults` and returning an `...Outputs`,
//! or equivalent), std-only, with no dependency on this crate's X-Plane/FBW
//! wiring (`Vars`, `Xplm`, `systems::simulation::*`) so it compiles and is
//! tested on its own; see `docs/deep/BRIEF.md`. Nothing outside this
//! directory references these modules yet — wiring them to real datarefs,
//! the electrical buses and the ECAM/crew-call annunciation is future
//! integration work for whoever wires `deep` into the plugin.
//!
//! No FlyByWire source exists to port for any of these systems: neither
//! `fbw-common/src/wasm/systems/systems/src` nor
//! `fbw-a380x/.../a380_systems/src` has a water, waste, lavatory, IFE/seat-
//! power, galley-thermal or slide/door-seal module (a search of both trees
//! for `water`/`waste`/`toilet`/`lavatory`/`galley`/`potable` found only the
//! galley *electrical shed flag* in `a380_systems/src/electrical/galley.rs`,
//! a boolean with no physical model behind it). Every model below is
//! therefore a native addition built from public reference material, in the
//! same spirit as this crate's existing `oxygen.rs`.

pub mod crew_calls;
pub mod doors_slides;
pub mod galley;
pub mod ife;
pub mod registry;
pub mod waste;
pub mod water;

/// The cabin zones every submodule here groups its equipment and faults by:
/// a coarse forward/mid/aft split of the cabin, the level of granularity the
/// brief asks for ("lavatories inoperative by zone", "seat electronics boxes
/// per seat zone"). GENERIC: no public A380 zone chart is used; this is a
/// simplification any of the cabin-crew-facing modules can share so a fault
/// in one (a full waste tank, a shed galley bus) is expressed in the same
/// coordinates as another (an IFE seat zone, a lavatory).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Zone {
    Fwd,
    Mid,
    Aft,
}

impl Zone {
    pub const ALL: [Zone; 3] = [Zone::Fwd, Zone::Mid, Zone::Aft];
    pub const COUNT: usize = 3;

    pub fn index(self) -> usize {
        match self {
            Zone::Fwd => 0,
            Zone::Mid => 1,
            Zone::Aft => 2,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Zone::Fwd => "FWD",
            Zone::Mid => "MID",
            Zone::Aft => "AFT",
        }
    }
}

impl Default for Zone {
    fn default() -> Self {
        Zone::Fwd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zone_index_is_stable_and_matches_all() {
        for (i, z) in Zone::ALL.iter().enumerate() {
            assert_eq!(z.index(), i);
        }
        assert_eq!(Zone::ALL.len(), Zone::COUNT);
    }
}

pub mod live;

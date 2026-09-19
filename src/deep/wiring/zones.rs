//! Airframe zones a wire bundle segment can be routed through.
//!
//! Nine of these match the zone list `fire_ice::fire_loops` documents as
//! coming from `docs/deep/BRIEF.md` backlog item 1 (read as context only --
//! this module imports nothing from `fire_ice`, per this push's
//! self-contained-directory rule): the four engine nacelle/pylon zones, the
//! APU bay, the main gear bay, the two cargo compartments and the main
//! avionics bay. Five more are added here, GENERIC (no shared list covers
//! them, but A380 wiring plausibly runs through all five): `UpperAvionics`
//! (the smaller upper equipment bay behind the cockpit, matching
//! `physics::bays.rs`'s second avionics bay, read as context), `Cockpit`
//! (the flight-deck panels themselves), `NoseGearBay` (distinct from the
//! main gear bay), `WingRoot` (centre-fuselage/wing-root structure the
//! hydraulic pumps, bleed ducts and engine feeder cables run through,
//! matching `physics::bays.rs`'s `WingRootBleed`) and `TailCone` (APU
//! intake/exhaust structure aft of the rear pressure bulkhead, where the
//! APU's own feeder run starts).

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Zone {
    /// 1..=4, left-to-right (1/2 on the left wing, 3/4 on the right --
    /// standard four-engine transport layout, public/general knowledge).
    Engine(u8),
    Apu,
    MainGearBay,
    NoseGearBay,
    CargoFwd,
    CargoAft,
    MainAvionics,
    UpperAvionics,
    Cockpit,
    WingRoot,
    TailCone,
}

pub const ALL_ZONES: &[Zone] = &[
    Zone::Engine(1),
    Zone::Engine(2),
    Zone::Engine(3),
    Zone::Engine(4),
    Zone::Apu,
    Zone::MainGearBay,
    Zone::NoseGearBay,
    Zone::CargoFwd,
    Zone::CargoAft,
    Zone::MainAvionics,
    Zone::UpperAvionics,
    Zone::Cockpit,
    Zone::WingRoot,
    Zone::TailCone,
];

impl Zone {
    pub fn name(self) -> String {
        match self {
            Zone::Engine(n) => format!("ENGINE_{n}"),
            Zone::Apu => "APU".into(),
            Zone::MainGearBay => "MAIN_GEAR_BAY".into(),
            Zone::NoseGearBay => "NOSE_GEAR_BAY".into(),
            Zone::CargoFwd => "CARGO_FWD".into(),
            Zone::CargoAft => "CARGO_AFT".into(),
            Zone::MainAvionics => "MAIN_AVIONICS".into(),
            Zone::UpperAvionics => "UPPER_AVIONICS".into(),
            Zone::Cockpit => "COCKPIT".into(),
            Zone::WingRoot => "WING_ROOT".into(),
            Zone::TailCone => "TAIL_CONE".into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_zone_has_a_distinct_name() {
        let mut names: Vec<String> = ALL_ZONES.iter().map(|z| z.name()).collect();
        let n = names.len();
        names.sort();
        names.dedup();
        assert_eq!(names.len(), n, "duplicate zone name");
        assert_eq!(n, 14);
    }
}

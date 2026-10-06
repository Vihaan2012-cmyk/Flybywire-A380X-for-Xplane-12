#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Zone {
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

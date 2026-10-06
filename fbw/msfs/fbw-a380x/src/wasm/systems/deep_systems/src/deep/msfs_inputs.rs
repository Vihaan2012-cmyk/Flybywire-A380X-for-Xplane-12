#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Convert {
    None,
    Scale(f64),
    ScaleNegated(f64),
}

impl Convert {
    pub fn apply(self, raw: f64) -> f64 {
        match self {
            Convert::None => raw,
            Convert::Scale(k) => raw * k,
            Convert::ScaleNegated(k) => -raw * k,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Confidence {
    Verified,
    Unverified,
}

pub struct Input {
    pub field: &'static str,
    pub xplane: &'static str,
    pub msfs: (&'static str, &'static str),
    pub convert: Convert,
    pub confidence: Confidence,
    pub note: &'static str,
}

const M_TO_FT: f64 = 3.280_839_895_013_123;
const DEG_PER_RAD: f64 = 57.295_779_513_082_32;
const KT_TO_M_S: f64 = 0.514_444_444_444_444_4;
const LB_TO_KG: f64 = 0.453_592_37;

pub const INPUTS: &[Input] = &[
    Input {
        field: "altitude_ft",
        xplane: "sim/flightmodel/position/elevation",
        msfs: ("PLANE ALTITUDE", "feet"),
        convert: Convert::None,
        confidence: Confidence::Verified,
        note: "X-Plane reports metres MSL, MSFS feet MSL. Truth wants feet.",
    },
    Input {
        field: "on_ground",
        xplane: "sim/flightmodel/failures/onground_any",
        msfs: ("SIM ON GROUND", "bool"),
        convert: Convert::None,
        confidence: Confidence::Verified,
        note: "Non-zero means on ground in both.",
    },
    Input {
        field: "aircraft_mass_kg",
        xplane: "sim/flightmodel/weight/m_total",
        msfs: ("TOTAL WEIGHT", "pounds"),
        convert: Convert::Scale(LB_TO_KG),
        confidence: Confidence::Verified,
        note: "MSFS weights are pounds throughout; X-Plane's are kilograms.",
    },
    Input {
        field: "pitch_deg",
        xplane: "sim/flightmodel/position/theta",
        msfs: ("PLANE PITCH DEGREES", "radians"),
        convert: Convert::ScaleNegated(DEG_PER_RAD),
        confidence: Confidence::Verified,
        note: "Named DEGREES, returns radians. MSFS positive is nose DOWN; \
               X-Plane and Truth want positive nose UP.",
    },
    Input {
        field: "groundspeed_m_s",
        xplane: "sim/flightmodel/position/groundspeed",
        msfs: ("GROUND VELOCITY", "knots"),
        convert: Convert::Scale(KT_TO_M_S),
        confidence: Confidence::Verified,
        note: "MSFS speeds are knots unless the unit string says otherwise.",
    },
    Input {
        field: "alpha_deg",
        xplane: "sim/flightmodel/position/alpha",
        msfs: ("INCIDENCE ALPHA", "radians"),
        convert: Convert::Scale(DEG_PER_RAD),
        confidence: Confidence::Verified,
        note: "FlyByWire read the same variable for their own angle of attack.",
    },
    Input {
        field: "radio_height_ft",
        xplane: "sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot",
        msfs: ("RADIO HEIGHT", "feet"),
        convert: Convert::None,
        confidence: Confidence::Verified,
        note: "Feet both sides.",
    },
    Input {
        field: "sink_rate_m_s",
        xplane: "sim/flightmodel/position/local_vy",
        msfs: ("VELOCITY WORLD Y", "feet per second"),
        convert: Convert::Scale(1.0 / M_TO_FT),
        confidence: Confidence::Unverified,
        note: "Positive up in both. Used for touchdown sink rate, so the \
               unit matters: the certification limits it is checked against \
               are 3.05 m/s at MLW and 1.83 at MTOW.",
    },
    Input {
        field: "speedbrake_ratio",
        xplane: "sim/cockpit2/controls/speedbrake_ratio",
        msfs: ("SPOILERS HANDLE POSITION", "percent over 100"),
        convert: Convert::None,
        confidence: Confidence::Unverified,
        note: "0..1 both sides. FlyByWire may publish their own lever \
               position as an LVar, which would be better than the simvar \
               because it is the one their own systems act on.",
    },
    Input {
        field: "sun_elevation_deg",
        xplane: "sim/graphics/scenery/sun_pitch_degrees",
        msfs: ("", ""),
        convert: Convert::None,
        confidence: Confidence::Unverified,
        note: "NO KNOWN COUNTERPART. MSFS exposes no sun elevation directly. \
               It is derivable from ZULU TIME plus latitude and longitude by \
               the standard solar position formulae, which is honest work \
               rather than a guess -- but it is work, and until it is done \
               this input has no source.",
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_input_the_plugin_reads_from_x_plane_is_accounted_for() {
        assert_eq!(INPUTS.len(), 10);
    }

    #[test]
    fn a_row_without_a_source_says_so_rather_than_naming_a_plausible_variable() {
        let unsourced: Vec<&str> =
            INPUTS.iter().filter(|i| i.msfs.0.is_empty()).map(|i| i.field).collect();
        assert_eq!(
            unsourced,
            vec!["sun_elevation_deg"],
            "a field with no MSFS source must be empty and explained, not filled with a guess"
        );
        let sun = INPUTS.iter().find(|i| i.field == "sun_elevation_deg").unwrap();
        assert!(sun.note.contains("NO KNOWN COUNTERPART"));
    }

    #[test]
    fn the_pitch_conversion_flips_sign_as_well_as_unit() {
        let pitch = INPUTS.iter().find(|i| i.field == "pitch_deg").unwrap();
        assert_eq!(pitch.msfs.1, "radians");
        let msfs_raw = -10.0 / DEG_PER_RAD;
        let truth = pitch.convert.apply(msfs_raw);
        assert!((truth - 10.0).abs() < 1e-9, "nose up must come out positive, got {truth}");
    }

    #[test]
    fn conversions_round_trip_against_their_own_constants() {
        let mass = INPUTS.iter().find(|i| i.field == "aircraft_mass_kg").unwrap();
        let pounds = 575_000.0 / LB_TO_KG;
        assert!((mass.convert.apply(pounds) - 575_000.0).abs() < 1e-6);

        let gs = INPUTS.iter().find(|i| i.field == "groundspeed_m_s").unwrap();
        assert!((gs.convert.apply(100.0) - 51.444_444_444_444_44).abs() < 1e-9);
    }

    #[test]
    fn a_unit_that_needs_no_conversion_is_marked_none_not_scaled_by_one() {
        for input in INPUTS {
            if let Convert::Scale(k) = input.convert {
                assert!(
                    (k - 1.0).abs() > f64::EPSILON,
                    "{} uses Scale(1.0); say Convert::None and mean it",
                    input.field
                );
            }
        }
    }

    #[test]
    fn unverified_rows_are_named_so_they_can_be_checked_before_being_trusted() {
        let unverified: Vec<&str> = INPUTS
            .iter()
            .filter(|i| i.confidence == Confidence::Unverified)
            .map(|i| i.field)
            .collect();
        assert_eq!(
            unverified,
            vec!["sink_rate_m_s", "speedbrake_ratio", "sun_elevation_deg"],
            "three rows are not yet confirmed against the simulator"
        );
    }
}

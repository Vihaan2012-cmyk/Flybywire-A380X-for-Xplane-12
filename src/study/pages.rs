//! The rest of the study pages: fuel drawn across the wing, flight control
//! surfaces as gauges, the systems best read as boxes of fields, every
//! variable the simulation holds, and one variable followed.

use std::collections::VecDeque;
use std::ffi::c_int;

use super::canvas::{
    arinc, format_value, group, looks_packed, lamp, num, number, palette as p, unpack_arinc, arinc_status, Action, Canvas, Field,
    Group, Reading, Show, Theme, DESIGN_W,
};
use super::PageKind;
use crate::xp::{FONT_BASIC, FONT_PROPORTIONAL};

/// The pastel colours the boxes take in turn.
const THEMES: [Theme; 7] = [p::GREEN, p::BLUE, p::YELLOW, p::ORANGE, p::MAGENTA, p::CYAN, p::RED];

pub(super) fn tint(i: usize) -> Theme {
    THEMES[i % THEMES.len()]
}

/// A box that leaves out the fields the simulation does not hold.
fn hidden(g: Group) -> Group {
    Group { hide_missing: true, ..g }
}

pub(super) fn hidden_group(g: Group) -> Group {
    hidden(g)
}

/// The boxes of a page that is read as boxes.
pub fn groups(kind: PageKind) -> Vec<Group> {
    let mut groups = base_groups(kind);
    groups.extend(super::depth::extra(kind));
    groups
}

fn base_groups(kind: PageKind) -> Vec<Group> {
    match kind {
        PageKind::Apu => apu(),
        PageKind::Bleed => bleed(),
        PageKind::AirConditioning => air_conditioning(),
        PageKind::Pressurisation => pressurisation(),
        PageKind::GearBrakes => gear_brakes(),
        PageKind::AirData => air_data(),
        PageKind::Fire => fire(),
        PageKind::Loadsheet => loadsheet(),
        _ => Vec::new(),
    }
}

fn apu() -> Vec<Group> {
    let gen = |n: usize| {
        let g = format!("A32NX_ELEC_APU_GEN_{n}");
        vec![
            num("Potential", format!("{g}_POTENTIAL"), "V", 0),
            num("Frequency", format!("{g}_FREQUENCY"), "Hz", 0),
            num("Load", format!("{g}_LOAD"), "%", 0),
            num("Current", format!("{g}_CURRENT"), "A", 0),
        ]
    };
    vec![
        group(
            "Rotor",
            tint(0),
            vec![
                num("N", "A32NX_APU_N", "%", 1),
                num("N unfiltered", "A32NX_APU_N_RAW", "%", 1),
                num("N2", "A32NX_APU_N2", "%", 1),
            ],
        ),
        group("Exhaust", tint(3), vec![arinc("EGT", "A32NX_APU_EGT", "C", 0)]),
        group(
            "Shutdown",
            tint(6),
            vec![
                lamp("Auto shutdown", "A32NX_APU_IS_AUTO_SHUTDOWN"),
                lamp("Emergency shutdown", "A32NX_APU_IS_EMERGENCY_SHUTDOWN"),
                lamp("Low fuel pressure", "A32NX_APU_LOW_FUEL_PRESSURE_FAULT"),
                lamp("On fire", "A32NX_APU_ON_FIRE"),
            ],
        ),
        group(
            "Air Intake Flap",
            tint(1),
            vec![
                num("Open", "A32NX_APU_FLAP_OPEN_PERCENTAGE", "%", 0),
                lamp("Fully open", "A32NX_APU_FLAP_FULLY_OPEN"),
            ],
        ),
        group(
            "Bleed",
            tint(5),
            vec![
                lamp("Bleed valve open", "A32NX_APU_BLEED_AIR_VALVE_OPEN"),
                arinc("Bleed pressure", "A32NX_APU_BLEED_AIR_PRESSURE", "psi", 1),
            ],
        ),
        group("Generator A", tint(2), gen(1)),
        group("Generator B", tint(2), gen(2)),
        group(
            "Controls",
            tint(4),
            vec![
                lamp("Master switch on", "A32NX_OVHD_APU_MASTER_SW_PB_IS_ON"),
                lamp("Master switch fault", "A32NX_OVHD_APU_MASTER_SW_PB_HAS_FAULT"),
                lamp("Start pb on", "A32NX_OVHD_APU_START_PB_IS_ON"),
                lamp("Start pb available", "A32NX_OVHD_APU_START_PB_IS_AVAILABLE"),
            ],
        ),
        group("Fuel", tint(3), vec![num("Fuel used", "A32NX_APU_FUEL_USED", "", 1)]),
    ]
}

fn bleed() -> Vec<Group> {
    let mut groups: Vec<Group> = (1..=4)
        .map(|n| {
            let e = |what: &str| format!("A32NX_PNEU_ENG_{n}_{what}");
            group(
                &format!("Engine {n}"),
                tint(n - 1),
                vec![
                    num("HP pressure", e("HP_PRESSURE"), "psi", 1),
                    num("HP temperature", e("HP_TEMPERATURE"), "C", 1),
                    lamp("HP valve open", e("HP_VALVE_OPEN")),
                    num("IP pressure", e("INTERMEDIATE_TRANSDUCER_PRESSURE"), "psi", 1),
                    num("IP temperature", e("IP_TEMPERATURE"), "C", 1),
                    lamp("IP valve open", e("IP_VALVE_OPEN")),
                    lamp("PR valve open", e("PR_VALVE_OPEN")),
                    num("Precooler in", e("PRECOOLER_INLET_TEMPERATURE"), "C", 1),
                    num("Precooler out", e("PRECOOLER_OUTLET_TEMPERATURE"), "C", 1),
                    num("Regulated", e("REGULATED_TRANSDUCER_PRESSURE"), "psi", 1),
                    num("Transfer", e("TRANSFER_TRANSDUCER_PRESSURE"), "psi", 1),
                    num("Transfer temp", e("TRANSFER_TEMPERATURE"), "C", 1),
                    lamp("Bleed pb auto", format!("A32NX_OVHD_PNEU_ENG_{n}_BLEED_PB_IS_AUTO")),
                    lamp("Bleed pb fault", format!("A32NX_OVHD_PNEU_ENG_{n}_BLEED_PB_HAS_FAULT")),
                ],
            )
        })
        .collect();
    groups.push(group(
        "APU Bleed",
        tint(5),
        vec![
            lamp("Valve open", "A32NX_APU_BLEED_AIR_VALVE_OPEN"),
            arinc("Pressure", "A32NX_APU_BLEED_AIR_PRESSURE", "psi", 1),
        ],
    ));
    groups
}

fn air_conditioning() -> Vec<Group> {
    let zone = |label: &str, zone: &str| {
        vec![
            num(&format!("{label} temp"), format!("A32NX_COND_{zone}_TEMP"), "C", 1),
            num(&format!("{label} duct"), format!("A32NX_COND_{zone}_DUCT_TEMP"), "C", 1),
            num(&format!("{label} trim valve"), format!("A32NX_COND_{zone}_TRIM_AIR_VALVE_POSITION"), "", 2),
        ]
    };
    let mut main_deck: Vec<Field> = Vec::new();
    for n in 1..=8 {
        main_deck.extend(zone(&format!("Zone {n}"), &format!("MAIN_DECK_{n}")));
    }
    let mut upper_deck: Vec<Field> = Vec::new();
    for n in 1..=7 {
        upper_deck.extend(zone(&format!("Zone {n}"), &format!("UPPER_DECK_{n}")));
    }
    let mut groups = vec![
        group("Cockpit", tint(0), zone("Cockpit", "CKPT")),
        hidden(group("Main Deck", tint(1), main_deck)),
        hidden(group("Upper Deck", tint(2), upper_deck)),
        group(
            "Cargo",
            tint(3),
            zone("Forward", "CARGO_FWD").into_iter().chain(zone("Bulk", "CARGO_BULK")).collect(),
        ),
        group(
            "Controllers",
            tint(6),
            vec![
                lamp("FDAC 1 channel 1 failed", "A32NX_COND_FDAC_1_CHANNEL_1_FAILURE"),
                lamp("FDAC 1 channel 2 failed", "A32NX_COND_FDAC_1_CHANNEL_2_FAILURE"),
                lamp("FDAC 2 channel 1 failed", "A32NX_COND_FDAC_2_CHANNEL_1_FAILURE"),
                lamp("FDAC 2 channel 2 failed", "A32NX_COND_FDAC_2_CHANNEL_2_FAILURE"),
            ],
        ),
    ];
    groups.push(hidden(group(
            "Overhead",
            tint(4),
            vec![
                lamp("Pack 1 on", "A32NX_OVHD_COND_PACK_1_PB_IS_ON"),
                lamp("Pack 2 on", "A32NX_OVHD_COND_PACK_2_PB_IS_ON"),
                lamp("Hot air 1 on", "A32NX_OVHD_COND_HOT_AIR_1_PB_IS_ON"),
                lamp("Hot air 2 on", "A32NX_OVHD_COND_HOT_AIR_2_PB_IS_ON"),
                lamp("Ram air on", "A32NX_OVHD_COND_RAM_AIR_PB_IS_ON"),
                num("Cockpit selector", "A32NX_OVHD_COND_CKPT_SELECTOR_KNOB", "", 0),
                num("Cabin selector", "A32NX_OVHD_COND_CABIN_SELECTOR_KNOB", "", 0),
                num("Purser selection", "A32NX_COND_PURS_SEL_TEMPERATURE", "C", 1),
            ],
    )));
    groups
}

fn pressurisation() -> Vec<Group> {
    let mut groups: Vec<Group> = (1..=4)
        .map(|b| {
            let v = |what: &str| format!("A32NX_PRESS_{what}_B{b}");
            group(
                &format!("CPIOM B{b}"),
                tint(b - 1),
                vec![
                    arinc("Cabin altitude", v("CABIN_ALTITUDE"), "ft", 0),
                    arinc("Target altitude", v("CABIN_ALTITUDE_TARGET"), "ft", 0),
                    arinc("Cabin v/s", v("CABIN_VS"), "ft/min", 0),
                    arinc("Target v/s", v("CABIN_VS_TARGET"), "ft/min", 0),
                    arinc("Differential", v("CABIN_DELTA_PRESSURE"), "psi", 2),
                ],
            )
        })
        .collect();
    groups.push(group(
        "Overhead",
        tint(4),
        vec![
            lamp("Man altitude pb auto", "A32NX_OVHD_PRESS_MAN_ALTITUDE_PB_IS_AUTO"),
            lamp("Man altitude pb fault", "A32NX_OVHD_PRESS_MAN_ALTITUDE_PB_HAS_FAULT"),
            num("Man altitude knob", "A32NX_OVHD_PRESS_MAN_ALTITUDE_KNOB", "", 0),
            lamp("Man v/s pb auto", "A32NX_OVHD_PRESS_MAN_VS_CTL_PB_IS_AUTO"),
            lamp("Man v/s pb fault", "A32NX_OVHD_PRESS_MAN_VS_CTL_PB_HAS_FAULT"),
            num("Man v/s knob", "A32NX_OVHD_PRESS_MAN_VS_CTL_KNOB", "", 0),
            lamp("Ditching on", "A32NX_OVHD_PRESS_DITCHING_PB_IS_ON"),
        ],
    ));
    groups
}

fn gear_brakes() -> Vec<Group> {
    let lgciu: Vec<Field> = ["LEFT", "RIGHT", "NOSE"]
        .iter()
        .flat_map(|leg| {
            let title = match *leg {
                "LEFT" => "Left",
                "RIGHT" => "Right",
                _ => "Nose",
            };
            vec![
                lamp(&format!("{title} downlocked"), format!("A32NX_LGCIU_1_{leg}_GEAR_DOWNLOCKED")),
                lamp(&format!("{title} unlocked"), format!("A32NX_LGCIU_1_{leg}_GEAR_UNLOCKED")),
                lamp(&format!("{title} compressed"), format!("A32NX_LGCIU_1_{leg}_GEAR_COMPRESSED")),
            ]
        })
        .collect();
    vec![
        group(
            "Gear Position",
            tint(0),
            vec![
                num("Left", "A32NX_GEAR_LEFT_POSITION", "", 2),
                num("Right", "A32NX_GEAR_RIGHT_POSITION", "", 2),
                num("Centre", "A32NX_GEAR_CENTER_POSITION", "", 2),
                num("Centre small", "A32NX_GEAR_CENTER_SMALL_POSITION", "", 2),
                num("Handle", "A32NX_GEAR_HANDLE_POSITION", "", 2),
                lamp("Lever locked", "A32NX_GEAR_LEVER_LOCKED"),
            ],
        ),
        group(
            "Doors and Tilt",
            tint(1),
            vec![
                num("Left door", "A32NX_GEAR_DOOR_LEFT_POSITION", "", 2),
                num("Right door", "A32NX_GEAR_DOOR_RIGHT_POSITION", "", 2),
                num("Centre door", "A32NX_GEAR_DOOR_CENTER_POSITION", "", 2),
                num("Bogie 1 tilt", "A32NX_GEAR_1_TILT_POSITION", "", 2),
                num("Bogie 2 tilt", "A32NX_GEAR_2_TILT_POSITION", "", 2),
                num("Bogie 3 tilt", "A32NX_GEAR_3_TILT_POSITION", "", 2),
                num("Bogie 4 tilt", "A32NX_GEAR_4_TILT_POSITION", "", 2),
            ],
        ),
        group("LGCIU 1", tint(5), lgciu),
        group(
            "Ground Contact",
            tint(2),
            vec![
                num("Weight on wheels", "A32NX_GROUND_WEIGHT_ON_WHEELS_RATIO", "", 2),
                num("Nose compression", "CONTACT POINT COMPRESSION:0", "%", 1),
                num("Leg 1 compression", "CONTACT POINT COMPRESSION:1", "%", 1),
                num("Leg 2 compression", "CONTACT POINT COMPRESSION:2", "%", 1),
                num("Leg 3 compression", "CONTACT POINT COMPRESSION:3", "%", 1),
                num("Leg 4 compression", "CONTACT POINT COMPRESSION:4", "%", 1),
            ],
        ),
        group(
            "Brake Pressure",
            tint(6),
            vec![
                num("Normal left", "A32NX_HYD_BRAKE_NORM_LEFT_PRESS", "psi", 0),
                num("Normal right", "A32NX_HYD_BRAKE_NORM_RIGHT_PRESS", "psi", 0),
                num("Normal accumulator", "A32NX_HYD_BRAKE_NORM_ACC_PRESS", "psi", 0),
                num("Alternate left", "A32NX_HYD_BRAKE_ALTN_LEFT_PRESS", "psi", 0),
                num("Alternate right", "A32NX_HYD_BRAKE_ALTN_RIGHT_PRESS", "psi", 0),
                num("Alternate accumulator", "A32NX_HYD_BRAKE_ALTN_ACC_PRESS", "psi", 0),
                num("Left force", "BRAKE LEFT FORCE FACTOR", "", 2),
                num("Right force", "BRAKE RIGHT FORCE FACTOR", "", 2),
                lamp("Antiskid", "ANTISKID BRAKES ACTIVE"),
            ],
        ),
        group(
            "Autobrake",
            tint(3),
            vec![
                lamp("Active", "A32NX_AUTOBRAKES_ACTIVE"),
                num("Armed mode", "A32NX_AUTOBRAKES_ARMED_MODE", "", 0),
                num("Selected mode", "A32NX_AUTOBRAKES_SELECTED_MODE", "", 0),
                lamp("Decel light", "A32NX_AUTOBRAKES_DECEL_LIGHT"),
                lamp("RTO armed", "A32NX_AUTOBRAKES_RTO_ARMED"),
            ],
        ),
        hidden(group(
            "Brake Temperature",
            tint(4),
            (1..=16)
                .map(|n| num(&format!("Brake {n}"), format!("A32NX_BRAKE_TEMPERATURE_{n}"), "C", 0))
                .collect(),
        )),
    ]
}

fn air_data() -> Vec<Group> {
    let mut groups: Vec<Group> = (1..=3)
        .map(|n| {
            let a = |what: &str| format!("A32NX_ADIRS_ADR_{n}_{what}");
            group(
                &format!("ADR {n}"),
                tint(n - 1),
                vec![
                    arinc("Computed airspeed", a("COMPUTED_AIRSPEED"), "kt", 1),
                    arinc("True airspeed", a("TRUE_AIRSPEED"), "kt", 1),
                    arinc("Mach", a("MACH"), "", 3),
                    arinc("Altitude", a("ALTITUDE"), "ft", 0),
                    arinc("Baro altitude", a("BARO_CORRECTED_ALTITUDE_1"), "ft", 0),
                    arinc("Vertical speed", a("BAROMETRIC_VERTICAL_SPEED"), "ft/min", 0),
                    arinc("Static air temp", a("STATIC_AIR_TEMPERATURE"), "C", 1),
                    arinc("Total air temp", a("TOTAL_AIR_TEMPERATURE"), "C", 1),
                    arinc("Angle of attack", a("ANGLE_OF_ATTACK"), "deg", 1),
                ],
            )
        })
        .collect();
    groups.extend((1..=3).map(|n| {
        let i = |what: &str| format!("A32NX_ADIRS_IR_{n}_{what}");
        group(
            &format!("IR {n}"),
            tint(n + 2),
            vec![
                arinc("Pitch", i("PITCH"), "deg", 2),
                arinc("Roll", i("ROLL"), "deg", 2),
                arinc("Heading", i("HEADING"), "deg", 1),
                arinc("Pitch rate", i("BODY_PITCH_RATE"), "deg/s", 2),
                arinc("Roll rate", i("BODY_ROLL_RATE"), "deg/s", 2),
                arinc("Yaw rate", i("BODY_YAW_RATE"), "deg/s", 2),
                arinc("Normal accel", i("BODY_NORMAL_ACC"), "g", 2),
            ],
        )
    }));
    groups.push(group(
        "From X-Plane",
        p::STATION,
        vec![
            num("Indicated airspeed", "AIRSPEED INDICATED", "kt", 1),
            num("True airspeed", "AIRSPEED TRUE", "kt", 1),
            num("Mach", "AIRSPEED MACH", "", 3),
            num("Pressure altitude", "PRESSURE ALTITUDE", "ft", 0),
            num("Ambient temperature", "AMBIENT TEMPERATURE", "C", 1),
            num("Ambient pressure", "AMBIENT PRESSURE", "inHg", 2),
            num("Pitch", "PLANE PITCH DEGREES", "deg", 2),
            num("Bank", "PLANE BANK DEGREES", "deg", 2),
            num("Heading true", "PLANE HEADING DEGREES TRUE", "deg", 1),
            num("Vertical speed", "VELOCITY WORLD Y", "ft/min", 0),
            lamp("On ground", "SIM ON GROUND"),
        ],
    ));
    groups
}

/// The loadsheet as the browser panel shows it.
///
/// The in-sim window draws its own richer version (`study::loadsheet`),
/// with the boarding and SimBrief buttons; this is the same figures as
/// plain variable groups, because that is what the panel renders. Without
/// it the page appeared in the menu and came up blank -- `base_groups`
/// falls through to an empty list for any kind it does not name.
fn loadsheet() -> Vec<Group> {
    let weights = group(
        "Weights and Balance",
        tint(0),
        vec![
            num("Zero fuel weight", "A32NX_AIRFRAME_ZFW", "kg", 0),
            num("Zero fuel CG", "A32NX_AIRFRAME_ZFW_CG_PERCENT_MAC", "% MAC", 1),
            num("Gross weight", "A32NX_AIRFRAME_GW", "kg", 0),
            num("Gross weight CG", "A32NX_AIRFRAME_GW_CG_PERCENT_MAC", "% MAC", 1),
            num("Take-off weight", "A32NX_AIRFRAME_TOW", "kg", 0),
            num("Take-off CG", "A32NX_AIRFRAME_TO_CG_PERCENT_MAC", "% MAC", 1),
            // X-Plane's own, which is what actually flies. It differs from
            // the above whenever no loadsheet has been published and the
            // plugin is therefore leaving X-Plane's balance alone
            // (`weight_balance.rs`), and that difference is the thing worth
            // seeing.
            num("X-Plane total weight", "TOTAL WEIGHT", "lb", 0),
        ],
    );
    let desired = group(
        "Boarding Toward",
        tint(1),
        vec![
            num("Zero fuel weight", "A32NX_AIRFRAME_ZFW_DESIRED", "kg", 0),
            num("Zero fuel CG", "A32NX_AIRFRAME_ZFW_CG_PERCENT_MAC_DESIRED", "% MAC", 1),
            num("Gross weight", "A32NX_AIRFRAME_GW_DESIRED", "kg", 0),
            num("Boarding in progress", "A32NX_BOARDING_STARTED_BY_USR", "", 0),
            num("Boarding rate", "A32NX_BOARDING_RATE", "", 0),
        ],
    );
    let pax = group(
        "Passengers",
        tint(2),
        super::loadsheet::PAX_ZONES.iter().map(|(label, var)| num(label, *var, "pax", 0)).collect(),
    );
    let cargo = group(
        "Cargo",
        tint(3),
        super::loadsheet::CARGO_HOLDS.iter().map(|(label, var)| num(label, *var, "kg", 0)).collect(),
    );
    vec![weights, desired, pax, cargo]
}

fn fire() -> Vec<Group> {
    let mut groups: Vec<Group> = (1..=4)
        .map(|n| {
            group(
                &format!("Engine {n}"),
                tint(n - 1),
                vec![
                    lamp("Fire detected", format!("A32NX_FIRE_DETECTED_ENG{n}")),
                    lamp("On fire", format!("A32NX_ENG_{n}_ON_FIRE")),
                    lamp("Fire pb released", format!("A32NX_FIRE_BUTTON_ENG{n}")),
                    lamp("Squib 1 armed", format!("A32NX_FIRE_SQUIB_1_ENG_{n}_IS_ARMED")),
                    lamp("Bottle 1 discharged", format!("A32NX_FIRE_SQUIB_1_ENG_{n}_IS_DISCHARGED")),
                    lamp("Squib 2 armed", format!("A32NX_FIRE_SQUIB_2_ENG_{n}_IS_ARMED")),
                    lamp("Bottle 2 discharged", format!("A32NX_FIRE_SQUIB_2_ENG_{n}_IS_DISCHARGED")),
                    lamp("Agent 1 pressed", format!("A32NX_OVHD_FIRE_AGENT_1_ENG_{n}_IS_PRESSED")),
                    lamp("Agent 2 pressed", format!("A32NX_OVHD_FIRE_AGENT_2_ENG_{n}_IS_PRESSED")),
                ],
            )
        })
        .collect();
    groups.push(group(
        "APU",
        tint(4),
        vec![
            lamp("Fire detected", "A32NX_FIRE_DETECTED_APU"),
            lamp("On fire", "A32NX_APU_ON_FIRE"),
            lamp("Fire pb released", "A32NX_FIRE_BUTTON_APU"),
            lamp("Squib armed", "A32NX_FIRE_SQUIB_1_APU_1_IS_ARMED"),
            lamp("Bottle discharged", "A32NX_FIRE_SQUIB_1_APU_1_IS_DISCHARGED"),
        ],
    ));
    groups.push(group(
        "Other",
        tint(6),
        vec![
            lamp("Main gear bay fire", "A32NX_FIRE_DETECTED_MLG"),
            lamp("Fire test pressed", "A32NX_OVHD_FIRE_TEST_PB_IS_PRESSED"),
        ],
    ));
    groups
}

/// Boxes flowing across the page and down it, scrolled by `scroll` pixels.
/// Returns how far the content runs past the foot of the page.
pub fn flow(cv: &mut Canvas, groups: &[Group], scroll: c_int) -> c_int {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);
    let gap = 10;
    let (mut x, mut y) = (l + gap, t - gap + scroll);
    let mut lowest = y;
    for g in groups {
        let fields: Vec<&Field> = g
            .fields
            .iter()
            .filter(|f| !g.hide_missing || cv.snap.find(&f.name).is_some())
            .collect();
        if fields.is_empty() {
            continue;
        }
        let width = cv.fit_width(&g.title, &fields).min(r - l - gap * 2);
        if x + width > r - gap && x > l + gap {
            x = l + gap;
            y = lowest - gap;
        }
        let height = cv.box_height(fields.len());
        // Boxes wholly above or below the page are measured, not drawn.
        if y - height < t && y > b {
            cv.box_px(x, y, width, &g.title, &g.theme, &fields);
        }
        lowest = lowest.min(y - height);
        x += width + gap;
    }
    (b + gap - (lowest - scroll)).max(0)
}

/// The A380's eleven tanks as FlyByWire numbers them, with where each sits
/// across the wing on the design sheet and its capacity in US gallons.
const TANKS: [(usize, &str, f32, f32, f64); 11] = [
    (1, "OUTER", 20., 85., 2731.5),
    (2, "FEED 1", 110., 70., 7299.6),
    (3, "MID", 185., 95., 9632.),
    (5, "FEED 2", 285., 70., 7753.2),
    (4, "INNER", 360., 105., 12189.4),
    (7, "INNER", 535., 105., 12189.4),
    (6, "FEED 3", 645., 70., 7753.2),
    (8, "MID", 720., 95., 9632.),
    (9, "FEED 4", 820., 70., 7299.6),
    (10, "OUTER", 895., 85., 2731.5),
    (11, "TRIM", 430., 140., 6260.3),
];

/// The tank layout, for the web Study tab's fuel page (id, label, x, width,
/// capacity in US gallons; the same numbers [`fuel`] draws from).
pub(super) fn tanks() -> &'static [(usize, &'static str, f32, f32, f64)] {
    &TANKS
}

/// The fuel system as a live diagram: the eleven tanks positioned as
/// [`fuel`] draws them, plus the pump and quantity/weight boxes.
pub(super) fn fuel_topology() -> super::elec::Topology {
    use super::elec::{field_to_topo, node, TopoNode, Topology};
    let mut nodes: Vec<TopoNode> = Vec::new();
    for &(n, label, x, w, cap) in TANKS.iter() {
        let y = if n == 11 { 470. } else { 176. };
        let name = format!("A32NX_FUEL_TANK_QUANTITY_{n}");
        let _ = cap;
        node(&mut nodes, &format!("tank{n}"), label, x, y, w, "bus", vec![super::elec::lfield("Qty", name, "gal", 0)]);
    }
    for (x, y, w, title, _theme, fields) in fuel_extra_stations() {
        let topo_fields = fields.iter().map(field_to_topo).collect();
        node(&mut nodes, title, title, x, y, w, "source", topo_fields);
    }
    Topology { design_w: DESIGN_W, design_h: 600., nodes, links: Vec::new() }
}

pub fn fuel(cv: &mut Canvas) {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);

    // The aeroplane from above, behind the tanks.
    let fuselage = [(470., 30.), (485., 12.), (515., 12.), (530., 30.), (530., 560.), (500., 590.), (470., 560.)];
    cv.poly(&fuselage, [0.80, 0.80, 0.88, 1.]);
    cv.outline(&fuselage, p::EDGE, 1.);
    let left_wing = [(470., 150.), (470., 340.), (12., 310.), (12., 190.)];
    let right_wing: Vec<(f32, f32)> = left_wing.iter().map(|&(x, y)| (DESIGN_W - x, y)).collect();
    for wing in [left_wing.to_vec(), right_wing] {
        cv.poly(&wing, [0.80, 0.80, 0.88, 1.]);
        cv.outline(&wing, p::EDGE, 1.);
    }

    let mut total = 0.;
    let mut all_read = true;
    for &(n, label, x, w, capacity) in &TANKS {
        let y = if n == 11 { 470. } else { 176. };
        let h = if n == 11 { 80. } else { 150. };
        let name = format!("A32NX_FUEL_TANK_QUANTITY_{n}");
        let (lx, ty, rx, by) = (cv.x(x), cv.y(y), cv.x(x + w), cv.y(y + h));
        cv.fill_px(lx, ty, rx, by, p::STATION.body);
        let lh = cv.line_h;
        match cv.read(&name) {
            Reading::Missing => {
                all_read = false;
                cv.text_centred_px((lx + rx) / 2, ty - lh + 3, p::INK, FONT_BASIC, label);
                cv.text_centred_px((lx + rx) / 2, ty - 2 * lh + 3, p::INK_DIM, FONT_BASIC, "not modelled");
            }
            Reading::Value { index, value, source } => {
                let gallons = if value.is_finite() { value } else { 0. };
                total += gallons;
                let level = (gallons / capacity).clamp(0., 1.);
                let fill_top = by + ((ty - by) as f64 * level) as c_int;
                cv.fill_px(lx, fill_top, rx, by, p::CYAN.body);
                if cv.hovering(lx, ty, rx, by) {
                    cv.fill_px(lx, ty, rx, by, p::HOVER);
                    cv.tip = Some(index);
                }
                let ink = if source == 0 { p::INK_DIM } else { p::INK };
                cv.text_centred_px((lx + rx) / 2, ty - lh + 3, p::INK, FONT_BASIC, label);
                cv.text_centred_px((lx + rx) / 2, ty - 2 * lh + 3, ink, FONT_BASIC, &format!("{gallons:.0}"));
                cv.text_centred_px((lx + rx) / 2, ty - 3 * lh + 3, ink, FONT_BASIC, "gal");
                cv.text_centred_px((lx + rx) / 2, ty - 4 * lh + 3, ink, FONT_BASIC, &format!("{:.0}%", level * 100.));
                if source == 0 {
                    cv.fill_px(lx + 2, ty - 2, lx + 5, by + 2, p::UNFED);
                }
                cv.hit_px(lx, ty, rx, by, Action::Follow(index));
            }
        }
        cv.frame_px(lx, ty, rx, by, p::EDGE, 1.);
    }
    if all_read {
        cv.text(560., 60., p::INK, FONT_PROPORTIONAL, &format!("Sum of the tank readings: {total:.0} gal"));
    }

    for (x, y, w, title, theme, fields) in fuel_extra_stations() {
        cv.station(x, y, w, title, &theme, &fields);
    }
}

/// The pump and quantity/weight boxes below the tanks: their design-sheet
/// position and fields, drawn by [`fuel`] and read by the web Study tab.
pub(super) fn fuel_extra_stations() -> Vec<(f32, f32, f32, &'static str, Theme, Vec<Field>)> {
    let pumps = |from: usize| -> Vec<Field> {
        (from..from + 10)
            .map(|k| lamp(&format!("Pump {k}"), format!("FUELSYSTEM PUMP ACTIVE:{k}")))
            .collect()
    };
    vec![
        (20., 350., 160., "Pumps 1 to 10", p::GREEN, pumps(1)),
        (190., 350., 160., "Pumps 11 to 20", p::GREEN, pumps(11)),
        (
            600.,
            350.,
            385.,
            "Quantity and Weight",
            p::ORANGE,
            vec![
                num("Total on board (FQMS)", "A32NX_FQMS_TOTAL_FUEL_ON_BOARD", "", 0),
                num("Gross weight (FQMS)", "A32NX_FQMS_GROSS_WEIGHT", "", 0),
                num("Fuel desired", "A32NX_FUEL_DESIRED", "", 0),
                num("Total weight", "TOTAL WEIGHT", "lb", 0),
                num("Line fuel flow", "FUELSYSTEM LINE FUEL FLOW:141", "gal/h", 1),
                num("APU fuel used", "A32NX_APU_FUEL_USED", "", 1),
                num("Refuel rate setting", "A32NX_EFB_REFUEL_RATE_SETTING", "", 0),
            ],
        ),
    ]
}

/// Rows of gauges down a column.
struct Column {
    l: c_int,
    r: c_int,
    y: c_int,
}

fn heading(cv: &Canvas, col: &mut Column, text: &str) {
    col.y -= 4;
    cv.text_px(col.l + 4, col.y - cv.line_h + 4, p::ECAM_CYAN, FONT_PROPORTIONAL, text);
    cv.line_px(col.l, col.y - cv.line_h, col.r, col.y - cv.line_h, p::ECAM_DEAD, 1.);
    col.y -= cv.line_h + 3;
}

#[allow(clippy::too_many_arguments)]
fn gauge(cv: &mut Canvas, col: &mut Column, label: &str, name: &str, low: f64, high: f64, unit: &'static str) {
    cv.gauge_px(col.l, col.r, col.y, label, name, low, high, unit);
    col.y -= cv.line_h;
}

/// One gauge on the flight controls page: its label, variable, and the range
/// its bar spans.
pub(super) struct GaugeSpec {
    pub label: String,
    pub name: String,
    pub low: f64,
    pub high: f64,
    pub unit: &'static str,
}

fn gspec(label: impl Into<String>, name: impl Into<String>, low: f64, high: f64, unit: &'static str) -> GaugeSpec {
    GaugeSpec { label: label.into(), name: name.into(), low, high, unit }
}

/// The flight controls page's two columns, each a list of headed sections of
/// gauges: the same data the drawn page and the web Study tab's JSON export
/// both read.
pub(super) fn flight_controls_columns() -> [Vec<(&'static str, Vec<GaugeSpec>)>; 2] {
    let mut ailerons = Vec::new();
    for side in ["LEFT", "RIGHT"] {
        for part in ["INWARD", "MIDDLE", "OUTWARD"] {
            let label = format!("{} {}", title_case(side), part.to_lowercase());
            ailerons.push(gspec(label, format!("A32NX_HYD_AIL_{side}_{part}_DEFLECTION"), 0., 1., ""));
        }
    }
    let mut elevators = Vec::new();
    for side in ["LEFT", "RIGHT"] {
        for part in ["INWARD", "OUTWARD"] {
            let label = format!("{} {}", title_case(side), part.to_lowercase());
            elevators.push(gspec(label, format!("A32NX_HYD_ELEV_{side}_{part}_DEFLECTION"), 0., 1., ""));
        }
    }
    let rudder = vec![
        gspec("Upper rudder", "A32NX_HYD_UPPER_RUD_DEFLECTION", 0., 1., ""),
        gspec("Lower rudder", "A32NX_HYD_LOWER_RUD_DEFLECTION", 0., 1., ""),
        gspec("Stabiliser", "A32NX_HYD_FINAL_THS_DEFLECTION", -15., 15., "deg"),
    ];
    let mut spoilers = Vec::new();
    for n in 1..=8 {
        for side in ["LEFT", "RIGHT"] {
            let label = format!("{} {n}", title_case(side));
            spoilers.push(gspec(label, format!("A32NX_HYD_SPOILER_{n}_{side}_DEFLECTION"), 0., 1., ""));
        }
    }
    let flaps = vec![
        gspec("Handle", "A32NX_FLAPS_HANDLE_INDEX", 0., 5., ""),
        gspec("Configuration", "A32NX_FLAPS_CONF_INDEX", 0., 5., ""),
        gspec("Left flaps", "A32NX_LEFT_FLAPS_ANGLE", 0., 40., "deg"),
        gspec("Left slats 1", "A32NX_LEFT_SLATS_1_ANGLE", 0., 30., "deg"),
        gspec("Flap FPPU", "A32NX_FLAPS_FPPU_ANGLE", 0., 360., "deg"),
        gspec("Flap IPPU", "A32NX_FLAPS_IPPU_ANGLE", 0., 360., "deg"),
    ];
    [
        vec![("AILERONS (position as FlyByWire writes it)", ailerons), ("ELEVATORS", elevators), ("RUDDER AND STABILISER", rudder)],
        vec![("SPOILERS", spoilers), ("FLAPS AND SLATS", flaps)],
    ]
}

pub fn flight_controls(cv: &mut Canvas) {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::ECAM_GROUND);
    let mid = (l + r) / 2;
    let mut cols = [Column { l: l + 8, r: mid - 10, y: t - 4 }, Column { l: mid + 10, r: r - 8, y: t - 4 }];
    cv.line_px(mid, t, mid, b, p::ECAM_DEAD, 1.);

    let columns = flight_controls_columns();
    for (col, sections) in cols.iter_mut().zip(columns.iter()) {
        for (title, gauges) in sections {
            heading(cv, col, title);
            for g in gauges {
                gauge(cv, col, &g.label, &g.name, g.low, g.high, g.unit);
            }
        }
    }
}

fn title_case(word: &str) -> String {
    let lower = word.to_lowercase();
    let mut chars = lower.chars();
    match chars.next() {
        Some(c) => c.to_uppercase().chain(chars).collect(),
        None => String::new(),
    }
}

/// The snapshot's names in alphabetical order, kept until the names change.
static mut ORDER: (u64, Vec<usize>) = (u64::MAX, Vec::new());

#[allow(static_mut_refs)]
fn sorted(snap: &crate::Snapshot) -> &'static [usize] {
    // Only the main thread draws, so the order is never shared.
    unsafe {
        let order = &mut *(&raw mut ORDER);
        if order.0 != snap.generation || order.1.len() != snap.names.len() {
            let mut indices: Vec<usize> = (0..snap.names.len()).collect();
            indices.sort_by(|&a, &b| snap.names[a].cmp(&snap.names[b]));
            *order = (snap.generation, indices);
        }
        &order.1
    }
}


/// The radios: the manual LS selection with its controls across the top
/// (the MFD POSITION/NAVAIDS LS entry, radios.rs), the receivers as boxes.
pub fn radios(cv: &mut Canvas, scroll: c_int) -> c_int {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);
    let lh = cv.line_h;
    let band_b = t - (lh * 2 + 26);
    cv.fill_px(l, t, r, band_b, p::BAND);
    let d = crate::radios::display();
    let ls = match (d.ls.frequency_mhz, d.ls.course_deg) {
        (None, None) => "LS  no manual selection (FMS tuned)".to_string(),
        (f, c) => format!(
            "LS  manual  {}  CRS {}",
            f.map_or("---.--".into(), |f| format!("{f:.2} MHz")),
            c.map_or("---".into(), |c| format!("{c:03.0}"))
        ),
    };
    let ident = |s: &str| if s.is_empty() { "----".to_string() } else { s.to_string() };
    let idents = format!(
        "IDENT  NAV1 {}  NAV2 {}  MMR1 {}  MMR2 {}  ADF1 {}  ADF2 {}",
        ident(&d.nav_ident[0]),
        ident(&d.nav_ident[1]),
        ident(&d.nav_ident[2]),
        ident(&d.nav_ident[3]),
        ident(&d.adf_ident[0]),
        ident(&d.adf_ident[1])
    );
    cv.text_px(l + 10, t - lh - 4, p::INK_LIGHT, FONT_BASIC, &ls);
    cv.text_px(l + 10, t - lh * 2 - 12, p::INK_LIGHT, FONT_BASIC, &idents);
    if let Some(c) = crate::radios::ls_controls() {
        let buttons = [
            ("FREQ -1", c.frequency_down_coarse),
            ("FREQ -.05", c.frequency_down),
            ("FREQ +.05", c.frequency_up),
            ("FREQ +1", c.frequency_up_coarse),
            ("CRS -10", c.course_down_coarse),
            ("CRS -1", c.course_down),
            ("CRS +1", c.course_up),
            ("CRS +10", c.course_up_coarse),
            ("CLR", c.clear),
        ];
        let w = 10 * cv.char_w + 8;
        let mut x = r - 10 - buttons.len() as c_int * (w + 4);
        let (bt, bb) = (t - 6, t - lh - 12);
        for (label, command) in buttons {
            cv.button_px(x, bt, x + w, bb, label, false, Action::Press(command));
            x += w + 4;
        }
    }

    let groups = radios_groups();
    cv.clip = (l, band_b - 2, r, b);
    let overflow = flow(cv, &groups, scroll);
    cv.clip = (l, t, r, b);
    overflow
}

/// The nav/adf/com/LS boxes of the radios page: the same data the drawn page
/// flows into boxes, and the web Study tab's JSON export.
pub(super) fn radios_groups() -> Vec<Group> {
    let nav = |n: usize| {
        let v = |what: &str| format!("{what}:{n}");
        let mut fields = vec![
            num("Active", v("NAV ACTIVE FREQUENCY"), "MHz", 2),
            num("Standby", v("NAV STANDBY FREQUENCY"), "MHz", 2),
            num("OBS", v("NAV OBS"), "deg", 0),
            lamp("Receiving", v("NAV HAS NAV")),
            num("To/from", v("NAV TOFROM"), "", 0),
            num("Relative bearing", v("NAV RELATIVE BEARING TO STATION"), "deg", 1),
            num("Localizer course", v("NAV LOCALIZER"), "deg", 1),
            lamp("DME", v("NAV HAS DME")),
            num("DME distance", v("NAV DME"), "nm", 1),
            num("Radial error", v("NAV RADIAL ERROR"), "deg", 1),
            num("Volume", v("NAV VOLUME"), "%", 0),
            lamp("Ident audio", v("NAV SOUND")),
        ];
        if n == 3 {
            fields.push(lamp("Glide slope", "NAV HAS GLIDE SLOPE:3"));
            fields.push(num("G/S error", "NAV GLIDE SLOPE ERROR:3", "deg", 2));
        }
        let title = match n {
            1 => "VOR 1 (NAV 1)".to_string(),
            2 => "VOR 2 (NAV 2)".to_string(),
            n => format!("MMR {} (NAV {n})", n - 2),
        };
        group(&title, tint(n - 1), fields)
    };
    let mut groups: Vec<Group> = (1..=4).map(nav).collect();
    groups.extend((1..=2).map(|n| {
        let v = |what: &str| format!("{what}:{n}");
        group(
            &format!("ADF {n}"),
            tint(n + 3),
            vec![
                num("Active", v("ADF ACTIVE FREQUENCY"), "kHz", 1),
                num("Bearing (radial)", v("ADF RADIAL"), "deg", 1),
                num("Volume", v("ADF VOLUME"), "", 2),
                lamp("Ident audio", v("ADF SOUND")),
            ],
        )
    }));
    groups.extend((1..=3).map(|n| {
        let v = |what: &str| format!("{what}:{n}");
        group(
            &format!("VHF {n} (COM {n})"),
            tint(n + 5),
            vec![
                num("Active (BCD32)", v("COM ACTIVE FREQUENCY"), "", 0),
                num("Standby (BCD32)", v("COM STANDBY FREQUENCY"), "", 0),
                num("Volume", v("COM VOLUME"), "%", 0),
                lamp("Receive", v("COM RECEIVE")),
                lamp("Transmit", v("COM TRANSMIT")),
            ],
        )
    }));
    groups.push(group(
        "LS and markers",
        p::STATION,
        vec![
            num("FM LS course", "A32NX_FM_LS_COURSE", "deg", 1),
            lamp("LOC valid", "A32NX_RADIO_RECEIVER_LOC_IS_VALID"),
            lamp("Marker audio", "MARKER SOUND"),
            num("Marker beacon", "MARKER BEACON STATE", "", 0),
        ],
    ));
    groups
}

/// Every variable the simulation holds, in columns, scrolled by `scroll`.
pub fn all(cv: &mut Canvas, scroll: c_int) -> c_int {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::ECAM_GROUND);
    let order = sorted(cv.snap);
    let lh = cv.line_h;
    let columns: usize = if r - l > 1500 { 3 } else { 2 };
    let col_w = (r - l) / columns as c_int;
    let per_column = order.len().div_ceil(columns);
    let first = (scroll / lh).max(0) as usize;
    let rows_visible = ((t - b) / lh) as usize + 2;

    for column in 0..columns {
        let cl = l + column as c_int * col_w + 6;
        let cr = cl + col_w - 14;
        for row in first..(first + rows_visible).min(per_column) {
            let k = column * per_column + row;
            let Some(&index) = order.get(k) else { break };
            let top = t - row as c_int * lh + scroll;
            let bottom = top - lh;
            if bottom < b {
                break;
            }
            let name = &cv.snap.names[index];
            let value = cv.snap.values.get(index).copied().unwrap_or(f64::NAN);
            let source = cv.snap.sources.get(index).copied().unwrap_or(0);
            if cv.hovering(cl, top, cr, bottom) {
                cv.fill_px(cl, top, cr, bottom, [1., 1., 1., 0.10]);
                cv.tip = Some(index);
            }
            let shown = if looks_packed(value) {
                format_value(value, Show::Arinc(2), "")
            } else {
                number(value)
            };
            let ink = if source == 0 { p::ECAM_GREY } else { p::ECAM_GREEN };
            let label = name.strip_prefix("A32NX_").unwrap_or(name);
            let room = ((cr - cl) / cv.char_w) as usize;
            let label_room = room.saturating_sub(shown.chars().count() + 2);
            let label: String = label.chars().take(label_room).collect();
            cv.text_px(cl + 6, bottom + 4, p::ECAM_WHITE, FONT_BASIC, &label);
            cv.text_px(cr - cv.text_width(&shown) - 4, bottom + 4, ink, FONT_BASIC, &shown);
            if source == 0 {
                cv.fill_px(cl, top - 2, cl + 2, bottom + 2, p::UNFED);
            }
            cv.hit_px(cl, top, cr, bottom, Action::Follow(index));
        }
    }
    (per_column as c_int * lh - (t - b)).max(0)
}

/// Readings a followed variable has had, recorded about four times a second.
pub struct Trace {
    pub samples: VecDeque<f64>,
    pub min: f64,
    pub max: f64,
    pub changes: u64,
    pub last: f64,
    pub first_seen: f64,
    pub last_sampled: f64,
}

/// How many readings a trace keeps: a minute at four a second.
pub const TRACE_LEN: usize = 240;

impl Trace {
    pub fn new(value: f64, time: f64) -> Self {
        Self {
            samples: VecDeque::with_capacity(TRACE_LEN),
            min: value,
            max: value,
            changes: 0,
            last: value,
            first_seen: time,
            last_sampled: f64::NEG_INFINITY,
        }
    }

    pub fn record(&mut self, value: f64, time: f64) {
        if !value.is_finite() {
            return;
        }
        if value != self.last {
            self.changes += 1;
            self.last = value;
        }
        self.min = self.min.min(value);
        self.max = self.max.max(value);
        if time - self.last_sampled >= 0.25 {
            self.last_sampled = time;
            if self.samples.len() == TRACE_LEN {
                self.samples.pop_front();
            }
            self.samples.push_back(value);
        }
    }
}

/// A label and a value on one line of a light box.
fn pair(cv: &Canvas, l: c_int, r: c_int, top: c_int, key: &str, value: &str) {
    let base = top - cv.line_h + 4;
    cv.text_px(l + 8, base, p::INK_DIM, FONT_BASIC, key);
    cv.text_px(l + 8 + 14 * cv.char_w, base, p::INK, FONT_BASIC, value);
    let _ = r;
}

/// Break text into lines of at most `width` characters, at spaces.
pub fn wrap(text: &str, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut line = String::new();
    for word in text.split(' ') {
        if !line.is_empty() && line.chars().count() + 1 + word.chars().count() > width {
            lines.push(std::mem::take(&mut line));
        }
        if !line.is_empty() {
            line.push(' ');
        }
        line.push_str(word);
    }
    if !line.is_empty() {
        lines.push(line);
    }
    lines
}

/// What a variable's source means, in words.
pub fn source_text(name: &str, source: u8) -> String {
    match source {
        1 => match crate::source_dataref(name) {
            Some(dataref) => format!("Read from X-Plane every tick: {dataref}"),
            None => "Worked out from X-Plane's own state every tick".into(),
        },
        2 => "Written by FlyByWire's systems".into(),
        _ => "No source yet: nothing writes this, so it holds the value it started at".into(),
    }
}

/// One variable followed: its reading, its source, and a plot of its history.
pub fn variable(cv: &mut Canvas, name: &str, back_to: &str, trace: Option<&Trace>) {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);
    let lh = cv.line_h;
    let back = format!("< {back_to}");
    let back_w = cv.text_width(&back) + 28;
    cv.button_px(l + 10, t - 10, l + 10 + back_w, t - 18 - lh, &back, false, Action::Back);
    let mut top = t - 30 - lh;

    let Reading::Value { index, value, source } = cv.read(name) else {
        cv.text_px(l + 12, top - lh, p::INK, FONT_PROPORTIONAL, "The simulation no longer holds this variable.");
        return;
    };

    // The reading.
    let half = (r - l) / 2;
    let reading_r = l + half - 5;
    let mut rows: Vec<(&str, String)> = vec![("Name", name.to_string()), ("Value", number(value))];
    if looks_packed(value) {
        let (v, ssm) = unpack_arinc(value);
        let status = match arinc_status(ssm) {
            Some("FW") => "failure warning",
            Some("NCD") => "no computed data",
            Some("FT") => "functional test",
            _ => "normal operation",
        };
        rows.push(("ARINC 429", format!("{v} ({status})")));
    }
    let dataref = cv.snap.datarefs.get(index).cloned().unwrap_or_default();
    rows.push(("Dataref", dataref));
    let box_h = (lh + 6) + rows.len() as c_int * lh + 6;
    cv.fill_px(l + 10, top, reading_r, top - box_h, p::STATION.body);
    cv.fill_px(l + 10, top, reading_r, top - lh - 6, p::GREEN.title);
    cv.frame_px(l + 10, top, reading_r, top - box_h, p::EDGE, 1.);
    cv.text_px(l + 18, top - lh + 1, p::INK, FONT_PROPORTIONAL, "Reading");
    let mut y = top - lh - 8;
    for (key, value) in &rows {
        pair(cv, l + 10, reading_r, y, key, value);
        y -= lh;
    }

    // Where it comes from.
    let source_l = l + half + 5;
    let width_chars = ((r - 10 - source_l - 16) / cv.char_w).max(20) as usize;
    let mut lines = wrap(&source_text(name, source), width_chars);
    if let Some(trace) = trace {
        lines.push(String::new());
        lines.push(format!("Watched for {:.0} s", (cv.snap.time - trace.first_seen).max(0.)));
        lines.push(format!("Changed {} times", trace.changes));
        lines.push(format!("Lowest {}   highest {}", number(trace.min), number(trace.max)));
    }
    let source_h = ((lh + 6) + lines.len() as c_int * lh + 6).max(box_h);
    let theme = if source == 0 { p::ORANGE } else { p::BLUE };
    cv.fill_px(source_l, top, r - 10, top - source_h, theme.body);
    cv.fill_px(source_l, top, r - 10, top - lh - 6, theme.title);
    cv.frame_px(source_l, top, r - 10, top - source_h, p::EDGE, 1.);
    cv.text_px(source_l + 8, top - lh + 1, p::INK, FONT_PROPORTIONAL, "Source");
    let mut y = top - lh - 8;
    for line in &lines {
        cv.text_px(source_l + 8, y - lh + 4, p::INK, FONT_BASIC, line);
        y -= lh;
    }
    top -= box_h.max(source_h) + 12;

    // The history.
    let (pl, pt, pr, pb) = (l + 10, top, r - 10, b + 10);
    if pt - pb < lh * 4 {
        return;
    }
    cv.fill_px(pl, pt, pr, pb, [0.07, 0.08, 0.11, 1.]);
    cv.frame_px(pl, pt, pr, pb, p::EDGE, 1.);
    let Some(trace) = trace.filter(|t| t.samples.len() >= 2) else {
        cv.text_px(pl + 10, pt - lh, [0.75, 0.77, 0.82], FONT_BASIC, "collecting readings...");
        return;
    };
    let low = trace.samples.iter().copied().fold(f64::INFINITY, f64::min);
    let high = trace.samples.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    // A reading that has not moved draws a flat line through the middle.
    let (low, high) = if (high - low).abs() < f64::EPSILON { (low - 1., high + 1.) } else { (low, high) };
    let label_w = 11 * cv.char_w;
    let (gl, gt, gr, gb) = (pl + label_w, pt - 10, pr - 10, pb + lh + 6);
    for i in 0..=4 {
        let gy = gb + (gt - gb) * i / 4;
        cv.line_px(gl, gy, gr, gy, [0.22, 0.24, 0.30, 1.], 1.);
        let at = low + (high - low) * i as f64 / 4.;
        cv.text_px(pl + 6, gy - 4, [0.70, 0.73, 0.80], FONT_BASIC, &number(at));
    }
    let seconds = trace.samples.len() as f64 / 4.;
    cv.text_px(gl, pb + 5, [0.70, 0.73, 0.80], FONT_BASIC, &format!("last {seconds:.0} s, newest on the right"));
    let count = trace.samples.len();
    let span = (gr - gl) as f32;
    let points: Vec<(f32, f32)> = trace
        .samples
        .iter()
        .enumerate()
        .map(|(i, v)| {
            let x = gl as f32 + span * i as f32 / (TRACE_LEN - 1) as f32 + span * (TRACE_LEN - count) as f32 / (TRACE_LEN - 1) as f32;
            let y = gb as f32 + ((v - low) / (high - low)) as f32 * (gt - gb) as f32;
            (x, y)
        })
        .collect();
    let colour = if source == 0 { [0.60, 0.62, 0.68, 1.] } else { [0.38, 0.90, 0.48, 1.] };
    // The plot area sits inside the page, so the strip needs no clipping.
    let inside = points
        .iter()
        .all(|&(x, y)| x >= l as f32 && x <= r as f32 && y >= b as f32 && y <= t as f32);
    if inside {
        cv.xp.strip(&points, colour, 2., false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_box_page_has_boxes() {
        for kind in [
            PageKind::Apu,
            PageKind::Bleed,
            PageKind::AirConditioning,
            PageKind::Pressurisation,
            PageKind::GearBrakes,
            PageKind::AirData,
            PageKind::Fire,
        ] {
            assert!(!groups(kind).is_empty(), "{kind:?} has no boxes");
        }
    }

    #[test]
    fn the_eleven_tanks_are_each_drawn_once() {
        let mut numbers: Vec<usize> = TANKS.iter().map(|t| t.0).collect();
        numbers.sort();
        assert_eq!(numbers, (1..=11).collect::<Vec<_>>());
    }

    #[test]
    fn wing_tanks_do_not_overlap() {
        let mut spans: Vec<(f32, f32)> = TANKS.iter().filter(|t| t.0 != 11).map(|t| (t.2, t.2 + t.3)).collect();
        spans.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        for pair in spans.windows(2) {
            assert!(pair[0].1 <= pair[1].0, "tanks overlap: {:?}", pair);
        }
    }

    #[test]
    fn traces_keep_a_minute_and_count_changes() {
        let mut trace = Trace::new(0., 0.);
        for i in 0..1000 {
            trace.record((i % 3) as f64, i as f64 * 0.25);
        }
        assert_eq!(trace.samples.len(), TRACE_LEN);
        assert_eq!(trace.max, 2.);
        assert!(trace.changes > 600);
    }

    #[test]
    fn text_wraps_at_spaces() {
        assert_eq!(wrap("read from X-Plane every tick", 12), vec!["read from", "X-Plane", "every tick"]);
    }

    #[test]
    fn packed_words_are_recognised_and_plain_numbers_are_not() {
        assert!(looks_packed(6_442_450_944.));
        assert!(!looks_packed(31_133_584.7));
        assert!(!looks_packed(250.));
    }
}

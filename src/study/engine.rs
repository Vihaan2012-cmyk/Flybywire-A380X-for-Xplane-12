//! One engine, drawn as a cutaway the way a study panel draws it: the
//! nacelle, fan, and the Trent 900's three spools, with a box of readings
//! for each part it models and a line from the box to the part, and a strip
//! of coloured summary boxes along the foot.
//!
//! The A380's Trent 900 has three spools: the fan (N1), the intermediate
//! pressure spool (N2) and the high pressure spool (N3). FlyByWire names
//! them that way. Every box here holds only what their simulation carries:
//! rotor speeds, thrust lever, starter and ignition, the bleed ports and
//! precooler, the generator, the engine driven pumps and fire detection.

use super::canvas::{lamp, num, palette as p, Canvas, Field, Theme, DESIGN_W};
use crate::xp::FONT_BASIC;

/// Where a box's leader line lands on the drawing, and the station label.
pub(super) struct Station {
    pub title: String,
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub theme: Theme,
    pub fields: Vec<Field>,
    pub to: Option<(f32, f32, &'static str)>,
}

fn station(title: &str, x: f32, y: f32, w: f32, fields: Vec<Field>, to: Option<(f32, f32, &'static str)>) -> Station {
    Station { title: title.into(), x, y, w, theme: p::STATION, fields, to }
}

/// The boxes for engine `n`, laid out across the top of the page: the same
/// data the drawn cutaway and the web Study tab's JSON export both read.
pub(super) fn stations(n: usize) -> Vec<Station> {
    let pneu = |what: &str| format!("A32NX_PNEU_ENG_{n}_{what}");
    vec![
        station(
            "2: Fan",
            8.,
            8.,
            180.,
            vec![
                num("N1", format!("A32NX_ENGINE_N1:{n}"), "%", 1),
                num("N1 corr", format!("TURB ENG CORRECTED N1:{n}"), "%", 1),
                num("Thrust", format!("TURB ENG JET THRUST:{n}"), "lbf", 0),
            ],
            Some((311., 262., "2")),
        ),
        station(
            "25: IP Compressor",
            196.,
            8.,
            180.,
            vec![
                num("N2", format!("A32NX_ENGINE_N2:{n}"), "%", 1),
                num("N2 corr", format!("TURB ENG CORRECTED N2:{n}"), "%", 1),
            ],
            Some((395., 314., "25")),
        ),
        station(
            "3: HP Compressor",
            384.,
            8.,
            170.,
            vec![
                num("N3", format!("A32NX_ENGINE_N3:{n}"), "%", 1),
                num("State", format!("A32NX_ENGINE_STATE:{n}"), "", 0),
                num("Lever", format!("A32NX_AUTOTHRUST_TLA:{n}"), "deg", 1),
            ],
            Some((495., 320., "3")),
        ),
        station(
            "IP Bleed Port",
            562.,
            8.,
            205.,
            vec![
                num("Temp", pneu("IP_TEMPERATURE"), "C", 1),
                num("Press", pneu("INTERMEDIATE_TRANSDUCER_PRESSURE"), "psi", 1),
                lamp("Valve open", pneu("IP_VALVE_OPEN")),
            ],
            Some((452., 286., "IP")),
        ),
        station(
            "HP Bleed Port",
            775.,
            8.,
            217.,
            vec![
                num("Press", pneu("HP_PRESSURE"), "psi", 1),
                num("Temp", pneu("HP_TEMPERATURE"), "C", 1),
                lamp("Valve open", pneu("HP_VALVE_OPEN")),
            ],
            Some((545., 272., "HP")),
        ),
        station(
            "Starter",
            8.,
            100.,
            230.,
            vec![
                lamp("Starter active", format!("GENERAL ENG STARTER ACTIVE:{n}")),
                num("Ignition mode", format!("TURB ENG IGNITION SWITCH EX1:{n}"), "", 0),
                lamp("Start valve open", pneu("STARTER_VALVE_OPEN")),
            ],
            Some((355., 501., "S")),
        ),
        station(
            "Fire Detection",
            246.,
            100.,
            150.,
            vec![
                lamp("Detected", format!("A32NX_FIRE_DETECTED_ENG{n}")),
                lamp("On fire", format!("A32NX_ENG_{n}_ON_FIRE")),
                lamp("Fire pb", format!("A32NX_FIRE_BUTTON_ENG{n}")),
            ],
            Some((575., 274., "F")),
        ),
        station(
            "4: Combustor",
            404.,
            100.,
            150.,
            vec![
                num("EGT", format!("A32NX_ENGINE_EGT:{n}"), "C", 0),
                num("FF", format!("A32NX_ENGINE_FF:{n}"), "kg/h", 0),
                num("Used", format!("A32NX_FUEL_USED:{n}"), "kg", 0),
            ],
            Some((578., 316., "4")),
        ),
        station(
            "Precooler",
            562.,
            100.,
            205.,
            vec![
                num("Inlet", pneu("PRECOOLER_INLET_TEMPERATURE"), "C", 1),
                num("Outlet", pneu("PRECOOLER_OUTLET_TEMPERATURE"), "C", 1),
                lamp("PR valve open", pneu("PR_VALVE_OPEN")),
            ],
            Some((645., 211., "PC")),
        ),
        station(
            "Bleed Delivery",
            775.,
            100.,
            217.,
            vec![
                num("Regulated", pneu("REGULATED_TRANSDUCER_PRESSURE"), "psi", 1),
                num("Transfer", pneu("TRANSFER_TRANSDUCER_PRESSURE"), "psi", 1),
                num("Transfer temp", pneu("TRANSFER_TEMPERATURE"), "C", 1),
            ],
            Some((735., 207., "BD")),
        ),
    ]
}

/// The summary strip along the foot of the page.
pub(super) fn summaries(n: usize) -> Vec<(&'static str, Theme, Vec<Field>)> {
    let (system, a, b) = if n <= 2 {
        ("GREEN", format!("{n}A"), format!("{n}B"))
    } else {
        ("YELLOW", format!("{n}A"), format!("{n}B"))
    };
    let gen = format!("A32NX_ELEC_ENG_GEN_{n}");
    vec![
        (
            "Summary",
            p::MAGENTA,
            vec![
                num("State", format!("A32NX_ENGINE_STATE:{n}"), "", 0),
                num("EGT", format!("A32NX_ENGINE_EGT:{n}"), "C", 0),
            ],
        ),
        ("LP Rotor", p::GREEN, vec![num("N1", format!("A32NX_ENGINE_N1:{n}"), "%", 1)]),
        ("IP Rotor", p::BLUE, vec![num("N2", format!("A32NX_ENGINE_N2:{n}"), "%", 1)]),
        ("HP Rotor", p::BLUE, vec![num("N3", format!("A32NX_ENGINE_N3:{n}"), "%", 1)]),
        (
            "Oil",
            p::YELLOW,
            vec![
                num("Qty", format!("A32NX_ENGINE_OIL_QTY:{n}"), "", 1),
                num("Press", format!("GENERAL ENG OIL PRESSURE:{n}"), "psi", 0),
            ],
        ),
        (
            "Generator",
            p::ORANGE,
            vec![
                num("Volts", format!("{gen}_POTENTIAL"), "V", 0),
                num("Freq", format!("{gen}_FREQUENCY"), "Hz", 0),
            ],
        ),
        (
            "Hyd Pumps",
            p::CYAN,
            vec![
                lamp(&format!("EDP {a}"), format!("A32NX_HYD_{system}_{a}_EDPUMP_ACTIVE")),
                lamp(&format!("EDP {b}"), format!("A32NX_HYD_{system}_{b}_EDPUMP_ACTIVE")),
            ],
        ),
        (
            "IDG",
            p::RED,
            vec![
                lamp("Connected", format!("{gen}_IDG_IS_CONNECTED")),
                num("Oil", format!("{gen}_IDG_OIL_OUTLET_TEMPERATURE"), "C", 0),
            ],
        ),
    ]
}

/// Mirror a point on the upper half of the engine to the lower half.
fn lower(points: &[(f32, f32)]) -> Vec<(f32, f32)> {
    points.iter().map(|&(x, y)| (x, 2. * AXIS - y)).collect()
}

/// The engine's centre line on the design sheet.
const AXIS: f32 = 355.;

const NACELLE: [f32; 4] = [0.84, 0.84, 0.87, 1.];
const CORE: [f32; 4] = [0.97, 0.97, 0.99, 1.];
const FAN: [f32; 4] = [0.60, 0.88, 0.63, 1.];
const IP: [f32; 4] = [0.62, 0.77, 0.93, 1.];
const HP: [f32; 4] = [0.50, 0.66, 0.88, 1.];
const LP: [f32; 4] = [0.58, 0.86, 0.60, 1.];
const BURNER: [f32; 4] = [0.99, 0.72, 0.44, 1.];
const BYPASS: [f32; 4] = [0.45, 0.88, 0.96, 1.];
const JET: [f32; 4] = [0.93, 0.47, 0.47, 1.];

/// One piece of the cutaway on the design sheet: the XPLM window draws these
/// through [`Canvas`], the web Study tab's JSON carries them as they are.
pub(super) enum Shape {
    /// A filled convex shape with an edge.
    Poly(Vec<(f32, f32)>, Rgba),
    /// A filled rectangle with an edge.
    Rect(f32, f32, f32, f32, Rgba),
    Line(f32, f32, f32, f32, Rgba, f32),
    /// An arrow pointing right: x, y, length, half height.
    Arrow(f32, f32, f32, f32, Rgba),
    Text(f32, f32, &'static str),
}

type Rgba = [f32; 4];

/// The cutaway itself: a Trent 900 in section, drawn about the axis with
/// every convex piece mirrored top and bottom (the XPLM window can only
/// fill convex shapes).
pub(super) fn cutaway() -> Vec<Shape> {
    use Shape::*;
    let mut out = Vec::new();
    let up = |x: f32, r: f32| (x, AXIS - r);
    let both = |out: &mut Vec<Shape>, pts: Vec<(f32, f32)>, colour: Rgba| {
        out.push(Poly(lower(&pts), colour));
        out.push(Poly(pts, colour));
    };
    // A band between two radius profiles, one convex quad per segment.
    let band = |out: &mut Vec<Shape>, xs: &[f32], outer: &[f32], inner: &[f32], colour: Rgba| {
        for i in 0..xs.len() - 1 {
            let q = vec![(xs[i], AXIS - outer[i]), (xs[i + 1], AXIS - outer[i + 1]), (xs[i + 1], AXIS - inner[i + 1]), (xs[i], AXIS - inner[i])];
            out.push(Poly(lower(&q), colour));
            out.push(Poly(q, colour));
        }
    };
    // A blade row: one rect above the axis and one below, hub to tip.
    let row = |out: &mut Vec<Shape>, x: f32, w: f32, hub: f32, tip: f32, colour: Rgba| {
        out.push(Rect(x, AXIS - tip, w, tip - hub, colour));
        out.push(Rect(x, AXIS + hub, w, tip - hub, colour));
    };

    // Free stream coming in.
    out.push(Line(20., 278., 140., 278., BYPASS, 2.));
    out.push(Line(20., AXIS, 205., AXIS, BYPASS, 2.));
    out.push(Text(24., 262., "0: Free stream"));

    // Core gas path: casing to hub, filled, the stages drawn over it.
    let core_x = [345., 400., 470., 545., 610., 680., 750., 810.];
    band(&mut out, &core_x, &[52., 54., 48., 44., 50., 56., 62., 54.], &[30., 30., 28., 28., 30., 34., 36., 32.], CORE);

    // Shafts: LP innermost and longest, IP round it, HP shortest and outermost.
    out.push(Rect(290., AXIS - 3., 470., 6., LP));
    out.push(Line(350., AXIS - 9., 640., AXIS - 9., IP_SHAFT, 2.));
    out.push(Line(350., AXIS + 9., 640., AXIS + 9., IP_SHAFT, 2.));
    out.push(Line(460., AXIS - 15., 622., AXIS - 15., HP, 2.));
    out.push(Line(460., AXIS + 15., 622., AXIS + 15., HP, 2.));

    // Eight-stage IP compressor, six-stage HP compressor, blades shortening.
    for i in 0..8 {
        let x = 352. + i as f32 * 13.;
        row(&mut out, x, 7., 30., 54. - i as f32 * 0.9, IP);
    }
    for i in 0..6 {
        let x = 468. + i as f32 * 12.;
        row(&mut out, x, 7., 28., 47. - i as f32 * 0.9, HP);
    }

    // Annular combustor with its fuel injector.
    both(&mut out, vec![up(550., 34.), up(556., 44.), up(578., 50.), up(600., 47.), up(607., 38.), up(596., 31.), up(572., 29.)], BURNER);
    out.push(Line(532., AXIS - 66., 552., AXIS - 42., INJECTOR, 2.));
    out.push(Line(532., AXIS + 66., 552., AXIS + 42., INJECTOR, 2.));

    // Turbines: one HP stage, one IP stage, five LP stages growing outwards.
    row(&mut out, 614., 9., 32., 48., HP);
    row(&mut out, 632., 9., 33., 52., IP);
    for i in 0..5 {
        let x = 656. + i as f32 * 18.;
        row(&mut out, x, 9., 35., 54. + i as f32 * 2.5, LP);
    }

    // Core cowl round the gas path, with its splitter lip.
    band(&mut out, &core_x, &[58., 76., 82., 82., 80., 76., 68., 58.], &[52., 54., 48., 44., 50., 56., 62., 54.], COWL);

    // Tail cone.
    out.push(Poly(vec![(745., AXIS - 32.), (800., AXIS - 29.), (860., AXIS - 14.), (885., AXIS), (860., AXIS + 14.), (800., AXIS + 29.), (745., AXIS + 32.)], NACELLE));

    // Nacelle with its inlet lip, upper and lower.
    band(
        &mut out,
        &[150., 162., 190., 260., 360., 480., 600., 720., 800.],
        &[140., 148., 154., 158., 159., 158., 152., 140., 126.],
        &[128., 130., 136., 138., 138., 138., 134., 124., 114.],
        NACELLE,
    );

    // Fan outlet guide vanes across the bypass duct.
    for x in [392., 404.] {
        both(&mut out, vec![up(x - 4., 84.), up(x + 6., 137.), up(x + 12., 137.), up(x + 2., 84.)], OGV);
    }

    // Spinner, fan disc and the swept fan blades.
    out.push(Poly(vec![(220., AXIS), (236., AXIS - 16.), (262., AXIS - 30.), (296., AXIS - 36.), (296., AXIS + 36.), (262., AXIS + 30.), (236., AXIS + 16.)], FAN));
    row(&mut out, 296., 30., 0., 36., FAN_DISC);
    both(&mut out, vec![up(298., 36.), up(300., 134.), up(318., 128.), up(326., 36.)], FAN);
    out.push(Line(305., AXIS - 40., 312., AXIS - 126., FAN_EDGE, 1.));
    out.push(Line(305., AXIS + 40., 312., AXIS + 126., FAN_EDGE, 1.));

    // Bleed: IP and HP ports up through the bypass to the precooler on the
    // pylon side, and on to delivery.
    out.push(Line(445., AXIS - 52., 470., 240., DUCT, 3.));
    out.push(Line(530., AXIS - 44., 560., 240., DUCT, 3.));
    out.push(Line(470., 240., 560., 240., DUCT, 3.));
    out.push(Line(560., 240., 620., 212., DUCT, 3.));
    out.push(Rect(620., 204., 50., 14., PRECOOLER));
    out.push(Line(670., 211., 760., 206., DUCT, 3.));

    // Accessory gearbox on the fan case, driven off the HP spool.
    out.push(Line(470., AXIS + 15., 400., 494., GEARBOX_DRIVE, 2.));
    out.push(Rect(300., 494., 110., 14., GEARBOX));

    // Air leaving: the bypass stream and the core jet.
    out.push(Arrow(810., AXIS - 90., 90., 12., BYPASS));
    out.push(Arrow(810., AXIS + 90., 90., 12., BYPASS));
    out.push(Arrow(890., AXIS, 70., 16., JET));
    out
}

const COWL: [f32; 4] = [0.74, 0.75, 0.79, 1.];
const OGV: [f32; 4] = [0.70, 0.72, 0.78, 1.];
const FAN_DISC: [f32; 4] = [0.42, 0.66, 0.46, 1.];
const FAN_EDGE: [f32; 4] = [0.25, 0.50, 0.28, 1.];
const IP_SHAFT: [f32; 4] = [0.30, 0.45, 0.75, 1.];
const INJECTOR: [f32; 4] = [0.80, 0.45, 0.20, 1.];
const DUCT: [f32; 4] = [0.86, 0.60, 0.38, 1.];
const PRECOOLER: [f32; 4] = [0.95, 0.80, 0.45, 1.];
const GEARBOX: [f32; 4] = [0.66, 0.66, 0.70, 1.];
const GEARBOX_DRIVE: [f32; 4] = [0.50, 0.66, 0.88, 1.];

fn engine_drawing(cv: &Canvas) {
    for shape in cutaway() {
        match shape {
            Shape::Poly(points, colour) => {
                cv.poly(&points, colour);
                cv.outline(&points, p::EDGE, 1.);
            }
            Shape::Rect(x, y, w, h, colour) => {
                cv.fill(x, y, w, h, colour);
                cv.frame(x, y, w, h, p::EDGE, 1.);
            }
            Shape::Line(x1, y1, x2, y2, colour, width) => cv.line(x1, y1, x2, y2, colour, width),
            Shape::Arrow(x, y, len, half, colour) => cv.arrow(x, y, len, half, colour),
            Shape::Text(x, y, s) => cv.text(x, y, p::INK, FONT_BASIC, s),
        }
    }
}

/// The engine as a live diagram: one node per station box, at the same
/// position [`draw`] places it, plus the summary strip along the foot.
pub(super) fn topology(n: usize) -> super::elec::Topology {
    use super::elec::{field_to_topo, TopoNode, Topology};
    let mut nodes: Vec<TopoNode> = Vec::new();
    for s in stations(n) {
        nodes.push(TopoNode { id: s.title.clone(), title: s.title.clone(), x: s.x, y: s.y, w: s.w, kind: "source", fields: s.fields.iter().map(field_to_topo).collect() });
    }
    for (i, (title, _theme, fields)) in summaries(n).into_iter().enumerate() {
        nodes.push(TopoNode {
            id: format!("sum{i}"),
            title: title.into(),
            x: 6. + i as f32 * 124.5,
            y: 527.,
            w: 118.,
            kind: "source",
            fields: fields.iter().map(field_to_topo).collect(),
        });
    }
    Topology { design_w: DESIGN_W, design_h: 600., nodes, links: Vec::new() }
}

pub fn draw(cv: &mut Canvas, n: usize) {
    // The page's three bands: stations, engine, summaries.
    cv.fill(0., 0., DESIGN_W, 196., p::SLATE);
    cv.fill(0., 196., DESIGN_W, 324., p::PERIWINKLE);
    cv.fill(0., 520., DESIGN_W, 80., p::SLATE);
    cv.line(0., 196., DESIGN_W, 196., p::EDGE, 1.);
    cv.line(0., 520., DESIGN_W, 520., p::EDGE, 1.);

    engine_drawing(cv);

    // Boxes last, with their leader lines, so a line never hides a reading.
    for s in stations(n) {
        let bottom = cv.station(s.x, s.y, s.w, &s.title, &s.theme, &s.fields);
        if let Some((tx, ty, label)) = s.to {
            let from_x = cv.x(s.x + s.w / 2.);
            cv.line_px(from_x, bottom, cv.x(tx), cv.y(ty), p::LEADER, 1.);
            cv.marker(tx, ty, label);
        }
    }

    for (i, (title, theme, fields)) in summaries(n).into_iter().enumerate() {
        let x = 6. + i as f32 * 124.5;
        cv.station(x, 527., 118., title, &theme, &fields);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_box_names_its_own_engine() {
        for n in 1..=4 {
            let names: Vec<String> = stations(n)
                .into_iter()
                .flat_map(|s| s.fields.into_iter().map(|f| f.name))
                .chain(summaries(n).into_iter().flat_map(|(_, _, f)| f.into_iter().map(|f| f.name)))
                .collect();
            for other in (1..=4).filter(|&o| o != n) {
                assert!(
                    !names.iter().any(|name| name.ends_with(&format!(":{other}"))
                        || name.contains(&format!("ENG_{other}_"))
                        || name.contains(&format!("ENG{other}"))
                        || name.contains(&format!("GEN_{other}_"))),
                    "engine {n}'s page reads engine {other}"
                );
            }
        }
    }

    #[test]
    fn engines_one_and_two_drive_green_pumps_three_and_four_yellow() {
        let pumps = |n| summaries(n).into_iter().find(|s| s.0 == "Hyd Pumps").unwrap().2;
        assert!(pumps(2)[0].name.contains("GREEN_2A"));
        assert!(pumps(3)[1].name.contains("YELLOW_3B"));
    }

    #[test]
    fn the_lower_nacelle_mirrors_the_upper() {
        assert_eq!(lower(&[(100., AXIS - 50.)]), vec![(100., AXIS + 50.)]);
    }
}

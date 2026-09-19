//! The hydraulic network, drawn as the aircraft's HYD page draws it: the
//! A380's two systems side by side, green fed by engines 1 and 2 and yellow
//! by engines 3 and 4, each with four engine driven pumps, two electric
//! pumps, a reservoir, and the pressure the system delivers.
//!
//! Pump outputs light green when the pump runs; the suction line is blue
//! when the reservoir holds fluid; the delivery line lights when the system
//! is pressurised. Pump section pressures and fire shutoff valves are listed
//! by FlyByWire's own numbering rather than tied to a pump name, because
//! their code numbers sections, not pumps.

use super::canvas::{lamp, num, palette as p, Canvas, Ink, DESIGN_H, DESIGN_W};
use super::elec::{flag, lfield, llamp, node, pipe, plink, source, tfield, tflag, value, warning, wire, wlink, Gate, TopoLink, TopoNode, Topology};
use crate::xp::FONT_PROPORTIONAL;

/// Pressure above which a system counts as delivering, in psi.
const PRESSURISED: f64 = 1450.;

struct System {
    name: &'static str,
    title: &'static str,
    ink: Ink,
    engine_pumps: [&'static str; 4],
    electric_pumps: [&'static str; 2],
    engines: [usize; 2],
}

const GREEN: System = System {
    name: "GREEN",
    title: "GREEN SYSTEM",
    ink: p::ECAM_GREEN,
    engine_pumps: ["1A", "1B", "2A", "2B"],
    electric_pumps: ["GA", "GB"],
    engines: [1, 2],
};

const YELLOW: System = System {
    name: "YELLOW",
    title: "YELLOW SYSTEM",
    ink: p::ECAM_YELLOW,
    engine_pumps: ["3A", "3B", "4A", "4B"],
    electric_pumps: ["YA", "YB"],
    engines: [3, 4],
};

pub fn draw(cv: &mut Canvas) {
    cv.fill(0., 0., DESIGN_W, DESIGN_H, p::ECAM_GROUND);
    cv.line(500., 0., 500., DESIGN_H, p::ECAM_DEAD, 1.);
    half(cv, 0., &GREEN);
    half(cv, 500., &YELLOW);
}

fn half(cv: &mut Canvas, ox: f32, sys: &System) {
    let c = sys.name;
    cv.text(ox + 14., 4., sys.ink, FONT_PROPORTIONAL, sys.title);

    // What the system delivers.
    let pressure_name = format!("A32NX_HYD_{c}_SYSTEM_1_SECTION_PRESSURE");
    let delivery = source(
        cv,
        ox + 150.,
        26.,
        200.,
        "SYSTEM",
        &[
            value(pressure_name.clone(), "PSI", 0, None),
            flag(format!("{pressure_name}_SWITCH"), "PRESS SWITCH"),
        ],
    );

    // The six pumps.
    let mut pumps = Vec::with_capacity(6);
    for (i, id) in sys.engine_pumps.iter().enumerate() {
        let active = format!("A32NX_HYD_{c}_{id}_EDPUMP_ACTIVE");
        let rect = source(
            cv,
            ox + 12. + i as f32 * 80.,
            146.,
            72.,
            &format!("EDP {id}"),
            &[flag(active.clone(), "ON")],
        );
        pumps.push((rect, cv.on(&active)));
    }
    for (i, id) in sys.electric_pumps.iter().enumerate() {
        let active = format!("A32NX_HYD_{id}_EPUMP_ACTIVE");
        let rect = source(
            cv,
            ox + 332. + i as f32 * 80.,
            146.,
            72.,
            &format!("ELEC {}", &id[1..]),
            &[
                flag(active.clone(), "ON"),
                value(format!("A32NX_HYD_{id}_EPUMP_RPM"), "RPM", 0, None),
                warning(format!("A32NX_HYD_{id}_EPUMP_LOW_PRESS"), "LO PR"),
            ],
        );
        pumps.push((rect, cv.on(&active)));
    }

    // Delivery manifold above the pumps, suction line below.
    let manifold = cv.y(118.);
    let suction = cv.y(252.);
    let any_running = pumps.iter().any(|(_, on)| *on);
    let (left, right) = (cv.x(ox + 40.), cv.x(ox + 460.));
    wire(cv, &[(left, manifold), (right, manifold)], any_running);
    let pressurised = cv.value(&pressure_name).is_some_and(|v| v > PRESSURISED);
    wire(cv, &[(delivery.cx(), manifold), (delivery.cx(), delivery.b)], pressurised);

    let level_name = format!("A32NX_HYD_{c}_RESERVOIR_LEVEL");
    let has_fluid = cv.value(&level_name).is_some_and(|v| v > 0.);
    pipe(cv, &[(left, suction), (right, suction)], has_fluid);
    for (rect, running) in &pumps {
        wire(cv, &[(rect.cx(), rect.t), (rect.cx(), manifold)], *running);
        pipe(cv, &[(rect.cx(), rect.b), (rect.cx(), suction)], has_fluid);
    }

    // Reservoir, feeding the suction line.
    let reservoir = source(
        cv,
        ox + 20.,
        272.,
        220.,
        "RESERVOIR",
        &[
            value(level_name.clone(), "GAL", 1, None),
            warning(format!("{level_name}_IS_LOW"), "LEVEL LOW"),
            value(format!("A32NX_HYD_{c}_RESERVOIR_AIR_PRESSURE"), "PSI", 1, None),
            warning(format!("A32NX_HYD_{c}_RESERVOIR_AIR_PRESSURE_IS_LOW"), "AIR PRESS LOW"),
            warning(format!("A32NX_HYD_{c}_RESERVOIR_OVHT"), "OVERHEAT"),
        ],
    );
    pipe(cv, &[(reservoir.cx(), reservoir.t), (reservoir.cx(), suction)], has_fluid);

    // Section pressures and fire shutoff valves, as FlyByWire numbers them.
    let sections: Vec<_> = (1..=6)
        .map(|k| num(&format!("Section {k}"), format!("A32NX_HYD_{c}_PUMP_{k}_SECTION_PRESSURE"), "psi", 0))
        .collect();
    cv.station(ox + 260., 272., 220., "PUMP SECTIONS", &p::ECAM_BOX, &sections);

    let valves: Vec<_> = (1..=6)
        .map(|k| lamp(&format!("Fire valve {k} open"), format!("A32NX_HYD_{c}_PUMP_{k}_FIRE_VALVE_OPENED")))
        .collect();
    cv.station(ox + 20., 404., 220., "FIRE SHUTOFF VALVES", &p::ECAM_BOX, &valves);

    let disconnects: Vec<_> = sys
        .engines
        .iter()
        .map(|e| lamp(&format!("Engine {e} pumps disc"), format!("A32NX_HYD_ENG_{e}AB_PUMP_DISC")))
        .chain(
            sys.electric_pumps
                .iter()
                .map(|id| lamp(&format!("Elec {} off pb", &id[1..]), format!("A32NX_OVHD_HYD_EPUMP{id}_OFF_PB_IS_AUTO"))),
        )
        .collect();
    cv.station(ox + 260., 420., 220., "PUMP CONTROL", &p::ECAM_BOX, &disconnects);
}

/// Pressure above which a system counts as delivering.
const PRESSURISED_PSI: f64 = PRESSURISED;

/// The hydraulic network as data, for the web Study tab: the same two
/// systems, pumps, reservoir and section pressures [`draw`] paints.
pub(super) fn topology() -> Topology {
    let mut nodes = Vec::new();
    let mut links = Vec::new();
    for (ox, sys) in [(0., &GREEN), (500., &YELLOW)] {
        half_topology(&mut nodes, &mut links, ox, sys);
    }
    Topology { design_w: DESIGN_W, design_h: DESIGN_H, nodes, links }
}

fn half_topology(nodes: &mut Vec<TopoNode>, links: &mut Vec<TopoLink>, ox: f32, sys: &System) {
    let c = sys.name;
    let pressure_name = format!("A32NX_HYD_{c}_SYSTEM_1_SECTION_PRESSURE");
    let delivery = node(
        nodes,
        &format!("{c}_delivery"),
        sys.title,
        ox + 150.,
        26.,
        200.,
        "source",
        vec![tfield(pressure_name.clone(), "psi", 0), tflag(format!("{pressure_name}_SWITCH"), "PRESS SWITCH")],
    );

    let mut pumps = Vec::with_capacity(6);
    for (i, id) in sys.engine_pumps.iter().enumerate() {
        let active = format!("A32NX_HYD_{c}_{id}_EDPUMP_ACTIVE");
        let rect = node(nodes, &format!("{c}_edp_{id}"), &format!("EDP {id}"), ox + 12. + i as f32 * 80., 146., 72., "source", vec![tflag(active.clone(), "ON")]);
        pumps.push((rect, active));
    }
    for (i, id) in sys.electric_pumps.iter().enumerate() {
        let active = format!("A32NX_HYD_{id}_EPUMP_ACTIVE");
        let rect = node(
            nodes,
            &format!("{c}_epump_{id}"),
            &format!("ELEC {}", &id[1..]),
            ox + 332. + i as f32 * 80.,
            146.,
            72.,
            "source",
            vec![tflag(active.clone(), "ON"), tfield(format!("A32NX_HYD_{id}_EPUMP_RPM"), "RPM", 0), tflag(format!("A32NX_HYD_{id}_EPUMP_LOW_PRESS"), "LO PR")],
        );
        pumps.push((rect, active));
    }

    let (manifold_y, suction_y) = (118., 252.);
    let (left, right) = (ox + 40., ox + 460.);
    let any_running = Gate::Any(pumps.iter().map(|(_, a)| Gate::On(a.clone())).collect());
    links.push(wlink(vec![(left, manifold_y), (right, manifold_y)], any_running));
    links.push(wlink(vec![(delivery.0, manifold_y), (delivery.0, delivery.1)], Gate::Gt(pressure_name.clone(), PRESSURISED_PSI)));

    let level_name = format!("A32NX_HYD_{c}_RESERVOIR_LEVEL");
    let has_fluid = || Gate::Gt(level_name.clone(), 0.);
    links.push(plink(vec![(left, suction_y), (right, suction_y)], has_fluid()));
    for (rect, active) in &pumps {
        links.push(wlink(vec![(rect.0, rect.1), (rect.0, manifold_y)], Gate::On(active.clone())));
        links.push(plink(vec![(rect.0, rect.2), (rect.0, suction_y)], has_fluid()));
    }

    let reservoir = node(
        nodes,
        &format!("{c}_reservoir"),
        "RESERVOIR",
        ox + 20.,
        272.,
        220.,
        "source",
        vec![
            tfield(level_name.clone(), "gal", 1),
            tflag(format!("{level_name}_IS_LOW"), "LEVEL LOW"),
            tfield(format!("A32NX_HYD_{c}_RESERVOIR_AIR_PRESSURE"), "psi", 1),
            tflag(format!("A32NX_HYD_{c}_RESERVOIR_AIR_PRESSURE_IS_LOW"), "AIR PRESS LOW"),
            tflag(format!("A32NX_HYD_{c}_RESERVOIR_OVHT"), "OVERHEAT"),
        ],
    );
    links.push(plink(vec![(reservoir.0, reservoir.1), (reservoir.0, suction_y)], has_fluid()));

    let sections: Vec<_> = (1..=6).map(|k| lfield(&format!("Section {k}"), format!("A32NX_HYD_{c}_PUMP_{k}_SECTION_PRESSURE"), "psi", 0)).collect();
    node(nodes, &format!("{c}_sections"), "PUMP SECTIONS", ox + 260., 272., 220., "source", sections);

    let valves: Vec<_> = (1..=6).map(|k| llamp(&format!("Fire valve {k} open"), format!("A32NX_HYD_{c}_PUMP_{k}_FIRE_VALVE_OPENED"))).collect();
    node(nodes, &format!("{c}_valves"), "FIRE SHUTOFF VALVES", ox + 20., 404., 220., "source", valves);

    let mut disconnects: Vec<_> = sys.engines.iter().map(|e| llamp(&format!("Engine {e} pumps disc"), format!("A32NX_HYD_ENG_{e}AB_PUMP_DISC"))).collect();
    disconnects.extend(
        sys.electric_pumps
            .iter()
            .map(|id| llamp(&format!("Elec {} off pb", &id[1..]), format!("A32NX_OVHD_HYD_EPUMP{id}_OFF_PB_IS_AUTO"))),
    );
    node(nodes, &format!("{c}_control"), "PUMP CONTROL", ox + 260., 420., 220., "source", disconnects);
}

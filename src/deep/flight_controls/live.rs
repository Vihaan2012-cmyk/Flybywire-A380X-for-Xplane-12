//! The live flight-control surfaces: one instance of every physical
//! surface on the A380, stepped every frame from [`Truth`] with every
//! failure this directory's `registry.rs` registers applied to the exact
//! model field that entry names.
//!
//! What this owns, matching `docs/deep/integration.md`'s own table of the
//! 37 physical surfaces:
//!
//! | Group | Count | Model |
//! |---|---|---|
//! | Ailerons | 3/wing x 2 | `surface::ControlSurface<2>` |
//! | Elevators | 2/side x 2 | `surface::ControlSurface<2>` |
//! | Rudders | 2 | `surface::ControlSurface<2>` |
//! | Spoilers | 8/wing x 2 | `surface::ControlSurface<1>` |
//! | THS | 1 | `ths::TrimmableHorizontalStabilizer` |
//! | Rudder trim | 1 | `ths::RudderTrimActuator` |
//! | Flap / slat / droop-nose drive lines | 2 each | `high_lift::HighLiftPair` (inboard + outboard station each, so 8 high-lift stations) |
//!
//! plus one `spoiler::GroundSpoilerLogic` and one `sensors::DualTransducer`
//! per surface (the computers' own position monitoring, separate from each
//! actuator's internal servo feedback).
//!
//! Actuator count per surface and which supply each actuator draws from
//! come from `allocation.rs`, which took them from FlyByWire's own
//! `prim.rs`; nothing new is invented here.
//!
//! ## Feeding `integration::flight_control_surfaces::PhysicalSurfaces`
//!
//! [`FlightControlsLive::surface_angles`] returns every live surface angle
//! in degrees, in exactly the layout `PhysicalSurfaces` wants (ailerons
//! `[side][inward, middle, outward]`, elevators `[side][inward, outward]`,
//! rudders `[upper, lower]`, spoilers `[side][1..=8]`, THS nose-up
//! degrees). See that method's own doc comment for the literal call.
//!
//! ## Inputs that are not on `Truth` yet
//!
//! Three, none of them faked here; each has a setter, and until the plugin
//! calls it the model runs at a value that is honest about not knowing:
//!
//! * **The commanded surface positions**
//!   ([`FlightControlsLive::set_commands`]). This area models what a
//!   surface *physically does* with the position FlyByWire's own PRIM/SEC
//!   asked for; that command is not on `Truth`. Until it arrives every
//!   surface is commanded to neutral, which means a jam at neutral looks
//!   like a healthy surface -- a runaway, a disconnect, blow-back and
//!   supply loss all still act, but a jam cannot be seen.
//! * **Angle of attack** ([`FlightControlsLive::set_alpha_rad`]), which
//!   `hinge_moment.rs` needs for the `Ch_alpha` term. Zero until supplied:
//!   the `Ch_delta` (deflection) term, which is the larger one and the one
//!   blow-back depends on, is unaffected.
//! * **Ground-spoiler arming and go-around selection**
//!   ([`FlightControlsLive::set_ground_spoiler_controls`]) -- cockpit
//!   discretes, not aircraft state. With the lever unarmed the
//!   ground-spoiler logic never deploys, so its own two failures cannot be
//!   reached.

use std::collections::BTreeMap;

use crate::deep::live::{Area as LiveArea, Faults, Truth};

use super::actuator::{ActuatorFaults, ActuatorMode, ActuatorGeometry, ElectricPumpFaults, PowerControlUnit, HYDRAULIC_SUPPLY_PA};
use super::allocation::{
    aileron_inboard, aileron_midboard, aileron_outboard, elevator_inboard, elevator_outboard, mode_for_surface, rudder_lower, rudder_upper, ths_motors, ActuatorAllocation,
    ComputerHealth, PowerAvailability,
};
use super::high_lift::{HighLiftFaults, HighLiftPair, HighLiftSystem};
use super::hinge_moment::HingeMomentCoefficients;
use super::sensors::{DualTransducer, TransducerFaults};
use super::spoiler::{GroundSpoilerInputs, GroundSpoilerLogic, GroundSpoilerLogicFaults};
use super::surface::{inertia_uniform_plate_kg_m2, AeroInputs, ControlSurface, SurfaceDamping, SurfaceFaults, SurfaceLimits, SurfaceOutput};
use super::ths::{RudderTrimActuator, ThsFaults, TrimmableHorizontalStabilizer};

// ---------------------------------------------------------------------------
// Constants.
// ---------------------------------------------------------------------------

/// Dry air specific gas constant, CIPM-2007 (the same 287.052 87
/// J/(kg K) `integration::weather_truth` uses for its own Mach
/// calculation), restated here so this directory keeps no dependency on
/// another area.
const R_AIR_J_KGK: f64 = 287.052_87;

/// Aileron and elevator travel, `docs/deep/integration.md`'s own per-
/// surface table: -20 deg (trailing edge down) to +30 deg (up), the range
/// `flight_controls.rs`'s `n = (20 - deg) / 50` conversion spans exactly.
const AILERON_MIN_DEG: f64 = -20.0;
const AILERON_MAX_DEG: f64 = 30.0;
/// Rudder travel, same table: +-30 deg, spanning `n = (30 - deg) / 60`.
const RUDDER_LIMIT_DEG: f64 = 30.0;
/// Spoiler travel, same table: 0..50 deg up, spanning `n = deg / 50`.
const SPOILER_MAX_DEG: f64 = 50.0;

/// GENERIC: the position disagreement between an actuator's two transducer
/// channels that a flight control computer treats as a monitoring fault,
/// and how long it must persist. Two degrees over one second is a
/// representative transport position-monitor threshold (no A380 figure is
/// public); it has to sit above normal tracking error and below anything
/// that would change the aircraft's response.
const TRANSDUCER_DISAGREE_RAD: f64 = 2.0 * std::f64::consts::PI / 180.0;
const TRANSDUCER_DISAGREE_TIMER_S: f64 = 1.0;

/// GENERIC: the left/right high-lift position difference the SFCC's own
/// asymmetry monitor acts on, and its debounce. Five degrees over one
/// second -- large enough that normal transit skew never trips it, small
/// enough to stop a real runaway well before it becomes a rolling moment
/// the ailerons cannot hold.
const HIGH_LIFT_ASYMMETRY_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;
const HIGH_LIFT_ASYMMETRY_TIMER_S: f64 = 1.0;

/// A 115 V AC bus is energised (same threshold the hydraulics live system
/// uses; half nominal is far below contactor hold-in and far above noise).
const AC_BUS_LIVE_V: f64 = 100.0;

/// m/s to knots.
const MS_TO_KT: f64 = 1.943_844_492_440_605;

// ---------------------------------------------------------------------------
// Failure-id binding: read straight out of `registry::register`.
// ---------------------------------------------------------------------------

fn registered_failures() -> BTreeMap<String, Vec<u64>> {
    let mut r = crate::deep::api::Registry::default();
    super::registry::register(&mut r);
    r.components.into_iter().map(|c| (c.id, c.failures)).collect()
}

fn take(map: &BTreeMap<String, Vec<u64>>, component: &str, count: usize) -> Vec<u64> {
    let ids = map.get(component).unwrap_or_else(|| panic!("flight_controls registry has no component {component}"));
    assert_eq!(ids.len(), count, "component {component} registers {} failures, live.rs binds {count}", ids.len());
    ids.clone()
}

/// The eleven failures every servo-hydraulic surface registers, in
/// `registry::surface_fields`' own order.
#[derive(Clone, Copy, Debug)]
struct SurfaceIds {
    jam: u64,
    runaway: u64,
    supply_loss: u64,
    transducer_fault: u64,
    disconnect: u64,
    flutter_damper_loss: u64,
    valve_leakage: u64,
    piston_seal_wear: u64,
    transducer_drift: u64,
    transducer_open: u64,
    transducer_intermittent: u64,
}

impl SurfaceIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, component: &str) -> Self {
        let i = take(map, component, 11);
        Self {
            jam: i[0],
            runaway: i[1],
            supply_loss: i[2],
            transducer_fault: i[3],
            disconnect: i[4],
            flutter_damper_loss: i[5],
            valve_leakage: i[6],
            piston_seal_wear: i[7],
            transducer_drift: i[8],
            transducer_open: i[9],
            transducer_intermittent: i[10],
        }
    }

    /// The actuator-level faults, for every actuator on this surface.
    ///
    /// The catalogue registers one entry per *surface*, not per actuator
    /// (`27_fctl.ail_l1 actuator jam`, not `... actuator 1 jam`), so an
    /// armed magnitude applies to each of that surface's actuators alike:
    /// a surface-level jam pins the surface, which is what the registered
    /// effect describes.
    ///
    /// `travel_span_rad` normalises the `transducer_fault` magnitude: the
    /// registry defines it as "0 healthy .. 1 feedback frozen at the angle
    /// seen when the fault began (or biased, for a partial fault)", so a
    /// partial magnitude is that fraction of the transducer's own full
    /// scale -- which for a surface position transducer is the surface's
    /// travel -- and the endpoint is the freeze the text names.
    fn actuator_faults(&self, faults: &Faults, travel_span_rad: f64) -> ActuatorFaults {
        let transducer = faults.get(self.transducer_fault);
        ActuatorFaults {
            supply_loss: faults.get(self.supply_loss),
            jam: faults.get(self.jam),
            runaway: faults.get(self.runaway),
            // The catalogue registers one runaway per surface with no
            // direction parameter, so it drives in the positive sense
            // (trailing edge up / spoiler extending / rudder right). An
            // opposite-direction hardover would need its own failure id.
            runaway_sign: 1.0,
            transducer_frozen: transducer >= 1.0,
            transducer_bias_rad: if transducer >= 1.0 { 0.0 } else { transducer * travel_span_rad },
            valve_leakage: faults.get(self.valve_leakage),
            piston_seal_wear: faults.get(self.piston_seal_wear),
        }
    }

    fn surface_faults(&self, faults: &Faults) -> SurfaceFaults {
        SurfaceFaults {
            // Registered magnitude: "boolean in practice: 0 linked, 1
            // sheared" -- a linkage either transmits torque or it does not.
            disconnected: faults.get(self.disconnect) >= 0.5,
            flutter_damper_loss: faults.get(self.flutter_damper_loss),
        }
    }

    /// The computers' own monitoring channel A. The catalogue registers
    /// one drift/open/intermittent failure per surface rather than one per
    /// channel, so it is armed on channel A and channel B stays healthy --
    /// which is the case that actually exercises the dual-channel
    /// disagreement monitor.
    fn transducer_faults(&self, faults: &Faults) -> TransducerFaults {
        TransducerFaults {
            drift: faults.get(self.transducer_drift),
            open_circuit: faults.get(self.transducer_open),
            intermittent: faults.get(self.transducer_intermittent),
        }
    }
}

/// THS failures, in `registry.rs`'s order.
#[derive(Clone, Copy, Debug)]
struct ThsIds {
    motor_green_supply_loss: u64,
    motor_yellow_supply_loss: u64,
    no_back_failure: u64,
    ballscrew_jam: u64,
    transducer_drift: u64,
    transducer_open: u64,
}

/// Rudder trim failures, in order.
#[derive(Clone, Copy, Debug)]
struct RudderTrimIds {
    motor_failure: u64,
    jam: u64,
}

/// One high-lift drive line's seven failures, in order.
#[derive(Clone, Copy, Debug)]
struct HighLiftIds {
    pcu_jam: u64,
    pcu_runaway: u64,
    pcu_supply_loss: u64,
    limiter_bypass: u64,
    inboard_shaft_break: u64,
    outboard_shaft_break: u64,
    wingtip_brake_fail: u64,
}

impl HighLiftIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, component: &str) -> Self {
        let i = take(map, component, 7);
        Self {
            pcu_jam: i[0],
            pcu_runaway: i[1],
            pcu_supply_loss: i[2],
            limiter_bypass: i[3],
            inboard_shaft_break: i[4],
            outboard_shaft_break: i[5],
            wingtip_brake_fail: i[6],
        }
    }

    fn faults(&self, faults: &Faults) -> HighLiftFaults {
        HighLiftFaults {
            pcu: ActuatorFaults {
                jam: faults.get(self.pcu_jam),
                runaway: faults.get(self.pcu_runaway),
                runaway_sign: 1.0,
                supply_loss: faults.get(self.pcu_supply_loss),
                ..ActuatorFaults::default()
            },
            limiter_bypass: faults.get(self.limiter_bypass),
            inboard_shaft_break: faults.get(self.inboard_shaft_break),
            outboard_shaft_break: faults.get(self.outboard_shaft_break),
            wingtip_brake_fail: faults.get(self.wingtip_brake_fail),
        }
    }
}

// ---------------------------------------------------------------------------
// Component id tables (the ids `registry.rs` uses, in this module's own
// index order).
// ---------------------------------------------------------------------------

/// `[side][inward, middle, outward]` -- the order
/// `integration::flight_control_surfaces::PhysicalSurfaces` uses. The
/// registry numbers them the other way round (`ail_l1` is the *outward*
/// panel), which is why this table is spelled out rather than generated.
const AILERON_COMPONENTS: [[&str; 3]; 2] = [
    ["27_fctl.ail_l3", "27_fctl.ail_l2", "27_fctl.ail_l1"],
    ["27_fctl.ail_r3", "27_fctl.ail_r2", "27_fctl.ail_r1"],
];
/// `[side][inward, outward]`.
const ELEVATOR_COMPONENTS: [[&str; 2]; 2] = [
    ["27_fctl.elev_l_inbd", "27_fctl.elev_l_outbd"],
    ["27_fctl.elev_r_inbd", "27_fctl.elev_r_outbd"],
];
/// `[upper, lower]`.
const RUDDER_COMPONENTS: [&str; 2] = ["27_fctl.rud_upper", "27_fctl.rud_lower"];
/// `[flap, slat, droop nose]` x `[left, right]`.
const HIGH_LIFT_COMPONENTS: [[&str; 2]; 3] =
    [["27_fctl.flap_l", "27_fctl.flap_r"], ["27_fctl.slat_l", "27_fctl.slat_r"], ["27_fctl.droop_l", "27_fctl.droop_r"]];

fn spoiler_component(side: usize, index: usize) -> String {
    format!("27_fctl.splr_{}{}", if side == 0 { 'l' } else { 'r' }, index + 1)
}

/// `registry::fault_var`'s own rule, restated: the component's bare name,
/// upper-cased, as `FCTL_<NAME>_FAULT`.
fn fault_var(component: &str) -> String {
    let bare = component.split('.').next_back().unwrap_or(component);
    format!("FCTL_{}_FAULT", bare.to_uppercase())
}

/// A component's bare name, for the physical (non-trigger) study variables.
fn bare(component: &str) -> String {
    component.split('.').next_back().unwrap_or(component).to_uppercase()
}

// ---------------------------------------------------------------------------
// Surface construction. Masses and chords are FlyByWire's own A380 rigid
// bodies (`a380_systems/src/hydraulic/mod.rs`); everything else comes from
// this directory's own cited constructors.
// ---------------------------------------------------------------------------

fn limits(min_deg: f64, max_deg: f64) -> SurfaceLimits {
    SurfaceLimits { min_rad: min_deg.to_radians(), max_rad: max_deg.to_radians() }
}

/// `panel`: 0 inward, 1 middle, 2 outward. Masses/chords `mod.rs:449-458`
/// (inward 108 kg / 1.6 m, middle 118 kg / 1.37 m, outward 128 kg /
/// 1.4 m).
fn aileron_surface(panel: usize) -> ControlSurface<2> {
    let (mass_kg, chord_m) = match panel {
        0 => (108.0, 1.6),
        1 => (118.0, 1.37),
        _ => (128.0, 1.4),
    };
    ControlSurface::new(
        std::array::from_fn(|_| PowerControlUnit::new(ActuatorGeometry::aileron())),
        HingeMomentCoefficients::aileron(),
        inertia_uniform_plate_kg_m2(mass_kg, chord_m),
        limits(AILERON_MIN_DEG, AILERON_MAX_DEG),
        SurfaceDamping::aileron(),
        0.0,
    )
}

/// `panel`: 0 inward, 1 outward. Masses/chords `mod.rs:784-794` (inner
/// 189 kg / 2.49 m, outer 243 kg / 2.23 m).
fn elevator_surface(panel: usize) -> ControlSurface<2> {
    let (mass_kg, chord_m) = if panel == 0 { (189.0, 2.49) } else { (243.0, 2.23) };
    ControlSurface::new(
        std::array::from_fn(|_| PowerControlUnit::new(ActuatorGeometry::elevator())),
        HingeMomentCoefficients::elevator(),
        inertia_uniform_plate_kg_m2(mass_kg, chord_m),
        limits(AILERON_MIN_DEG, AILERON_MAX_DEG),
        SurfaceDamping::elevator(),
        0.0,
    )
}

/// `panel`: 0 upper, 1 lower. Masses/chords `mod.rs:964-974` (upper 357 kg
/// / 2.9 m, lower 304 kg / 3.41 m).
fn rudder_surface(panel: usize) -> ControlSurface<2> {
    let (mass_kg, chord_m) = if panel == 0 { (357.0, 2.9) } else { (304.0, 3.41) };
    ControlSurface::new(
        std::array::from_fn(|_| PowerControlUnit::new(ActuatorGeometry::rudder())),
        HingeMomentCoefficients::rudder(),
        inertia_uniform_plate_kg_m2(mass_kg, chord_m),
        limits(-RUDDER_LIMIT_DEG, RUDDER_LIMIT_DEG),
        SurfaceDamping::rudder(),
        0.0,
    )
}

/// Mass/chord `mod.rs:624-632` (42 kg, 0.685 m).
fn spoiler_surface() -> ControlSurface<1> {
    ControlSurface::new(
        [PowerControlUnit::new(ActuatorGeometry::spoiler())],
        HingeMomentCoefficients::spoiler(),
        inertia_uniform_plate_kg_m2(42.0, 0.685),
        limits(0.0, SPOILER_MAX_DEG),
        SurfaceDamping::spoiler(),
        0.0,
    )
}

/// Which hydraulic circuit each spoiler's single actuator is fed from:
/// odd-numbered panels yellow, even-numbered green, exactly as
/// FlyByWire's own `SpoilerGroup::update` wires them
/// (`a380_systems/src/hydraulic/mod.rs:6930-6968`). Panel 6 additionally
/// has an AC ESS electrical backup (`SPOILER_6_EBHA_BUS`, `mod.rs:574`).
fn spoiler_is_green(index: usize) -> bool {
    index % 2 == 1
}

// ---------------------------------------------------------------------------
// Inputs the plugin supplies.
// ---------------------------------------------------------------------------

/// What FlyByWire's own PRIM/SEC asked each surface to do this tick, in
/// the same body-angle degrees `integration::flight_control_surfaces`
/// converts to and from. All zero = every surface commanded to neutral and
/// every high-lift device retracted.
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceCommands {
    /// `[side][inward, middle, outward]`, degrees, positive TE up.
    pub ailerons_deg: [[f64; 3]; 2],
    /// `[side][inward, outward]`, degrees, positive TE up.
    pub elevators_deg: [[f64; 2]; 2],
    /// `[upper, lower]`, degrees.
    pub rudders_deg: [f64; 2],
    /// `[side][spoiler 1..=8]`, degrees up.
    pub spoilers_deg: [[f64; 8]; 2],
    /// Degrees, positive nose up.
    pub ths_deg: f64,
    pub rudder_trim_deg: f64,
    /// One commanded position per high-lift device (both wings are
    /// commanded alike; asymmetry is a *consequence*, not a command).
    pub flap_deg: f64,
    pub slat_deg: f64,
    pub droop_deg: f64,
}

/// Every live surface angle, degrees, in the layout
/// `integration::flight_control_surfaces::PhysicalSurfaces` uses.
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceAngles {
    /// `[side][inward, middle, outward]`, positive TE up.
    pub ailerons_deg: [[f64; 3]; 2],
    /// `[side][inward, outward]`, positive TE up.
    pub elevators_deg: [[f64; 2]; 2],
    /// `[upper, lower]`.
    pub rudders_deg: [f64; 2],
    /// `[side][spoiler 1..=8]`, degrees up.
    pub spoilers_deg: [[f64; 8]; 2],
    /// Degrees, positive nose up.
    pub ths_deg: f64,
    pub rudder_trim_deg: f64,
    /// `[side][inboard station, outboard station]`. These are **not**
    /// calibrated against a real A380 travel range
    /// (`high_lift::HighLiftSystem`'s sizing is GENERIC), which is why
    /// `PhysicalSurfaces::flap_deg`/`slat_deg` must stay `None` --
    /// `docs/deep/integration.md`'s own known gap.
    pub flap_deg: [[f64; 2]; 2],
    pub slat_deg: [[f64; 2]; 2],
    pub droop_deg: [[f64; 2]; 2],
}

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

pub struct FlightControlsLive {
    // Surfaces, indexed as `SurfaceAngles` is.
    ailerons: [[ControlSurface<2>; 3]; 2],
    elevators: [[ControlSurface<2>; 2]; 2],
    rudders: [ControlSurface<2>; 2],
    spoilers: [[ControlSurface<1>; 8]; 2],
    ths: TrimmableHorizontalStabilizer,
    rudder_trim: RudderTrimActuator,
    /// `[flap, slat, droop nose]`.
    high_lift: [HighLiftPair; 3],
    ground_spoiler: GroundSpoilerLogic,

    // The computers' own position monitoring, one dual channel per surface.
    aileron_monitors: [[DualTransducer; 3]; 2],
    elevator_monitors: [[DualTransducer; 2]; 2],
    rudder_monitors: [DualTransducer; 2],
    spoiler_monitors: [[DualTransducer; 8]; 2],
    ths_monitor: DualTransducer,

    // Failure ids, read out of `registry.rs` itself.
    aileron_ids: [[SurfaceIds; 3]; 2],
    elevator_ids: [[SurfaceIds; 2]; 2],
    rudder_ids: [SurfaceIds; 2],
    spoiler_ids: [[SurfaceIds; 8]; 2],
    ths_ids: ThsIds,
    rudder_trim_ids: RudderTrimIds,
    high_lift_ids: [[HighLiftIds; 2]; 3],
    ground_spoiler_ids: [u64; 2],
    /// `(FCTL_<COMPONENT>_FAULT, that component's failure ids)` for all 37
    /// components -- the aggregate variable every ECAM trigger in this
    /// area's `registry.rs` reads.
    fault_vars: Vec<(String, Vec<u64>)>,
    /// This tick's armed magnitude for every failure in this area, kept so
    /// `publish` (which is handed no `Faults`) can report the aggregate
    /// `FCTL_<COMPONENT>_FAULT` value the ECAM triggers read.
    armed: BTreeMap<u64, f64>,

    // Live state.
    angles: SurfaceAngles,
    aileron_out: [[SurfaceOutput; 3]; 2],
    elevator_out: [[SurfaceOutput; 2]; 2],
    rudder_out: [SurfaceOutput; 2],
    spoiler_out: [[SurfaceOutput; 8]; 2],
    ths_out: super::ths::ThsOutput,
    monitor_fault: BTreeMap<String, bool>,
    ground_spoiler_deployed: bool,
    high_lift_brake: [bool; 3],

    // Inputs `Truth` does not carry yet (see module doc).
    commands: SurfaceCommands,
    alpha_rad: f64,
    ground_spoiler_lever_armed: bool,
    go_around_selected: bool,
}

impl Default for FlightControlsLive {
    fn default() -> Self {
        Self::new()
    }
}

impl FlightControlsLive {
    pub fn new() -> Self {
        let map = registered_failures();

        let mut fault_vars: Vec<(String, Vec<u64>)> = Vec::new();
        for component in map.keys() {
            fault_vars.push((fault_var(component), map[component].clone()));
        }

        let ths = take(&map, "27_fctl.ths", 6);
        let trim = take(&map, "27_fctl.rudder_trim", 2);
        let gnd = take(&map, "27_fctl.gnd_splr_logic", 2);

        Self {
            ailerons: std::array::from_fn(|_| std::array::from_fn(aileron_surface)),
            elevators: std::array::from_fn(|_| std::array::from_fn(elevator_surface)),
            rudders: std::array::from_fn(rudder_surface),
            spoilers: std::array::from_fn(|_| std::array::from_fn(|_| spoiler_surface())),
            ths: TrimmableHorizontalStabilizer::new_generic(),
            rudder_trim: RudderTrimActuator::new_generic(),
            high_lift: [
                HighLiftPair::new(HighLiftSystem::new_flap(), HighLiftSystem::new_flap(), HIGH_LIFT_ASYMMETRY_TIMER_S),
                HighLiftPair::new(HighLiftSystem::new_slat(), HighLiftSystem::new_slat(), HIGH_LIFT_ASYMMETRY_TIMER_S),
                HighLiftPair::new(HighLiftSystem::new_droop_nose(), HighLiftSystem::new_droop_nose(), HIGH_LIFT_ASYMMETRY_TIMER_S),
            ],
            ground_spoiler: GroundSpoilerLogic::new(),

            aileron_monitors: std::array::from_fn(|_| std::array::from_fn(|_| DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S))),
            elevator_monitors: std::array::from_fn(|_| std::array::from_fn(|_| DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S))),
            rudder_monitors: std::array::from_fn(|_| DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S)),
            spoiler_monitors: std::array::from_fn(|_| std::array::from_fn(|_| DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S))),
            ths_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),

            aileron_ids: std::array::from_fn(|side| std::array::from_fn(|p| SurfaceIds::build(&map, AILERON_COMPONENTS[side][p]))),
            elevator_ids: std::array::from_fn(|side| std::array::from_fn(|p| SurfaceIds::build(&map, ELEVATOR_COMPONENTS[side][p]))),
            rudder_ids: std::array::from_fn(|p| SurfaceIds::build(&map, RUDDER_COMPONENTS[p])),
            spoiler_ids: std::array::from_fn(|side| std::array::from_fn(|i| SurfaceIds::build(&map, &spoiler_component(side, i)))),
            ths_ids: ThsIds {
                motor_green_supply_loss: ths[0],
                motor_yellow_supply_loss: ths[1],
                no_back_failure: ths[2],
                ballscrew_jam: ths[3],
                transducer_drift: ths[4],
                transducer_open: ths[5],
            },
            rudder_trim_ids: RudderTrimIds { motor_failure: trim[0], jam: trim[1] },
            high_lift_ids: std::array::from_fn(|d| std::array::from_fn(|side| HighLiftIds::build(&map, HIGH_LIFT_COMPONENTS[d][side]))),
            ground_spoiler_ids: [gnd[0], gnd[1]],
            fault_vars,
            armed: BTreeMap::new(),

            angles: SurfaceAngles::default(),
            aileron_out: Default::default(),
            elevator_out: Default::default(),
            rudder_out: Default::default(),
            spoiler_out: Default::default(),
            ths_out: Default::default(),
            monitor_fault: BTreeMap::new(),
            ground_spoiler_deployed: false,
            high_lift_brake: [false; 3],

            commands: SurfaceCommands::default(),
            alpha_rad: 0.0,
            ground_spoiler_lever_armed: false,
            go_around_selected: false,
        }
    }

    /// What FlyByWire's own pipeline asked for this tick. The plugin reads
    /// the same normalised `HYD_*_DEFLECTION` variables
    /// `integration::flight_control_surfaces::SurfaceOverrideWriter`
    /// already caches identifiers for, converts them back to body degrees
    /// with `flight_controls.rs`'s own `aileron_or_elevator_down_deg` /
    /// `rudder_right_deg` / `spoiler_up_deg`, and passes them here.
    pub fn set_commands(&mut self, commands: SurfaceCommands) {
        self.commands = commands;
    }

    /// Angle of attack at the tail/wing, radians, for `hinge_moment.rs`'s
    /// `Ch_alpha` term.
    pub fn set_alpha_rad(&mut self, alpha_rad: f64) {
        self.alpha_rad = alpha_rad;
    }

    /// The two cockpit discretes the ground-spoiler logic needs: the
    /// speedbrake lever armed for automatic deployment, and a go-around
    /// selected.
    pub fn set_ground_spoiler_controls(&mut self, lever_armed: bool, go_around_selected: bool) {
        self.ground_spoiler_lever_armed = lever_armed;
        self.go_around_selected = go_around_selected;
    }

    /// Every live surface angle, degrees.
    ///
    /// This is what fills
    /// `integration::flight_control_surfaces::PhysicalSurfaces`. The whole
    /// call, in the plugin:
    ///
    /// ```ignore
    /// let a = flight_controls_live.surface_angles();
    /// let physical = PhysicalSurfaces {
    ///     ailerons_deg: a.ailerons_deg.map(|side| side.map(Some)),
    ///     elevators_deg: a.elevators_deg.map(|side| side.map(Some)),
    ///     rudders_deg: a.rudders_deg.map(Some),
    ///     spoilers_deg: a.spoilers_deg.map(|side| side.map(Some)),
    ///     ths_deg: Some(a.ths_deg),
    ///     // `high_lift`'s travel range is GENERIC and not yet calibrated
    ///     // against the real A380, so these stay `None` and FlyByWire's
    ///     // own flap/slat position is left alone -- see
    ///     // `docs/deep/integration.md`'s known gaps.
    ///     flap_deg: [None; 2],
    ///     slat_deg: [None; 2],
    /// };
    /// surface_override_writer.apply(&mut vars, &physical);
    /// ```
    pub fn surface_angles(&self) -> SurfaceAngles {
        self.angles
    }

    /// Whether the ground spoilers are currently commanded out.
    pub fn ground_spoilers_deployed(&self) -> bool {
        self.ground_spoiler_deployed
    }

    fn aero(&self, truth: &Truth) -> AeroInputs {
        // Ideal gas from the real X-Plane static pressure and temperature
        // `integration::weather_truth` reads, then `q = 0.5 rho V^2`.
        let t_k = (truth.environment.sat_c + 273.15).max(1.0);
        let rho = truth.environment.ambient_pressure_pa.max(0.0) / (R_AIR_J_KGK * t_k);
        let tas = truth.environment.tas_ms.max(0.0);
        AeroInputs {
            dynamic_pressure_pa: 0.5 * rho * tas * tas,
            alpha_rad: self.alpha_rad,
            mach: truth.environment.mach(),
            // `AeroInputs`' own documented default critical Mach; this area
            // has no A380-specific figure to put here.
            ..AeroInputs::default()
        }
    }

    fn power(truth: &Truth) -> PowerAvailability {
        let frac = |pa: f64| (pa / HYDRAULIC_SUPPLY_PA).clamp(0.0, 1.0);
        PowerAvailability {
            green: frac(truth.hydraulic_pressure_pa[0]),
            yellow: frac(truth.hydraulic_pressure_pa[1]),
            // An EHA/EBHA's own motor-pump runs off an AC bus
            // (`a380_systems/src/hydraulic/mod.rs:81,390,574`: the 247XP
            // and AC ESS buses). `Truth` carries the four main AC bus
            // voltages but not those two named buses, so an EHA counts as
            // powered while any AC bus is live.
            eha: if truth.ac_bus_volts.iter().any(|&v| v > AC_BUS_LIVE_V) { 1.0 } else { 0.0 },
        }
    }

    /// Per-actuator supply fraction for one surface, from the allocation's
    /// own declared power source for each actuator.
    fn pressures<const N: usize>(allocations: &[ActuatorAllocation; N], power: &PowerAvailability) -> [f64; N] {
        std::array::from_fn(|i| {
            use super::allocation::PowerSource;
            match allocations[i].options.first().map(|&(_, source)| source) {
                Some(PowerSource::Green) => power.green,
                Some(PowerSource::Yellow) => power.yellow,
                Some(PowerSource::Eha) => power.eha,
                None => 0.0,
            }
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn step_surface<const N: usize>(
        surface: &mut ControlSurface<N>,
        monitor: &mut DualTransducer,
        ids: &SurfaceIds,
        allocations: &[ActuatorAllocation; N],
        health: &ComputerHealth,
        power: &PowerAvailability,
        faults: &Faults,
        commanded_deg: f64,
        travel_span_rad: f64,
        aero: &AeroInputs,
        dt: f64,
    ) -> (SurfaceOutput, bool) {
        let (modes, _) = mode_for_surface(allocations, health, power);
        let pressures = Self::pressures(allocations, power);
        let actuator_faults = ids.actuator_faults(faults, travel_span_rad);
        let out = surface.step(
            modes,
            commanded_deg.to_radians(),
            pressures,
            [actuator_faults; N],
            &ids.surface_faults(faults),
            aero,
            dt,
        );
        let monitoring = monitor
            .step(out.angle_rad, &ids.transducer_faults(faults), &TransducerFaults::default(), TRANSDUCER_DISAGREE_RAD, dt)
            .monitoring_fault;
        (out, monitoring)
    }
}

impl LiveArea for FlightControlsLive {
    fn name(&self) -> &'static str {
        "flight_controls"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        self.armed.clear();
        for (_, ids) in &self.fault_vars {
            for &id in ids {
                let magnitude = faults.get(id);
                if magnitude > 0.0 {
                    self.armed.insert(id, magnitude);
                }
            }
        }
        let aero = self.aero(truth);
        let power = Self::power(truth);
        // Computer availability: this area does not model the PRIM/SEC
        // power distribution (that is `deep::electrical`/
        // `deep::avionics_network`) and `Truth` names no per-computer bus,
        // so all six are treated as available while the aircraft has AC
        // power at all, and unavailable when it has none.
        let computers_powered = truth.ac_bus_volts.iter().any(|&v| v > AC_BUS_LIVE_V);
        let health = ComputerHealth { prim: [computers_powered; 3], sec: [computers_powered; 3] };

        let aileron_span = (AILERON_MAX_DEG - AILERON_MIN_DEG).to_radians();
        let rudder_span = (2.0 * RUDDER_LIMIT_DEG).to_radians();
        let spoiler_span = SPOILER_MAX_DEG.to_radians();

        let aileron_allocations = [aileron_inboard(), aileron_midboard(), aileron_outboard()];
        let elevator_allocations = [elevator_inboard(), elevator_outboard()];
        let rudder_allocations = [rudder_upper(), rudder_lower()];

        for side in 0..2 {
            for panel in 0..3 {
                let (out, monitoring) = Self::step_surface(
                    &mut self.ailerons[side][panel],
                    &mut self.aileron_monitors[side][panel],
                    &self.aileron_ids[side][panel],
                    &aileron_allocations[panel],
                    &health,
                    &power,
                    faults,
                    self.commands.ailerons_deg[side][panel],
                    aileron_span,
                    &aero,
                    dt,
                );
                self.aileron_out[side][panel] = out;
                self.angles.ailerons_deg[side][panel] = out.angle_rad.to_degrees();
                self.monitor_fault.insert(bare(AILERON_COMPONENTS[side][panel]), monitoring);
            }
            for panel in 0..2 {
                let (out, monitoring) = Self::step_surface(
                    &mut self.elevators[side][panel],
                    &mut self.elevator_monitors[side][panel],
                    &self.elevator_ids[side][panel],
                    &elevator_allocations[panel],
                    &health,
                    &power,
                    faults,
                    self.commands.elevators_deg[side][panel],
                    aileron_span,
                    &aero,
                    dt,
                );
                self.elevator_out[side][panel] = out;
                self.angles.elevators_deg[side][panel] = out.angle_rad.to_degrees();
                self.monitor_fault.insert(bare(ELEVATOR_COMPONENTS[side][panel]), monitoring);
            }
        }

        for panel in 0..2 {
            let (out, monitoring) = Self::step_surface(
                &mut self.rudders[panel],
                &mut self.rudder_monitors[panel],
                &self.rudder_ids[panel],
                &rudder_allocations[panel],
                &health,
                &power,
                faults,
                self.commands.rudders_deg[panel],
                rudder_span,
                &aero,
                dt,
            );
            self.rudder_out[panel] = out;
            self.angles.rudders_deg[panel] = out.angle_rad.to_degrees();
            self.monitor_fault.insert(bare(RUDDER_COMPONENTS[panel]), monitoring);
        }

        // ---- Ground spoilers. Weight-on-wheels and wheel speed come from
        // `Truth`; no radio altimeter reading exists there, so height is
        // zero on the ground by definition and pressure altitude in the
        // air -- which only matters to the wheel-spin-up branch, and that
        // branch cannot be reached without being on the ground anyway.
        let on_ground = truth.on_ground;
        let ground_spoiler_faults = GroundSpoilerLogicFaults {
            fails_to_deploy: faults.get(self.ground_spoiler_ids[0]),
            fails_to_retract: faults.get(self.ground_spoiler_ids[1]),
        };
        let ground_spoiler_out = self.ground_spoiler.step(
            &GroundSpoilerInputs {
                lever_armed: self.ground_spoiler_lever_armed,
                main_gear_wow: [on_ground; 2],
                wheel_speed_kt: if on_ground { truth.environment.tas_ms * MS_TO_KT } else { 0.0 },
                radio_alt_ft: if on_ground { 0.0 } else { truth.altitude_ft },
                go_around_selected: self.go_around_selected,
            },
            &ground_spoiler_faults,
        );
        self.ground_spoiler_deployed = ground_spoiler_out.deployed;

        for side in 0..2 {
            for i in 0..8 {
                // Every A380 spoiler panel is ground-spoiler capable, so
                // the ground-spoiler command drives each of them fully out
                // unless the flight-spoiler command already asks for more.
                let commanded_deg = self.commands.spoilers_deg[side][i].max(ground_spoiler_out.deploy_command * SPOILER_MAX_DEG);
                let source = if spoiler_is_green(i) { power.green } else { power.yellow };
                // Panel 6 (index 5) is an EBHA: it keeps working off its
                // own electrical motor-pump when its circuit is lost.
                let supply = if i == 5 { source.max(power.eha) } else { source };
                let ids = &self.spoiler_ids[side][i];
                let actuator_faults = ids.actuator_faults(faults, spoiler_span);
                let out = self.spoilers[side][i].step(
                    [if supply > 0.5 { ActuatorMode::Active } else { ActuatorMode::Damping }],
                    commanded_deg.to_radians(),
                    [supply],
                    [actuator_faults],
                    &ids.surface_faults(faults),
                    &aero,
                    dt,
                );
                let monitoring = self.spoiler_monitors[side][i]
                    .step(out.angle_rad, &ids.transducer_faults(faults), &TransducerFaults::default(), TRANSDUCER_DISAGREE_RAD, dt)
                    .monitoring_fault;
                self.spoiler_out[side][i] = out;
                self.angles.spoilers_deg[side][i] = out.angle_rad.to_degrees();
                self.monitor_fault.insert(bare(&spoiler_component(side, i)), monitoring);
            }
        }

        // ---- THS: two hydraulic motors, green and yellow
        // (`allocation::ths_motors`). Active while the commanded trim
        // differs from where it is, Damping otherwise -- the real
        // operational pattern `ths.rs::step` documents (the no-back device,
        // not the motors, holds position between inputs).
        let ths_allocations = ths_motors();
        let (mut ths_modes, _) = mode_for_surface(&ths_allocations, &health, &power);
        let ths_command_rad = self.commands.ths_deg.to_radians();
        if (ths_command_rad - self.ths.angle_rad()).abs() < 1.0e-4 {
            ths_modes = [ActuatorMode::Damping; 2];
        }
        let ths_faults = ThsFaults {
            motor_green: ActuatorFaults { supply_loss: faults.get(self.ths_ids.motor_green_supply_loss), ..ActuatorFaults::default() },
            motor_yellow: ActuatorFaults { supply_loss: faults.get(self.ths_ids.motor_yellow_supply_loss), ..ActuatorFaults::default() },
            no_back_failure: faults.get(self.ths_ids.no_back_failure),
            ballscrew_jam: faults.get(self.ths_ids.ballscrew_jam),
        };
        self.ths_out = self.ths.step(ths_modes, ths_command_rad, [power.green, power.yellow], &ths_faults, &aero, dt);
        self.angles.ths_deg = self.ths.angle_deg();
        let ths_monitoring = self
            .ths_monitor
            .step(
                self.ths.angle_rad(),
                &TransducerFaults {
                    drift: faults.get(self.ths_ids.transducer_drift),
                    open_circuit: faults.get(self.ths_ids.transducer_open),
                    intermittent: 0.0,
                },
                &TransducerFaults::default(),
                TRANSDUCER_DISAGREE_RAD,
                dt,
            )
            .monitoring_fault;
        self.monitor_fault.insert("THS".to_string(), ths_monitoring);

        // ---- Rudder trim: its own electric motor-pump, off any live AC bus.
        let trim_power = if computers_powered { 1.0 } else { 0.0 };
        self.angles.rudder_trim_deg = self
            .rudder_trim
            .step(
                self.commands.rudder_trim_deg.to_radians(),
                trim_power,
                &ElectricPumpFaults { motor_failure: faults.get(self.rudder_trim_ids.motor_failure) },
                &ActuatorFaults { jam: faults.get(self.rudder_trim_ids.jam), runaway_sign: 1.0, ..ActuatorFaults::default() },
                dt,
            )
            .to_degrees();

        // ---- High lift. Each drive line's PCU is a hydraulic rotary motor
        // fed from the flight-control circuits; with no per-motor split
        // registered in this area, the better of the two circuits is what
        // it can draw on.
        let high_lift_supply = power.green.max(power.yellow);
        let high_lift_commands = [self.commands.flap_deg, self.commands.slat_deg, self.commands.droop_deg];
        for d in 0..3 {
            let device_faults = [self.high_lift_ids[d][0].faults(faults), self.high_lift_ids[d][1].faults(faults)];
            let (left, right, tripped) = self.high_lift[d].step(
                if high_lift_supply > 0.5 { ActuatorMode::Active } else { ActuatorMode::Damping },
                high_lift_commands[d].to_radians(),
                [high_lift_supply; 2],
                device_faults,
                aero.dynamic_pressure_pa,
                HIGH_LIFT_ASYMMETRY_RAD,
                dt,
            );
            self.high_lift_brake[d] = tripped;
            let target = match d {
                0 => &mut self.angles.flap_deg,
                1 => &mut self.angles.slat_deg,
                _ => &mut self.angles.droop_deg,
            };
            target[0] = [left.inboard_angle_rad.to_degrees(), left.outboard_angle_rad.to_degrees()];
            target[1] = [right.inboard_angle_rad.to_degrees(), right.outboard_angle_rad.to_degrees()];
        }
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        // The 37 aggregate variables every ECAM trigger in this area's
        // `registry.rs` reads: "0..1, the worst of that component's active
        // fault magnitudes" (that file's own definition).
        for (name, ids) in &self.fault_vars {
            let worst = ids.iter().map(|&id| self.armed.get(&id).copied().unwrap_or(0.0)).fold(0.0_f64, f64::max);
            out(name, worst);
        }

        // The emergent physical state behind them, for the Study pages and
        // for anything that wants the real position rather than the armed
        // magnitude.
        for side in 0..2 {
            for panel in 0..3 {
                let n = bare(AILERON_COMPONENTS[side][panel]);
                out(&format!("FCTL_{n}_DEFLECTION_DEG"), self.angles.ailerons_deg[side][panel]);
                out(&format!("FCTL_{n}_BLOWN_BACK"), f64::from(u8::from(self.aileron_out[side][panel].blown_back)));
            }
            for panel in 0..2 {
                let n = bare(ELEVATOR_COMPONENTS[side][panel]);
                out(&format!("FCTL_{n}_DEFLECTION_DEG"), self.angles.elevators_deg[side][panel]);
                out(&format!("FCTL_{n}_BLOWN_BACK"), f64::from(u8::from(self.elevator_out[side][panel].blown_back)));
            }
            for i in 0..8 {
                let n = bare(&spoiler_component(side, i));
                out(&format!("FCTL_{n}_DEFLECTION_DEG"), self.angles.spoilers_deg[side][i]);
                out(&format!("FCTL_{n}_BLOWN_BACK"), f64::from(u8::from(self.spoiler_out[side][i].blown_back)));
            }
        }
        for panel in 0..2 {
            let n = bare(RUDDER_COMPONENTS[panel]);
            out(&format!("FCTL_{n}_DEFLECTION_DEG"), self.angles.rudders_deg[panel]);
            out(&format!("FCTL_{n}_BLOWN_BACK"), f64::from(u8::from(self.rudder_out[panel].blown_back)));
        }
        out("FCTL_THS_DEFLECTION_DEG", self.angles.ths_deg);
        out("FCTL_THS_NO_BACK_ENGAGED", f64::from(u8::from(self.ths_out.no_back_engaged)));
        out("FCTL_RUDDER_TRIM_DEFLECTION_DEG", self.angles.rudder_trim_deg);

        const HIGH_LIFT_NAMES: [&str; 3] = ["FLAP", "SLAT", "DROOP"];
        let high_lift_angles = [self.angles.flap_deg, self.angles.slat_deg, self.angles.droop_deg];
        for d in 0..3 {
            for (side, tag) in [(0usize, 'L'), (1usize, 'R')] {
                out(&format!("FCTL_{}_{tag}_INBOARD_DEG", HIGH_LIFT_NAMES[d]), high_lift_angles[d][side][0]);
                out(&format!("FCTL_{}_{tag}_OUTBOARD_DEG", HIGH_LIFT_NAMES[d]), high_lift_angles[d][side][1]);
            }
            out(&format!("FCTL_{}_WINGTIP_BRAKE_ON", HIGH_LIFT_NAMES[d]), f64::from(u8::from(self.high_lift_brake[d])));
        }

        out("FCTL_GND_SPLR_DEPLOYED", f64::from(u8::from(self.ground_spoiler_deployed)));

        for (component, fault) in &self.monitor_fault {
            out(&format!("FCTL_{component}_POSITION_MONITOR_FAULT"), f64::from(u8::from(*fault)));
        }
    }
}

/// This area's live system, for `deep::live::Deep::with_area`.
pub fn live_system() -> Box<dyn LiveArea> {
    Box::new(FlightControlsLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Registry;

    /// Sea level, 200 m/s true: about 24.5 kPa of dynamic pressure, both
    /// hydraulic circuits at their regulated pressure, AC power on.
    fn flying_truth() -> Truth {
        let mut t = Truth {
            dt_s: 0.02,
            on_ground: false,
            altitude_ft: 5000.0,
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            hydraulic_pressure_pa: [HYDRAULIC_SUPPLY_PA; 2],
            ..Truth::default()
        };
        t.environment.tas_ms = 200.0;
        t
    }

    /// Stationary on the ground with both circuits pressurised: no
    /// aerodynamic hinge moment at all, so an actuator fault is the only
    /// thing that can move a surface off its command.
    fn parked_truth() -> Truth {
        Truth {
            dt_s: 0.02,
            on_ground: true,
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            hydraulic_pressure_pa: [HYDRAULIC_SUPPLY_PA; 2],
            ..Truth::default()
        }
    }

    fn run(area: &mut FlightControlsLive, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let ticks = (seconds / truth.dt_s).round() as usize;
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
        let mut published = BTreeMap::new();
        area.publish(&mut |name, value| {
            published.insert(name.to_string(), value);
        });
        published
    }

    fn ids() -> FlightControlsLive {
        FlightControlsLive::new()
    }

    fn push_surface(s: &SurfaceIds, bound: &mut Vec<u64>) {
        bound.extend([
            s.jam,
            s.runaway,
            s.supply_loss,
            s.transducer_fault,
            s.disconnect,
            s.flutter_damper_loss,
            s.valve_leakage,
            s.piston_seal_wear,
            s.transducer_drift,
            s.transducer_open,
            s.transducer_intermittent,
        ]);
    }

    #[test]
    fn every_registered_failure_id_reaches_a_model_field() {
        let live = ids();
        let mut bound: Vec<u64> = Vec::new();
        for side in 0..2 {
            for p in 0..3 {
                push_surface(&live.aileron_ids[side][p], &mut bound);
            }
            for p in 0..2 {
                push_surface(&live.elevator_ids[side][p], &mut bound);
            }
            for i in 0..8 {
                push_surface(&live.spoiler_ids[side][i], &mut bound);
            }
        }
        for p in 0..2 {
            push_surface(&live.rudder_ids[p], &mut bound);
        }
        let t = live.ths_ids;
        bound.extend([t.motor_green_supply_loss, t.motor_yellow_supply_loss, t.no_back_failure, t.ballscrew_jam, t.transducer_drift, t.transducer_open]);
        bound.extend([live.rudder_trim_ids.motor_failure, live.rudder_trim_ids.jam]);
        for d in 0..3 {
            for side in 0..2 {
                let h = live.high_lift_ids[d][side];
                bound.extend([h.pcu_jam, h.pcu_runaway, h.pcu_supply_loss, h.limiter_bypass, h.inboard_shaft_break, h.outboard_shaft_break, h.wingtip_brake_fail]);
            }
        }
        bound.extend(live.ground_spoiler_ids);
        bound.sort_unstable();

        let mut r = Registry::default();
        super::super::registry::register(&mut r);
        let mut registered: Vec<u64> = r.failures.iter().map(|f| f.id).collect();
        registered.sort_unstable();
        assert_eq!(bound, registered, "every registered flight-control failure must reach a model field");
    }

    #[test]
    fn all_thirty_seven_components_publish_the_fault_variable_their_ecam_trigger_reads() {
        let live = ids();
        assert_eq!(live.fault_vars.len(), 37);
        let published = run(&mut ids(), &parked_truth(), &Faults::default(), 0.1);
        for name in [
            "FCTL_AIL_L1_FAULT",
            "FCTL_ELEV_R_OUTBD_FAULT",
            "FCTL_RUD_UPPER_FAULT",
            "FCTL_SPLR_R8_FAULT",
            "FCTL_THS_FAULT",
            "FCTL_RUDDER_TRIM_FAULT",
            "FCTL_FLAP_L_FAULT",
            "FCTL_SLAT_R_FAULT",
            "FCTL_DROOP_R_FAULT",
            "FCTL_GND_SPLR_LOGIC_FAULT",
        ] {
            assert!(published.contains_key(name), "missing {name}");
            assert_eq!(published[name], 0.0, "{name} must read healthy with nothing armed");
        }
    }

    #[test]
    fn a_jammed_aileron_pins_at_its_jam_angle_while_its_neighbours_follow_the_command() {
        // Registered effect: "servo authority falls toward zero and a
        // strong resistive spring pins the surface near its jam angle".
        let live = ids();
        let faults = Faults::from_pairs([(live.aileron_ids[0][2].jam, 1.0)]);
        let mut commands = SurfaceCommands::default();
        for side in 0..2 {
            commands.ailerons_deg[side] = [20.0; 3];
        }

        let mut jammed = ids();
        jammed.set_commands(commands);
        let published = run(&mut jammed, &parked_truth(), &faults, 3.0);

        // `ail_l1` is the left *outward* panel (index 2 in this module's
        // inward-first order).
        assert!(
            published["FCTL_AIL_L1_DEFLECTION_DEG"].abs() < 1.0,
            "the jammed panel must stay where it seized: {} deg",
            published["FCTL_AIL_L1_DEFLECTION_DEG"]
        );
        assert!(
            published["FCTL_AIL_L3_DEFLECTION_DEG"] > 18.0,
            "its unjammed neighbour must still follow the command: {} deg",
            published["FCTL_AIL_L3_DEFLECTION_DEG"]
        );
        assert_eq!(published["FCTL_AIL_L1_FAULT"], 1.0, "and the ECAM trigger variable must see it");
        assert_eq!(published["FCTL_AIL_L3_FAULT"], 0.0);
    }

    #[test]
    fn a_servo_hardover_drives_an_elevator_to_its_stop_with_nothing_commanded() {
        // Registered effect: "the surface drives toward one stop unless the
        // other actuators on the same surface, or the aerodynamic hinge
        // moment, have enough authority to override it".
        let live = ids();
        let faults = Faults::from_pairs([(live.elevator_ids[1][1].runaway, 1.0)]);
        let published = run(&mut ids(), &parked_truth(), &faults, 3.0);
        assert!(
            (published["FCTL_ELEV_R_OUTBD_DEFLECTION_DEG"] - AILERON_MAX_DEG).abs() < 0.5,
            "a full hardover must reach the stop: {} deg",
            published["FCTL_ELEV_R_OUTBD_DEFLECTION_DEG"]
        );
        assert!(
            published["FCTL_ELEV_L_OUTBD_DEFLECTION_DEG"].abs() < 0.5,
            "the other side must not move: {} deg",
            published["FCTL_ELEV_L_OUTBD_DEFLECTION_DEG"]
        );
    }

    #[test]
    fn losing_a_rudder_actuators_supply_lets_the_airload_carry_it_back_to_neutral() {
        // Registered effect: "at 1.0 the actuator can only be moved by the
        // other actuators on the surface or the airload".
        let live = ids();
        let faults = Faults::from_pairs([(live.rudder_ids[0].supply_loss, 1.0)]);
        let mut commands = SurfaceCommands::default();
        commands.rudders_deg = [25.0, 25.0];

        let truth = flying_truth();
        let mut healthy = ids();
        healthy.set_commands(commands);
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 4.0);

        let mut starved = ids();
        starved.set_commands(commands);
        let starved_out = run(&mut starved, &truth, &faults, 4.0);

        assert!(
            healthy_out["FCTL_RUD_UPPER_DEFLECTION_DEG"] > 15.0,
            "a healthy upper rudder holds most of its command at 24 kPa: {} deg",
            healthy_out["FCTL_RUD_UPPER_DEFLECTION_DEG"]
        );
        assert!(
            starved_out["FCTL_RUD_UPPER_DEFLECTION_DEG"] < 5.0,
            "with no supply the airload wins: {} deg",
            starved_out["FCTL_RUD_UPPER_DEFLECTION_DEG"]
        );
        assert!(
            starved_out["FCTL_RUD_LOWER_DEFLECTION_DEG"] > 15.0,
            "the lower rudder is a separate surface and must be unaffected: {} deg",
            starved_out["FCTL_RUD_LOWER_DEFLECTION_DEG"]
        );
    }

    #[test]
    fn a_disconnected_surface_free_floats_instead_of_holding_its_command() {
        // Registered effect: "zero actuator torque reaches the surface; it
        // free-floats, weathervaning under its own aerodynamic restoring
        // moment".
        let live = ids();
        let faults = Faults::from_pairs([(live.aileron_ids[1][0].disconnect, 1.0)]);
        let mut commands = SurfaceCommands::default();
        commands.ailerons_deg[1] = [20.0; 3];
        let mut area = ids();
        area.set_commands(commands);
        let published = run(&mut area, &flying_truth(), &faults, 4.0);
        assert!(
            published["FCTL_AIL_R3_DEFLECTION_DEG"].abs() < 3.0,
            "a sheared inward aileron weathervanes to neutral: {} deg",
            published["FCTL_AIL_R3_DEFLECTION_DEG"]
        );
        assert!(published["FCTL_AIL_R1_DEFLECTION_DEG"] > 10.0, "its neighbours still track");
    }

    #[test]
    fn a_ths_ballscrew_jam_freezes_the_trim_where_it_seized() {
        // Registered effect: "trim freezes at the jam angle regardless of
        // motor command".
        let live = ids();
        let faults = Faults::from_pairs([(live.ths_ids.ballscrew_jam, 1.0)]);
        let mut commands = SurfaceCommands::default();
        commands.ths_deg = 8.0;

        let truth = parked_truth();
        let mut healthy = ids();
        healthy.set_commands(commands);
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 20.0);

        let mut jammed = ids();
        jammed.set_commands(commands);
        let jammed_out = run(&mut jammed, &truth, &faults, 20.0);

        assert!(healthy_out["FCTL_THS_DEFLECTION_DEG"] > 7.0, "a healthy THS reaches its trim: {} deg", healthy_out["FCTL_THS_DEFLECTION_DEG"]);
        // A seizure is a very stiff spring, not an infinitely stiff one:
        // the two motors' combined 160 kN*m against `ths.rs`'s own
        // `jam_spring_nm_per_rad` (160 kN*m x 50) winds the screw about
        // 0.02 rad -- a degree or so -- and no further. That residual is
        // the model being honest about a finite structure, not the trim
        // running.
        assert!(
            jammed_out["FCTL_THS_DEFLECTION_DEG"] < 1.5,
            "a jammed ballscrew does not: {} deg",
            jammed_out["FCTL_THS_DEFLECTION_DEG"]
        );
        assert!(jammed_out["FCTL_THS_DEFLECTION_DEG"] < healthy_out["FCTL_THS_DEFLECTION_DEG"] * 0.25);
        assert_eq!(jammed_out["FCTL_THS_FAULT"], 1.0);
    }

    #[test]
    fn ground_spoiler_logic_that_fails_to_deploy_leaves_every_panel_down() {
        // Registered effect: "loss of lift dump and reduced wheel braking
        // effectiveness on landing".
        let live = ids();
        let faults = Faults::from_pairs([(live.ground_spoiler_ids[0], 1.0)]);
        let truth = parked_truth();

        let mut armed = ids();
        armed.set_ground_spoiler_controls(true, false);
        let deployed = run(&mut armed, &truth, &Faults::default(), 4.0);
        assert_eq!(deployed["FCTL_GND_SPLR_DEPLOYED"], 1.0);
        assert!(
            deployed["FCTL_SPLR_L1_DEFLECTION_DEG"] > 45.0,
            "armed and on the ground, the panels come out: {} deg",
            deployed["FCTL_SPLR_L1_DEFLECTION_DEG"]
        );

        let mut failed = ids();
        failed.set_ground_spoiler_controls(true, false);
        let stowed = run(&mut failed, &truth, &faults, 4.0);
        assert_eq!(stowed["FCTL_GND_SPLR_DEPLOYED"], 0.0);
        assert!(
            stowed["FCTL_SPLR_L1_DEFLECTION_DEG"] < 1.0,
            "with the logic failed they stay down: {} deg",
            stowed["FCTL_SPLR_L1_DEFLECTION_DEG"]
        );
        assert_eq!(stowed["FCTL_GND_SPLR_LOGIC_FAULT"], 1.0);
    }

    #[test]
    fn a_high_lift_shaft_break_costs_the_outboard_station_its_drive() {
        // Registered effect of `outboard_shaft_break`: "only the outboard
        // station loses drive; the inboard station still tracks".
        let live = ids();
        let faults = Faults::from_pairs([(live.high_lift_ids[0][0].outboard_shaft_break, 1.0)]);
        let mut commands = SurfaceCommands::default();
        commands.flap_deg = 20.0;
        let truth = parked_truth();

        let mut broken = ids();
        broken.set_commands(commands);
        let out = run(&mut broken, &truth, &faults, 12.0);
        assert!(out["FCTL_FLAP_L_INBOARD_DEG"] > 10.0, "the inboard station still drives: {} deg", out["FCTL_FLAP_L_INBOARD_DEG"]);
        assert!(
            out["FCTL_FLAP_L_OUTBOARD_DEG"] < out["FCTL_FLAP_L_INBOARD_DEG"] * 0.5,
            "the outboard station is left behind: {} vs {} deg",
            out["FCTL_FLAP_L_OUTBOARD_DEG"],
            out["FCTL_FLAP_L_INBOARD_DEG"]
        );
    }

    #[test]
    fn a_drifting_position_transducer_eventually_trips_the_dual_channel_monitor() {
        // Registered effect: "the computer's own position monitoring slowly
        // diverges from truth, eventually tripping a dual-channel
        // disagreement".
        let live = ids();
        let faults = Faults::from_pairs([(live.rudder_ids[1].transducer_drift, 1.0)]);
        let truth = parked_truth();
        let early = run(&mut ids(), &truth, &faults, 1.0);
        assert_eq!(early["FCTL_RUD_LOWER_POSITION_MONITOR_FAULT"], 0.0, "a fresh drift has not diverged yet");
        let late = run(&mut ids(), &truth, &faults, 60.0);
        assert_eq!(late["FCTL_RUD_LOWER_POSITION_MONITOR_FAULT"], 1.0, "60 s of drift must trip the monitor");
        assert_eq!(late["FCTL_RUD_UPPER_POSITION_MONITOR_FAULT"], 0.0);
    }

    #[test]
    fn the_surface_angle_getter_matches_what_is_published() {
        let live = ids();
        let faults = Faults::from_pairs([(live.elevator_ids[0][0].runaway, 1.0)]);
        let mut area = ids();
        let published = run(&mut area, &parked_truth(), &faults, 2.0);
        let angles = area.surface_angles();
        assert_eq!(angles.elevators_deg[0][0], published["FCTL_ELEV_L_INBD_DEFLECTION_DEG"]);
        assert_eq!(angles.rudders_deg[0], published["FCTL_RUD_UPPER_DEFLECTION_DEG"]);
        assert_eq!(angles.spoilers_deg[1][7], published["FCTL_SPLR_R8_DEFLECTION_DEG"]);
        assert_eq!(angles.ths_deg, published["FCTL_THS_DEFLECTION_DEG"]);
        // Every published angle stays inside the surface's own travel, so
        // `integration::flight_control_surfaces`' conversions never have to
        // clamp a physically impossible position.
        for side in 0..2 {
            for p in 0..3 {
                assert!((AILERON_MIN_DEG..=AILERON_MAX_DEG).contains(&angles.ailerons_deg[side][p]));
            }
            for i in 0..8 {
                assert!((0.0..=SPOILER_MAX_DEG).contains(&angles.spoilers_deg[side][i]));
            }
        }
        assert!(angles.rudders_deg.iter().all(|d| d.abs() <= RUDDER_LIMIT_DEG + 1e-9));
    }

    #[test]
    fn nothing_is_nan_on_the_very_first_frame_of_a_cold_aircraft() {
        let mut live = ids();
        live.tick(&Truth::default(), &Faults::default());
        let mut ok = true;
        live.publish(&mut |name, value| {
            if !value.is_finite() {
                println!("non-finite {name}");
                ok = false;
            }
        });
        assert!(ok);
    }

    #[test]
    fn the_area_plugs_into_deep_through_the_live_contract() {
        use crate::deep::live::Deep;
        let mut deep = Deep::new().with_area(super::live_system());
        assert_eq!(deep.area_names(), vec!["flight_controls"]);
        let mut published = BTreeMap::new();
        deep.tick(parked_truth(), &Faults::default(), &mut |name, value| {
            published.insert(name.to_string(), value);
        });
        assert!(published.contains_key("FCTL_AIL_L1_FAULT"));
        assert!(published.contains_key("FCTL_THS_DEFLECTION_DEG"));
    }
}

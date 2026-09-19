//! Turns a `deep::flight_controls` surface's modelled physical position
//! (jammed at an angle, runaway, free-floating, blown back) into what
//! X-Plane actually flies, by overriding the exact variable FlyByWire's own
//! actuator model writes -- so the unmodified `flight_controls.rs`/
//! `handling.rs` pipeline (span-blending, degree conversion, dataref
//! writes) drives X-Plane with the physical position with no further code
//! change, and anything else in the plugin that reads the same variable
//! (ECAM/study diagrams, and -- see the "feedback" section below -- one
//! tick later, any of FlyByWire's own ported systems that read a peer
//! actuator's published position rather than their own private state) sees
//! it too.
//!
//! ## The existing pipeline this overrides (read in full, not duplicated)
//! `flight_controls.rs`'s own module doc: FlyByWire's hydraulic actuators
//! (`a380_systems`) move each surface and **write its position, normalised
//! 0..1**, into named `Var`s (`HYD_AIL_{side}_{part}_DEFLECTION` etc,
//! `flight_controls.rs:258-269`); `FlightControls::update` then reads those
//! same `Var`s, converts to degrees with `aileron_or_elevator_down_deg`/
//! `rudder_right_deg`/`spoiler_up_deg`, span-blends FlyByWire's panels onto
//! X-Plane's coarser surface sets, and writes X-Plane's own
//! `sim/flightmodel2/wing/*_deg` datarefs. `handling.rs` does the
//! equivalent for flap/slat from `LEFT_FLAPS_ANGLE`/`RIGHT_FLAPS_ANGLE`/
//! `LEFT_SLATS_ANGLE`/`RIGHT_SLATS_ANGLE` (already in degrees, one ratio
//! applied uniformly across every X-Plane flap element -- see `PROGRESS.md`
//! for the asymmetry this loses).
//!
//! This module writes the **same named `Var`s**, computed from
//! `deep::flight_controls::surface::ControlSurface::angle_rad`/
//! `deep::flight_controls::high_lift::HighLiftSystem`'s station angles
//! instead of from `a380_systems`' own (undamaged) actuator model, using
//! the exact inverse of `flight_controls.rs`'s own documented request
//! formulas (round-trip-tested against its real, unmodified public
//! functions below, so this can never silently drift from that file).
//!
//! ## Body-angle convention (matches `deep::flight_controls::surface`
//! exactly, both cite the same FlyByWire body geometry)
//! - Aileron/elevator: degrees, positive trailing edge up, -20..+30
//!   (`a380_aileron_body`/`a380_elevator_body`, cited in both
//!   `flight_controls.rs` and `deep::flight_controls::surface`'s own test
//!   `SurfaceLimits`).
//! - Rudder: degrees, -30..+30, FlyByWire's own body sign (positive as its
//!   `order` term in `flight_controls.rs`'s cited formula, *not* X-Plane's
//!   positive-right convention -- the conversion below applies that sign
//!   flip, exactly reproducing `flight_controls.rs`'s documented
//!   `request = 0.5 - order/60`).
//! - Spoiler: degrees up, 0..50, no sign flip (FlyByWire's own spoiler body
//!   angle already is "degrees up", `flight_controls.rs`'s cited
//!   `request = surface degrees / 50`).
//! - THS: degrees, positive nose up, -2..+10, written unconverted (the
//!   `Var` is already in degrees, matching `flight_controls.rs`'s own
//!   `ths_deg` field).
//! - Flap/slat: degrees (whatever `deep::flight_controls::high_lift`
//!   calibrates a fully-extended station to, once it publishes a real A380
//!   travel figure -- see `PROGRESS.md`), one representative value per
//!   side (the inboard/outboard split X-Plane's own flap surfaces cannot
//!   express, per `handling.rs`'s single per-side `Var`).
//!
//! ## Feedback into FlyByWire's own PRIM/SEC
//! Overriding the shared `Var` guarantees X-Plane and every other plugin
//! reader of it see the real position **this tick** (placed right before
//! `FlightControls::update`/`Handling`'s own publish step, both already
//! documented as running "after FlyByWire's systems have moved the
//! actuators this tick" -- see `docs/deep/integration.md`'s exact lib.rs
//! ordering patch). Whether FlyByWire's own control-law computer inside
//! `a380_systems` (compiled into this plugin, not a separate WASM sandbox)
//! itself reacts to the override depends on whether its SFCC/PRIM position
//! monitors read a peer actuator's *published* `Var` (as opposed to their
//! own already-computed internal state) as their feedback -- if they do,
//! `Simulation::tick`'s own read-then-update-then-write cycle means they
//! see this override starting the **next** tick, the same one-tick lag
//! `extra_backend_fbw.rs` already accepts for the reverser force path
//! ("the same one tick of lag FlyByWire's own `ExecuteOn::PreTick` read
//! has in MSFS"). This is not claimed to be instantaneous same-tick
//! closure into `a380_systems`' own internals, which would need a change
//! to that read-only reference tree; see `PROGRESS.md` for what a fuller
//! closure would require.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}
impl Side {
    fn name(self) -> &'static str {
        match self {
            Side::Left => "LEFT",
            Side::Right => "RIGHT",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AileronPanel {
    Inward,
    Middle,
    Outward,
}
impl AileronPanel {
    fn name(self) -> &'static str {
        match self {
            AileronPanel::Inward => "INWARD",
            AileronPanel::Middle => "MIDDLE",
            AileronPanel::Outward => "OUTWARD",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ElevatorPanel {
    Inward,
    Outward,
}
impl ElevatorPanel {
    fn name(self) -> &'static str {
        match self {
            ElevatorPanel::Inward => "INWARD",
            ElevatorPanel::Outward => "OUTWARD",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RudderPanel {
    Upper,
    Lower,
}
impl RudderPanel {
    fn name(self) -> &'static str {
        match self {
            RudderPanel::Upper => "UPPER",
            RudderPanel::Lower => "LOWER",
        }
    }
}

// ---------------------------------------------------------------------------
// Var names, reproducing `flight_controls.rs`'s own construction exactly
// (`flight_controls.rs:260-269`) so `vars.get(...)` resolves to the same
// slot FlyByWire's actuators and `FlightControls::read` already use.
// ---------------------------------------------------------------------------

pub fn aileron_var_name(side: Side, panel: AileronPanel) -> String {
    format!("HYD_AIL_{}_{}_DEFLECTION", side.name(), panel.name())
}
pub fn elevator_var_name(side: Side, panel: ElevatorPanel) -> String {
    format!("HYD_ELEV_{}_{}_DEFLECTION", side.name(), panel.name())
}
pub fn rudder_var_name(panel: RudderPanel) -> String {
    format!("HYD_{}_RUD_DEFLECTION", panel.name())
}
pub fn spoiler_var_name(side: Side, k: u8) -> String {
    format!("HYD_SPOILER_{}_{}_DEFLECTION", k, side.name())
}
pub const THS_VAR_NAME: &str = "HYD_FINAL_THS_DEFLECTION";
pub fn flap_var_name(side: Side) -> String {
    format!("{}_FLAPS_ANGLE", side.name())
}
pub fn slat_var_name(side: Side) -> String {
    format!("{}_SLATS_ANGLE", side.name())
}

// ---------------------------------------------------------------------------
// Conversions: the exact inverse of `flight_controls.rs`'s documented
// request formulas, so the round trip through its own real, unmodified
// public functions reproduces the input body angle exactly (see tests).
// ---------------------------------------------------------------------------

/// Aileron/elevator: `flight_controls.rs` documents `request = 20/50 -
/// order/50` where `order` is the body angle in degrees; this is that
/// formula solved for the normalised request, clamped to the actuator's
/// physical 0..1 range.
pub fn normalized_aileron_or_elevator(body_deg: f64) -> f64 {
    ((20.0 - body_deg) / 50.0).clamp(0.0, 1.0)
}

/// Rudder: `flight_controls.rs` documents `request = 0.5 - order/60`.
pub fn normalized_rudder(body_deg: f64) -> f64 {
    ((30.0 - body_deg) / 60.0).clamp(0.0, 1.0)
}

/// Spoiler: `flight_controls.rs` documents `request = surface degrees / 50`.
pub fn normalized_spoiler(body_deg: f64) -> f64 {
    (body_deg / 50.0).clamp(0.0, 1.0)
}

/// Every A380 flight-control surface's physical position for one tick, in
/// the body-angle convention documented above -- populated by whichever
/// wrapper owns live `ControlSurface`/`HighLiftSystem` instances (this
/// module takes plain data so it never depends on that aggregate's
/// as-yet-unbuilt shape; see `PROGRESS.md`).
///
/// Every field is `Option`: `None` means "no live deep-model instance for
/// this surface yet, leave FlyByWire's own (undamaged) actuator output
/// alone". This matters correctness-wise, not just for convenience --
/// `deep::flight_controls` will not model all 37 A380 surfaces on day one,
/// and unconditionally overriding an unmodelled surface with e.g. `0.0`
/// would fight FlyByWire's real command on every surface this crate has not
/// gotten to yet, which is worse than not overriding it at all.
#[derive(Clone, Copy, Debug, Default)]
pub struct PhysicalSurfaces {
    /// `[side][inward, middle, outward]`, degrees, positive TE up.
    pub ailerons_deg: [[Option<f64>; 3]; 2],
    /// `[side][inward, outward]`, degrees, positive TE up.
    pub elevators_deg: [[Option<f64>; 2]; 2],
    /// `[upper, lower]`, degrees, FlyByWire body sign (see module doc).
    pub rudders_deg: [Option<f64>; 2],
    /// `[side][spoiler 1..=8]`, degrees up, 0..50.
    pub spoilers_deg: [[Option<f64>; 8]; 2],
    /// Degrees, positive nose up, written unconverted.
    pub ths_deg: Option<f64>,
    /// `[side]`, degrees; also `None` while `deep::flight_controls::
    /// high_lift` has no cited real travel range yet (see `PROGRESS.md`).
    pub flap_deg: [Option<f64>; 2],
    pub slat_deg: [Option<f64>; 2],
}

/// Caches every surface's `VariableIdentifier` at construction (like
/// `flight_controls.rs::FlightControls`'s own `Ids`), then overrides all of
/// them from a [`PhysicalSurfaces`] snapshot every tick.
pub struct SurfaceOverrideWriter {
    ailerons: [[VariableIdentifier; 3]; 2],
    elevators: [[VariableIdentifier; 2]; 2],
    rudders: [VariableIdentifier; 2],
    spoilers: [[VariableIdentifier; 8]; 2],
    ths: VariableIdentifier,
    flaps: [VariableIdentifier; 2],
    slats: [VariableIdentifier; 2],
}

const SIDES: [Side; 2] = [Side::Left, Side::Right];

impl SurfaceOverrideWriter {
    pub fn new<V: VariableRegistry>(vars: &mut V) -> Self {
        let ailerons = SIDES.map(|s| [AileronPanel::Inward, AileronPanel::Middle, AileronPanel::Outward].map(|p| vars.get(aileron_var_name(s, p))));
        let elevators = SIDES.map(|s| [ElevatorPanel::Inward, ElevatorPanel::Outward].map(|p| vars.get(elevator_var_name(s, p))));
        let rudders = [RudderPanel::Upper, RudderPanel::Lower].map(|p| vars.get(rudder_var_name(p)));
        let spoilers = SIDES.map(|s| std::array::from_fn(|i| vars.get(spoiler_var_name(s, i as u8 + 1))));
        let ths = vars.get(THS_VAR_NAME.to_owned());
        let flaps = SIDES.map(|s| vars.get(flap_var_name(s)));
        let slats = SIDES.map(|s| vars.get(slat_var_name(s)));
        Self { ailerons, elevators, rudders, spoilers, ths, flaps, slats }
    }

    /// Overrides only the surfaces `s` gives a value for (see
    /// [`PhysicalSurfaces`]'s own doc for why `None` must mean "leave
    /// FlyByWire's value alone", not "override with zero"). Must run after
    /// FlyByWire's own systems have written this tick's (undamaged)
    /// actuator positions and before `flight_controls.rs`/`handling.rs`
    /// publish to X-Plane (`docs/deep/integration.md`'s lib.rs ordering
    /// patch).
    pub fn apply<V: SimulatorReaderWriter>(&self, vars: &mut V, s: &PhysicalSurfaces) {
        for side in 0..2 {
            for i in 0..3 {
                if let Some(deg) = s.ailerons_deg[side][i] {
                    vars.write(&self.ailerons[side][i], normalized_aileron_or_elevator(deg));
                }
            }
            for i in 0..2 {
                if let Some(deg) = s.elevators_deg[side][i] {
                    vars.write(&self.elevators[side][i], normalized_aileron_or_elevator(deg));
                }
            }
            for i in 0..8 {
                if let Some(deg) = s.spoilers_deg[side][i] {
                    vars.write(&self.spoilers[side][i], normalized_spoiler(deg));
                }
            }
            if let Some(deg) = s.flap_deg[side] {
                vars.write(&self.flaps[side], deg);
            }
            if let Some(deg) = s.slat_deg[side] {
                vars.write(&self.slats[side], deg);
            }
        }
        for i in 0..2 {
            if let Some(deg) = s.rudders_deg[i] {
                vars.write(&self.rudders[i], normalized_rudder(deg));
            }
        }
        if let Some(deg) = s.ths_deg {
            vars.write(&self.ths, deg);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight_controls::{aileron_or_elevator_down_deg, rudder_right_deg, spoiler_up_deg};

    #[test]
    fn var_names_match_flight_controls_rs_construction() {
        assert_eq!(aileron_var_name(Side::Left, AileronPanel::Inward), "HYD_AIL_LEFT_INWARD_DEFLECTION");
        assert_eq!(elevator_var_name(Side::Right, ElevatorPanel::Outward), "HYD_ELEV_RIGHT_OUTWARD_DEFLECTION");
        assert_eq!(rudder_var_name(RudderPanel::Upper), "HYD_UPPER_RUD_DEFLECTION");
        assert_eq!(spoiler_var_name(Side::Right, 8), "HYD_SPOILER_8_RIGHT_DEFLECTION");
        assert_eq!(flap_var_name(Side::Left), "LEFT_FLAPS_ANGLE");
        assert_eq!(slat_var_name(Side::Right), "RIGHT_SLATS_ANGLE");
    }

    #[test]
    fn aileron_and_elevator_round_trip_through_flight_controls_rs_real_conversion() {
        for body_deg in [-20.0, -12.5, 0.0, 7.0, 20.0, 30.0] {
            let n = normalized_aileron_or_elevator(body_deg);
            let back = aileron_or_elevator_down_deg(n);
            assert!((back - body_deg).abs() < 1e-9, "{body_deg} -> {n} -> {back}");
        }
    }

    #[test]
    fn rudder_round_trips_with_flight_controls_rs_sign_convention() {
        // flight_controls.rs: `rudder_right_deg(n) = 60n - 30`; its own
        // `request = 0.5 - order/60` composed with that gives
        // `rudder_right_deg(request(order)) = -order` (both files' own
        // documented formulas, not an assumption this test introduces).
        for body_deg in [-30.0, -10.0, 0.0, 15.0, 30.0] {
            let n = normalized_rudder(body_deg);
            let xp_right_deg = rudder_right_deg(n);
            assert!((xp_right_deg - (-body_deg)).abs() < 1e-9, "{body_deg} -> {n} -> {xp_right_deg}");
        }
    }

    #[test]
    fn spoiler_round_trips_through_flight_controls_rs_real_conversion() {
        for body_deg in [0.0, 12.5, 25.0, 50.0] {
            let n = normalized_spoiler(body_deg);
            let back = spoiler_up_deg(n);
            assert!((back - body_deg).abs() < 1e-9, "{body_deg} -> {n} -> {back}");
        }
    }

    #[test]
    fn conversions_clamp_rather_than_go_out_of_the_actuators_0_1_range() {
        assert_eq!(normalized_aileron_or_elevator(999.0), 0.0);
        assert_eq!(normalized_aileron_or_elevator(-999.0), 1.0);
        assert_eq!(normalized_rudder(999.0), 0.0);
        assert_eq!(normalized_spoiler(-5.0), 0.0);
        assert_eq!(normalized_spoiler(500.0), 1.0);
    }

    #[test]
    fn a_jammed_aileron_overrides_the_shared_var_with_its_physical_angle() {
        // End-to-end through the crate's own `aspects::test_vars::TestVars`
        // (the same `VariableRegistry`/`SimulatorReaderWriter` test double
        // `physics/xp_effects.rs`'s own tests use): construct the writer,
        // pretend FlyByWire's systems just wrote a healthy commanded
        // position, then confirm `apply` overrides it with the physically
        // jammed one.
        // `vars.get(name)` is used (not `TestVars`'s own `set`/`value`
        // convenience helpers, which bypass `VariableRegistry` and would
        // resolve to a different slot than the writer's -- see
        // `physics/xp_effects.rs`'s own test comment on exactly this trap),
        // so this interoperates with `flight_controls.rs`'s real `get` the
        // same way production code would, regardless of prefixing scheme.
        use crate::aspects::test_vars::TestVars;
        let mut vars = TestVars::default();
        let writer = SurfaceOverrideWriter::new(&mut vars);
        let id = vars.get(aileron_var_name(Side::Left, AileronPanel::Inward));
        vars.write(&id, normalized_aileron_or_elevator(20.0)); // FBW's healthy command
        let jammed_body_deg = -14.0; // physically stuck well off that command
        let mut s = PhysicalSurfaces::default();
        s.ailerons_deg[0][0] = Some(jammed_body_deg);
        writer.apply(&mut vars, &s);
        let n = vars.read(&id);
        assert!((aileron_or_elevator_down_deg(n) - jammed_body_deg).abs() < 1e-9);
        assert_ne!(n, normalized_aileron_or_elevator(20.0), "must differ from the healthy commanded position it replaced");
    }

    #[test]
    fn ths_and_flap_slat_write_through_unconverted_degrees() {
        use crate::aspects::test_vars::TestVars;
        let mut vars = TestVars::default();
        let writer = SurfaceOverrideWriter::new(&mut vars);
        let ths_id = vars.get(THS_VAR_NAME.to_owned());
        let left_flap_id = vars.get(flap_var_name(Side::Left));
        let right_flap_id = vars.get(flap_var_name(Side::Right));
        let right_slat_id = vars.get(slat_var_name(Side::Right));
        let mut s = PhysicalSurfaces { ths_deg: Some(6.5), ..Default::default() };
        s.flap_deg[0] = Some(12.0);
        s.slat_deg[1] = Some(20.0);
        writer.apply(&mut vars, &s);
        assert_eq!(vars.read(&ths_id), 6.5);
        assert_eq!(vars.read(&left_flap_id), 12.0);
        assert_eq!(vars.read(&right_slat_id), 20.0);
        // A side with no override (`None`) must not be written at all.
        assert_eq!(vars.read(&right_flap_id), 0.0);
    }

    #[test]
    fn a_surface_with_no_live_deep_model_instance_keeps_flybywires_own_command() {
        // The correctness point `PhysicalSurfaces`'s own doc makes: `None`
        // must leave whatever FlyByWire already wrote alone, never force it
        // to a fabricated default (e.g. 0.0 body degrees), since most
        // surfaces will have no live `ControlSurface` instance for a long
        // time yet.
        use crate::aspects::test_vars::TestVars;
        let mut vars = TestVars::default();
        let writer = SurfaceOverrideWriter::new(&mut vars);
        let rudder_id = vars.get(rudder_var_name(RudderPanel::Upper));
        let flybywires_healthy_command = normalized_rudder(5.0);
        vars.write(&rudder_id, flybywires_healthy_command);
        writer.apply(&mut vars, &PhysicalSurfaces::default()); // every field None
        assert_eq!(vars.read(&rudder_id), flybywires_healthy_command, "an unmodelled surface must be left exactly as FlyByWire wrote it");
    }
}

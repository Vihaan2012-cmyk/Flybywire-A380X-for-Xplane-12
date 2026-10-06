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
//!
//! ## A distinct override namespace, not FlyByWire's own `Var`s (2026-09-25
//! revision)
//!
//! An earlier version of this file wrote straight into the shared `Var`s
//! named above, unconditionally, once `deep::flight_controls` had a live
//! instance for a surface -- which is what the "Feedback into FlyByWire's
//! own PRIM/SEC" section above still describes as the mechanism's whole
//! point. Review (W125, `E:/fbw-debug/fixes/W124.md`) caught the flaw:
//! `deep.tick()` runs strictly after FlyByWire's own `simulation.tick()`
//! has already written a healthy commanded position into the same `Var`,
//! so an *unconditional* override replaces FlyByWire's own validated
//! actuator output with this crate's independent physics model's output on
//! every healthy frame too, not only a faulted one -- two simulations
//! racing to own one `Var`, agreeing only by luck.
//!
//! [`SurfaceOverrideWriter`] therefore targets a **separate,
//! `DEEP_`-prefixed namespace** ([`aileron_deflection_var_name`] and
//! siblings, e.g. `DEEP_HYD_AIL_LEFT_INWARD_OVERRIDE_DEFLECTION`) and
//! writes an explicit **active flag** beside every value, every tick
//! ([`aileron_active_var_name`] and siblings), true only while
//! `SurfaceIds`/`ThsIds::position_fault_active` says a fault that actually
//! moves that surface off command is armed (`deep::flight_controls::live`'s
//! own doc for the full list -- jam, runaway, supply loss, disconnect,
//! flutter-damper loss, valve leakage, piston-seal wear; deliberately not
//! the transducer-* faults, which corrupt only the monitoring channel).
//! FlyByWire's own `HYD_*`/`HYD_FINAL_THS_DEFLECTION` `Var`s are never
//! written by this crate. The consumer (`flight_controls.rs::FlightControls
//! ::read`) is the one taught to prefer the override, and only while the
//! paired active flag is set -- see that file's own doc for the read side
//! of this same mechanism. The feedback-into-PRIM/SEC property the section
//! above describes still holds while a fault is active (that is exactly
//! when the override is live); it simply no longer holds *unconditionally*,
//! which was the bug.

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
// Var names. NOT FlyByWire's own `HYD_*_DEFLECTION` names (see this file's
// module doc, "A distinct override namespace") -- these are a new,
// `DEEP_`-prefixed pair per surface (value + active flag), read only by
// `flight_controls.rs::FlightControls::read`, which builds the identical
// strings from its own side/part loops (its own doc comment on `Ids`, kept
// in step with this file's naming on purpose: reproducing a handful of
// `format!` patterns twice is cheaper and safer here than giving
// `flight_controls.rs` -- otherwise entirely independent of `deep::` -- a
// dependency on this module just to share four functions).
// ---------------------------------------------------------------------------

pub fn aileron_deflection_var_name(side: Side, panel: AileronPanel) -> String {
    format!("DEEP_HYD_AIL_{}_{}_OVERRIDE_DEFLECTION", side.name(), panel.name())
}
pub fn aileron_active_var_name(side: Side, panel: AileronPanel) -> String {
    format!("DEEP_HYD_AIL_{}_{}_OVERRIDE_ACTIVE", side.name(), panel.name())
}
pub fn elevator_deflection_var_name(side: Side, panel: ElevatorPanel) -> String {
    format!("DEEP_HYD_ELEV_{}_{}_OVERRIDE_DEFLECTION", side.name(), panel.name())
}
pub fn elevator_active_var_name(side: Side, panel: ElevatorPanel) -> String {
    format!("DEEP_HYD_ELEV_{}_{}_OVERRIDE_ACTIVE", side.name(), panel.name())
}
pub fn rudder_deflection_var_name(panel: RudderPanel) -> String {
    format!("DEEP_HYD_{}_RUD_OVERRIDE_DEFLECTION", panel.name())
}
pub fn rudder_active_var_name(panel: RudderPanel) -> String {
    format!("DEEP_HYD_{}_RUD_OVERRIDE_ACTIVE", panel.name())
}
pub fn spoiler_deflection_var_name(side: Side, k: u8) -> String {
    format!("DEEP_HYD_SPOILER_{}_{}_OVERRIDE_DEFLECTION", k, side.name())
}
pub fn spoiler_active_var_name(side: Side, k: u8) -> String {
    format!("DEEP_HYD_SPOILER_{}_{}_OVERRIDE_ACTIVE", k, side.name())
}
pub const THS_DEFLECTION_VAR_NAME: &str = "DEEP_HYD_FINAL_THS_OVERRIDE_DEFLECTION";
pub const THS_ACTIVE_VAR_NAME: &str = "DEEP_HYD_FINAL_THS_OVERRIDE_ACTIVE";
/// Unchanged, still FlyByWire's own name: `PhysicalSurfaces::flap_deg` is
/// always `None` today (uncalibrated, see this file's own doc), so this is
/// never written by [`SurfaceOverrideWriter::apply`] either way. Move this
/// to the same `DEEP_` scheme as the surfaces above when that stops being
/// true, for the same reason W125 flagged for the others.
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
/// `flight_controls.rs::FlightControls`'s own `Ids`) -- both the override
/// value and its paired active flag (this file's module doc, "A distinct
/// override namespace") -- then overrides all of them from a
/// [`PhysicalSurfaces`] snapshot every tick.
pub struct SurfaceOverrideWriter {
    ailerons: [[VariableIdentifier; 3]; 2],
    ailerons_active: [[VariableIdentifier; 3]; 2],
    elevators: [[VariableIdentifier; 2]; 2],
    elevators_active: [[VariableIdentifier; 2]; 2],
    rudders: [VariableIdentifier; 2],
    rudders_active: [VariableIdentifier; 2],
    spoilers: [[VariableIdentifier; 8]; 2],
    spoilers_active: [[VariableIdentifier; 8]; 2],
    ths: VariableIdentifier,
    ths_active: VariableIdentifier,
    flaps: [VariableIdentifier; 2],
    slats: [VariableIdentifier; 2],
}

const SIDES: [Side; 2] = [Side::Left, Side::Right];

impl SurfaceOverrideWriter {
    pub fn new<V: VariableRegistry>(vars: &mut V) -> Self {
        let ailerons = SIDES.map(|s| [AileronPanel::Inward, AileronPanel::Middle, AileronPanel::Outward].map(|p| vars.get(aileron_deflection_var_name(s, p))));
        let ailerons_active = SIDES.map(|s| [AileronPanel::Inward, AileronPanel::Middle, AileronPanel::Outward].map(|p| vars.get(aileron_active_var_name(s, p))));
        let elevators = SIDES.map(|s| [ElevatorPanel::Inward, ElevatorPanel::Outward].map(|p| vars.get(elevator_deflection_var_name(s, p))));
        let elevators_active = SIDES.map(|s| [ElevatorPanel::Inward, ElevatorPanel::Outward].map(|p| vars.get(elevator_active_var_name(s, p))));
        let rudders = [RudderPanel::Upper, RudderPanel::Lower].map(|p| vars.get(rudder_deflection_var_name(p)));
        let rudders_active = [RudderPanel::Upper, RudderPanel::Lower].map(|p| vars.get(rudder_active_var_name(p)));
        let spoilers = SIDES.map(|s| std::array::from_fn(|i| vars.get(spoiler_deflection_var_name(s, i as u8 + 1))));
        let spoilers_active = SIDES.map(|s| std::array::from_fn(|i| vars.get(spoiler_active_var_name(s, i as u8 + 1))));
        let ths = vars.get(THS_DEFLECTION_VAR_NAME.to_owned());
        let ths_active = vars.get(THS_ACTIVE_VAR_NAME.to_owned());
        let flaps = SIDES.map(|s| vars.get(flap_var_name(s)));
        let slats = SIDES.map(|s| vars.get(slat_var_name(s)));
        Self { ailerons, ailerons_active, elevators, elevators_active, rudders, rudders_active, spoilers, spoilers_active, ths, ths_active, flaps, slats }
    }

    /// Writes every surface's active flag unconditionally (`1.0`/`0.0`, so a
    /// fault that just cleared reads `0.0` this same tick rather than
    /// holding a stale `1.0`), and the override value itself only while `s`
    /// gives one (see [`PhysicalSurfaces`]'s own doc for why `None` must
    /// mean "no override", not "override with zero"; harmless either way,
    /// since the consumer only reads the value while the active flag beside
    /// it is `1.0`). Must run after FlyByWire's own systems have written
    /// this tick's (undamaged) actuator positions -- so this tick's fault
    /// state is what decides the flag -- and before
    /// `flight_controls.rs::FlightControls::update` reads both
    /// (`docs/deep/integration.md`'s lib.rs ordering patch;
    /// `flight_controls.rs`'s own doc for the read side).
    pub fn apply<V: SimulatorReaderWriter>(&self, vars: &mut V, s: &PhysicalSurfaces) {
        let flag = |active: bool| if active { 1.0 } else { 0.0 };
        for side in 0..2 {
            for i in 0..3 {
                vars.write(&self.ailerons_active[side][i], flag(s.ailerons_deg[side][i].is_some()));
                if let Some(deg) = s.ailerons_deg[side][i] {
                    vars.write(&self.ailerons[side][i], normalized_aileron_or_elevator(deg));
                }
            }
            for i in 0..2 {
                vars.write(&self.elevators_active[side][i], flag(s.elevators_deg[side][i].is_some()));
                if let Some(deg) = s.elevators_deg[side][i] {
                    vars.write(&self.elevators[side][i], normalized_aileron_or_elevator(deg));
                }
            }
            for i in 0..8 {
                vars.write(&self.spoilers_active[side][i], flag(s.spoilers_deg[side][i].is_some()));
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
            vars.write(&self.rudders_active[i], flag(s.rudders_deg[i].is_some()));
            if let Some(deg) = s.rudders_deg[i] {
                vars.write(&self.rudders[i], normalized_rudder(deg));
            }
        }
        vars.write(&self.ths_active, flag(s.ths_deg.is_some()));
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
    fn var_names_are_a_distinct_deep_prefixed_namespace_with_active_flags() {
        assert_eq!(aileron_deflection_var_name(Side::Left, AileronPanel::Inward), "DEEP_HYD_AIL_LEFT_INWARD_OVERRIDE_DEFLECTION");
        assert_eq!(aileron_active_var_name(Side::Left, AileronPanel::Inward), "DEEP_HYD_AIL_LEFT_INWARD_OVERRIDE_ACTIVE");
        assert_eq!(elevator_deflection_var_name(Side::Right, ElevatorPanel::Outward), "DEEP_HYD_ELEV_RIGHT_OUTWARD_OVERRIDE_DEFLECTION");
        assert_eq!(rudder_deflection_var_name(RudderPanel::Upper), "DEEP_HYD_UPPER_RUD_OVERRIDE_DEFLECTION");
        assert_eq!(rudder_active_var_name(RudderPanel::Upper), "DEEP_HYD_UPPER_RUD_OVERRIDE_ACTIVE");
        assert_eq!(spoiler_deflection_var_name(Side::Right, 8), "DEEP_HYD_SPOILER_8_RIGHT_OVERRIDE_DEFLECTION");
        assert_eq!(THS_DEFLECTION_VAR_NAME, "DEEP_HYD_FINAL_THS_OVERRIDE_DEFLECTION");
        assert_eq!(THS_ACTIVE_VAR_NAME, "DEEP_HYD_FINAL_THS_OVERRIDE_ACTIVE");
        // None of these collide with FlyByWire's own names, on purpose
        // (this file's module doc, "A distinct override namespace").
        assert_ne!(aileron_deflection_var_name(Side::Left, AileronPanel::Inward), "HYD_AIL_LEFT_INWARD_DEFLECTION");
        // Flap/slat are unchanged (still `None` always, see their own doc).
        assert_eq!(flap_var_name(Side::Left), "LEFT_FLAPS_ANGLE");
        assert_eq!(slat_var_name(Side::Right), "RIGHT_SLATS_ANGLE");
    }

    #[test]
    fn aileron_and_elevator_round_trip_through_flight_controls_rs_real_conversion() {
        // `flight_controls.rs`'s own conversion is `down_deg = 20 - 50*n`
        // over the actuator's physical `n` in 0..1, so the representable
        // body-angle range is exactly 20 deg trailing edge down (n = 0) to
        // -30 deg, i.e. 30 deg up (n = 1). Those endpoints are the surface's
        // travel stops, so the round trip is asserted across that whole
        // range and at both ends of it.
        for body_deg in [-30.0, -20.0, -12.5, 0.0, 7.0, 20.0] {
            let n = normalized_aileron_or_elevator(body_deg);
            let back = aileron_or_elevator_down_deg(n);
            assert!((back - body_deg).abs() < 1e-9, "{body_deg} -> {n} -> {back}");
        }
        // Outside that range there is no normalised position to round trip
        // to: the conversion clamps to the travel stop, which is the right
        // answer (a surface cannot be driven past its stop) and is what the
        // deep model's own `SurfaceLimits` would have produced anyway.
        assert_eq!(normalized_aileron_or_elevator(30.0), 0.0);
        assert_eq!(aileron_or_elevator_down_deg(normalized_aileron_or_elevator(30.0)), 20.0);
        assert_eq!(normalized_aileron_or_elevator(-45.0), 1.0);
        assert_eq!(aileron_or_elevator_down_deg(normalized_aileron_or_elevator(-45.0)), -30.0);
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
    fn a_jammed_aileron_sets_the_active_flag_and_the_override_value_to_its_physical_angle() {
        // End-to-end through the crate's own `aspects::test_vars::TestVars`
        // (the same `VariableRegistry`/`SimulatorReaderWriter` test double
        // `physics/xp_effects.rs`'s own tests use). Unlike the pre-W125
        // version of this test, this never touches FlyByWire's own `HYD_*`
        // var at all -- see this file's module doc, "A distinct override
        // namespace" -- so there is nothing here for it to "overwrite";
        // what is asserted is the new namespace's own two Vars.
        use crate::aspects::test_vars::TestVars;
        let mut vars = TestVars::default();
        let writer = SurfaceOverrideWriter::new(&mut vars);
        let active_id = vars.get(aileron_active_var_name(Side::Left, AileronPanel::Inward));
        let value_id = vars.get(aileron_deflection_var_name(Side::Left, AileronPanel::Inward));
        let jammed_body_deg = -14.0;
        let mut s = PhysicalSurfaces::default();
        s.ailerons_deg[0][0] = Some(jammed_body_deg);
        writer.apply(&mut vars, &s);
        assert_eq!(vars.read(&active_id), 1.0, "a jam must set the active flag");
        let n = vars.read(&value_id);
        assert!((aileron_or_elevator_down_deg(n) - jammed_body_deg).abs() < 1e-9);
    }

    #[test]
    fn a_healthy_aileron_clears_the_active_flag_and_does_not_touch_the_value() {
        // The correctness point W125 raised: `None` (this area either has
        // no live instance for the surface, or has one and it is healthy)
        // must write the active flag `0.0` -- deterministically, every
        // tick, so a fault that just cleared cannot leave a stale `1.0`
        // behind -- and must not touch the value var, which the consumer is
        // never supposed to read while inactive anyway.
        use crate::aspects::test_vars::TestVars;
        let mut vars = TestVars::default();
        let writer = SurfaceOverrideWriter::new(&mut vars);
        let active_id = vars.get(rudder_active_var_name(RudderPanel::Upper));
        let value_id = vars.get(rudder_deflection_var_name(RudderPanel::Upper));
        writer.apply(&mut vars, &PhysicalSurfaces::default()); // every field None
        assert_eq!(vars.read(&active_id), 0.0, "a healthy/unmodelled surface must clear the active flag");
        assert_eq!(vars.read(&value_id), 0.0, "and never write a fabricated value for it");
    }

    #[test]
    fn ths_and_flap_slat_write_through_unconverted_degrees() {
        use crate::aspects::test_vars::TestVars;
        let mut vars = TestVars::default();
        let writer = SurfaceOverrideWriter::new(&mut vars);
        let ths_active_id = vars.get(THS_ACTIVE_VAR_NAME.to_owned());
        let ths_id = vars.get(THS_DEFLECTION_VAR_NAME.to_owned());
        let left_flap_id = vars.get(flap_var_name(Side::Left));
        let right_flap_id = vars.get(flap_var_name(Side::Right));
        let right_slat_id = vars.get(slat_var_name(Side::Right));
        let mut s = PhysicalSurfaces { ths_deg: Some(6.5), ..Default::default() };
        s.flap_deg[0] = Some(12.0);
        s.slat_deg[1] = Some(20.0);
        writer.apply(&mut vars, &s);
        assert_eq!(vars.read(&ths_active_id), 1.0);
        assert_eq!(vars.read(&ths_id), 6.5);
        assert_eq!(vars.read(&left_flap_id), 12.0);
        assert_eq!(vars.read(&right_slat_id), 20.0);
        // A side with no override (`None`) must not be written at all.
        assert_eq!(vars.read(&right_flap_id), 0.0);
    }
}

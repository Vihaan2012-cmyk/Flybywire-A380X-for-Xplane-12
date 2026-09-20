//! Integration: wires the deep systems' physical models onto the real
//! aircraft in X-Plane, in both directions, and documents (in
//! `docs/deep/integration.md`) the exact, small patches other areas' files
//! need so the wiring below actually runs -- this directory itself may not
//! edit any file outside `src/deep/integration/` (`docs/deep/BRIEF.md` hard
//! rule 1).
//!
//! `docs/deep/BRIEF.md`'s Integration backlog, in order:
//! 1. [`flight_control_surfaces`] -- turns a `deep::flight_controls`
//!    surface's modelled position (jammed, blown back, free-floating) into
//!    the exact normalised value FlyByWire's own actuator model would have
//!    written, so overriding that one shared variable drives both X-Plane's
//!    surface datarefs (unchanged `flight_controls.rs`/`handling.rs`) and
//!    whatever inside FlyByWire's own simulation reads the same variable
//!    back as position feedback.
//! 2. [`weather_truth`] -- reads X-Plane's real weather/atmosphere
//!    (`XPLMGetWeatherAtLocation`, `sim/weather/aircraft/*`) into one
//!    `EnvironmentTruth`, plus the physics (compressible Mach, an EASA
//!    CS-25 Appendix-C-shaped icing envelope) needed to turn "temperature
//!    and cloud type" into "liquid water content and droplet size" -- no
//!    X-Plane dataref publishes LWC/MVD directly.
//! 3. [`fire_ice_adapter`], [`sensors_adapter`], [`thermal_zones_adapter`],
//!    [`environment_events_adapter`] -- pure functions from
//!    `EnvironmentTruth` (plus, where a model needs it, a little
//!    already-computed flight state) to each area's own public input
//!    types, read from their own source rather than re-derived.
//! 4. [`xp_consequences`] -- the reverse direction: aerodynamic/structural
//!    outputs already computed by other areas' models (ice mass/shape
//!    drag, bird-strike/hail leading-edge dent drag, a collapsed gear leg)
//!    turned into X-Plane's own `sim/flightmodel/forces/*_plug_acf`
//!    plugin-force/moment datarefs (the SDK's own mechanism for exactly
//!    this, X-Plane 10.30+) or, where no dataref exists at all (a broken
//!    strut's shape), the closest available real consequence
//!    (`sim/flightmodel2/gear/deploy_ratio`).
//! 5. [`registry`] -- registers this directory's own components/failures/
//!    ECAM alerts through `crate::deep::api::Registry` (`Area::Integration`).
//!
//! Per hard rule 2, nothing outside this directory references this code
//! yet; unlike most areas, Integration's whole purpose is to depend on
//! already-existing crate internals (`crate::xp`, `crate::flight_controls`'s
//! public conversions, `systems::simulation`, other deep areas' public
//! input/output types) rather than staying self-contained -- that is what
//! "integration" means here, and the task explicitly calls for it.

pub mod weather_truth;

pub mod flight_control_surfaces;

pub mod fire_ice_adapter;
pub mod sensors_adapter;
pub mod thermal_zones_adapter;
pub mod environment_events_adapter;

pub mod xp_consequences;

pub mod registry;

pub mod failure_audit;

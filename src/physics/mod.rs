//! Plugin-side physical simulation that FlyByWire's own ported systems (a
//! WASM/MSFS-facing crate) have no equivalent of. FlyByWire's systems stay
//! authoritative over control laws and computer logic; these modules turn
//! their commands into the underlying physics (mass flow, energy, torque)
//! that the flight model and displayed simulator variables should show.
//!
//! Brief: `D:\fbw-xp-systems\docs\briefs\hyperrealism.md`. Parameter sources
//! are cited in `docs/physics/<area>.md`.

pub mod engine;

// hyperrealism.md physics workstream 5 (fluids).
pub mod fluids;
pub mod gas;
pub mod hydraulics;

// hyperrealism.md physics workstream 2 (electrical): a real Kirchhoff/
// Ohm's-law circuit over FlyByWire's own A380 topology (generators, TRUs,
// batteries, external power, buses -- patches/fbw-rust/electrical.patch),
// the shared engine-load contract's electrical term, and a circuit-
// protection (breaker/SSPC trip) foundation FBW's crate has no equivalent
// of at all. `docs/physics/electrical.md`.
pub mod electrical;

// Breaker-coupling workstream: shared DC/AC motor-current model (back-EMF
// mechanical-load relation, winding-insulation partial-short relation) every
// "physical cause raises this motor's current" coupling uses, so the
// breaker's own I^2t/magnetic curve trips from a real current, not an
// authored flag. `docs/physics/breakers.md`.
pub mod motor;

// hyperrealism.md physics workstream 4 (navigation sensors): strapdown IRS,
// pitot-static ADR and the radio altimeter's terrain probe.
// `docs/physics/adirs.md`.
pub mod adirs;

// Physics workstream 6 (failures, damage, MEL, persistence): exceedance and
// wear tracking that arms failures.rs's catalogue. `docs/physics/failures.md`.
pub mod damage;

// Physics workstream 6 continued: per-wheel tyre nitrogen pressure/
// temperature/leak/wear-pin model, and the fuse-plug melt it can arm.
// `docs/physics/failures.md`.
pub mod tyre;

// hyperrealism.md physics workstream 3 (air): bleed, packs, distribution,
// cabin thermal model and pressurisation. Almost all of it lands as a
// patch to FlyByWire's own Rust systems crate
// (patches/fbw-rust/air.patch); this module is only the shared engine-
// load contract's bleed term. `docs/physics/air.md`.
pub mod air;

// Equipment bay thermal model (emergence goal, docs/briefs/hyperrealism.md:
// a pneumatic bleed-duct leak heats a bay, and the hotter bay derates a
// breaker on a load that is fine when cold). Publishes
// `BAY_<NAME>_TEMPERATURE_C`, matched to `breakers.rs`'s already-landed
// `bay_for` consumer (`AVIONICS`/`CARGO_FWD`/`CARGO_AFT`/`WING_ROOT`) so
// its `trip_step_with_ambient` coupling reads real values instead of its
// `REFERENCE_AMBIENT_C` default. `physics::bays`'s own module doc has the
// full derivation and constant sources.
pub mod bays;

// X-Plane visible/physical failure effects workstream: turns this plugin's
// already-causal internal state into X-Plane's own native failure/effect
// datarefs (fire, cockpit smoke). `docs/physics/fire.md`.
pub mod xp_effects;

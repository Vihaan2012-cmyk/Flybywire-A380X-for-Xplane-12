//! Deep sensor physics: the individual physical sensing elements feeding
//! the A380's ADIRUs, radio altimeters, GPS receivers and assorted discrete
//! instrumentation, modelled at the level of the probe/transducer itself
//! (heater heat balances, icing, mechanical/electrical failure modes,
//! pneumatic lag) rather than at the level of the computed air-data output.
//!
//! `docs/deep/BRIEF.md`'s original sensors backlog, in order:
//! 1. [`pitot`] -- pitot probe (heater, icing, drain-hole-dependent
//!    blockage behaviour, insect/tape blockage, pneumatic lag).
//! 2. [`static_port`] -- static ports (blockage, leak to cabin, position
//!    error).
//! 3. [`aoa_vane`] -- AoA vane (heater, icing jam, mechanical stuck,
//!    resolver drift, damage).
//! 4. [`tat_probe`] -- TAT probe (heater, icing, recovery factor,
//!    self-heating error).
//! 5. [`adr`] -- ADR computation from the physical inputs above (CAS, Mach,
//!    altitude, TAS, SAT), plus a 3-way voter/monitor.
//! 6. [`radio_altimeter`] -- transceiver/tx-antenna/rx-antenna faults,
//!    false readings, multipath, range limits.
//! 7. [`gps`] -- receiver/antenna faults, geometry-free position error,
//!    loss of fix, jamming, spoofing.
//! 8. [`discrete`] -- proximity sensors, fuel/oil quantity capacitance
//!    probes, temperature sensors, pressure transducers.
//!
//! Second-pass (lead's "go much deeper" instruction) additions, one physical
//! sensor technology each, reused across many registered instances in
//! `registry.rs` rather than duplicated per instance in code:
//! 9. [`ice_detector`] -- magnetostrictive ice detector (resonant-frequency
//!    mass loading, detect/deice duty cycle).
//! 10. [`engine_sensors`] -- N1/N2/N3 speed pickups (VR sensor amplitude
//!     vs. air gap), TGT thermocouple harness averaging, vibration
//!     pickups, fuel flow transmitter (turbine K-factor).
//! 11. [`smoke_detector`] -- photoelectric light-scattering smoke detector.
//!
//! Several other requested instances (P30/T25 engine probes, engine
//! oil/hydraulic pressure and temperature, brake temperature, tyre
//! pressure, duct temperature, oxygen bottle pressure, CPC cabin pressure,
//! standby OAT) are the *same* physical sensor technology this directory
//! already models generically ([`discrete::PressureTransducer`],
//! [`discrete::temperature_sensor_reading_c`],
//! [`discrete::oil_probe_indicated_level`]) at a different installation
//! point -- `registry.rs` registers each as its own instance/component
//! against the shared model rather than this module re-implementing the
//! same physics under a new name (see `docs/deep/BRIEF.md`: "a failure
//! only if it changes a modelled output" -- a differently-named copy of an
//! identical function changes nothing).
//!
//! Third pass (the lead's deferred-items list) additions:
//! 12. [`float_level`] -- float-type liquid level transmitter (thermal
//!     expansion vs. a real leak, float lag/binding), used for hydraulic
//!     reservoir quantity.
//! 13. [`brake_wear`] -- brake wear indication, modelling indicated wear
//!     diverging from real wear under sensor binding.
//! 14. [`sideslip`] -- a synthetic angle-of-sideslip *estimator* (the
//!     A380, like the rest of the Airbus FBW family, has no physical
//!     sideslip vane); deliberately has no `Faults` struct and is not
//!     registered in `registry.rs` since there is no physical part to
//!     fail -- see that module's doc comment.
//! [`engine_sensors::tgt_harness_average_c`] was extended with real
//! per-junction circumferential position and an optional
//! [`engine_sensors::HotStreak`] plain input (for a combustor hot streak
//! from, e.g., a coked fuel nozzle group modelled by another agent).
//! [`static_port::average_pair`] adds left/right static-port pneumatic
//! averaging and what a single-side blockage does to it.
//!
//! ATA 28 fuel quantity/temperature is deliberately **not** registered by
//! this directory (a first-pass version was removed) -- a dedicated fuel
//! agent now owns `src/deep/fuel/` end to end, including per-tank/per-probe
//! fuel gauging; see `registry.rs`'s note above `register_hydraulic_sensors`
//! and `PROGRESS.md`.
//!
//! Per `docs/deep/BRIEF.md` hard rule 2, this directory is self-contained:
//! nothing here depends on any other module's internals (each submodule
//! that needs randomness or a physics helper used elsewhere in the crate --
//! e.g. the Zukauskas convective-heat-transfer correlation, also used by
//! `src/physics/adirs.rs` and `src/physics/engine/oil.rs` -- re-derives it
//! independently rather than importing it), and nothing outside this
//! directory references it yet.

mod rng;

pub mod adr;
pub mod aoa_vane;
pub mod brake_wear;
pub mod discrete;
pub mod engine_sensors;
pub mod float_level;
pub mod gps;
pub mod ice_detector;
pub mod live;
pub mod live_discrete;
pub mod pitot;
pub mod radio_altimeter;
pub mod registry;
pub mod sideslip;
pub mod smoke_detector;
pub mod static_port;
pub mod tat_probe;

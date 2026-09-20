# fire_ice progress

Directory: `D:\fbw-xp-systems\src\deep\fire_ice\`. Self-contained per
`docs/deep/BRIEF.md` rule 2 (no `crate::` dependency) except `registry.rs`,
which is the one file allowed to use `crate::deep::api` per the lead's
registration-API instruction.

- [done] Backlog item 1 (dual-loop fire detection) — `fire_loops.rs` — nine
  zones (4 engines, APU, MLG bay, cargo fwd/aft, avionics), each with a
  thermistor loop (Loop A, NTC resistance law) and a pneumatic loop (Loop
  B, Amontons' law average response + discrete getter-material hot-spot
  response). Open circuit reads out-of-physical-range (loop fault); short
  circuit reads indistinguishably from real heat (false fire) -- both
  emerge from the same physical law, not a scripted flag. AND/OR zone
  logic with single-loop fallback and a both-loops-faulted failsafe
  (matches the real, documented Airbus-style "simultaneous total loss ->
  presumed fire" convention). 11 unit tests.
- [done] Backlog item 2 (per-zone combustion + spread) — `combustion.rs` —
  fuel/air-limited burn rate (stoichiometric AFR), self-sustaining once
  ignited until fuel/air/suppression removes it or the zone cools below a
  quench margin, zone thermal balance, and a conductive inter-zone link
  (`conductive_link_w`, same "one literal wire" pattern as
  `physics::bays.rs`'s `heat_link_enabled`) that lets one zone's fire
  genuinely ignite a neighbour's own fuel source once its autoignition
  temperature is crossed -- proven both ways (spreads with the link,
  doesn't with it cut) in `heat_crossing_a_link_can_ignite...` /
  `cutting_the_link_prevents_the_same_spread`. 8 unit tests.
- [done] Backlog item 3 (extinguishing) — `extinguishing.rs` — Halon 1301
  bottles (Amontons' law pressure vs temperature, with the real two-phase
  liquid/vapour behaviour: pressure holds flat with fill level until a
  residual-liquid fraction, then falls off), one-shot irreversible squib
  discharge through an orifice model, a slow independent leak path, zone
  agent-concentration decay against ventilation (first-order gas washout,
  NFPA 12A design concentration), cross-feed valve routing, cargo optical
  smoke detection (Beer-Lambert obscuration, soot-yield mass balance,
  public UL217-range alarm threshold) with a lens-obscured fault, two-stage
  (high-rate knockdown then metered) cargo suppression whose metered
  duration is an emergent mass/flow-rate result, and lavatory smoke
  detection plus a passive, irreversible fusible-link extinguisher
  (CS-25.854) with an aging/degraded-link fault. 14 unit tests.
- [done] Backlog item 4 (ice accretion) — `icing.rs` — droplet inertia
  parameter and the Langmuir-Blodgett-derived collection-efficiency
  approximation (`util::droplet_inertia_parameter`/
  `collection_efficiency_beta0`), the classical Messinger surface energy
  balance (`util::messinger_freezing_fraction`) giving freezing fraction
  and ice mass/thickness on wing leading edge, nacelle inlet, probe and
  windshield presets, plus a GENERIC saturating aerodynamic-penalty
  correlation (Cl_max loss, Cd increase) for lifting surfaces. Verified:
  probes collect more efficiently than the wing at the same conditions;
  high-TAS recovery heating can fully prevent freezing even in a
  supercooled cloud (with the important caveat, corrected during testing,
  that this needs a much higher Mach number at cold-cruise temperatures
  than at milder icing-envelope temperatures -- documented in
  `util::recovery_temperature_c`'s doc comment). 8 unit tests.
- [done] Backlog item 5 (anti-ice) — `anti_ice.rs` — wing/nacelle hot-
  bleed-air heat balance solved as a genuine self-consistent equilibrium
  (`util::equilibrium_surface_c_with_bleed`, bisected directly rather than
  iterated tick-to-tick, which is numerically unconditionally stable
  regardless of bleed-flow magnitude), probe and windshield-film electrical
  resistive heat with bang-bang thermostatic control, a real thermal-mass
  relaxation (`util::relax_toward_equilibrium_c`, exact exponential step)
  so a bang-bang controller settles into a small oscillation band around
  its setpoint instead of swinging instantly between extremes, valve
  stuck-open/closed and duct-leak faults, probe/window stuck-warm sensor
  faults and controller faults (including a "controller fault sticks the
  heater full on" dangerous failure mode), a film-defect hot-spot model
  (same `1/(1-x)^2` concentration law as `physics::engine::oil.rs`'s
  filter clog) driving irreversible delamination/crack thresholds, and a
  windshield rain-removal water-film mass balance with a jet-shear fault.
  16 unit tests. This module went through substantial mid-development
  correction (see below) before landing on physically sound numbers.
- [done] Shared physics (`util.rs`) — orifice compressible flow, Tetens
  saturation vapor pressure, recovery/kinetic-heating temperature, the
  Messinger surface energy balance and its self-consistent
  heater/bleed-equilibrium solvers, thermal-mass relaxation, droplet
  inertia/collection-efficiency, Sutherland viscosity. 14 unit tests.
- [done] Registration — `registry.rs` — every failure, component and ECAM
  alert above registered through `crate::deep::api::Registry`
  (`Area::FireIce`), per the lead's mid-task instruction (superseding an
  earlier CATALOGUE.md/ECAM.md instruction; neither file was ever created,
  so nothing needed deleting).

## Corrections made while validating by hand (no `cargo test` available;
verified every nontrivial numeric claim by re-deriving the same formulas in
a scratch Python script and checking the assertions would actually hold)

- `util::surface_net_loss_w_m2`'s evaporative term was originally applied
  unconditionally from ambient humidity alone; fixed to gate on
  impingement being present (no water on the surface -> no evaporative
  cooling possible), which is what let the anti-ice "stuck valve, no icing
  demand" scenario actually reach a stable, physically sensible overheat
  temperature instead of a runaway one.
- `anti_ice::BleedAntiIceSurface` originally computed its own heater term
  from the *previous* tick's surface temperature (a lagged fixed-point
  iteration); for a large bleed-flow-to-area ratio this does not converge
  (diverges/oscillates). Replaced with a genuinely self-consistent
  equilibrium solved by bisection each tick
  (`util::equilibrium_surface_c_with_bleed`), which is unconditionally
  stable, plus real thermal-mass relaxation for smooth per-tick dynamics.
- The probe/window bang-bang thermostats originally had zero thermal mass,
  so the modelled surface swung instantly between two extreme equilibria
  every tick depending on whichever side of the setpoint it landed on --
  not real behaviour for anything with mass. Added
  `util::relax_toward_equilibrium_c` (exact exponential first-order-lag
  step, per this project's own conventions) and per-surface thermal time
  constants; tests now check a settled tail window rather than one
  arbitrary tick, since a real bang-bang controller with any plant inertia
  settles into a small oscillation band, not a single fixed value.
- The probe/window "sensor fault" was originally modelled as blending the
  sensed reading toward the real ambient temperature -- which, in the
  realistic case where ambient is colder than the heated surface, is a
  complete no-op (verified numerically: identical results with the fault
  on or off). Replaced with a stuck-at-a-fixed-warm-value fault, the
  actual common real failure mode for a resistive/thermocouple element,
  which is unconditionally effective.
- `icing.rs`/`util.rs`'s original "high Mach prevents icing" test used
  -50 C/260 m/s and asserted recovery heating crossed freezing; it does
  not (recovers to about -20 C, not >0 C) -- fixed the test to a milder,
  still-realistic icing-envelope temperature (-10 C) where the same speed
  genuinely does cross freezing, and corrected the overclaiming doc
  comment that had asserted a fixed "Mach 0.6-0.7" universal threshold.

## New simulator variables (`Vars`) this backlog's eventual wiring needs

None of the above is wired into the plugin's update loop or dataref
registry yet (per BRIEF: "nothing else in the crate references your code
yet"). `registry.rs`'s ECAM triggers name the following not-yet-published
Vars, which whoever wires `fire_ice` into the simulation must publish under
these exact names: `FIRE_DETECTED_ENG:n`, `FIRE_DETECTED_APU`,
`FIRE_DETECTED_MLG`, `FIRE_BUTTON_ENG:n`, `FIRE_BUTTON_APU`,
`FIRE_SQUIB_{1,2}_ENG_n_IS_DISCHARGED`, `FIRE_SQUIB_1_APU_1_IS_DISCHARGED`,
`CARGO_{FWD,AFT}_SMOKE_DETECTED`, `CARGO_{FWD,AFT}_SUPPRESSION_ARMED`,
`FIRE_LOOP_{A,B}_<ZONE>_FAULT`, `FIRE_BOTTLE_ENGn_{1,2}_LOW_PRESSURE`,
`ANTI_ICE_{WING_L,WING_R}_VALVE_OPEN`, `ANTI_ICE_NACELLEn_{VALVE_OPEN,
OVERHEAT}`, `ANTI_ICE_{WING_L,WING_R}_OVERHEAT`, `WINDOW_HEAT_{L,R}_FAULT`,
`PROBE_HEAT_PITOTn_FAULT`, `AUTOTHRUST_TLA:n`, `ENGINE_MASTER:n`.

## Natural next extensions (not started, in priority order)

1. Wire `fire_loops`/`combustion`/`extinguishing` zone state together into
   one `FireZone` aggregate per zone (detection -> combustion ->
   suppression feedback loop) so a single struct owns one zone end to end;
   currently each model is independently tested but the caller must wire
   the three together itself.
2. Runback icing (liquid fraction from a partially-frozen Messinger result
   flowing aft of the protected zone and refreezing) -- `icing.rs`
   currently reports the unfrozen fraction implicitly (`1 -
   freezing_fraction`) but does not track where it goes.
3. A `NacelleInlet`-specific anti-ice/icing coupling test proving the same
   spread/overheat mechanisms as the wing case (only the wing case has a
   dedicated test currently; `NACELLE_ANTI_ICE`'s constants are exercised
   only indirectly through the shared `BleedAntiIceSurface` code path).
4. Cross-feed (`extinguishing::CrossFeed`) is modelled as a routing
   decision but not yet exercised end-to-end with two real `Bottle`/zone
   pairs in one test.

- [done] live system — `live.rs` (`live_system() -> Box<dyn deep::live::Area>`), `mod.rs` —
  owns the nine `ZoneDetector`/`ZoneCombustion`/`ZoneConcentration` sets, 8 engine bottles +
  APU bottle + 2 cargo suppression systems, both cargo smoke detectors, the lavatory link, 2
  wing + 4 nacelle anti-ice surfaces, 8 probe heaters, 2 window heaters, 2 rain-removal
  systems and the wing/nacelle icing surfaces. Driven from `Truth`: the icing environment via
  `integration::weather_truth`'s real cloud sample, `engine_running`/`apu_running` as the only
  ignition sources, `on_ground` for the APU's automatic agent discharge, and
  `precipitation_on_aircraft_ratio` x TAS for the windshield water catch. Consumes all 36 loop
  failures, all 9 leak sources, all 22 bottle/squib failures, both smoke detectors, the
  lavatory link and all 50 ATA 30 failures. Publishes `FIRE_DETECTED_*`, `FIRE_LOOP_A/B_*_FAULT`,
  `FIRE_BOTTLE_ENG<n>_<b>_LOW_PRESSURE`, `FIRE_SQUIB_*_IS_DISCHARGED`, `CARGO_<bay>_SMOKE_DETECTED`,
  `ANTI_ICE_*_VALVE_OPEN`/`_OVERHEAT`, `PROBE_HEAT_*_FAULT`, `WINDOW_HEAT_*_FAULT` and the
  underlying temperatures/ice thicknesses. 11 tests.
  Still missing from `Truth`: fire/agent pushbuttons (only bottle *leaks* and the APU ground
  discharge are observable), wing/nacelle anti-ice selection, rain-removal selection, and a
  cabin/lavatory local temperature. No bulk hold exists in `fire_loops::ZONES`, so
  `CARGO_BULK_SMOKE_DETECTED` is not published here (that alert reaches its trigger through
  `thermal_zones`' contribution instead).

# Integration area — progress

Directory: `src/deep/integration/`. Area: `Area::Integration` (18). Per
`docs/deep/BRIEF.md` hard rule 1, only files inside this directory (plus
this file, `FAILURES.md`, and the shared `docs/deep/integration.md`) were
written; every change needed in another file is an exact patch in
`docs/deep/integration.md`, not applied here.

Unlike a modelling area, Integration's whole purpose is to *depend on*
already-existing crate internals (`crate::xp`, `crate::flight_controls`'s
public conversions, `systems::simulation`, other deep areas' public input/
output types) rather than stay self-contained — the task explicitly calls
for this (`docs/deep/BRIEF.md`'s task brief, "your area is Area::
Integration"). Because none of the areas it depends on are declared from
`src/deep/mod.rs` yet (nor is this one), **none of this code can compile
into the crate until the lead applies `docs/deep/integration.md`'s
`deep/mod.rs` patch** — this matches how every other area's own
`PROGRESS.md` already notes it is not wired in yet.

## Backlog item 1 — flight-control surface override (`flight_control_surfaces.rs`)

- [done] Per-A380-surface `Var` name reproduction (`HYD_AIL_*`,
  `HYD_ELEV_*`, `HYD_*_RUD_DEFLECTION`, `HYD_SPOILER_*`,
  `HYD_FINAL_THS_DEFLECTION`, `LEFT/RIGHT_FLAPS_ANGLE`,
  `LEFT/RIGHT_SLATS_ANGLE`) matching `flight_controls.rs`/`handling.rs`'s
  own construction exactly, plus the exact inverse of `flight_controls.rs`'s
  documented aileron/elevator, rudder and spoiler request formulas,
  round-trip-tested against its real, unmodified public functions
  (`aileron_or_elevator_down_deg`, `rudder_right_deg`, `spoiler_up_deg`) so
  this can never silently drift from that file. 9 tests.
- [done] `SurfaceOverrideWriter`: caches every surface's `VariableIdentifier`
  once, then overrides from a `PhysicalSurfaces` snapshot each tick — the
  mechanism that makes a jammed/blown-back/free-floating deep
  `flight_controls::surface::ControlSurface`/`high_lift::HighLiftSystem`
  angle reach X-Plane through the **unmodified** existing pipeline (no edit
  to `flight_controls.rs`/`handling.rs` needed at all — see
  `docs/deep/integration.md` for the one lib.rs ordering patch this does
  need). Every `PhysicalSurfaces` field is `Option<f64>`, not a bare
  `f64`: `None` means "no live deep-model instance for this surface yet,
  leave FlyByWire's own command alone" rather than silently forcing an
  unmodelled surface to a fabricated default — a correctness fix made
  during this pass (an all-`f64` version would have fought FlyByWire's real
  commands on every surface `deep::flight_controls` has not modelled yet),
  tested explicitly (`a_surface_with_no_live_deep_model_instance_keeps_flybywires_own_command`).
- **Known gap, not fabricated**: `PhysicalSurfaces` takes plain per-surface
  angles because `deep::flight_controls` has not yet built (and, per its own
  directory rule, cannot yet have) an aggregate "all 37 A380 surfaces"
  struct — no `mod.rs`/`registry.rs` exists there as of this writing, only
  `actuator.rs`/`surface.rs`/`hinge_moment.rs`/`high_lift.rs`. Whoever wires
  a live `ControlSurface`/`HighLiftSystem` instance per real surface must
  fill `PhysicalSurfaces` from each one's own `angle_rad`/
  `inboard_angle_rad`/`outboard_angle_rad` getter, in the body-angle
  convention `flight_control_surfaces.rs`'s module doc documents exactly.
- **Known gap**: flap/slat real travel range. `deep::flight_controls::
  high_lift::HighLiftSystem::new_generic()` is explicitly GENERIC (no cited
  A380 flap/slat travel in degrees), so `PhysicalSurfaces::flap_deg`/
  `slat_deg` are `Option<f64>` and this module cannot itself invent a
  calibration (would violate the no-fake-values rule) — pass `None` (leaves
  FlyByWire's own value in place) until that area publishes a real degree
  range, then feed the calibrated angle straight through; no code change
  needed here.
- **Known gap**: flap/slat asymmetry. `handling.rs` drives every X-Plane
  flap element from one ratio per side (`flap_deg`/`slat_deg` here are
  already "one representative value per side" for exactly this reason) —
  X-Plane's own converted `.acf` cannot show an inboard/outboard split even
  once `high_lift.rs` models one. Exact patch to lift this limitation (a
  second `Var`/degree array) is in `docs/deep/integration.md`, not applied.
- **Feedback-path honesty** (see the module's own long doc comment): this
  guarantees X-Plane and any other `Var`-registry reader see the real
  position the same tick; it does **not** claim same-tick closure into
  `a380_systems`' own internal control-law state, which would need a change
  to the read-only `D:\fbw-aircraft` reference tree. If any of its SFCC/
  PRIM logic reads a peer actuator's *published* `Var` as feedback (as
  opposed to its own private field), it sees the override starting the next
  `Simulation::tick()` — the same one-tick lag `extra_backend_fbw.rs`
  already accepts for the reverser force path.

## Backlog item 2 — environment (`weather_truth.rs`, `fire_ice_adapter.rs`, `sensors_adapter.rs`, `thermal_zones_adapter.rs`, `environment_events_adapter.rs`)

- [done] `EnvironmentTruth`: SAT, leading-edge (recovery) temperature,
  ambient pressure, TAS, precipitation-on-aircraft ratio and
  `XPLMGetWeatherAtLocation`'s own sample (precip rate, turbulence ratio, up
  to 3 cloud layers' type/coverage/base/top), every field's exact real
  source cited in the module doc. `WeatherTruthReader` reads it each tick
  via the plugin's existing `"AMBIENT TEMPERATURE"`/`"AIRSPEED TRUE"`
  `Var`s plus `Option<DataRef>`/`Option<&Xplm>` for the rest, matching
  `physics/xp_effects.rs::XpEffects`'s own degrade-gracefully pattern. 12
  tests on the pure derivations (Mach, cloud classification, LWC/MVD
  envelope shape, convective/hail intensity).
- [done] LWC/droplet-size derivation: X-Plane's weather API has no LWC/MVD
  field at all (confirmed: neither `sim/weather/*` nor
  `XPLMWeatherInfo_t`'s member list represents liquid water content) — a
  GENERIC Gaussian stand-in for 14 CFR/CS-25 Appendix C's published
  temperature-banded icing envelope shape, gated on real cloud type/
  coverage/temperature, scaled to the peak order of magnitude
  `deep::fire_ice::icing`'s own module doc already cites. Explicitly not a
  digitisation of the actual Appendix C chart (documented as such).
- [done] `fire_ice_adapter::icing_environment` -> `fire_ice::icing::
  IcingEnvironment` (5 tests, including a full `IcingSurface::step` run
  under the derived environment).
- [done] `sensors_adapter::probe_environment` -> the `(tas_ms, sat_c,
  lwc_gm3)` triple `pitot`/`tat_probe`/`aoa_vane` all take directly, plus
  `true_total_pressure_pa` (compressible pitot-static relation) for
  `PitotProbe`'s true-pressure input. Explicit unit-conversion test (LWC
  kg/m^3 -> g/m^3, the one place `fire_ice` and `sensors` chose different
  units independently) plus end-to-end runs of `PitotProbe::step`/
  `TatProbe::step` under the derived environment. 6 tests.
- [done] `thermal_zones_adapter::outside_air` -> `thermal_zones::network::
  OutsideAir` (2 tests).
- [done] `environment_events_adapter`: `convective_intensity`/
  `hail_intensity` (real cumulonimbus-cell detection from cloud type +
  X-Plane's own `turbulence_alt`/`precip_rate_alt`), `turbulence_intensity`
  (bands X-Plane's **real** `turbulence_alt` ratio into `wind_shear::
  TurbulenceIntensity` — the one input in this whole file that needs no
  inference at all, X-Plane already reports it), `ice_water_content_g_m3`
  (deep glaciated convective core, gated on both cell strength and
  temperature), `contaminant_from_weather` (best-effort, explicitly
  low-confidence — see its own doc, X-Plane has no runway-contamination
  dataref at all), `night_from_sun`/`month_from_day_of_year` (the two
  genuinely-environmental fields of `bird_strike::FlightState`, reusing
  `extra_backend_fbw::time_of_day_from_sun` rather than re-deriving it), and
  `WindShearSampler` (Bowles F-factor inputs from real wind direction/speed
  finite-differenced against aircraft heading). 12 tests.
- **Cross-area blocker found, not fixed here (not this directory)**:
  `src/deep/environment/mod.rs` declares `mod rng;` (private). `lightning::
  LightningModel::step`, `hail::HailModel::step`, `wind_shear::
  TurbulenceModel::step` and `bird_strike::BirdStrikeModel::step` are all
  `pub fn` but take `&mut environment::rng::Rng` in their signature — a type
  that cannot be named or constructed from outside `deep::environment` as
  currently declared, so none of those four `step` functions can actually
  be called by this (or any other) adapter yet. Exact one-line patch in
  `docs/deep/integration.md`. This is why this directory's tests stop at
  its own pure outputs (`convective_intensity`, `turbulence_intensity`,
  etc.) rather than driving those models end-to-end — the correct test
  boundary for an adapter regardless, but noted so the gap is not silently
  assumed away.
- **Out of scope, not filler**: `deep::environment::volcanic_ash::
  AshInputs` needs an ash concentration X-Plane's weather system has no
  concept of whatsoever (grepped `sim/weather/*` and every
  `XPLMWeatherInfo_t` member: none). There is nothing to adapt from real
  weather; an ash encounter is necessarily a scripted/scenario input
  (`scenarios.rs`), not a weather-truth one, so no adapter was written for
  it (a stub would be exactly the "placeholder function that does nothing"
  the brief forbids).
- **New `Var`s this directory's own registry needs published, if the lead
  wants the Components-page diagnostic wired live**: none required by code
  in this directory — `18_int.weather_truth_feed`'s `weather_api_available`
  reads `crate::xp::has_weather_api()` directly, no new `Var` needed.

## Backlog item 3 — X-Plane consequences (`xp_consequences.rs`)

- [done] Found and documented X-Plane's real plugin-force mechanism,
  `sim/flightmodel/forces/{fside,fnrml,faxil}_plug_acf` (N) /
  `{L,M,N}_plug_acf` (N*m) (`developer.x-plane.com/article/
  movingtheplane`, X-Plane 10.30+) — the physically correct path for a
  continuous force/moment (ice drag, a dragging collapsed gear leg), as
  opposed to `extra_backend_fbw.rs::apply_reverser_thrust`'s velocity/
  yaw-rate nudge (a different, pre-existing mechanism in that file, not
  changed here). Documented that X-Plane zeroes these every frame, so
  every function here returns a **delta**, applied through the mandatory
  read-add-write pattern (`add_plug_force`).
- [done] `extra_drag_force_n`/`asymmetric_drag_yaw_moment_nm`: one pair of
  functions serving both `fire_ice::icing::IcingOutputs.cd_increase_fraction`
  and `environment::bird_strike::StrikeOutcome.
  leading_edge_dent_drag_delta_cd` (both are, physically, an extra
  profile-drag coefficient on a reference area — `D = q*S*dCd`), plus the
  ideal-gas dynamic-pressure helper and the public A380 wing reference area
  (845 m^2, Airbus's own published Aircraft Characteristics figure). 6
  tests.
- [done] Collapsed gear leg (`gear_structure::LegOutput.collapsed`):
  `gear_deploy_override` retracts that gear's `sim/flightmodel2/gear/
  deploy_ratio` element (X-Plane then computes zero ground reaction there
  on its own — the corner of the airframe settles under its own weight from
  X-Plane's real physics, not a scripted animation) plus
  `collapsed_leg_drag_force_n` (a dragging bare strut's sliding friction
  against its static load share, GENERIC coefficient, cited/bounded against
  a rolling tyre's and a locked tyre's real figures) injected via the same
  plug-force path while on the ground. Honest caveat documented in the
  module doc: this *looks* like a retracted gear, not a visibly bent strut
  — the SDK has nothing better.
- **Deliberately not modelled** (would need a fabricated number): a
  Cl_max-loss lift penalty from ice. X-Plane's own stall AoA/Cl_max is
  internal to its flight model with no writable override, and applying a
  lift-reducing force at *all* angles of attack (not just near the real
  stall margin the Cl_max loss actually affects) would misrepresent the
  physics rather than approximate it — left out rather than faked; only the
  always-correct drag consequence is applied.
- **Not modelled, no X-Plane hook exists**: bird-strike/hail radome,
  windshield and nose-gear damage stay avionics/systems-level facts (no
  airframe-force consequence of their own); their already-registered
  `Area::Environment` ECAM alerts are the correct and sufficient
  consequence for those.

## Registration (`registry.rs`)

- [done] 7 `ComponentDef`s (`weather_truth_feed`, `airframe_ice_state`, one
  `gear_xp_relay` per leg), **zero** new `FailureDef`s and **zero** new
  `EcamAlert`s — see `registry.rs`'s own module doc for why: Integration
  relays other areas' already-registered physical faults and their already-
  registered ECAM alerts rather than originating new ones of its own. 2
  tests confirm `Registry::validate()` passes clean and every id is unique.

## Next most valuable items if continued

1. Once `deep::flight_controls` publishes its own `mod.rs`/aggregate
   surface struct, build the small glue that fills `PhysicalSurfaces` from
   live `ControlSurface`/`HighLiftSystem` instances each tick (this
   directory's own scope stopped at the pure conversion layer per the
   "nothing calls this yet" convention every area follows).
2. Apply `docs/deep/integration.md`'s `environment/mod.rs` `rng`-visibility
   patch, then extend `environment_events_adapter.rs`'s tests to drive
   `LightningModel`/`HailModel`/`TurbulenceModel`/`BirdStrikeModel::step`
   end-to-end.
3. A second flap/slat `Var`/degree-array pair so `handling.rs` can show
   inboard/outboard asymmetry once `high_lift.rs` calibrates real travel.
4. Wire `sensors::ice_detector`'s output (new since this directory's
   research pass began) into whichever ECAM alert governs "ICE NOT
   DETECTED" — noted here for whoever owns that alert; not this directory's
   component to add without overlapping `Area::Sensors`'/`Area::FireIce`'s
   own registrations.

- [done] `contaminant_from_weather`'s wet/dry snow split moved from -15 C to
  -5 C (`WET_SNOW_MIN_C`). Wet snow's cohesion comes from liquid water in the
  snowpack, which only survives within a few degrees of melting (ICAO Doc
  9981 / EASA RCAM define the two by cohesion); -15 C classified ordinary dry
  continental snowfall as wet, the more optimistic of the two for braking
  action. File: environment_events_adapter.rs.
- [done] `normalized_aileron_or_elevator`'s round-trip test now covers the
  conversion's actual domain. `flight_controls.rs` is `down_deg = 20 - 50*n`
  over `n` in 0..1, i.e. -30 (30 up) .. +20 (20 down) deg; the test asked for
  a round trip at +30 deg, which is past the travel stop and correctly
  clamps. Both stops are now asserted explicitly. File: flight_control_surfaces.rs.

# Environment area — progress

Directory: `src/deep/environment/`. Coupling agent: self-contained models
producing documented output structs; nothing in the crate references this
code yet. See `registry.rs` for the failure/component/ECAM registration
(replaces the earlier CATALOGUE.md/ECAM.md instruction) and `FAILURES.md`
for the same faults in prose.

- [done] Bird strike — `bird_strike.rs`, `rng.rs`, `registry.rs` — FAA
  National Wildlife Strike Database altitude/phase distribution, CS-25.631 /
  CS-25.775(b) / CS-E 800 bird masses, frontal-area-weighted target
  selection (4 engines, 6 windshield panels, radome, 12 wing LE segments,
  nose gear, 6 air data probes), flock splitting, manual trigger API
  (`arm`/`TriggerCondition::{Now, AtAltitudeAglM, DuringPhase}`) plus an
  optional random background mode (`BirdStrikeModel::random_mode`). 13
  tests. New Vars the datarefs layer still needs to publish from
  `bird_strike::StrikeOutcome` for the registered ECAM alerts to ever
  trigger at runtime: `ENV_BIRD_FAN_DAMAGE:<1-4>`, `ENV_BIRD_CORE_FOD:<1-4>`,
  `ENV_BIRD_WINDSHIELD_DAMAGE:<1-6>`, `ENV_BIRD_RADOME_DAMAGE`,
  `ENV_BIRD_WING_LE_DRAG:<1-12>`, `ENV_BIRD_NOSE_GEAR_DAMAGE`,
  `ENV_BIRD_PROBE_BLOCKED:<1-6>` (documented in `registry.rs`'s header too).
- [done] Lightning — `lightning.rs`, `registry.rs` — SAE ARP5414 zoning/
  attachment pattern (nose/wingtip entry, tail/nacelle exit), SAE ARP5412
  component A current (200 kA / 2e6 A^2s design value) with a GENERIC
  log-normal typical-strike distribution (median 30 kA, Berger et al.),
  per-bus GENERIC exposure factors (17 buses swept every strike:
  PRIM/SEC/FMGC/ADIRS/standby instruments/IFE/4x FADEC, `all_buses()`),
  standby-compass deviation, radome
  diverter-strip damage split, composite-extremity burn. 9 tests. New Vars:
  see `registry.rs` header (`ENV_LTG_*`).
- [done] Hail — `hail.rs`, `registry.rs` — TORRO Hailstorm Intensity Scale
  classification, radome hail resistance (C-grade 20mm/H2, public
  radome-industry figure), GENERIC windshield/leading-edge references,
  GENERIC hail terminal-velocity relation added to TAS for impact energy,
  GENERIC exponential size distribution for random mode (mean 8mm, NWS
  severe-hail criterion 25mm noted as the "large hail" boundary in the
  module doc). Manual `trigger(target, diameter_mm)` plus `random_mode`
  keyed to a `hail_intensity` weather input. 8 tests. New Vars: see
  `registry.rs` header (`ENV_HAIL_*`).
- [done] Volcanic ash — `volcanic_ash.rs`, `registry.rs` — ICAO/EASA
  concentration bands (<2/2-4/>4 mg/m^3), GENERIC linear melt ramp
  (1000-1200 C) splitting ingested ash into a molten fraction that glasses
  the NGVs (flow-capacity loss) and a solid fraction that erodes the
  compressor (GENERIC velocity^2.5 erosion scaling), cumulative windshield
  abrasion and pitot blockage, cabin odour, and `relight_possible` once
  concentration returns to ~0 (BA9/KLM867 precedent, damage itself does
  not heal). 8 tests. New Vars: see `registry.rs` header (`ENV_ASH_*`).
- [done] Ice crystal icing — `ice_crystal_icing.rs`, `registry.rs` — Mason,
  Strapp & Chow (AIAA 2006-206) mechanism: crystals only adhere within a
  GENERIC +-4 C window around a 0 C surface (too cold = bounces off, too
  warm = melts and washes away), accreting on the IPC front stage/splitter
  until a GENERIC 0.6 flow-capacity-loss threshold sheds most of it, with
  roll-back risk scaling as loss^2 and a flameout-risk spike during
  shedding. 6 tests. New Vars: see `registry.rs` header (`ENV_ICE_*`).
- [done] Runway contamination — `runway_contamination.rs`, `registry.rs` —
  public RCAM (ICAO Doc 9981 / FAA AC 150/5200-30D) contaminant/depth/
  temperature -> RWYCC 0-6 -> qualitative braking-action mapping, GENERIC
  representative mu per band (the RCAM itself is deliberately qualitative,
  no public numeric mu), Horne's NASA hydroplaning speed `9*sqrt(psi)`
  with a GENERIC onset ramp collapsing friction toward a residual once
  groundspeed passes it. Pure-function API (`friction(contaminant,
  tire_pressure_psi, groundspeed_kt)`), no persistent state needed. 7
  tests. New Vars: see `registry.rs` header (`ENV_RWY_*`).
- [done] Wind shear and turbulence — `wind_shear.rs`, `registry.rs` —
  Bowles' F-factor windshear-hazard metric with a GENERIC idealised
  microburst spatial profile (manual `trigger`/`random_mode`, same shape
  as the other event models); a three-axis Dryden turbulence model per
  MIL-HDBK-1797 (public low/medium-altitude scale-length and intensity
  formulas), each axis stepped as an exact Ornstein-Uhlenbeck
  discretisation -- documented deliberate simplification: the vertical
  channel's numerator zero is dropped so every step stays numerically
  exact rather than an uncontrolled approximation (mirrors
  `physics::engine`'s own documented simplification convention). 10
  tests. New Vars: `ENV_F_FACTOR` (turbulence has no ECAM entry, see
  `registry.rs` header).

Backlog (docs/deep/BRIEF.md) complete; then extended per the lead's
follow-up request:

- [done] `weather_cells.rs` — `WeatherCell`/`WeatherCellField`: position,
  radius, base/top height, a 0..1 intensity proxy, GENERIC updraft/
  downdraft (loosely parcel-theory-consistent), from which
  `convective_intensity` (feeds `lightning`/`wind_shear`'s own parameter
  of that name directly), `hail_intensity` (feeds `hail`, via
  `hail::max_sustainable_diameter_mm`'s updraft-vs-hail-growth inversion),
  `ice_water_content_g_m3` (feeds `ice_crystal_icing`, peaking in the
  cell's upper third per HAIC/HIWC) and a `TurbulenceIntensity` are all
  derived at one aircraft position from one consistent picture, rather
  than being set independently per model. `WeatherCell::from_xp_point_weather`
  is the concrete recipe for `src\deep\integration\`: X-Plane's
  `XPLMGetWeatherAtLocation` point weather (precip rate, thunderstorm
  fraction, cloud top, both 0..1/metres) in, one synthetic cell out;
  richer feeds can instead call `WeatherCellField::add` directly. Position
  fields are named `x_m`/`z_m` to match X-Plane's own `local_x`/`local_z`
  (metres, no geodesy conversion needed). 7 tests.
- [done] `dispatch.rs` — `EnvironmentEvent` wraps every discrete/per-step
  model's own outcome type; `route()` turns each into `ConsumerEffects`
  (`engines`/`sensors`/`structure`/`electrical`/`thermal` `Vec`s, plus a
  `cabin_odor_intensity` pass-through that fits none of the five). Reuses
  the same `EngineEffect` fields across bird/hail/ash/ice so the engine
  owner does not need to know which hazard caused a given flow-capacity/
  efficiency/flameout number. One new coupling introduced here (not
  registered yet, see its own doc comment): lightning's nacelle/radome
  arc-root heat pulse (`ARC_ROOT_J_PER_KA`, GENERIC). `wind_shear`'s
  F-factor/turbulence stay outside `EnvironmentEvent` (continuous kinematic
  inputs the flight model reads directly, not discrete component-damage
  events). 8 tests.
- [done] **Hail rewrite** (priority: "hail must cause real damage with
  real consequences"): `hail.rs` is now stateful (`HailDamageState`,
  persists for the flight, never self-heals) instead of one-shot. Every
  hit adds `energy / area` to that specific part's running total
  (radome/6 windshields/12 LE-slat segments/4 engines/4 nacelles/6
  probes), compared to a per-part `GENERIC` threshold (anchored to the
  same 20 mm/30 mm-at-VMO references as before, scaled by a GENERIC
  15-hits-for-full-damage count) so damage is cumulative, not one-hit.
  New consequences, each read by a real consumer: radome
  `wxr_attenuation_frac` (WXR model) + drag; windshield
  `window_heat_fault`/`visibility_loss_frac`/`leak_area_m2` (window-heat
  system, cabin visibility, pressurisation); wing LE/slat `clmax_delta` +
  `slat_jam_risk_frac` (flight model, slat mechanism); engine
  `fan_damage_frac`/`compressor_efficiency_loss_frac`/`flameout_risk_frac`
  (the last worse at low N1, citing CS-E 790/14 CFR 33.68's low-power
  hail/rain-ingestion certification concern) -- these three field names
  are shared with `bird_strike`'s/`ash`'s engine outputs via `dispatch.rs`;
  probe/antenna and nacelle/cowl damage fractions. `registry.rs` gained 5
  new components, 6 new failures (72:7, 34:4, 71:1, plus enriched
  53:4/56:2/57:2) and 3 new ECAM alerts (`ENV_HAIL_WINDOW_HEAT_FAULT`,
  `ENV_HAIL_SLAT_JAM_RISK`, `ENV_HAIL_ENG_1_FLAMEOUT_RISK`, alongside the
  existing radome/windshield ones). 14 tests, including a stronger/
  longer/higher-airspeed encounter damaging more, damage persisting after
  a zero-intensity "post-storm" step, low-N1 ingestion being worse than
  high-N1 for the same ice mass, and every new consequence changing its
  own field independent of the others.

Next most valuable, not yet started: (a) a matching persistent
`LightningDamageState`-style nacelle/radome arc-root-heating registry
entry (currently only routed in `dispatch.rs`, see its doc comment); (b)
wiring `weather_cells::CellInfluence` into a small end-to-end example/test
that drives all of bird/lightning/hail/ice from one `WeatherCellField`
plus aircraft position, to prove the whole chain composes.

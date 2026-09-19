# Integration — pipeline reference and exact patches

Area: `Area::Integration` (18). Code: `src/deep/integration/` (read that
directory's `PROGRESS.md`/`FAILURES.md` for what is built and what is
known-incomplete). Per `docs/deep/BRIEF.md` hard rule 1, this directory may
not edit any file outside itself — every change another file needs is an
**exact patch** below, not yet applied anywhere.

## 1. Flight-control surface override — the pipeline

FlyByWire's own ported `a380_systems` computes each surface's commanded
position and writes it, normalised 0..1, into named `Var`s
(`flight_controls.rs:258-269`; `handling.rs:325-334` for flap/slat).
`flight_controls.rs`/`handling.rs` then read those same `Var`s and drive
X-Plane's own `sim/flightmodel2/wing/*_deg` datarefs. `deep::flight_controls`
models the *physical* consequence of a fault (jam, runaway, blow-back,
disconnect) as a different angle than what was commanded, computed by
`surface::ControlSurface::angle_rad`/`high_lift::HighLiftSystem::
{inboard,outboard}_angle_rad`. `deep::integration::flight_control_surfaces`
overrides the shared `Var` with the exact normalised value that angle
implies (the precise inverse of `flight_controls.rs`'s own documented
request formulas), so the **unmodified** existing pipeline drives X-Plane
with the physical position.

### Per-surface mapping (37 distinct A380 flight-control surfaces)

| Surface | Count | Shared `Var` name | Body-angle convention | Normalised conversion |
|---|---|---|---|---|
| Aileron | 3/wing x 2 = 6 | `HYD_AIL_{LEFT,RIGHT}_{INWARD,MIDDLE,OUTWARD}_DEFLECTION` | deg, +TE up, -20..+30 | `n = (20 - deg) / 50` |
| Elevator | 2/side x 2 = 4 | `HYD_ELEV_{LEFT,RIGHT}_{INWARD,OUTWARD}_DEFLECTION` | deg, +TE up, -20..+30 | `n = (20 - deg) / 50` |
| Rudder | 2 | `HYD_{UPPER,LOWER}_RUD_DEFLECTION` | deg, FlyByWire body sign, -30..+30 | `n = (30 - deg) / 60` |
| Spoiler | 8/wing x 2 = 16 | `HYD_SPOILER_{1..8}_{LEFT,RIGHT}_DEFLECTION` | deg up, 0..50 | `n = deg / 50` |
| THS | 1 | `HYD_FINAL_THS_DEFLECTION` | deg, +nose up, -2..+10 | written unconverted |
| Flap | 1/side x 2 = 2 (no inboard/outboard split reaches X-Plane) | `{LEFT,RIGHT}_FLAPS_ANGLE` | deg (real travel not yet calibrated by `deep::flight_controls`) | written unconverted |
| Slat | 1/side x 2 = 2 | `{LEFT,RIGHT}_SLATS_ANGLE` | deg (ditto) | written unconverted |

Total: 6 + 4 + 2 + 16 + 1 + 2 + 2 = **33** named `Var` targets covering
**37** physical surfaces (flap/slat's per-side `Var` each stand in for 2
stations, inboard+outboard, since X-Plane's converted `.acf` cannot show
the split — see below).

Every conversion above is round-trip-tested in
`flight_control_surfaces.rs` against `flight_controls.rs`'s own real,
unmodified `aileron_or_elevator_down_deg`/`rudder_right_deg`/
`spoiler_up_deg` functions, so it can never silently drift from that file.

### Feedback into FlyByWire's own PRIM/SEC

Overriding the shared `Var` guarantees X-Plane, and any other reader of
that same `Var` this tick, see the real (possibly jammed) position. Whether
`a380_systems`' own control-law computer reacts depends on whether its own
SFCC/PRIM position monitors read a peer actuator's *published* `Var` as
feedback (as opposed to their own already-computed private state) — if they
do, `Simulation::tick`'s read-then-update-then-write cycle means they see
the override starting the **next** tick, the same one-tick lag
`extra_backend_fbw.rs` already accepts for the reverser force path. This is
not same-tick closure into `a380_systems`' own internals, which would need
a change to the read-only `D:\fbw-aircraft` reference tree and is out of
scope for this crate entirely.

### Known gaps (see `PROGRESS.md` for full detail, not repeated here)

- No aggregate "all 37 surfaces" struct exists in `deep::flight_controls`
  yet (no `mod.rs`/`registry.rs` there as of this writing) — whoever wires
  live instances fills `PhysicalSurfaces` from each one's own angle getter.
- `high_lift::HighLiftSystem::new_generic()` has no cited real flap/slat
  degree range, so `PhysicalSurfaces::flap_deg`/`slat_deg` are `Option<f64>`
  (`None` until a calibrated figure exists).
- Flap/slat asymmetry: see patch 4 below for the `handling.rs` change a
  fuller fix would need (not applied).

## 2. Environment — real X-Plane datarefs used, and their consumers

| Real X-Plane source | Field on `EnvironmentTruth` | Consumer(s) |
|---|---|---|
| `"AMBIENT TEMPERATURE"` `Var` (`sim/weather/aircraft/temperature_ambient_deg_c`) | `sat_c` | `fire_ice::icing`, `sensors::{pitot,tat_probe,aoa_vane}`, `thermal_zones::network::OutsideAir`, `environment::ice_crystal_icing` |
| `"AIRSPEED TRUE"` `Var`, kt->m/s | `tas_ms` | same, plus Mach derivation |
| `sim/weather/aircraft/temperature_leadingedge_deg_c` (raw) | `leading_edge_c` | available; not yet consumed (X-Plane's own real recovery temperature, kept for a future consumer preferring it over the derived one) |
| `sim/weather/aircraft/barometer_current_pas` (raw) | `ambient_pressure_pa` | `fire_ice::icing`, `sensors_adapter::true_total_pressure_pa`, `xp_consequences::dynamic_pressure_pa` |
| `sim/weather/aircraft/precipitation_on_aircraft_ratio` (raw) | `precipitation_on_aircraft_ratio` | `environment_events_adapter::contaminant_from_weather` |
| `XPLMGetWeatherAtLocation` (`crate::xp::weather_at_location`) | `weather: Option<WeatherSample>` (precip rate, **turbulence_alt**, up to 3 cloud layers' type/coverage/base/top) | LWC/MVD derivation, `convective_intensity`, `hail_intensity`, `turbulence_intensity`, `ice_water_content_g_m3` |
| `"PLANE LATITUDE"` `Var` + raw `sim/flightmodel/position/{longitude,elevation}` | (feeds `weather_at_location`'s query point only) | — |
| `"AMBIENT WIND DIRECTION"`/`"AMBIENT WIND VELOCITY"` `Var`s + raw `sim/flightmodel/position/psi` (heading) | (not on `EnvironmentTruth`; consumed directly by `WindShearSampler::step`) | `wind_shear::f_factor` |
| `sim/time/local_date_days` (raw, not yet wired to a reader — see gap below) | — | `environment_events_adapter::month_from_day_of_year` |

**No X-Plane dataref exists for**: liquid water content, droplet size,
runway contamination depth/type, or airborne volcanic ash concentration —
confirmed by grepping this crate's own `sim/weather/*` uses and
`XPLMWeatherInfo_t`'s full member list (`src/xp.rs`'s `WeatherInfoRaw`).
Where a deep model needs one of these, `weather_truth.rs`/
`environment_events_adapter.rs` derive a GENERIC, clearly-labelled stand-in
from what X-Plane *does* give (temperature + cloud type for LWC; the same
convective signature for hail/lightning/ice-crystal intensity;
precipitation + temperature for a best-effort runway-contaminant guess);
volcanic ash has no such derivation at all (out of scope, `PROGRESS.md`).

### Cross-area blocker: `environment::rng::Rng` is private

`src/deep/environment/mod.rs` declares `mod rng;` (no `pub`). But
`lightning::LightningModel::step`, `hail::HailModel::step`,
`wind_shear::TurbulenceModel::step` and `bird_strike::BirdStrikeModel::step`
are all `pub fn` taking `&mut Rng` from that module — a type nameable only
from inside `deep::environment` as currently declared, so none of those
four `pub` step functions can be called from any other area. See patch 3.

## 3. X-Plane consequences — the plugin-force mechanism

`sim/flightmodel/forces/{fside,fnrml,faxil}_plug_acf` (N) and
`{L,M,N}_plug_acf` (N*m) (`developer.x-plane.com/article/movingtheplane`,
X-Plane 10.30+): body axes at the aircraft's CG, positive `faxil` aft
(drag positive, thrust negative), positive `L` right roll, `M` nose up, `N`
yaw right. **X-Plane resets all six to zero every frame**; `deep::
integration::xp_consequences::add_plug_force` does the mandatory
read-add-write. Used for: ice/dent extra drag (`extra_drag_force_n`,
`D = q*S*dCd`), asymmetric-icing/dent yaw (`asymmetric_drag_yaw_moment_nm`),
and a collapsed gear leg still dragging on the ground
(`collapsed_leg_drag_force_n`). A collapsed leg additionally forces its
`sim/flightmodel2/gear/deploy_ratio` element to 0
(`gear_deploy_override`) so X-Plane's own physics stops giving that corner
a ground reaction at all — see `xp_consequences.rs`'s module doc for the
full reasoning and honest caveats.

## Exact patches (none applied; for the lead)

### Patch A — `src/deep/mod.rs`

Every area `deep::integration` depends on (and `integration` itself) needs
declaring. Current file:

```rust
pub mod api;
```

Replace with:

```rust
pub mod api;

pub mod environment;
pub mod fire_ice;
pub mod flight_controls;
pub mod gear_structure;
pub mod integration;
pub mod sensors;
pub mod thermal_zones;
```

(Add any other already-finished areas' modules in the same pass if this is
the first time `deep/mod.rs` is being filled in — `avionics_network`,
`cabin`, `electrical`, `engine_accessories`, `hydraulics`, `pneumatic_ducts`,
`wiring` are not touched by anything in `deep::integration` and are listed
here only so the lead does not have to re-derive the full set from scratch.)

### Patch B — `src/deep/environment/mod.rs`

Current:

```rust
pub mod registry;

mod rng;
```

Replace with:

```rust
pub mod registry;

pub(crate) mod rng;
```

(`pub(crate)`, not `pub`: `Rng` is an internal implementation detail no
other crate should depend on, but every in-crate caller of `lightning`/
`hail`/`wind_shear`/`bird_strike`'s `step` functions needs to name it.)

### Patch C — `src/lib.rs`: struct fields

After the existing field at line 835 (`flight_controls:
flight_controls::FlightControls,`), add:

```rust
    /// Area::Integration: overrides FlyByWire's own actuator `Var`s with
    /// `deep::flight_controls`' physical (possibly jammed/blown-back)
    /// surface positions before `flight_controls`/`handling` publish to
    /// X-Plane (`docs/deep/integration.md`).
    integration_surfaces: deep::integration::flight_control_surfaces::SurfaceOverrideWriter,
    /// Area::Integration: real X-Plane weather/atmosphere, read once per
    /// tick and adapted for `fire_ice`/`sensors`/`environment`/
    /// `thermal_zones` (`docs/deep/integration.md`).
    integration_weather: deep::integration::weather_truth::WeatherTruthReader,
```

### Patch D — `src/lib.rs`: construction

After the existing construction at line 1132-1133:

```rust
        let flight_controls = flight_controls::FlightControls::new(&mut vars, xplm);
        let handling = handling::Handling::new(&mut vars, xplm);
```

add:

```rust
        let integration_surfaces = deep::integration::flight_control_surfaces::SurfaceOverrideWriter::new(&mut vars);
        let integration_weather = deep::integration::weather_truth::WeatherTruthReader::new(&mut vars, xplm);
```

(and add both new names to whatever struct-literal construction follows,
the same way `flight_controls`/`handling` already are).

### Patch E — `src/lib.rs`: the tick call site

Immediately before the existing call at line 1541
(`self.flight_controls.update(&mut self.vars, xplm);`), which the
surrounding comment already documents as running "after the systems tick,
X-Plane's surfaces follow" — exactly where a physical override belongs:

```rust
        crate::perf::lap("tick-after-systems: integration_surfaces");
        // [slot tick-after-systems: integration_surfaces] Area::Integration:
        // override FlyByWire's own actuator Vars with the physical
        // (possibly jammed/blown-back/free-floating) surface positions
        // before flight_controls/handling publish to X-Plane
        // (docs/deep/integration.md). `physical_surfaces()` is a TODO until
        // deep::flight_controls exposes live ControlSurface/HighLiftSystem
        // instances -- until then this call is a documented no-op
        // (PhysicalSurfaces::default() is all-None, see that struct's own
        // doc for why an all-None default, not all-zero, is the only safe
        // placeholder).
        self.integration_surfaces.apply(&mut self.vars, &deep::integration::flight_control_surfaces::PhysicalSurfaces::default());
```

**Do not** wire a non-default `PhysicalSurfaces` in before
`deep::flight_controls` has a live instance for the surfaces being filled
in: every field defaults to `None` (leave FlyByWire's command alone)
specifically so this call is harmless to add now and becomes live
incrementally, one surface at a time, as `deep::flight_controls` grows real
instances — never by writing a placeholder numeric angle.

For the weather-truth read, add anywhere before whichever areas'
(`fire_ice`/`sensors`/`environment`/`thermal_zones`) own per-tick `step`
calls eventually land (none are wired into `Plugin::tick` yet, matching
those areas' own `PROGRESS.md` notes) — e.g. alongside patch E:

```rust
        let environment_truth = self.integration_weather.read(&mut self.vars, xplm);
```

and pass `environment_truth` (or the output of `deep::integration::
fire_ice_adapter::icing_environment(&environment_truth)` etc.) into each
area's `step` once that area's own live instances exist.

### Patch F (optional, not required for the above) — `src/handling.rs` flap/slat asymmetry

To let X-Plane show inboard/outboard flap/slat asymmetry once
`high_lift.rs` calibrates a real travel range, `handling.rs` would need a
second per-side `Var` (e.g. `{LEFT,RIGHT}_FLAPS_OUTBOARD_ANGLE`) and to
stop applying one uniform ratio to all 48 flap elements
(`handling.rs:584-587`); left unapplied since it changes `handling.rs`'s
own X-Plane-surface-writing logic, not just which `Var` value flows into
it, and is not required for this pass's own deliverables.

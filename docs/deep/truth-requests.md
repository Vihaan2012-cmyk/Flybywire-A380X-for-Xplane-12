# What the live areas need that `Truth` does not carry yet

Running list, filled in as each area grows a live system. Nothing here is
invented: each entry is a field an area needs to make a *registered*
failure do what its own `effect` text says. An area that lacks one has
said so rather than substituting a plausible number.

Apply as one pass once every area's live system exists, so the struct
does not change under agents mid-flight.

> **2026-09-20 pass (this file's own requester list, `deep::live`/`deep::plugin`
> only):** everything struck through below now has a real field on `Truth`,
> sourced and documented in `src/deep/plugin.rs`'s own module doc (the
> per-field table there, plus a dedicated `Controls` sub-table). **No area's
> own file was touched** -- consuming these fields (reading
> `truth.controls.*`, `truth.commanded_surfaces`, etc. instead of the
> hardcoded `ControlAssumptions`/interim setters/local commands structs
> areas built to stand in for them) is explicitly left for each area's own
> next pass, so two agents were never editing the same file. See "What an
> area now has to do" at the bottom of this file.

## Contract gap: an area cannot read another area's output

~~`live.rs`'s module doc says areas read the previous frame's values of
anything another area publishes. **There is no mechanism for that** --
`Area::tick(&Truth, &Faults)` has no access to published values. This is
a bug in the contract, not in any area.~~

**Already resolved, no action needed this pass.** `Truth` already carries
`pub published: PublishedFrame` and `Deep::tick` already fills it from the
previous frame's publish before calling any area's `tick` (`live.rs`'s own
`an_area_reads_what_another_published_on_the_previous_frame` test exercises
exactly this). Whoever wrote this paragraph was describing a state of the
contract that predates it; the fix landed before this pass touched the
file. Left visible (struck through, not deleted) so nobody re-does it.

What it unblocks, concretely: `pneumatic_ducts`' overheat detection loops
watch the bay each duct runs through, and those bay temperatures are
`thermal_zones`' output. Without it, a duct leak publishes its heat and
the loop that should trip on it never sees it -- the leak -> overheat ->
isolation chain the whole area exists for is cut in the middle. Every
zone is currently given recovery temperature, which is the right
baseline for an unheated ram-ventilated bay, so false-trip faults work
and real ones do not. (This paragraph is about *area code* reading
`truth.published.get_or(...)`, which is a per-area follow-up, not blocked
by the contract any more.)

## Cockpit control state

The largest single gap. Many registered failures are inert not because
the physics is missing but because nothing tells the model what the crew
selected -- a valve stuck *closed* is invisible when the valve was never
commanded open.

**Sourced this pass as `Truth::controls: Controls`** (`src/deep/live.rs`),
filled in `src/deep/plugin.rs::DeepLayer::truth`. Every field's exact
source (or, for the two with none, why) is in `plugin.rs`'s own
`Controls`-specific sourcing table.

| Field | Unblocks | Status |
|---|---|---|
| ~~Fire pushbutton + agent pushbutton, per engine and per cargo bay~~ | 20 bottle squib failures and the whole suppression chain (`fire_ice`). Only the APU's works today, because its ground discharge is the one path needing no crew action. | **Per engine and APU: done** (`controls.fire_pb_released`, `fire_pb_apu_released`, `fire_agent_pb_pressed`, `fire_agent_pb_apu_pressed`). **Per cargo bay: no real pushbutton or agent exists in this port** (`fire_and_smoke_protection.rs` models 8 engine bottles + 1 APU bottle only) -- not added, left for whoever eventually models cargo bottles to add both the physical bottles and this field together. |
| ~~Wing (x2) and nacelle (x4) anti-ice selection~~ | 12 anti-ice failures (`fire_ice`) and 3 wing anti-ice duct failures (`pneumatic_ducts`). Stuck-*open* works today because it floors flow regardless; stuck-closed and duct leak do not. | **Done.** `controls.wing_anti_ice_selected` (one pushbutton, both wings -- `BUTTON_OVHD_ANTI_ICE_WING_POSITION`, real, single knob) and `controls.nacelle_anti_ice_selected[4]` (`BUTTON_OVHD_ANTI_ICE_ENG_{n}_POSITION`, real, per engine). `fire_ice::live`'s own "held off" interim value still needs its own pass to read these instead. |
| ~~Engine/APU bleed pushbuttons, cross-bleed selector, pack flow control valve positions~~ | `pneumatic_ducts` currently runs on a documented `live::ControlAssumptions` (packs open, WAI off, starters off, cross-bleed open only when the APU is sole source). | **Done except the FCVs themselves.** `controls.engine_bleed_pb_auto[4]`, `apu_bleed_pb_on`, `cross_bleed_selector` (raw 0 SHUT/1 AUTO/2 OPEN), `pack_pb_on[2]` are all real and sourced. There is no per-FCV *valve position* to read (only the PACK pushbutton exists as a crew control in this port; the flow-control valve itself is `pneumatic_ducts`' own modelled component) -- `ControlAssumptions::pack_valve_open` is a physical valve state the area computes from `pack_pb_on` and its own logic, not a second cockpit control, so nothing more belongs on `Truth` for this one. |
| ~~Starter engagement, per engine~~ | 4 start-duct failures (`pneumatic_ducts`); the start ducts sit at ambient today. | **Done.** `controls.starter_engaged[4]`, recomputed in `plugin.rs` from the same real reads (`GENERAL ENG STARTER:n`, `TURB ENG IGNITION SWITCH EX1:n`, `ENGINE_STATE:n`, `ENGINE_TIMER:n`) `physics::engine`'s own `phys_inputs.starter_engaged` already uses -- not a new source, the same formula read twice. |
| ~~Rain removal selection~~ | 2 failures (`fire_ice`); no jet exists to degrade. | **Unsourced -- no real pushbutton in this port.** `controls.rain_removal_selected` stays at `Controls::default()` (off) every tick. Confirmed absent from `fire_and_smoke_protection.rs` and the rest of `a380_systems`; not guessed. |
| ~~Commanded gear door position (nose/wing/body)~~ | 3 door jam failures (`thermal_zones`); the jam latch works but has nothing to diverge from. | **Done, as `[nose, left, right]` (this aircraft's door groups; there is no separate wing/body door dataref, only center/left/right).** `controls.gear_door_commanded_open`, from FlyByWire's own (undamaged) door actuator output `GEAR_DOOR_{CENTER,LEFT,RIGHT}_POSITION` -- the same Vars `handling.rs` already mirrors onto X-Plane's gear animation. |

Also added this pass, not on the original list above but requested by name
elsewhere in the codebase while sourcing the above: `controls.gear_lever_down`
(`GEAR_HANDLE_POSITION`), `controls.parking_brake_on` (`PARK_BRAKE_LEVER_POS`),
`controls.brake_pedal_pos[2]` (X-Plane's own raw
`sim/cockpit2/controls/{left,right}_brake_ratio`, not the post-antiskid
`BRAKE * FORCE FACTOR`), `controls.engine_master_on[4]` (`GENERAL ENG
STARTER:n`, the same read `engine_commands.rs` already calls "master"),
`controls.eng_gen_pb_on[4]`/`apu_gen_pb_on[2]`/`bat_pb_auto[2]`
(`OVHD_ELEC_{ENG_GEN,APU_GEN,BAT}_*_PB_IS_*`),
`controls.ground_spoiler_lever_armed` (`sim/cockpit2/controls/
speedbrake_ratio` through `prim::SimReadings::spoilers_from_xplane`, the
exact function/dataref `Prims::read` already uses), `controls.
apu_master_sw_on`/`apu_start_pb_on` (`OVHD_APU_{MASTER_SW,START}_PB_IS_ON`,
named for `deep::apu::live`'s own doc comment asking for exactly these
two). **Deliberately not added:** a manual galley-shed pushbutton --
`deep::electrical`'s own load management already computes an automatic
`galley_shed_commanded` from the power budget, no real *manual* shed
switch was found in this port, and a second field under a different name
would only invite the two to drift.

## Environment and engine

| Field | Unblocks | Status |
|---|---|---|
| ~~Solar irradiance (or sun elevation)~~ | `ThermalNetwork::step` takes a solar flux and every zone carries a sun-exposure fraction. Passed as 0 today rather than inventing 800 W/m2. | **Elevation done, flux deliberately not.** `Truth::sun_elevation_deg` from `sim/graphics/scenery/sun_pitch_degrees` (real, X-Plane native). Turning elevation into a clear-sky flux needs the atmosphere's own optical depth, which this dataref does not carry -- an area wanting W/m^2 still has to derive it, which is that area's physics, not a `Truth` reading. |
| ~~`engine_hp_port_pressure_pa` / `_temp_k`, per engine~~ | The HP6 branch and its stuck-valve failure, and makes the precooler genuinely work -- HP6 is the hot source it exists to cool. `Truth` carries one bleed port; the model has a real IP8 tap *and* an HP6 valve. HP6 is fed 0 Pa today, which is below FBW's own 15 psi interlock so the valve correctly stays shut, rather than claiming HP6 = IP8, which is wrong by about 200 K. | **Done.** `Truth::engine_hp_port_pressure_pa[4]` / `engine_hp_port_temp_k[4]`, the same `ENGINE_HP_PORT_{PRESSURE_PA,TEMP_K}:n` `engine_bleed_pressure_pa` already reads conditionally, now also read unconditionally alongside it. |
| Cabin / lavatory local temperature | The lavatory fusible link (`fire_ice`). | **Cabin: done, as one representative zone.** `Truth::cabin_temp_k`, from `A32NX_COND_MAIN_DECK_1_TEMP` (real, +273.15). All fifteen zones (`COND_{CKPT,MAIN_DECK_1..8,UPPER_DECK_1..7,CARGO_FWD,CARGO_BULK}_TEMP`) are real and could be added individually if a specific lavatory zone turns out to need its own reading rather than the one representative cabin number `Truth` now carries -- not attempted here since "lavatory" was not resolved to one specific zone key in the time this pass had. |
| `engine_bypass_mdot_kg_s` from the crate's own engine model | Would replace `pneumatic_ducts`' N1-derived estimate of precooler cooling air. | **Not sourced.** `physics::engine::EngineOutputs::bypass_mdot_kg_s` is computed every tick but never published to a Var (only `core_mdot_kg_s`'s sibling, `w24_kg_s`/`w26_kg_s`, reaches a Var, for the bleed-limit calculation). Publishing it needs a change to `engine_commands.rs`, which is outside this pass's three files (`live.rs`/`plugin.rs`/this doc) and inside `src/` proper rather than `src/deep/`, so it was technically in scope but the safer call was to leave an engine-model publishing change to whoever owns `engine_commands.rs`'s own review, rather than editing a hot, heavily-cited file as a side effect of a `Truth`-only pass. Flagging for the next pass rather than doing it silently. |

### Also added this pass (not originally listed, sourced while covering the brief's own priority list)

- **`engine_n2_frac[4]` / `engine_n3_frac[4]`** (`ENGINE_N2:n`/`ENGINE_N3:n`,
  the same pattern as `engine_n1_frac`) -- explicitly requested by
  `hydraulics`' own entry below; done for every consumer, not just
  hydraulics.
- **Per-engine fuel flow**, `Truth::engine_fuel_flow_kg_s[4]`
  (`ENGINE_FUEL_DEMAND_KG_S:n`, real, SI).
- **Commanded Wf, N2, TGT, P30 -- deliberately not added as a "commanded"
  set.** The compiled FADEC bus (`fbw_controllers::BaseEec`) only carries a
  commanded *N1* (`AUTOTHRUST_N1_COMMANDED:n`, already real and already
  readable independently of this request). N2, N3, TGT (this engine's own
  name for EGT) and fuel flow are this physics model's *response* to that
  one commanded number, not independently commanded setpoints in this
  port -- inventing "commanded" versions of them would be exactly the kind
  of fabricated number the brief forbids. If a future engine-model change
  adds real per-parameter setpoints, they belong here as new fields then.
- **Per-engine core mass flow -- not sourced.** `physics::engine::
  EngineOutputs::core_mdot_kg_s` is computed every tick but never
  published to a Var (see `engine_bypass_mdot_kg_s` above; same root
  cause, same reason it was left to `engine_commands.rs`'s own owner).
- **`gpu_plugged_in`** -- done. `Truth::gpu_plugged_in`, from
  `EXT_PWR_AVAIL:{1..4}` (any != 0), the same real, plugin-managed Var
  `efb.rs`'s ground-power control and the cold-start setting already
  write. `deep::electrical::live`'s own `command_contactors`/`capacity_w`
  still hardcode `let gpu_plugged_in = false;` with a comment asking for
  this field -- reading `truth.gpu_plugged_in` there instead is that
  area's own next-pass job, not done here (its file is off limits to this
  pass).
- **Equipment-bay air temperature -- deliberately *not* added to `Truth`.**
  `Truth::published` already carries whatever `thermal_zones::live`
  publishes (bay temperatures included, by the same `get`/`get_or`
  mechanism the "Contract gap" section above describes, which already
  works). A `Truth` field would be a second, parallel way to read the same
  number the published-frame mechanism already exposes, and the two could
  drift if `thermal_zones` ever changes what it publishes without a
  matching edit here. `Truth` is "what the plugin knows from X-Plane/FBW
  directly"; a bay temperature is another area's own computed output, which
  is exactly what `published` exists for. The electrical area (or anyone
  else wanting a bay temperature) should call
  `truth.published.get_or("<thermal_zones' bay Var name>", ambient_fallback)`
  directly.
- **Commanded surface positions -- done**, see the dedicated section below
  (it was originally filed under "From hydraulics and flight controls").

## Failure granularity

`pneumatic_ducts`' registry deliberately registers one id per fault
mechanism per component *class* ("x4 engines"), so arming the duct leak
applies it to all four engine ducts at once. Per-engine arming needs
either per-instance failure ids or a per-instance channel in `Faults`.
Worth a decision before the EFB exposes these to the crew.

*(Untouched this pass -- a `Faults`/registry-shape question, not a `Truth`
sourcing one.)*

## Model gaps noted in passing

- Nothing downstream of the pneumatic ducts actually consumes air: a
  pack duct fills and stops, so at steady state every flow is zero and
  the precooler has nothing to exchange. A pack discharge into cabin
  pressure fixes it but needs a cabin pressure input and is a topology
  change. **`Truth::cabin_pressure_pa` now exists** (see below), which
  removes the "needs a cabin pressure input" half of this; the topology
  change itself is still `pneumatic_ducts`' own work.
- `CARGO_BULK_SMOKE_DETECTED` is published by nobody: `fire_ice` has no
  bulk hold and registers no bulk detector, so publishing it would be a
  hardcoded zero. The bulk alert still reaches its trigger through
  `thermal_zones`' contribution on the bay's smoke concentration. If the
  primary trigger should be live, a bulk detector needs registering.
  *(Untouched -- not a `Truth` field.)*

---

## From hydraulics and flight controls

Each of these has a public setter on the concrete live struct, so the
plugin can feed it before the consolidated `Truth` pass lands.

| Field | Unblocks | Status |
|---|---|---|
| ~~**Commanded surface positions** (flight controls)~~ | The whole point of the area: it models what a surface physically does with what PRIM/SEC asked for. | **Done, as `Truth::commanded_surfaces: CommandedSurfaces`.** The 29 `HYD_*_DEFLECTION` Vars `flight_controls.rs::FlightControls::new` resolves, read independently in `plugin.rs` (so `deep::live` never depends on `flight_controls.rs`'s private `Ids`) and converted to degrees with that file's own public `aileron_or_elevator_down_deg`/`rudder_right_deg`/`spoiler_up_deg`. Read *before* `SurfaceOverrideWriter`/`flight_controls.rs` run this tick (`deep.tick` precedes `self.flight_controls.update` in `lib.rs`), so this is FlyByWire's own commanded position, never last tick's physical output. `deep::flight_controls::live`'s own interim setter is still there; wiring it to read `truth.commanded_surfaces` instead is that area's own next pass. |
| ~~**`engine_n3_frac: [f64; 4]`** (hydraulics)~~ | The engine-driven pumps are driven off the HP spool; `Truth` carries only N1. The plugin already publishes `ENGINE_N3:n` from our own engine model, so this is a copy. | **Done** (see "Also added this pass" above; `engine_n2_frac` came along with it since the same two Vars are written together). |
| **Hydraulic consumer flow demand** (hydraulics) | `deep::flight_controls` knows its own half, but the `Area` trait is tick-then-publish with no inter-area channel. Same root cause as the contract gap above. | **Not this pass.** This is exactly the already-working `truth.published` mechanism (see "Contract gap" above): once `deep::flight_controls::live` publishes its own flow demand under a name, `deep::hydraulics::live` can already read it via `truth.published.get_or(...)` with no new `Truth` field. Both are area-owned files this pass could not touch. |
| **Fire handle position, per engine** (hydraulics) | The registered fire-shutoff-valve failures need a healthy counterpart to act against. | **Covered by `controls.fire_pb_released[4]`** (see "Cockpit control state" above) -- the A380's engine fire pushbutton *is* the fire handle in this port (`FirePushButton`/`FIRE_BUTTON_ENG{n}`); no separate "handle" dataref exists or is needed. |
| **Engine feed fuel flow and temperature** (hydraulics) | The fuel/hydraulic heat exchangers. | **Fuel flow: done** (`Truth::engine_fuel_flow_kg_s`, above). **Feed temperature: not sourced this pass** -- `engine_commands.rs` reads a per-engine `feed_fuel_temp` internally (`vars.read(&e.feed_fuel_temp) + 273.15` feeds the engine physics itself) but this pass did not add it to `Truth`; worth picking up alongside `engine_bypass_mdot_kg_s`/`core_mdot_kg_s` above since all three are already-computed values with no `Truth` field yet. |
| **Ground-spoiler lever armed / go-around selected** (flight controls) | The 2 ground-spoiler-logic failures. | **Lever armed: done** (`controls.ground_spoiler_lever_armed`, above). **Go-around selected: not sourced** -- no real TOGA-detent/go-around discrete was found on `Truth`'s existing engine/FADEC reads in the time this pass had; the closest real signal (`o.thrust_limit_type` indicating TOGA, already written per-engine in `engine_commands.rs` but not published to a Var) would need the same kind of `engine_commands.rs` publishing change flagged above. |
| **`alpha_rad`** (flight controls) | `hinge_moment.rs`'s Ch_alpha term. | **Done, as `Truth::angle_of_attack_deg`** (`sim/flightmodel/position/alpha`, real, X-Plane SDK). Degrees rather than radians, matching every other angle already on `Truth` (`pitch_deg`); convert at the call site (`.to_radians()`) rather than carrying two units for the same reading. |

Gated on "any AC bus live" for want of a named bus: the EHA/EBHA 247XP
and AC-ESS supplies. *(Untouched -- no new bus-naming information
surfaced this pass.)*

**Per-computer PRIM/SEC health: done** (2026-09-22 pass), as
`Truth::prim_healthy`/`sec_healthy: [bool; 3]`. Not a bus-naming problem
after all -- `src/prim.rs` already publishes `A32NX_PRIM_{1,2,3}_HEALTHY`/
`A32NX_SEC_{1,2,3}_HEALTHY` every tick from FlyByWire's own compiled
Simulink `prim_healthy`/`sec_healthy` discrete outputs, which already fold
in both `FAILURE_PRIM`/`FAILURE_SEC` injection and each computer's own
per-index power feed (108PH/247PP/DC_1); `deep::flight_controls::live` had
just never been wired to read it, and collapsed all six computers to one
bit off `ac_bus_volts` instead.

## Catalogue gaps found while wiring

*(All three below are pre-existing observations about area topology/
registries, not `Truth` sourcing gaps -- untouched this pass.)*

- **The green circuit is missing two real pumps.** `hydraulics/topology.rs`'s
  module doc asserts the A380 has no green electric pump, but FlyByWire's
  own `A380Hydraulic` constructs `green_electric_pump_a`/`_b` on AC 1 and
  AC 2 (`a380_systems/src/hydraulic/mod.rs:1772-1775`). So the green
  circuit has two fewer pumps than the aircraft and the registry has no
  failures for them. Pre-existing, in the area's topology rather than the
  live layer.
- **Runaway has no direction.** `ActuatorFaults.runaway_sign` is driven
  to +1 (TE-up / spoiler-extend / rudder-right); the catalogue registers
  one runaway per surface with no direction parameter. An
  opposite-direction hardover needs a second failure id.
- **Flight-control failures are per surface, not per actuator**, so an
  armed magnitude applies to every actuator on that surface. That matches
  the registered effect text (a surface-level jam pins the surface), but
  it means a single-actuator fault cannot be expressed.

## Airframe motion and structure (this pass's own priority items 8-9, not previously listed above by name)

| Field | Status |
|---|---|
| Aircraft mass | **Done.** `Truth::aircraft_mass_kg`, `sim/flightmodel/weight/m_total`, real. Cannot default to zero (an aircraft always weighs something); defaults to the A380-800 OEW, 277,000 kg, the same figure and citation `deep::gear_structure::live::OEW_KG` already uses. |
| Pitch attitude | **Done.** `Truth::pitch_deg`, `sim/flightmodel/position/theta`, real, positive nose up. |
| Groundspeed | **Done.** `Truth::groundspeed_m_s`, `sim/flightmodel/position/groundspeed`, real. |
| Angle of attack | **Done.** `Truth::angle_of_attack_deg` (see `alpha_rad` row above). |
| Radio height | **Done.** `Truth::radio_height_ft`, `sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot`, the same dataref `prim.rs`'s own `h_radio_ft` already reads. |
| Per-leg ground contact | **Done, at the resolution this port's real sensors actually have.** `Truth::leg_on_ground[5]` (`nose, l_wing, r_wing, l_body, r_body`), from `A32NX_LGCIU_1_{NOSE,LEFT,RIGHT}_GEAR_COMPRESSED` (FlyByWire's own primary LGCIU) ANDed with `on_ground`. **Caveat:** this port's LGCIU does not separate wing gear from body gear on the same side (`landing_gear/mod.rs`'s own `left_gear_sensor_compressed` ORs `LEFT`/`WINGLEFT` together) -- so `l_wing`/`l_body` (and `r_wing`/`r_body`) currently read the *same* real sensor rather than two independent ones. A genuinely independent body-gear reading does not exist anywhere in this port; `deep::gear_structure` gets a real signal for each side, not yet a real signal for each of its five legs individually. |
| Touchdown sink speed, per leg | **Done, as a computed edge, not a raw reading.** `Truth::leg_touchdown_sink_speed_ms[5]`, captured in `DeepLayer` (new `prev_leg_on_ground`/`held_sink_speed_ms` state, since an edge needs last frame's value) from `sim/flightmodel/position/local_vy` (real, X-Plane's own vertical speed) at the tick each leg's `leg_on_ground` goes false -> true, held until that leg next lifts off. Because all five legs share the aircraft's one rigid-body vertical speed, legs that touch down in the same instant (both mains, typically) get the identical number; a nose gear that derotates onto the runway later gets whatever the aircraft's vertical speed has decayed to by then, which is real physics, not an approximation layered on top. |
| Cabin pressure | **Done.** `Truth::cabin_pressure_pa`, `environment.ambient_pressure_pa` + FlyByWire's own ARINC 429 `A32NX_PRESS_CPC_1_CABIN_DELTA_PRESSURE` word (psi). Reads CPC 1's channel specifically, not CPC 2 or the manual-mode controller (`PRESS_MAN_*`); if CPC 1 is failed/off while CPC 2 or manual mode is actually flying the cabin, this will read stale. Not resolved this pass -- would need a "which channel is actually in control" read this pass did not chase down. |
| Cabin temperature | **Done, as one representative zone** (see "Cabin / lavatory local temperature" above). |
| Solar irradiance / sun elevation | **Done, elevation only** (see above). |

## What an area now has to do to consume these (for the next pass)

None of the above is wired into any area yet -- every area's `live.rs`
still runs on its own interim default/`ControlAssumptions`/hardcoded value
where one of these fields now exists for real. Concretely, per area:

- **`fire_ice::live`**: read `truth.controls.wing_anti_ice_selected`,
  `nacelle_anti_ice_selected`, `rain_removal_selected` (still off),
  `fire_pb_released`/`fire_agent_pb_pressed`(+APU) instead of the "held
  off"/interim values its own module doc lists.
- **`pneumatic_ducts::live`**: replace `ControlAssumptions`'s hardcoded
  `pack_valve_open`/`wai_selected`/`starter_engaged` with
  `truth.controls.pack_pb_on`, `wing_anti_ice_selected`/
  `nacelle_anti_ice_selected`, `starter_engaged`; read
  `truth.controls.cross_bleed_selector`/`engine_bleed_pb_auto`/
  `apu_bleed_pb_on` for the bleed/cross-bleed logic already gated on an
  APU-sole-source heuristic.
- **`hydraulics::live`**: read `truth.engine_n2_frac`/`engine_n3_frac`
  instead of the N1-interpolated stand-in; read `truth.controls.
  fire_pb_released` for the fire-shutoff-valve failures instead of nothing;
  read `truth.engine_fuel_flow_kg_s` for the fuel/hydraulic heat exchanger
  (feed temperature is still not on `Truth`, see above).
- **`flight_controls::live`**: read `truth.commanded_surfaces` instead of
  its own interim setter for PRIM/SEC's commanded position; read
  `truth.controls.ground_spoiler_lever_armed` for the ground-spoiler-logic
  failures (go-around selected is still not on `Truth`); read
  `truth.angle_of_attack_deg` for `hinge_moment.rs`'s Ch_alpha term.
- **`gear_structure::live`**: read `truth.aircraft_mass_kg`,
  `truth.pitch_deg`, `truth.groundspeed_m_s`, `truth.leg_on_ground`,
  `truth.leg_touchdown_sink_speed_ms`, `truth.controls.gear_lever_down`/
  `parking_brake_on`/`brake_pedal_pos`/`gear_door_commanded_open` instead
  of its own `GearCommands` interim struct (side load is still not on
  `Truth` -- no real per-leg lateral-load source was found).
- **`electrical::live`**: read `truth.gpu_plugged_in` instead of the
  hardcoded `let gpu_plugged_in = false;` in `command_contactors`/
  `capacity_w`; read `truth.controls.eng_gen_pb_on`/`apu_gen_pb_on`/
  `bat_pb_auto` if the pushbutton positions themselves (as opposed to bus
  potentials, already read) turn out to matter to that area's own logic.
- **`apu::live`**: read `truth.controls.apu_master_sw_on`/
  `apu_start_pb_on` instead of inferring both from the one `apu_running`
  bit its own module doc says it currently has to.
- **`engine_accessories::live`**: read `truth.engine_n2_frac`/
  `engine_n3_frac` and `truth.controls.engine_master_on`/
  `starter_engaged` wherever it currently has no crew-selection input.
- **`thermal_zones::live`**: read `truth.controls.gear_door_commanded_open`
  for the 3 door jam failures; read `truth.cabin_pressure_pa`/
  `cabin_temp_k` if the pressurisation topology change noted above gets
  picked up.
- **Any area wanting an equipment-bay temperature**: read
  `truth.published.get_or("<thermal_zones' own bay Var name>", fallback)`
  directly -- deliberately not a `Truth` field (see above).

None of this consuming work was done in this pass (`src/deep/<area>/` was
off limits); this section exists so whichever agent picks up each area
next does not have to re-derive which `Truth` field replaces which
hardcoded value.

# Ice and rain protection, and probe heat (ATA 30, ATA 34 probe heat)

Scope: wing and engine anti-ice, probe/window heat, X-Plane icing accretion
effects, rain repellent and wipers. Code: `src/lights.rs` (the output wiring
added by this pass), `src/aspects.rs` (FlyByWire's own wing/engine anti-ice
mirroring, pre-existing), `src/physics/adirs.rs` (ADIRS-owned: the pitot/
static probe icing/heat model, pre-existing — read here, not edited).

## Audit: what was already here

FlyByWire's own A380X systems (`a380_systems`/`systems_wasm`, read via
`fbw-aircraft`, not modified) already computes real ice-protection state:

- `PNEU_WING_ANTI_ICE_SYSTEM_ON`/`_HAS_FAULT`: FBW's own pneumatic wing
  anti-ice valve/fault logic (`a380_systems/src/pneumatic/wing_anti_ice.rs`).
- `ENG ANTI ICE:1..4` (MSFS-style var `aspects.rs` mirrors from the
  overhead pushbuttons): FBW's own per-engine anti-ice state.
- `physics/adirs.rs` (a separate workstream, ADIRS-owned, **not edited by
  this pass**) already has a genuinely causal pitot/static probe-icing
  model: real Airbus AUTO probe-heat logic (heat on whenever airborne or
  any engine running, not a temperature switch), ice accretion at
  `PROBE_ICE_ACCRETION_KG_S` while unheated and in icing conditions,
  blockage at `PROBE_BLOCK_MASS_KG`, and a frozen-pressure model that gives
  the classic "blocked pitot keeps the last airspeed" failure. It publishes
  `PROBE_HEAT_LOAD_W:1..3` (per ADIRU 1/2/3 = pilot/copilot/standby) as a
  documented shared contract ("the electrical workstream reads this as a
  bus consumer") and `ADIRS_STUDY_<n>_PITOT_BLOCKED`/`_PROBE_HEAT_W` for the
  Study panel. This is thorough and was left alone.
- `src/lights.rs` already had window heat and wipers (ATA 34 ICE-002):
  `ice_window_heat_on` and `wiper_speed_switch`, correctly gated by the
  `WipersLeft`/`WipersRIght` circuits, following the `GatedSwitch` pattern
  that lets a cockpit click still be told apart from this module's own
  power-loss forcing.

## The gap: the switches never reached X-Plane's own icing model

`src/aspects.rs` mirrors FlyByWire's wing/engine anti-ice pushbuttons onto
MSFS-style simulator variable names — `Variable::aircraft("ENG ANTI ICE", n)`
and `Variable::aircraft("STRUCTURAL DEICE SWITCH", 0)`. That mirroring
mechanism works by looking the name up in `lib.rs`'s `mapping()` table to
find the real X-Plane dataref underneath (`Vars::input_for`); everything
else that goes through it (`AIRSPEED INDICATED`, `SIM ON GROUND`, ...) has an
entry there. **"ENG ANTI ICE" and "STRUCTURAL DEICE SWITCH" do not** — grep
across `lib.rs` confirms no `deice`/`anti ice`/`anti_ice` mapping entry
exists at all. Unmapped simulator variables still work (`Vars::add`/
`publish`), but only as an `fbw/<name>` dataref nobody else reads: FBW's own
overhead ENG/WING ANTI ICE pushbuttons moved a value with **zero effect on
X-Plane's own icing physics** (`sim/flightmodel/failures/frm_ice`,
`inlet_ice_per_engine`) — the causal chain broke at the very last step.

X-Plane 12 actually ships exactly the right native switches
(`Resources/plugins/DataRefs.txt`, verified against the local X-Plane 12
install per `docs/team.md`):

| Function | Real X-Plane dataref |
|---|---|
| Engine cowl/inlet anti-ice, per engine | `sim/cockpit2/ice/cowling_thermal_anti_ice_per_engine[16]` |
| Wing anti-ice (hot bleed air — the A380's actual system, not electric boots) | `sim/cockpit2/ice/ice_surface_hot_bleed_air_left_on`/`_right_on` |
| AoA vane heat, pilot/copilot/standby | `sim/cockpit2/ice/ice_AOA_heat_on`/`_copilot`/`_stby` |
| TAT probe heat, pilot/copilot/standby | `sim/cockpit2/ice/ice_TAT_heat_on`/`_copilot`/`_stby` |
| Pitot heat, pilot/copilot/standby | `sim/cockpit2/ice/ice_pitot_heat_on_pilot`/`_copilot`/`_standby` |
| Static port heat, pilot/copilot/standby | `sim/cockpit2/ice/ice_static_heat_on_pilot`/`_copilot`/`_standby` |
| Rain repellent, left/right | `sim/cockpit2/switches/rain_repellent_switch[2]` |

X-Plane's own airframe/engine/probe icing simulation reacts to these
natively (accretion when off and cold/wet, no accretion/shedding when on) —
this is exactly the case where letting X-Plane's own physics do the work
(rather than reimplementing wing-ice aerodynamics or engine inlet blockage
ourselves) is the right, non-duplicative causal design.

## Fix: `src/lights.rs` forwards the already-computed state

`Lights` (already the module owning window heat/wipers) gained:

- `eng_cowl_anti_ice`, `wing_hot_bleed_left`/`_right`, `aoa_heat`/`tat_heat`/
  `pitot_heat`/`static_heat` (`[Option<DataRef>; 3]`, pilot/copilot/standby),
  `rain_repellent`: the real X-Plane datarefs above, found once in `new()`.
- `eng_anti_ice_src` (`ENG ANTI ICE:1..4`), `wing_anti_ice_src`
  (`PNEU_WING_ANTI_ICE_SYSTEM_ON`), `probe_heat_src`
  (`PROBE_HEAT_LOAD_W:1..3`), `rain_repellent_src`
  (`RAIN_REPELLENT_LEFT_ON`/`_RIGHT_ON`): the already-computed FlyByWire/
  ADIRS state, read by `vars.get`/`vars.read` — the same `Vars` slots
  `aspects.rs`/`adirs.rs` already write, not new state.
- `Lights::update` (`src/lights.rs`) forwards every tick: engine anti-ice
  and rain repellent per-engine/per-side into their array datarefs
  (`Xplm::set_vi_at`); wing anti-ice to both hot-bleed-air switches;
  probe heat (`load_w > 0.`) to AoA/TAT/pitot/static heat for each of the 3
  ADIRU channels.

This is a one-way read of `adirs.rs`'s own published, documented shared
contract (`PROBE_HEAT_LOAD_W:n` — its own comment already invites "the
electrical workstream reads this as a bus consumer"); ice protection is
just another reader of the same contract, and `physics/adirs.rs` itself
was not touched, per the ADIRS ownership boundary.

Two small pure predicates are unit-tested directly (`src/lights.rs` tests):
`on_state` (FlyByWire's own "nonzero, not just 1.0, is on" convention) and
`probe_heat_on` (watts to boolean). The dataref-writing side needs a real
`Xplm`, same as the rest of `Lights`, so it is exercised live rather than
unit-tested (see Remaining gaps).

## Rain repellent: FBW's own source marks it inoperative

`A380_Cockpit_Behavior.xml`'s `PUSH_OVHD_RAINRPLNTL`/`RAINRPNLTR` set
`L:A32NX_RAIN_REPELLENT_LEFT_ON`/`_RIGHT_ON` while held, with FBW's own
tooltip text reading "Dispense rain repellent (Inop.)" — no system in
FlyByWire ever reacts to that LVar, and (unlike the probes/window heat
button, which cites `A32NX_ELEC_AC_2_BUS_IS_POWERED`) no `SEQ_POWERED`
condition or circuit is given for these two buttons at all, so there is no
real bus to gate this on without inventing one. `src/lights.rs` now forwards
the two hold-simvars straight to X-Plane's own
`sim/cockpit2/switches/rain_repellent_switch[0|1]`, the real "rain
repellent other than wipers" system X-Plane itself simulates — turning
FBW's own admittedly-inert placeholder into a working one using X-Plane's
physics, deliberately left ungated because FBW's own source gives nothing
real to gate it on (not a shortcut of ours; a fact about the upstream
source, stated here rather than silently assumed away).

## Remaining gaps (ranked)

1. **Rain repellent click-side wiring is unverified.** This pass wires the
   *consumption* of `L:A32NX_RAIN_REPELLENT_LEFT_ON`/`_RIGHT_ON`; whether the
   converted X-Plane cockpit's `PUSH_OVHD_RAINRPLNTL`/`RAINRPNLTR`
   manipulators actually set that LVar on click-and-hold is cockpit control
   bindings territory (out of this workstream's file list) and was not
   checked live. If they don't yet, the button will still do nothing end to
   end despite this fix; worth a quick `dref.mjs` check of
   `fbw/RAIN_REPELLENT_LEFT_ON` while holding the button.
2. **AoA vane icing has no accretion model in `adirs.rs`.** `adirs.rs`
   already applies `AOA_VANE_UPWASH_FACTOR` to the sensed AoA but never
   checks `heat_on`/icing conditions for the vane the way it does for the
   pitot/static ports — an unheated AoA vane does not currently freeze the
   AoA reading. Since `physics/adirs.rs` is ADIRS-owned, this needs to be
   done there: mirror the existing `pitot_ice_kg`/`static_ice_kg` accretion-
   while-unheated-and-icing / reset-while-heated pattern (adirs.rs:1063-1134)
   for a third `aoa_ice_kg`, freezing `adr_aoa_deg` at its last value past
   some threshold instead of always recomputing
   `t.alpha_deg * AOA_VANE_UPWASH_FACTOR`. This report is the description of
   that needed change, per the coordination rule.
3. **`ENG ANTI ICE`/`STRUCTURAL DEICE SWITCH` are still unmapped in
   `lib.rs`'s `mapping()`.** Not a live bug any more (their value is now
   read by `lights.rs` directly through the same `Vars` slot, bypassing the
   mapping table entirely), but any other future consumer that assumes
   `mapping()` is the single source of truth for "is this a real X-Plane
   dataref" would still be misled. Left as-is rather than edited, since
   `lib.rs` edits are asked to stay minimal and no other agent's workstream
   currently depends on it being mapped there.
4. **Nacelle valve bleed-extraction cost** is FBW's own pneumatic system
   plus `physics/air.rs`'s `EngineBleedLoads` (air/bleed-owned, out of this
   workstream's files) — audited, not re-verified live under this pass's
   time budget, but the wiring (`fadec.rs` already reads
   `sim/cockpit2/ice/ice_inlet_heat_on_per_engine` into
   `is_anti_ice_active`) looks intact.
5. **Windshield fog/ice** relies entirely on X-Plane's native rendering
   reacting to `ice_window_heat_on` (already wired, pre-existing) — not
   independently re-verified live under this pass's time budget.

## Tests

`src/lights.rs`:
- `probe_heat_on_reads_any_nonzero_watt_load_as_on`
- `on_state_matches_flybywires_nonzero_convention`

(plus the pre-existing `GatedSwitch`/circuit-resolution tests, unaffected).

# Cockpit controls, Study panel and candidate circuit breakers — stage 2 analysis

Read-only analysis. No source files were changed. Scope: cockpit_bindings.txt (the converter's
behaviour-resolution report for the whole A380X cockpit), the converter's `src/behaviour/`
resolver, the MSFS model behaviour XML, and the plugin's `src/study/` module.

`docs/cl650-reference.md` does not exist yet (checked: `D:\A380\fbw-xp-systems\docs\` has no such
file), so the CL650 comparison in part 2 uses general, well-known Hot Start CL650 study-level
features (CB panel, failures page, ground services page, aircraft-state persistence) rather than
citing a document. Nothing about the real A380 or CL650 below is invented: where I could not
confirm a number or name from FBW source, the MSFS package, X-Plane, or the CL650's published
feature list, I said so instead of guessing.

This pass is evidence-driven and not a line-by-line audit of all 1,545 lines of
`cockpit_bindings.txt` (472 of them are "runs the control's original MSFS code in SASL" and 204
are unresolved `PUSH_KBD_*` keyboard keys for the EFB, which is out of scope per `docs/team.md`).
It groups the recurring failure patterns the file exposes, with file:line/line-content evidence
for every group, and flags the individual controls most worth fixing first.

## 1. Summary table

Sorted by realism impact (5 = highest), then by effort (S < M < L < XL).

| ID | System (ATA) | What's wrong | Impact | Effort |
|---|---|---|---|---|
| CTRL-002 | Autopilot/FCU (22) | Altitude knob and increment selector drive X-Plane's own autopilot altitude, not an FBW variable | 5 | M |
| CTRL-009 | Fire protection (26) | Fire guard and discharge buttons' path into FBW's fire logic is unverified from the resolver's output | 5 | S (verify) / M (fix) |
| CB-001 | Electrical (24) | The only 10 clickable CB-like buttons in the model toggle variables nothing reads | 4 | S |
| CTRL-006 | Pressurisation (21) | Landing-elevation and manual cabin V/S controls unresolved | 4 | M |
| CTRL-007 | Landing gear (32) | Parking brake lever left unresolved by the same resolver that handles every other lever | 4 | S |
| STUDY-001 | Study panel | No Circuit Breakers page exists | 4 | XL |
| STUDY-002 | Study panel | A real failure-injection system (`failures.rs`, 776 lines) exists but has no Study UI | 4 | M |
| LGT-001 | Exterior lights (33) | Beacon/strobe/nav/logo/wing/landing/taxi switches run X-Plane's native light system, no FBW bus gating | 3 | M |
| LGT-002 | Interior lighting (33) | Panel/pedestal/glareshield/ambient/reading dimming knobs run generic SASL code tied to no FBW variable | 3 | M |
| CTRL-003 | EFIS control panel (34) | Baro-reference knobs partly unresolved, one fires X-Plane's native barometer command instead of FBW's | 3 | M |
| CTRL-004 | Surveillance (34) | TCAS/WXR/XPDR/GS mode button backlights never get their `#INDICATOR_POWERED#` code | 3 | S/M |
| CTRL-005 | Overhead misc (21/23/28/52) | 13 overhead pushbuttons have no click code at all, even in the original MSFS model | 3 | L |
| STUDY-005 | Study panel | 6 system pages (Bleed, Air Cond, Pressurisation, Gear/Brakes, Air Data, Fire) are plain field lists, not schematics like Electrical/Hydraulics/Engine | 3 | L |
| STUDY-003 | Study panel | No Ground Services page (GPU, pushback, chocks, air start, fuel) | 3 | M |
| STUDY-004 | Study panel | No aircraft-state / persistence page; nothing in the plugin saves state across sessions | 3 | L |
| CB-002 | Electrical (24) | No overhead circuit-breaker panel modeled in the aircraft at all; only generic consumer names exist to build candidates from | 3 | XL |
| CTRL-008 | Audio/radar/comms | Audio selector, radar ground-clutter/multiscan switches, DLS selector unresolved | 2 | S |
| LGT-003 | EFIS backlights (34) | EFIS CS/FO BLANK and TAXI button backlight codes never given | 2 | S |
| LGT-004 | Cabin signage (33) | EMER EXIT LT / NO SMOKING 3-way overhead switches unresolved | 2 | S |
| STUDY-006 | Study panel | No maintenance page (deferred items, component wear) | 2 | L |
| CTRL-001 | Lighting knobs (33) | LH/RH sliding lighting-knob push-detent unresolved | 1 | S |
| CTRL-010 | Misc cabin (25) | Coffee-cup and hat-switch controls unresolved (cosmetic/camera, low stakes) | 1 | S |

## 2. Per-system sections

### 2.1 Autopilot / FCU (ATA 22) — CTRL-002

**Evidence:** `cockpit_bindings.txt`:
```
KNOB_AUTOPILOT_ALT: unresolved (ASOBO_GT_Knob_Infinite_PushPull), keeps fbw/cockpit/KNOB_AUTOPILOT_ALT:
  MSFS event K:AP_ALT_VAR_SET_ENGLISH has no FBW variable
KNOB_AUTOPILOT_SELECTOR_ALT: unresolved (ASOBO_AUTOPILOT_Switch_Altitude_Increment_Template), keeps
  fbw/cockpit/KNOB_AUTOPILOT_SELECTOR_ALT: Asobo template ...: acts on MSFS's own systems, not an FBW variable
KNOB_FCU_ALT: unresolved ...: MSFS event K:AP_ALT_VAR_SET_ENGLISH has no FBW variable
KNOB_FCU_SELECTOR_ALT: unresolved ...: acts on MSFS's own systems, not an FBW variable
```
Also confirmed in the model XML (`A380_COCKPIT.xml`, the AUTOPILOT component): the altitude knob
uses `ASOBO_AUTOPILOT_Switch_Altitude_Increment_Template`, an Asobo stock template, alongside FBW's
own `FBW_Airbus_FCU_Altitude_Knob` for the turn itself.

**What the real aircraft does:** the FCU's altitude selector and its 100 ft/1,000 ft increment
switch are read by the FMGC/FCU logic that FBW's own systems implement (target altitude, managed
vs. selected mode). This is exactly the class of gap the brief calls out: "places where X-Plane's
own physics or systems decide something FBW's model should." Right now, X-Plane's native
autopilot altitude bug and increment state substitute for FBW's own target-altitude variable.

**Proposal:** give the altitude knob and its increment selector real `fbw/` variables (as the
turn action `FBW_Airbus_FCU_Altitude_Knob` already gets via `PUSH_AUTOPILOT_ALT`/`PUSH_KNOB_FCU_ALT`
push-pull), and stop emitting `K:AP_ALT_VAR_SET_ENGLISH` for the plain rotate. Route the increment
selector through an `fbw/` toggle instead of `ASOBO_AUTOPILOT_Switch_Altitude_Increment_Template`.

**Impact 5, Effort M.**

### 2.2 Fire protection (ATA 26) — CTRL-009

**Evidence:** every fire-guard and fire-button node resolves the same way:
```
A380X_OVHD_APU_FIRE_GUARD: click command running the control's MSFS code in SASL
A380X_OVHD_ENG1_FIRE_GUARD: click command running the control's MSFS code in SASL
PUSH_OVHD_FIRE_ENG1: click command running the control's MSFS code in SASL
PUSH_OVHD_FIRE_ENG1_AGENT1: click command running the control's MSFS code in SASL
PUSH_OVHD_FIRE_ENG1_AGENT2: click command running the control's MSFS code in SASL
PUSH_OVHD_FIRE_APU: click command running the control's MSFS code in SASL
PUSH_OVHD_FIRE_AGENT: click command running the control's MSFS code in SASL
```
Unlike the FCU/HDG/altitude knobs elsewhere in the same file, which show which `fbw/hevent/...`
their SASL script fires when it does reach FBW (e.g. `KNOB_FCU_HDG: ... (fires
fbw/hevent/A320_Neo_FCU_HDG_DEC_HEADING, ...)`), none of the fire-guard or fire-button lines carry
that annotation. The one fire-panel control that is confirmed reaching FBW is the test button:
```
PUSH_FIRE_ENG1_TEST: holds fbw/A32NX_OVHD_FIRE_TEST_PB_IS_PRESSED at 1 while pressed (0 released)
```
FBW's own fire logic exists and is real (`D:\fbw-aircraft\fbw-a380x\src\wasm\systems\a380_systems\src\fire_and_smoke_protection.rs`).

**What the real aircraft does:** the fire push buttons arm/disarm the fire-extinguishing
squibs and their guards prevent inadvertent operation; both feed the FWS and the fire-and-smoke
protection logic that decides bottle discharge, engine shutdown interlocks, and ECAM logic.

**Proposal:** confirm (by reading the RPN each `Click::Script` carries, not just the resolver's
one-line description) which `L:` variables the fire guard/push/agent buttons actually write, and
check those names against what `fire_and_smoke_protection.rs` reads. If they line up (the A380X
already uses many `L:A32NX_*` names that match FBW's own dataref names 1:1), this may already work
and only needs a note in the resolver to stop reporting it as opaque; if they don't line up, give
these controls the same explicit `fbw/` binding the test button already has.

**Impact 5, Effort S to verify / M to fix.**

### 2.3 Cabin pressurisation (ATA 21) — CTRL-006

**Evidence:**
```
KNOB_OVHD_CABINPRESS_LDGELEV: unresolved (ASOBO_GT_MouseRect), keeps fbw/cockpit/KNOB_OVHD_CABINPRESS_LDGELEV:
  raw mouse rectangle callback (event-dispatch code) not translated
SWITCH_OVHD_CABINPRESS_MANVSCTL: unresolved (ASOBO_GT_Interaction_WheelAndContinuousLeft), keeps
  fbw/cockpit/SWITCH_OVHD_CABINPRESS_MANVSCTL: changes no FBW variable (MSFS-local state only)
```
**What the real aircraft does:** the landing-elevation selector feeds the cabin pressure
controller's scheduled landing-field elevation, and the manual V/S control lets the crew drive
cabin rate directly if the automatic controller is not used. Both are safety-relevant (cabin
altitude/rate).

**Proposal:** the `ASOBO_GT_MouseRect` callback needs translating into a proper up/down step (the
resolver already handles similar continuous knobs elsewhere, e.g. `KNOB_OVHD_AIRCOND_BULK` at
`Power:0 to 300 by 5`); give both controls real `fbw/` variables feeding the FBW cabin-pressure
controller.

**Impact 4, Effort M.**

### 2.4 Parking brake (ATA 32) — CTRL-007

**Evidence:**
```
lever_parking_brake: unresolved (ASOBO_GT_Switch_Code), keeps fbw/cockpit/lever_parking_brake:
  reads M:Event
```
Every other lever in the file (`flaps_lever`, `lever_landing_gear`, `lever_speed_brake`,
`throttle_lever_1..4`) resolves cleanly as `lever, keeps fbw/cockpit/<name>`. The parking brake is
the one lever left as a raw `M:Event` read, and `rig.rs` (the converter's exterior-animation
generator) already depends on the parking-brake state for ground-equipment visibility logic
(`PBRK > 0.5`, `rig.rs:398`), so a broken/unresolved parking-brake lever also breaks that.

**Proposal:** resolve it the same way the other levers are resolved; it is a state lever, not a
momentary control, so it should not need special-casing once the `M:Event` read is replaced.

**Impact 4, Effort S.**

### 2.5 Exterior lighting (ATA 33) — LGT-001

**Evidence:**
```
SWITCH_OVHD_EXTLT_BEACON: unresolved (ASOBO_LIGHTING_Switch_Light_Beacon_Template), keeps
  fbw/cockpit/SWITCH_OVHD_EXTLT_BEACON: Asobo template ...: acts on MSFS's own systems, not an FBW variable
SWITCH_OVHD_EXTLT_LANDL / _NOSE: ... ASOBO_LIGHTING_Switch_Light_Landing_Template: acts on MSFS's own systems ...
SWITCH_OVHD_EXTLT_LOGO / _NAVLOGO / _RWY / _STROBE / _WING: same pattern, one line each
```
And in the plugin itself, a test that documents the same gap on the handling side:
`D:\A380\fbw-xp-systems\src\handling\aspects.rs:783`:
```rust
assert!(!a.handle(&mut v, "TOGGLE_BEACON_LIGHTS", 0, 0.));
```
i.e. the handling-aspects module explicitly does *not* claim the beacon-light command; it is left
for X-Plane's native light system. `systems.cfg` (the MSFS package) carries real per-light circuit
definitions that this bypasses entirely, e.g. (`systems.cfg:15-30`):
```
circuit.15 = Type:CIRCUIT_LIGHT_BEACON:1#Connections:bus.2#Power:6, 8, 20.0#Name:Beacon_Light
circuit.23 = Type:CIRCUIT_LIGHT_STROBE:1#Connections:bus.3#Power:20, 25, 20.0#Name:Strobe_Light_1
circuit.27 = Type:CIRCUIT_LIGHT_WING:1#Connections:bus.2#Power:10, 15, 20.0#Name:Wing_Light
circuit.29 = Type:CIRCUIT_LIGHT_LOGO:1#Connections:bus.2#Power:10, 15, 20.0#Name:Logo_Light
```
**What the real aircraft does:** each exterior light is powered from a specific DC/AC bus through
a relay; losing that bus (or pulling the light's breaker) puts the light out regardless of switch
position.

**Proposal:** a small exterior-lights module (comparable in shape to `sensors.rs` or `doors.rs`)
that reads the switch position plus the bus-power state FBW's own electrical model already
computes, and only then drives the X-Plane light dataref/command — so a DC bus loss actually
extinguishes the light. Bus names to key off: `bus.2`/`bus.3` in `systems.cfg`, or the FBW
Rust-side bus objects in `alternating_current.rs`/`direct_current.rs`.

**Impact 3, Effort M.**

### 2.6 Interior/panel lighting and dimming (ATA 33) — LGT-002

**Evidence:**
```
LIGHTING_Knob_Ambient: up/down commands running the control's MSFS code in SASL
LIGHTING_Knob_Glareshield / _1 / _2 / _3 / _4: up/down commands running the control's MSFS code in SASL
LIGHTING_Knob_Panel: up/down commands running the control's MSFS code in SASL
LIGHTING_Knob_Pedestal: up/down commands running the control's MSFS code in SASL
KNOB_OVHD_INTLT_BRT: up/down commands running the control's MSFS code in SASL
KNOB_OVHD_READINGLTL / READINGLTR: up/down commands running the control's MSFS code in SASL
```
None of these carry the `(fires fbw/hevent/...)` annotation that the resolver adds when a SASL
script's RPN does reach an `fbw/` variable (contrast with `KNOB_FCU_HDG`, which does show that
annotation) — so these dimmers are not confirmed to touch anything FBW's systems can see. The
backlight emissive for the FCU screen also depends on the MSFS-native light potentiometer array,
not an FBW brightness variable:
```
SCREEN_BACKLIGHT_AUTOPILOT (): emissive code unresolved: RPN word "5*" [(A:LIGHT POTENTIOMETER:87,
  Percent over 100) (L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED, Bool) (L:A32NX_ELEC_DC_2_BUS_IS_POWERE]
```
`systems.cfg` shows the real consumer circuits this bypasses: `circuit.31/61/62/63`
(`CIRCUIT_LIGHT_PANEL`, 4 separate panel-light circuits), `circuit.34` (`CIRCUIT_LIGHT_PEDESTAL`),
`circuit.58-60` (`CIRCUIT_LIGHT_GLARESHIELD`, 3 circuits).

**What the real aircraft does:** panel, integral, pedestal, glareshield and dome/flood lighting
are each their own rheostat-controlled circuit, bus-powered, independently dimmable.

**Proposal:** give each dimmer a real `fbw/LIGHT_POTENTIOMETER_n`-style variable (two already
exist for the EFIS console lights — `SWITCH_EFIS_CS_CONSOLE`/`SWITCH_EFIS_FO_CONSOLE` step
`fbw/LIGHT_POTENTIOMETER_8`/`_9` cleanly, so the pattern is proven), and drive the cockpit texture
emissives and OpenGL panel-lighting overlay from those instead of `A:LIGHT POTENTIOMETER`.

**Impact 3, Effort M.**

### 2.7 EFIS control panel baro (ATA 34) — CTRL-003 / LGT-003

**Evidence:**
```
KNOB_EFIS_CS_BARO1: fires sim/instruments/barometer_up / sim/instruments/barometer_down
KNOB_EFIS_CS_BARO2: unresolved (ASOBO_GT_Switch_Code), keeps fbw/cockpit/KNOB_EFIS_CS_BARO2:
  changes no FBW variable (MSFS-local state only)
KNOB_EFIS_FO_BARO1: unresolved (ASOBO_GT_Knob_Infinite_PushPull), keeps fbw/cockpit/KNOB_EFIS_FO_BARO1:
  changes no FBW variable (MSFS-local state only)
KNOB_EFIS_FO_BARO2: unresolved ... changes no FBW variable (MSFS-local state only)
KNOB_EFIS_CS_BARO1 (FBW_Airbus_FCU_Baro_Knob_SubTemplate): emissive code unresolved: template
  parameter never given: #EMISSIVE_CODE# ...
PUSH_EFIS_CS_BLANK (FBW_A380X_BacklightIndicator_Button_Template): emissive code unresolved:
  template parameter never given: #INDICATOR_CODE# s0
PUSH_EFIS_CS_TAXI (FBW_A380X_BacklightIndicator_Button_Template): same
```
`KNOB_EFIS_CS_BARO1` firing `sim/instruments/barometer_up`/`_down` is X-Plane's own barometric
setting command, not FBW's QNH/QFE/STD reference — another instance of X-Plane's own systems
deciding something FBW's model should.

**What the real aircraft does:** the CP/FO baro-reference knob sets the altimeter's QNH/QFE
reference that FBW's own ADIRS/air-data and PFD altitude tape read; hPa/inHg selection is a
separate switch (`KNOB_EFIS_CS_BARO2`/`_FO_BARO2`).

**Proposal:** give the baro-set knob an `fbw/` altimeter-setting variable and stop firing
X-Plane's native barometer command; wire `#EMISSIVE_CODE#`/`#INDICATOR_CODE#` for the baro-knob
and BLANK/TAXI backlight templates so their legends actually light when powered.

**Impact 3, Effort M (knob) / S (backlights).**

### 2.8 Surveillance panel backlights (ATA 34) — CTRL-004

**Evidence:** every SURV mode button has the same unresolved indicator code:
```
PUSH_SURV_GS_MODE: unresolved (ASOBO_GT_Push_Button), keeps fbw/cockpit/PUSH_SURV_GS_MODE:
  template parameter never given: #INDICATOR_POWERED# if{
PUSH_SURV_GS_MODE_SEQ1 (FBW_A380X_BacklightIndicator_Button_Template): emissive code unresolved:
  template parameter never given: #INDICATOR_POWERED# s1
PUSH_SURV_TCAS_ABV / _BLW / _TAONLY: same, plus their _SEQ1 backlight
PUSH_SURV_WXR_TAWS_SYS1 / SYS2: same, plus their _SEQ1 backlight
PUSH_SURV_XPDR_TCAS_SYS1 / SYS2: same, plus their _SEQ1 backlight
```
8 buttons and their 8 backlight indicators (16 unresolved lines total) all miss the same template
parameter. This looks like a single template-instantiation bug in the model or the converter's
template-parameter substitution, not 8 independent ones.

**What the real aircraft does:** these buttons select TCAS TA/RA above/below normal and TAWS/WXR
system selection; their green legends light to show which mode/system is selected, amber for
fault, and go dark when unpowered.

**Proposal:** find why `#INDICATOR_POWERED#` is never substituted for this one template
(`FBW_A380X_BacklightIndicator_Button_Template` combined with `ASOBO_GT_Push_Button` on the SURV
panel) — likely a missing default in the model's `DefaultTemplateParameters` for this component,
fixable once and applying to all 8.

**Impact 3, Effort S once the root cause is found (likely one fix fixes all 8), M if each needs separate handling.**

### 2.9 Overhead pushbuttons with no click code (ATA 21/23/28/52) — CTRL-005

**Evidence:** these are not merely unresolved by the converter — the resolver reports "no click
code" for them, meaning the original MSFS model itself never wired a click action:
```
PUSH_OVHD_AUTO_GND_XFR: unresolved (ASOBO_GT_Push_Button_Airliner), keeps fbw/cockpit/PUSH_OVHD_AUTO_GND_XFR: no click code
PUSH_OVHD_AVNCS_GND_COOLG: ... no click code
PUSH_OVHD_CAB_DATA_TO_NSS: ... no click code
PUSH_OVHD_DLS_NEXT / _PREV / _SELCTL: ... no click code
PUSH_OVHD_GATELINK: ... no click code
PUSH_OVHD_OVHT_COND_FANS: ... no click code
PUSH_OVHD_REFUEL: ... no click code
PUSH_CKPT_DOOR: ... no click code
PUSH_DOOR_LCKG_SYS: ... no click code
PUSH_GLARESHIELD_CS_AUTOLAND / _FO_AUTOLAND: ... no click code
PUSH_GLARESHIELD_CS_SIDESTICK / _FO_SIDESTICK: ... no click code
```
13 buttons in total, spanning ATA 21 (`OVHT_COND_FANS`), 23 (`CAB_DATA_TO_NSS`, `GATELINK`, DLS),
28 (`REFUEL`, `AUTO_GND_XFR`), 52 (`CKPT_DOOR`, `DOOR_LCKG_SYS`), and 22/27 priority buttons
(`AUTOLAND`, `SIDESTICK`).

**What the real aircraft does:** REFUEL opens the fuel panel/refuel logic, AUTO GND XFR the
automatic ground fuel transfer, DOOR LCKG SYS the cockpit-door locking system, SIDESTICK/AUTOLAND
priority takeover, GATELINK/CAB DATA TO NSS the cabin datalink routing.

**Proposal:** since the MSFS model itself has no click code, these need new bindings added at the
model level (or synthesised by the converter from FBW's own dataref names, the way `FIRE_AGENT`
etc. already are) rather than something the resolver can "find" — flag for whoever owns model-XML
authoring, since this is missing at the source, not lost in translation.

**Impact 3 average (varies: REFUEL and DOOR_LCKG_SYS matter more than GATELINK), Effort L (13 controls, several systems).**

### 2.10 Audio, weather radar and data-load selector switches — CTRL-008

**Evidence:**
```
KNOB_OVHD_AUDIOSWITCH: unresolved (ASOBO_GT_Switch_Dummy), keeps fbw/cockpit/KNOB_OVHD_AUDIOSWITCH:
  changes no FBW variable (MSFS-local state only)
SWITCH_OVHD_DLS: unresolved (ASOBO_GT_Interaction_LeftSingle_Code), keeps fbw/cockpit/SWITCH_OVHD_DLS:
  changes no FBW variable (MSFS-local state only)
SWITCH_RADAR_GCS: unresolved (ASOBO_GT_Interaction_LeftSingle_Code), keeps fbw/cockpit/SWITCH_RADAR_GCS:
  changes no FBW variable (MSFS-local state only)
SWITCH_RADAR_MULTISCAN: unresolved ... changes no FBW variable (MSFS-local state only)
```
`SWITCH_RADAR_GCS`/`SWITCH_RADAR_MULTISCAN` are weather-radar controls and belong to the wxr
owner (`src/wxr/`, `docs/wxr.md`) per `docs/team.md`; noted here only because they surfaced in
this file, not analysed further (out of my scope).

**Impact 2, Effort S** (for the audio/DLS switches; radar switches are the wxr owner's).

### 2.11 Cabin signage (ATA 33) — LGT-004

**Evidence:**
```
SWITCH_OVHD_INTLT_EMEREXIT: unresolved (ASOBO_GT_Switch_3States), keeps fbw/cockpit/SWITCH_OVHD_INTLT_EMEREXIT:
  changes no FBW variable (MSFS-local state only)
SWITCH_OVHD_INTLT_NOSMOKING: unresolved (ASOBO_GT_Switch_3States), keeps fbw/cockpit/SWITCH_OVHD_INTLT_NOSMOKING:
  changes no FBW variable (MSFS-local state only)
```
Contrast with the panel's third switch, which does resolve cleanly:
```
SWITCH_OVHD_INTLT_ANNLT: steps fbw/A32NX_OVHD_INTLT_ANN from 0 to 2 by 1
```
and is confirmed consumed on the FBW side (`D:\A380\fbw-xp-systems\src\prim.rs:716`:
`d.lights_test = b(self.names.get(vars, "A32NX_OVHD_INTLT_ANN"));`) — so the annunciator-test
switch (part of the brief's "annunciator test" scope item) works; its two neighbours on the same
panel (emergency-exit signs, no-smoking signs) do not drive anything.

**Proposal:** give both 3-way switches `fbw/` variables analogous to `A32NX_OVHD_INTLT_ANN`.

**Impact 2, Effort S.**

### 2.12 Lighting-knob push detent and cosmetic controls — CTRL-001 / CTRL-010

```
LH_SLIDING_LIGHTING_KNOB: unresolved (ASOBO_Interaction_Base_Template (Push)), keeps
  fbw/cockpit/LH_SLIDING_LIGHTING_KNOB: input event without SET_STATE_EXTERNAL
RH_SLIDING_LIGHTING_KNOB: same
COCKPIT_COFFEE_L / _R: unresolved (ASOBO_GT_Interaction_LeftSingle_Leave_Code) ...
  changes no FBW variable (MSFS-local state only)
HAT03: unresolved (ASOBO_Interaction_Base_Template (Push)) ... input event without SET_STATE_EXTERNAL
HAT04: unresolved (ASOBO_GT_MouseRect) ... raw mouse rectangle callback not translated
```
Low-stakes cosmetic/camera items (coffee cups, hat-switch view controls); noted for completeness
but not worth prioritising. **Impact 1, Effort S.**

## 3. Study panel (`src/study/`)

`src/study/mod.rs` lists every page the Study menu can open (`ITEMS`, lines 45-63): four Engine
pages, APU, Electrical, Hydraulics, Flight Controls, Fuel, Bleed, Air Conditioning,
Pressurisation, Gear and Brakes, Air Data, Fire, Radios, and an "All Variables" dump. Each opens a
window whose title bar reports whether the simulation is running and how many variables are live
(`mod.rs:284-292`), and every reading is followed back to its source and traced on click
(`tooltip`, `mod.rs:361-406`) — the module comment's claim ("every figure comes from the running
simulation... nothing is invented") holds up: no hard-coded fallback values or `TODO`/stub markers
were found in `elec.rs`, `engine.rs`, `hyd.rs`, `pages.rs` or `canvas.rs` (grepped for
`TODO|FIXME|stub|placeholder|hard.?code|fake|dummy|unimplemented|simplified`; the only hit,
`elec.rs:324`, is a local variable named `stub_top` for a wire-drawing y-coordinate, not a stub
value).

So the gap here is not fake data — it is missing pages and missing depth, exactly as the brief
frames the CL650 comparison.

### 3.1 What's missing entirely — STUDY-001, 002, 003, 004, 006

| ID | Missing page | Evidence it's missing | CL650-style study aircraft equivalent |
|---|---|---|---|
| STUDY-001 | Circuit Breakers | `PageKind` enum (`mod.rs:27-42`) and `ITEMS` (`mod.rs:45-63`) have no CB entry; no CB drawing code anywhere in `src/study/` | A clickable overhead/avionics CB panel; pulling a breaker removes power from exactly the system it feeds |
| STUDY-002 | Failures | Same: no `PageKind::Failures`. But the capability exists and is substantial: `D:\A380\fbw-xp-systems\src\failures.rs` (776 lines) implements a full `FailureType`-mapped failure system tied to X-Plane's own failure datarefs (`XplaneFailures`, `failures.rs:525-620`), with real ATA-numbered ids (e.g. `24_106` electrical bus, `26_018` fire-detection loop, `29_017` engine pump overheat, `32_015`/`32_025` gear sensor/actuator) — it is simply never surfaced or made injectable from the Study UI | A failures page to arm/clear failures and see which are active, with search/filter by ATA chapter |
| STUDY-003 | Ground services | No `PageKind` entry; the only related plugin code found is `rig.rs`'s ground-equipment visibility clip (`fbw/anim/ground_equipment`, gated on `GND > 0.5 and PBRK > 0.5 and all N1 < 5`, `rig.rs:396-412`), which is a converter-side visual, not something the Study panel shows or lets you command | GPU connect/disconnect, pushback start/stop and direction, chocks, external air/fuel, jetway/stairs, all inspectable and some controllable from one page |
| STUDY-004 | Aircraft state / persistence | `start_state.rs` (403 lines) classifies the spawn situation (cold-and-dark, ready-to-fly, etc., `classify`, line 74) but has no save/load functions at all (grepped for `persist|save|load` — no hits beyond `override_path`, which reads a one-shot override file, not aircraft state) and nothing in `src/study/` exposes any of it | A page showing/saving aircraft configuration (fuel, payload, doors, CB pulls, active failures, wear) across sessions |
| STUDY-006 | Maintenance | No page, and no wear/degradation model found under this search either | Deferred-defects style maintenance log; component wear affecting performance over time |

### 3.2 What shows less than it could — STUDY-005

`mod.rs:312-321` dispatches by page kind:
```rust
PageKind::Engine(n) => engine::draw(&mut cv, n),
PageKind::Electrical => elec::draw(&mut cv),
PageKind::Hydraulics => hyd::draw(&mut cv),
PageKind::Fuel => pages::fuel(&mut cv),
PageKind::FlightControls => pages::flight_controls(&mut cv),
PageKind::All => overflow = pages::all(&mut cv, win.scroll),
PageKind::Radios => overflow = pages::radios(&mut cv, win.scroll),
other => overflow = pages::flow(&mut cv, &pages::groups(other), win.scroll),
```
`engine::draw`, `elec::draw` and `hyd::draw` are bespoke schematic renderers (420, 372 and 158
lines respectively) that draw the network the way the aircraft's own synoptic pages do, per the
module doc comment. Everything that falls into the `other` arm — **Bleed, Air Conditioning,
Pressurisation, Gear and Brakes, Air Data, and Fire** (`pages::groups`, `pages.rs:28-36`) — gets
the same generic `flow()` field-list layout as a catch-all, not a schematic. These are exactly the
systems where a schematic view matters most for a study panel (bleed duct routing, air-conditioning
pack/mix logic, cabin pressure schedule, gear-door/downlock sequence, ADIRS source selection, fire
loop/zone layout).

**Proposal:** give each of the six a dedicated `draw()` function analogous to `elec.rs`/`hyd.rs`,
in priority order Gear/Brakes and Fire first (closest to safety-relevant animations already in
scope), then Pressurisation, Bleed, Air Conditioning, Air Data.

**Impact 3, Effort L** (six pages).

## 4. Candidate circuit breakers

Per `docs/team.md`, "only the lead edits the converter" and CBs are stage 3 work; this section is
an inventory of what exists to build candidates from, not a wired implementation.

### 4.1 What the model actually has today

The MSFS model's only clickable CB-shaped controls are 10 "reset panel" pushbuttons, all using one
template (`model/behaviour/overhead/reset.xml`):
```xml
<Template Name="FBW_Airbus_RESET_PANEL_BUTTON">
  <Parameters Type="Default">
    <NODE_ID>CB_#NAME#</NODE_ID>
    ...
  </Parameters>
  <UseTemplate Name="FBW_Push_Toggle">
    <TOGGLE_SIMVAR>L:A32NX_RESET_PANEL_#NAME#</TOGGLE_SIMVAR>
  </UseTemplate>
</Template>
```
instantiated in `A380_COCKPIT.xml`'s `Overhead_Reset_Panel` component (around line 4731) for
`NAME` = `ARPT_NAV`, `FMC_A`, `FMC_B`, `FMC_C`, `FWS1`, `FWS2`, `AESU1`, `AESU2`, `NSS_AVNCS`,
`NSS_FLT_OPS` — matching the 10 lines the converter resolves cleanly in `cockpit_bindings.txt`
(`CB_AESU1: toggles fbw/A32NX_RESET_PANEL_AESU1 between 1 and 0`, etc.).

**CB-001 — these 10 buttons currently do nothing.** `grep -rn "RESET_PANEL"` across both
`D:\fbw-aircraft\fbw-a380x\src\wasm\systems` and `D:\A380\fbw-xp-systems\src` returns zero hits: no
Rust code, on either the FBW systems side or the plugin side, reads any `A32NX_RESET_PANEL_*`
variable. They are wired end-to-end (model → bound `fbw/` var) but the var is a dead end.

**Proposal for CB-001:** since these already exist as real clickable geometry and are the only
avionics-computer reset controls the model has, wire them first: on press, they should
power-cycle/reset the named computer (AESU1/2, the three FMCs, FWS1/2, the two NSS channels,
airport-nav database) the way real reset panels do, most likely by round-tripping through
`failures.rs`'s `Failures`/`XplaneFailures` machinery (which already models per-computer failure
injection) or a small dedicated reset handler. Effort S because the binding and the target vars
already exist — only the consuming logic is missing.

**CB-002 — no physical overhead/avionics-bay circuit-breaker panel is modeled at all.** No
`ModelBehaviorDefs`/`model/behaviour` file in the package defines a general CB panel (only the
10-button reset panel above), and `systems.cfg` (the MSFS package) has no `[CIRCUIT` sections used
as clickable objects — only as internal MSFS electrical-load bookkeeping (below). This is the
biggest structural gap in the CB scope item: there is nothing to "add 50+ working CBs" to yet;
stage 3 will need new clickable geometry (or a Study-panel-only CB page that doesn't require new
3D geometry, per STUDY-001) before any candidate below can be operated from the actual cockpit.

### 4.2 Candidate consumers from `systems.cfg`

The package's `systems.cfg` `[ELECTRICAL]` section defines 151 named circuits (`circuit.1`
through `circuit.151`), each with a bus connection and a wattage — MSFS's own generic electrical-
load bookkeeping, not read by FBW's Rust systems, but a real, non-invented source of consumer
names and groupings to build a CB list from. By category:

| Category | Count | Examples (name, systems.cfg line) |
|---|---|---|
| Fuel pumps | 25 | `Fuel_Pump1_Feed1` (372), `Fuel_Pump_Left_Outer` (380), `Fuel_Pump_APU` (497) |
| Fuel valves | 60 | `Fuel_Valve_Engine_1_LP` (392), `Crossfeed_Valve_1..4` (457-460), `APU_Iso_Valve`/`APU_LP_Valve` (461-462) |
| Lighting | 20 | `Beacon_Light` x2 (395-396), `Landing_Light`/`Taxi_Light` x6 (397-402), `Strobe_Light` x3 (403-405), `Wing_Light`/`Logo_Light`/`Nav_Light` x8, `Panel_Light` x4, `Pedestal_Light`, `Cabin_Light` x3, `CaptainGlareshieldLights`/`CaptainTableLight`/`FOTableLight` |
| Radios/nav | 12 | `NAV1..3`, `COM1..3`, `XPNDR 1`, `ADF_DME`, `Marker_Position`, `Audio`, `Directional_Gyro`, `FIS` |
| Avionics displays | 7 | `PFD`, `MFD`, `EICAS1`, `EICAS2`, `CDU`, `FCU`, `ADC_AHRS` |
| Other | 10 | `Gear_Motor`, `Gear_Warning`, `Pitot_Heat`, `Starter_1/2` (engine), `Starter_APU`, `STBY_Vacuum`, `HotBatteryCircuit`, `WipersLeft`/`WipersRight`, `Avionics`, `Recognition_Light`, `General_Panel`, autopilot |

None of these are ready-made CB names for the real A380 — they are MSFS's own generic circuit
model, useful only as a checklist of consumers that should each end up behind *some* candidate
breaker once real A380 CB nomenclature is available (AMM/FCOM, which this analysis does not have
access to — do not invent panel positions or breaker part numbers).

### 4.3 Candidate groupings from FBW's own bus topology

FBW's A380 Rust electrical model (`D:\fbw-aircraft\fbw-a380x\src\wasm\systems\a380_systems\src\electrical\`)
defines the real sub-bus structure a CB panel should be organised by — this is FBW's own code, not
invented:

- **DC** (`direct_current.rs:64-144`): `DirectCurrent(1)`, `DirectCurrent(2)`,
  `DirectCurrentEssential`, `DirectCurrentNamed("247PP")`, `DirectCurrentNamed("309PP")`,
  `DirectCurrentHot(1..4)` (battery hot buses 1/2/ESS/APU), `DirectCurrentGndFltService`,
  `DirectCurrentNamed("108PH")`, `DirectCurrentNamed("502PP")`.
- **AC** (`alternating_current.rs:42-59`): `AlternatingCurrent(1..4)`,
  `AlternatingCurrentEssentialShed`, `AlternatingCurrentEssential`,
  `AlternatingCurrentNamed("247XP")`, `AlternatingCurrentGndFltService`.
- APU start motor sub-bus: `ElectricalBusType::Sub("49-42-00")` (`direct_current.rs:16`).

**Proposal:** stage 3's CB inventory should be organised by these real sub-buses (each maps to a
physical CB/contactor group on the real aircraft), cross-referenced against the `systems.cfg`
consumer list above for plausible per-consumer breakers, and validated against `failures.rs`'s
existing `FailureType::ElectricalBus(...)` entries (already confirmed to include
`AlternatingCurrentNamed("247XP")` at `failures.rs:616`) so that pulling a candidate CB can reuse
the failure-injection path that already exists rather than inventing a second mechanism.

**Impact 3, Effort XL** (the full 50+ CB build-out; the groundwork above is what makes it
tractable).

## 5. Top 50 candidates

The brief asks for the 50 highest impact-per-effort gaps in scope for stage 2.5. This pass found
22 distinct, evidenced gaps (the list below); it does not reach 50 because the remaining
`cockpit_bindings.txt` surface — roughly 450 "runs the control's original MSFS code in SASL"
entries and ~200 unresolved `PUSH_KBD_*` EFB keyboard keys (out of scope) — was not individually
audited line-by-line within this pass, since most "runs in SASL" entries need reading their
underlying RPN script (not just the resolver's one-line summary) to tell whether they reach FBW,
which is a per-control investigation each. That follow-up pass is the natural stage-2 continuation
and should focus on: (a) the ~470 SASL-script lines, checking each RPN's `L:`/`H:` targets against
FBW's own dataref names, prioritising the fire panel (§2.2) and pedestal/overhead first; (b) the 6
generic-layout Study pages once their schematics are drafted (§3.2), to see whether the drafting
itself surfaces further data gaps.

Ranked by impact, then impact-per-effort:

1. CTRL-002 — FCU/AP altitude knob and increment selector bypass FBW (impact 5, effort M)
2. CTRL-009 — fire guard/discharge buttons' FBW path unverified (impact 5, effort S/M)
3. CTRL-007 — parking brake lever unresolved (impact 4, effort S)
4. CB-001 — 10 reset-panel CB buttons wired to nothing (impact 4, effort S)
5. CTRL-006 — cabin pressurisation landing-elevation/manual V/S unresolved (impact 4, effort M)
6. STUDY-002 — failures system exists but has no Study UI (impact 4, effort M)
7. STUDY-001 — no Circuit Breakers page (impact 4, effort XL)
8. LGT-001 — exterior lights bypass FBW bus gating (impact 3, effort M)
9. LGT-002 — interior dimming knobs unresolved (impact 3, effort M)
10. CTRL-003 — EFIS baro knobs partly unresolved / one fires X-Plane's native command (impact 3, effort M)
11. CTRL-004 — SURV panel backlights missing one template parameter (impact 3, effort S/M)
12. STUDY-003 — no Ground Services page (impact 3, effort M)
13. STUDY-005 — 6 Study pages are field lists, not schematics (impact 3, effort L)
14. CTRL-005 — 13 overhead buttons have no click code at source (impact 3, effort L)
15. STUDY-004 — no persistence / aircraft-state page (impact 3, effort L)
16. CB-002 — no physical CB panel modeled; groundwork only (impact 3, effort XL)
17. CTRL-008 — audio selector / DLS selector unresolved (impact 2, effort S)
18. LGT-003 — EFIS CS/FO BLANK and TAXI backlights unresolved (impact 2, effort S)
19. LGT-004 — EMER EXIT / NO SMOKING signage switches unresolved (impact 2, effort S)
20. STUDY-006 — no maintenance page (impact 2, effort L)
21. CTRL-001 — sliding lighting-knob push detent unresolved (impact 1, effort S)
22. CTRL-010 — coffee-cup/hat-switch cosmetic controls unresolved (impact 1, effort S)

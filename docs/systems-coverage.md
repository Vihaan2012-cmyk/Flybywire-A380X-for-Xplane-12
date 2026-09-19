# Systems coverage: what FlyByWire's A380X runs in MSFS, and where it runs here

Owner: systems coverage engineer. Statuses:

- **ported**: runs in the plugin from FlyByWire's code (named file);
- **JS runtime**: TypeScript that runs unchanged once the MSFS instrument runtime (src/js, js_bridge.rs) runs the host; notes say what the host still needs from the plugin;
- **teammate**: in progress in a teammate's area;
- **converter**: the MSFS behaviour XML's logic, translated by msfs2xp-aircraft (lead);
- **MISSING**: nothing runs it yet;
- **n/a**: MSFS-only machinery with no X-Plane counterpart to drive.

Source of the list: panel.cfg (package `SimObjects/AirPlanes/FlyByWire_A380_842/panel/panel.cfg`) VCockpit01-24.

## VCockpit21: WASM gauges

### systems.wasm: `a380_systems_wasm` (lib.rs) and the `A380` Rust systems

| piece | status | where |
|---|---|---|
| `A380` systems (electrical, hydraulic, pneumatic, air conditioning, pressurisation, APU, fuel quantity, landing gear/LGCIU, ADIRS, RA, fire, payload, BTV, ...) | ported (unchanged crate) | lib.rs `Simulation<A380>` |
| builder `with_electrical_buses` (MSFS bus lookups for MSFS circuits) | partly: fuel pump/valve circuits follow FBW buses; MSFS light circuits do not (X-Plane lights are X-Plane's) | fuel.rs `power_circuits` |
| `with_auxiliary_power_unit` (APU fuel valve 8, pump 21) | ported | fuel.rs / fuel_transfer.rs |
| `with_engine_anti_ice(4)`, `with_wing_anti_ice()` | ported | aspects.rs `a380` |
| `with_fuel_pumps(1..=21)` | ported | aspects.rs |
| `with_failures` (ids 21000-...) | ported | failures.rs |
| provided simvars (lib.rs:429-564) | ported | lib.rs `mapping`, sensors.rs |
| GSX bypass pin, APU/ENG GEN and EXT PWR aspects | ported | aspects.rs |
| aspects: brakes, autobrakes, nose/body wheel steering, flaps, gear | ported | handling/aspects.rs, handling/physics.rs |
| aspects: ailerons, elevators, rudder, spoilers, THS | ported | flight_controls.rs |
| aspects: reversers (reversers.rs: `ACCELERATION_BODY_Z_WITH_REVERSER`, and FBW's reverser thrust applied as `VELOCITY BODY Z` += `REVERSER_DELTA_SPEED`, yaw from `REVERSER_ANGULAR_ACCELERATION`) | ported: `REVERSER_DELTA_SPEED`/`REVERSER_ANGULAR_ACCELERATION` (already computed by the unchanged `ReverserForce`, engine/reverser_thrust.rs, inside `Simulation<A380>`) are applied to X-Plane's velocity and yaw rate the way pushback.rs nudges the aircraft; `engine_commands.rs` zeroes X-Plane's own reverse throttle so this is the sole source, avoiding double-counting. The `ACCELERATION_BODY_Z_WITH_REVERSER` input still reads X-Plane's raw `g_axil` (lib.rs mapping) rather than feeding back `REVERSER_DELTA_ACCEL`, and FlyByWire's MSFS-only low-speed friction workaround (reversers.rs:29-31) is left out; see extra_backend_fbw.rs's module doc. | extra_backend_fbw.rs, engine_commands.rs |
| aspects: cargo_doors, fire, payload, fuel | ported | aspects.rs, fuel.rs, weight_balance.rs, doors.rs |
| start state | ported | start_state.rs |
| fuel pump pushbuttons `CIRCUIT CONNECTION ON:n` switching the MSFS fuel pump circuits | fixed: fuel.rs's circuits now carry the pushbutton's `CIRCUIT CONNECTION ON:n` alongside their buses | fuel.rs |

### fbw.wasm: `fbw_a380` (FlyByWireInterface.cpp)

| piece (cpp function) | status | where |
|---|---|---|
| PRIM x3, SEC x3, FCU x2 (Simulink C++) | ported (compiled C++) | fbw_computers.rs, prim.rs |
| updateRa/Lgciu/Sfcc/Ils/Adirs/Fqms/Tcas/Aesu, updateFcu, updateFcuAfsLvars, updateFcuShim, updatePrim, updatePrimFgShim, updateSec, updateServoSolenoidStatus, handleFcuInitialization | ported | prim.rs, afs_events.rs |
| updateFadec (A380FadecComputer) | ported (compiled C++) | engine_commands.rs |
| throttle axis mapping | ported | throttle.rs |
| **FCDC x2** (fcdc/Fcdc.cpp, updateFcdc cpp:2219-2320): FCDC bus words for FWS, SD F/CTL, PFD | in progress (delegated by coverage engineer) | src/extra_backend_fcdc.rs |
| **updateSpoilers** (SpoilersHandler): `A32NX_SPOILERS_ARMED`, `_HANDLE_POSITION` (PFD, FWS, sound.xml, presets) | in progress (with FCDC) | src/extra_backend_fcdc.rs |
| updateFlyByWire: rudder pedal position | ported | handling.rs:416-423 |
| updateFlyByWire: `A32NX_SIDESTICK_POSITION_X/Y` (3D sidestick, sound), `A32NX_RUDDER_PEDAL_ANIMATION_POSITION`, `FLIGHT_CONTROLS_TRACKING_MODE` | ported (X-Plane has no slew/pause equivalent, so tracking mode follows the external override alone; see extra_backend_fbw.rs's module doc) | extra_backend_fbw.rs |
| updateRadioReceiver option (CalculatedRadioReceiver) | MISSING (option off, as FBW's default) | prim.rs `UNAVAILABLE` |
| FailuresConsumer for the C++ computers | ported: failures.rs's `COMPUTER_FAILURES` (Fcu1/2, Prim1-3, Sec1-3, Fcdc1/2, Rollout) feeds prim.rs's PRIM/SEC/FCU and extra_backend_fcdc.rs's FCDC/Rollout from `failures::active_ids()` | failures.rs, prim.rs, extra_backend_fcdc.rs |
| handleSimulationRate (limit sim rate while AP engaged) | ported, onto `sim/time/sim_speed` | extra_backend_fbw.rs |
| updatePerformanceMonitoring (`A32NX_PERFORMANCE_WARNING_ACTIVE`) | ported | extra_backend_fbw.rs |
| updateAltimeterSetting (sets MSFS altimeter 4 to STD) | n/a | - |
| FlightDataRecorder, updateBaseData/AircraftSpecificData | n/a (developer recording) | - |
| disconnect MSFS default autopilot | n/a | - |

### fadec-a380x.wasm

| piece | status | where |
|---|---|---|
| EngineControl_A380X, Polynomials, Table1502, ThrustLimits, fadec_common EngineRatios | ported (Rust translation) | fadec.rs |
| FuelConfiguration_A380X (tank levels ini) | ported | fuel.rs |
| quick mode (`AIRCRAFT_PRESET_QUICK_MODE`) | ported | fadec.rs |

### extra-backend-a380x.wasm (Gauge_Extra_Backend.cpp)

| module | status | where |
|---|---|---|
| LightingPresets_A380X + LightingPresets | ported (this area) | src/extra_backend/lighting_presets.rs |
| Pushback_A380X + Pushback | ported (this area) | src/extra_backend/pushback.rs |
| AircraftPresets + PresetProcedures + ProcedureStep (with `execute_calculator_code`) | ported (this area) | src/extra_backend/aircraft_presets.rs, procedures.rs, rpn.rs, sim.rs |
| ExampleModule | n/a (compiled only with EXAMPLES) | - |

### terronnd.wasm (ND terrain, VCockpit07/08)

teammate (map data): src/mapdata.

## VCockpit22: systems-host (fbw-a380x/src/systems/systems-host)

All **JS runtime** unless noted. What each needs from the plugin beyond variables:

| module | status / notes |
|---|---|
| CpiomC/FlightWarningSystem (FwsCore, Abnormal*, Memos, Limitations, InopSys, NormalChecklists, SystemDisplayLogic, FlightPhases, AutoCallouts) | JS runtime. Needs FCDC words (in progress) and SPOILERS LVars (in progress). Its aurals play through sound (see below). |
| FwsSoundManager (aurals: LVar sounds + `PLAY_INSTRUMENT_SOUND`) | JS runtime logic; **playback** is this area: src/sound (in progress) |
| CpiomD/atsu | JS runtime (needs SimBridge/Hoppie networking in the runtime; teammate) |
| CpiomF/LegacyFuel | **ported natively** (fuel_transfer.rs). If the runtime also runs systems-host, LegacyFuel runs twice: key_events.rs ignores its FUELSYSTEM_* events, but its LVar writes still double. Runtime engineer: skip `LegacyFuel` or the fuel.rs port must yield. |
| Misc/Communications (VhfRadio, AudioManagementUnit, SimAudioManager, Transponder, RmpAmuBusPublisher) | JS runtime; their K events: radios.rs (COM/NAV/ADF), key_events.rs (XPNDR_*, ELECTRICAL_CIRCUIT_TOGGLE) |
| Misc/EfisTawsBridge | JS runtime; terrain via mapdata (teammate) |
| Misc/LegacyGpws | JS runtime; callouts through sound (in progress) |
| Misc/LegacySoundManager | JS runtime logic (`Coherent.call('PLAY_INSTRUMENT_SOUND')`); playback: src/sound (in progress) |
| Misc/powersupply | JS runtime |
| Misc/tcas (LegacyTcasComputer) | JS runtime; traffic from mapdata (teammate); RA aurals through sound |
| PseudoPRIM/BrakeToVacate | JS runtime (needs nav database: teammate) |
| publishers (FuelSystem, Fqms, StallWarning, PseudoFwc, Fcdc, ResetPanel, CpiomAvailable, MsfsFlightModel, FmsSymbols, Egpwc, FGData, MsfsMisc, IrBus, RaBus, LgciuBus, AesuBus, SwitchingPanel, FmsMessage, WeightBalance, Btv, PilotSeat, Power) | JS runtime; MSFS simvars they read must be fed (sensors.rs, lib.rs mapping) |

## VCockpit23: extras-host (fbw-a380x/src/systems/extras-host)

| module | status |
|---|---|
| AircraftSync, VersionCheck, TelexCheck, MsfsVersionPopupMonitor, NotificationManager | JS runtime (popups/network only; no systems effect) |
| A380XKeyInterceptor | JS runtime; needs the runtime to deliver intercepted MSFS key events (runtime engineer) |
| LightSync (cabin/panel auto brightness, `LIGHT_POTENTIOMETER_SET`, `ELECTRICAL_CIRCUIT_TOGGLE`) | JS runtime; its K events are applied by key_events.rs. `E:TIME OF DAY` (civil twilight from the sun) and `A:ON ANY RUNWAY` (apt.dat's runways) are ported (extra_backend_fbw.rs, js_bridge.rs). `GLASSCOCKPIT AUTOMATIC BRIGHTNESS` is still **MISSING**: MSFS computes it internally and does not document how, so nothing is fed and LightSync's clamp gives its 15 % floor. |
| PushbuttonCheck | JS runtime |
| GPUManagement | ported natively (efb.rs, teammate); do not run twice |
| PilotSeatManager, GsxSync, BaroUnitSelector, publishers | JS runtime |
| `L:A32NX_IS_READY` = 1 when in game | lib.rs sets it every tick |

## VCockpit01-20, 24: instruments

MFD, EWD, SD/SDv2, PFD x2, ND x2, FCU, ISIS, Clock, RTPI, BAT, RMP x3: JS runtime + renderer + DOM (teammates). EFB (VCockpit15), OIT (19/20) and popup (24): out of scope.

## Model behaviour XML (MSFS-side logic)

Cockpit click code, animations and emissives (A380_Cockpit_Behavior.xml and includes, ModelBehaviorDefs): **converter** (msfs2xp-aircraft behaviour/, SASL Lua). Key events the XML fires that the converter does not map (e.g. the exterior light events) are listed in key_events.rs for its table.

## MSFS simulator key events

General K: event handling for scripts and presets: **ported (this area)**, src/key_events.rs. See its table.

## Sound

**In progress (this area)**: src/sound. FlyByWire's sounds are Wwise soundbanks in the package (sound/FBW_A320_NEO_1-3.PC.PCK, sound.xml triggers). They are read from the user's own installed package at run time and played with XPLMPlayPCMOnBus; nothing is copied or redistributed.

## Requests to teammates

- Runtime engineer: route `Coherent.call('PLAY_INSTRUMENT_SOUND', name)` to `crate::sound::play_instrument_sound(name)` (sound module), and `Coherent.call('TRIGGER_KEY_EVENT', key, bypass, v0, v1, v2)` to `crate::key_events::push(key, &[v0, v1, v2])`. Do not run LegacyFuel (native port exists).
- Lead (converter): `LIGHT_POTENTIOMETER_SET` stores percent / 100 in MSFS (`LIGHT POTENTIOMETER` is percent over 100); events.rs stores the raw percent. key_events.rs and the lighting presets use percent over 100.
- EFB engineer: `fbw/efb/pushback/attached` should also report FlyByWire's tug (`PUSHBACK ATTACHED`, pushback.rs), which the flyPad's pushback uses; X-Plane's tug runs only canned manoeuvres.

# EFB interface

The `fbw/efb/...` datarefs and commands let a third-party EFB do what FlyByWire's flyPad does,
on the systems' own variables. The plugin registers them at start-up, in src/efb.rs, and serves
them every flight loop. A value you write is applied on the next tick. Every number dataref is
readable as int, float and double.

- **rw** means writable. Writing a value makes a request, and reading it back shows the value in
  effect.
- **ro** means read-only.
- A command acts once for each press. Only the begin phase counts.

Sources are cited as paths under `fbw-common/src/systems/instruments/src/EFB/`, unless another
path is given.

## Ground services

Sources: `Ground/Pages/Services/A380_842/A380Services.tsx` and `fbw-a380x/src/systems/extras-host`
`GPUManagement.ts`.

| name | type | units / range | rw | meaning |
|---|---|---|---|---|
| `fbw/efb/ground/available` | int | 0/1 | ro | Ground services can be used: on the ground, `A32NX_IS_STATIONARY`, and no tug attached (`sim/aircraft/overflow/pushback_attached`), as in A380Services.tsx:118-121. The plugin writes `A32NX_IS_STATIONARY` itself: ground speed at or below 0.1 ft/s (A380_WINGS.xml:347). |
| `fbw/efb/ground/gpu_connected` | int | 0/1 | ro | Any `A32NX_EXT_PWR_AVAIL:1-4`. |

| command | does |
|---|---|
| `fbw/efb/ground/door_main1_left` | Toggles point 0 (M1L), as `K:TOGGLE_AIRCRAFT_EXIT` 1 does. |
| `fbw/efb/ground/door_main2_left` | Toggles point 2 (M2L). |
| `fbw/efb/ground/door_upper1_left` | Toggles point 10 (U1L). |
| `fbw/efb/ground/door_main4_right` | Toggles point 9 (M5R; the flyPad calls it "main 4 right" and sends exit 10). |
| `fbw/efb/ground/door_cargo_fwd` | Toggles point 16 (forward cargo). |
| `fbw/efb/ground/jetway` | `sim/ground_ops/jetway` (MSFS `K:TOGGLE_JETWAY`). |
| `fbw/efb/ground/stairs`, `baggage`, `catering`, `fuel_truck` | `sim/ground_ops/service_plane`. X-Plane has one truck service command, standing in for MSFS's `TOGGLE_RAMPTRUCK`, `REQUEST_LUGGAGE`, `REQUEST_CATERING` and `REQUEST_FUEL_KEY`. |
| `fbw/efb/ground/gpu` | GPUManagement `toggleGPU` (see the notes below). |

These commands do nothing while `available` is 0, and the plugin logs that. When `available`
goes from 1 to 0, the five service doors close if they are fully open (A380Services.tsx:500-525).

GPU notes:

- X-Plane's `sim/cockpit2/electrical/GPU_generator_on` takes the place of both MSFS's powered stand
  and its GPU cart. While it is on, interactive point 19 (the ground power cable) reads 100 %.
- A change in the connection sets or clears `A32NX_EXT_PWR_AVAIL:1-4`. Clearing them also sets
  `A32NX_OVHD_ELEC_EXT_PWR_n_PB_IS_ON` to 0.
- Pressing `gpu` with no external power connects it if the GPU is there. Otherwise it runs
  `sim/ground_ops/toggle_gpu_request`, which is also how a connected cart is sent away.
- When the aircraft starts moving faster than 0.3 kt, the GPU is toggled once, not on every frame.

The doors also have their own commands, handled by src/doors.rs:

- `fbw/door/<NAME>/toggle`, `open` and `close`, where NAME is one of M1L-M5R, U1L-U3R,
  CARGO_FWD, CARGO_AFT, FUEL_HOSE or GROUND_POWER.
- `fbw/door/<NAME>/open_ratio` (float, 0-1, ro).
- X-Plane's own `sim/flight_controls/door_toggle_N`, `door_open_N` and `door_close_N`, where
  N = point + 1.

The `fbw/door/...` and X-Plane door commands act as door handles: a passenger door stays shut
while the cabin differential pressure is 0.8 psi or more. `INTERACTIVE POINT OPEN:n` is in percent,
at `fbw/INTERACTIVE_POINT_OPEN_n`.

## Payload and boarding

Source: `Ground/Pages/Payload/WideBody/A380Payload.tsx` and `PayloadElements.tsx`. The flyPad's
own split logic is ported: passengers go station by station at `ceil(capacity/484 x 100)/100` of
the count, from the last station forward, and station 1 takes the rest. Seats are picked at random,
and cargo is split in proportion to hold size. FlyByWire's payload system (a380_systems payload)
then boards towards the `_DESIRED` values.

The four target entries are refused while boarding is in progress, as the page disables them.

| name | type | units / range | rw | meaning |
|---|---|---|---|---|
| `fbw/efb/payload/pax_target` | int | 0-484 | rw | Sets the passenger target (`setTargetPax(n)`), then `setTargetCargo(n, 0)`, so cargo becomes that many bags. Reads back as the desired passenger count. |
| `fbw/efb/payload/cargo_target_kg` | float | kg, 0-51400 | rw | `setTargetCargo(0, kg)`. Reads back as the total desired cargo. |
| `fbw/efb/payload/zfw_target_kg` | float | kg | rw | `processZfw`. Reads back `A32NX_AIRFRAME_ZFW_DESIRED`. Empty weight is flight_model.cfg's 661403 lb. |
| `fbw/efb/payload/gw_target_kg` | float | kg | rw | `processGw`, using the current fuel (GW - ZFW). Reads back `A32NX_AIRFRAME_GW_DESIRED`. |
| `fbw/efb/payload/pax_weight_kg` | float | kg, 10-250 | rw | `A32NX_WB_PER_PAX_WEIGHT`. Defaults to 84. |
| `fbw/efb/payload/bag_weight_kg` | float | kg, 1-250 | rw | `A32NX_WB_PER_BAG_WEIGHT`. Defaults to 20. |
| `fbw/efb/payload/boarding_started` | int | 0/1 | rw | `A32NX_BOARDING_STARTED_BY_USR`. |
| `fbw/efb/payload/boarding_rate` | int | 0 instant, 1 fast, 2 real | rw | `A32NX_BOARDING_RATE`, stored as the CONFIG_BOARDING_RATE setting. Only instant (0) can be chosen unless the aircraft is cold and dark (on the ground with engines 1 and 4 not running), and the rate is forced to instant otherwise (A380Payload.tsx:573-587). |
| `fbw/efb/payload/pax` | int | | ro | Passengers on board. |
| `fbw/efb/payload/pax_desired` | int | | ro | Desired passengers. |
| `fbw/efb/payload/cargo_kg` | float | kg | ro | Cargo loaded. |
| `fbw/efb/payload/cargo_desired_kg` | float | kg | ro | Desired cargo. |
| `fbw/efb/payload/station/<1-14>/pax` | int | | ro | Passengers on board in each station, in cabin.json5 order: 1 MAIN FWD A (28), 2 MAIN FWD B (28), 3 MID 1A (39), 4 MID 1B (50), 5 MID 1C (43), 6 MID 2A (48), 7 MID 2B (40), 8 MID 2C (36), 9 AFT A (42), 10 AFT B (40), 11 UPPER FWD (14), 12 UPPER MID A (30), 13 UPPER MID B (28), 14 UPPER AFT (18). |
| `fbw/efb/payload/station/<1-14>/pax_desired` | int | | ro | Desired passengers per station. |
| `fbw/efb/payload/cargo/<fwd,aft,bulk>/kg` | float | kg | ro | Cargo loaded per hold. |
| `fbw/efb/payload/cargo/<fwd,aft,bulk>/kg_desired` | float | kg, 0-28577 / 20310 / 2513 | rw | Desired cargo per hold, as clicking the hold does (`onClickCargo`). |
| `fbw/efb/payload/boarding_eta_s` | float | s | ro | `calculateBoardingTime`: 5 s (real) or 1 s (fast) per passenger or per 60 kg of cargo, divided by the number of open boarding doors (points 0, 2 and 10). |

| command | does |
|---|---|
| `fbw/efb/payload/deboard` | While not boarding, sets the targets to 0 passengers and 0 cargo, then starts boarding 0.5 s later. While boarding, stops boarding. This is `handleDeboarding` without the confirmation prompt. |

## Refuel

Source: `Ground/Pages/Fuel/A380_842/A380Fuel.tsx`. The page only sets the target, the rate and the
start flag. The distribution between tanks is done by FlyByWire's systems (fuel refuel), which read
these variables.

| name | type | units / range | rw | meaning |
|---|---|---|---|---|
| `fbw/efb/refuel/target_kg` | float | kg, 0-259755 | rw | `A32NX_FUEL_DESIRED`. The maximum is 85471.7 gal x 3.039 kg/gal, and larger values are clamped to it. |
| `fbw/efb/refuel/target_percent` | float | %, 0-100 | rw | The same target as a percentage of the maximum. Below 0.5 % counts as 0. |
| `fbw/efb/refuel/rate` | int | 0 real, 1 fast, 2 instant | rw | The REFUEL_RATE_SETTING setting, which sets `A32NX_EFB_REFUEL_RATE_SETTING`. |
| `fbw/efb/refuel/started` | int | 0/1 | rw | `A32NX_REFUEL_STARTED_BY_USR`. It can only be set to 1 when `allowed` is 1. |
| `fbw/efb/refuel/allowed` | int | 0/1 | ro | `isRefuelAllowed` without GSX: refuelling is already running, or the aircraft is on the ground with no engine running, or the rate is instant. |
| `fbw/efb/refuel/total_kg` | float | kg | ro | `A32NX_TOTAL_FUEL_QUANTITY`. |
| `fbw/efb/refuel/eta_s` | float | s | ro | `calculateEta` in seconds (the page shows minutes): the difference at 16 gal/s, times 5 when fast; 0 when instant or within 10 kg of the target. |

| command | does |
|---|---|
| `fbw/efb/refuel/start_stop` | `switchRefuelState`. |

## Pushback

Source: `Ground/Pages/Pushback/PushbackPage.tsx`. FlyByWire's pushback module
(`fbw-common/src/wasm/extra-backend/Pushback`) moves the aircraft from these variables.

| name | type | units / range | rw | meaning |
|---|---|---|---|---|
| `fbw/efb/pushback/system_enabled` | int | 0/1 | rw | `A32NX_PUSHBACK_SYSTEM_ENABLED`. |
| `fbw/efb/pushback/speed_factor` | float | -1 to 1 | rw | `A32NX_PUSHBACK_SPD_FACTOR`. |
| `fbw/efb/pushback/heading_factor` | float | -1 to 1 | rw | `A32NX_PUSHBACK_HDG_FACTOR`. |
| `fbw/efb/pushback/parking_brake` | int | 0/1 | rw | `A32NX_PARK_BRAKE_LEVER_POS`. |
| `fbw/efb/pushback/attached` | int | 0/1 | ro | `sim/aircraft/overflow/pushback_attached`. |

| command | does |
|---|---|
| `fbw/efb/pushback/stop` | Sets both factors to 0. |
| `fbw/efb/pushback/call_tug` | Sets `PUSHBACK WAIT` to 1. |
| `fbw/efb/pushback/release_tug` | Sets `PUSHBACK WAIT` to 0. |

Attaching the tug (`K:TOGGLE_PUSHBACK` and `PUSHBACK STATE`) is not handled here. sensors.rs sets
`PUSHBACK STATE` from X-Plane's tug.

## Settings

Sources: `Settings/sync.ts:40-251` and `fbw-a380x/src/systems/instruments/src/EFB/settingsSync.ts`.

Settings are stored in `Output/preferences/fbw_a380x_settings.ini` as `A380X_<KEY>=<value>`,
using the same key and value strings as MSFS's stored data (NXDataStore with prefix `A380X`). The
plugin writes every variable at start-up, and again whenever a setting changes.

Each setting is `fbw/efb/settings/<key in lower case>`, a float, rw. It holds the variable's
number. Anything written is converted back to the stored string.

| key | variable | default | values |
|---|---|---|---|
| SOUND_EXTERIOR_MASTER | A32NX_SOUND_EXTERIOR_MASTER | 0 | number |
| SOUND_INTERIOR_ENGINE | A32NX_SOUND_INTERIOR_ENGINE | 0 | number |
| SOUND_INTERIOR_WIND | A32NX_SOUND_INTERIOR_WIND | 0 | number |
| EFB_BRIGHTNESS | A32NX_EFB_BRIGHTNESS | 0 | number |
| EFB_USING_AUTOBRIGHTNESS | A32NX_EFB_USING_AUTOBRIGHTNESS | 1 | 0/1 |
| CABIN_MANUAL_BRIGHTNESS | A32NX_CABIN_MANUAL_BRIGHTNESS | 0 | number |
| CABIN_USING_AUTOBRIGHTNESS | A32NX_CABIN_USING_AUTOBRIGHTNESS | 1 | 0/1 |
| ISIS_BARO_UNIT_INHG | A32NX_ISIS_BARO_UNIT_INHG | 0 | 0/1 |
| REALISTIC_TILLER_ENABLED | A32NX_REALISTIC_TILLER_ENABLED | 0 | 0/1 |
| HOME_COCKPIT_ENABLED | A32NX_HOME_COCKPIT_ENABLED | 0 | 0/1 |
| SOUND_PASSENGER_AMBIENCE_ENABLED | A32NX_SOUND_PASSENGER_AMBIENCE_ENABLED | 1 | 0/1 |
| SOUND_ANNOUNCEMENTS_ENABLED | A32NX_SOUND_ANNOUNCEMENTS_ENABLED | 1 | 0/1 |
| SOUND_BOARDING_MUSIC_ENABLED | A32NX_SOUND_BOARDING_MUSIC_ENABLED | 1 | 0/1 |
| RADIO_RECEIVER_USAGE_ENABLED | A32NX_RADIO_RECEIVER_USAGE_ENABLED | 0 | 0/1 |
| FDR_ENABLED | A32NX_FDR_ENABLED | 1 | 0/1 |
| MODEL_WHEELCHOCKS_ENABLED | A32NX_MODEL_WHEELCHOCKS_ENABLED | 1 | 0/1 |
| MODEL_CONES_ENABLED | A32NX_MODEL_CONES_ENABLED | 1 | 0/1 |
| FO_SYNC_EFIS_ENABLED | A32NX_FO_SYNC_EFIS_ENABLED | 0 | 0/1 |
| MODEL_SATCOM_ENABLED | A32NX_SATCOM_ENABLED | 0 | 0/1 |
| CONFIG_PILOT_AVATAR_VISIBLE | A32NX_PILOT_AVATAR_VISIBLE_0 | 0 | 0/1 |
| CONFIG_FIRST_OFFICER_AVATAR_VISIBLE | A32NX_PILOT_AVATAR_VISIBLE_1 | 0 | 0/1 |
| GSX_PAYLOAD_SYNC | A32NX_GSX_PAYLOAD_SYNC_ENABLED | 0 | 0/1 |
| CONFIG_USING_METRIC_UNIT | A32NX_EFB_USING_METRIC_UNIT | true | 0/1 (stored as JSON true/false) |
| CONFIG_USING_PORTABLE_DEVICES | A32NX_CONFIG_USING_PORTABLE_DEVICES | 1 | 0/1 |
| REFUEL_RATE_SETTING | A32NX_EFB_REFUEL_RATE_SETTING | 0 | 0 real, 1 fast, 2 instant |
| CONFIG_BOARDING_RATE | A32NX_BOARDING_RATE | REAL | 0 INSTANT, 1 FAST, 2 REAL |
| CONFIG_ALIGN_TIME | A32NX_CONFIG_ADIRS_IR_ALIGN_TIME | REAL | 0 REAL, 1 INSTANT, 2 FAST |
| CONFIG_A380X_FWC_RADIO_AUTO_CALL_OUT_PINS | A380X_FWC_RADIO_AUTO_CALL_OUT_PINS | 1032265 | bit flags (AutoCallOuts.ts) |

## Failures

These datarefs use src/failures.rs. Failure ids are FlyByWire's (a380_systems_wasm lib.rs:86-428).

| name | type | rw | meaning |
|---|---|---|---|
| `fbw/efb/failures/activate` | int | rw | Write a failure id to activate that failure. |
| `fbw/efb/failures/deactivate` | int | rw | Write a failure id to clear that failure. |
| `fbw/efb/failures/toggle` | int | rw | Write a failure id to toggle that failure. |
| `fbw/efb/failures/count` | int | ro | Number of active failures. |
| `fbw/failure/<id>` | int 0/1 | rw | One failure. |
| `fbw/failures/active` | int[] | rw | The active ids, zero padded. Writing it replaces the whole set. |
| `fbw/failures/count` | int | ro | Number of active failures. |

Commands: `fbw/failure/<id>/toggle`.

## Start state

| name | type | rw | meaning |
|---|---|---|---|
| `fbw/efb/start_state` | int | ro | `A32NX_START_STATE`: 1 hangar, 2 apron, 3 taxi, 4 runway, 5 climb, 6 cruise, 7 approach, 8 final. |
| `fbw/efb/start_state_override` | int | rw | 1-8 writes `Output/preferences/fbw_a380x_start_state.txt`, and any other value deletes it. It takes effect the next time the aircraft loads (src/start_state.rs). |

## Weights and CG (read-only)

| name | type | units | source |
|---|---|---|---|
| `fbw/efb/wb/zfw_kg`, `zfw_desired_kg` | float | kg | `A32NX_AIRFRAME_ZFW`, `_DESIRED` |
| `fbw/efb/wb/gw_kg`, `gw_desired_kg` | float | kg | `A32NX_AIRFRAME_GW`, `_DESIRED` |
| `fbw/efb/wb/zfw_cg_mac`, `zfw_cg_mac_desired` | float | % MAC | `A32NX_AIRFRAME_ZFW_CG_PERCENT_MAC`, `_DESIRED` |
| `fbw/efb/wb/gw_cg_mac`, `gw_cg_mac_desired` | float | % MAC | `A32NX_AIRFRAME_GW_CG_PERCENT_MAC`, `_DESIRED` |
| `fbw/efb/wb/to_cg_mac` | float | % MAC | `A32NX_AIRFRAME_TO_CG_PERCENT_MAC` |
| `fbw/wb/payload_kg` | float | kg | Sum of the payload stations (src/weight_balance.rs). |
| `fbw/wb/gross_weight_kg` | float | kg | Empty weight + stations + tanks, as MSFS adds them. |
| `fbw/wb/cg_z_ft` | float | ft | MSFS CG, measured forward from the datum. |

## Radios (MFD manual LS)

These are in src/radios.rs.

| name | type | units / range | rw | meaning |
|---|---|---|---|---|
| `fbw/radio/ls/frequency_mhz` | float | 108.00-111.95, 0 clears | rw | `setManualIls`. Reads back the manual frequency, or NAV 3 when there is none. |
| `fbw/radio/ls/course_deg` | float | 0-360, negative clears | rw | `setIlsCourse`. |
| `fbw/radio/ls/manual` | int | 0/1 | ro | A manual selection is active. |
| `fbw/radio/ls/ident`, `fbw/radio/nav<1-4>/ident`, `fbw/radio/adf<1-2>/ident` | byte[8] | string | ro | X-Plane's received idents. |

Commands: `fbw/radio/ls/frequency_up`, `frequency_down` (0.05 MHz), `frequency_up_coarse`,
`frequency_down_coarse` (1 MHz), `course_up`, `course_down` (1 degree), `course_up_coarse`,
`course_down_coarse` (10 degrees), and `clear`.

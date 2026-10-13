# The MSFS interface contract FlyByWire's A380X depends on

Read-only research brief. Goal: enumerate everything the FBW A380X (`fbw-a380x`, plus the
`fbw-common` code it shares) takes from Microsoft Flight Simulator, so that an external process
emulating MSFS (the `fbw-xp-systems` X-Plane host, `D:\A380\fbw-xp-systems`) can be checked against the
full contract, not just the parts it has already hit in testing.

Sources read: `fbw-a380x/src/systems` and `fbw-common/src/systems` (TypeScript instruments/FMS/EFB),
`fbw-a380x/src/wasm` and `fbw-common/src/wasm` (Rust `systems_wasm`/`a380_systems_wasm` and C++
`fbw_a380`/`extra-backend`), the behaviour XML under `fbw-a380x/src/base/**/model/behaviour`, the
package's `.cfg`/`.FLT` files, and `node_modules/@microsoft/msfs-sdk/msfssdk.js`. Coverage was
checked against `D:\A380\fbw-xp-systems\src` (`lib.rs`, `js_bridge.rs`, `key_events.rs`, `afs_events.rs`,
`sensors.rs`, `fadec.rs`, `prim.rs`, `js/msfs/*`).

**Methodology note on the two largest tables (Layers 1 and 3).** The 481-row SimConnect data
definition (Layer 1.1) and the 243-entry `Events` enum (Layer 3.1) are exhaustive: every
`addDataDefinition`/`addInputDataDefinition` call in `SimConnectInterface.cpp` is listed. Their
"our coverage" column was produced by grepping each variable/event's exact MSFS name against
`fbw-xp-systems/src` (`lib.rs`'s `mapping()` table first, then `sensors.rs`/`prim.rs`/`fadec.rs`/
`key_events.rs`/`afs_events.rs`/`radios.rs`/`js/msfs/*.js`). A `MISSING` mark means the literal
string was not found anywhere in that tree — it does **not** distinguish "genuinely unimplemented"
from "handled generically" (e.g. `sensors.rs`'s and `key_events.rs`'s own doc-comment tables already
describe large swathes of coverage in prose rather than as a grep-able per-name mapping) or from
"this direction doesn't matter" (many `Events` entries are MSFS's *default* flight-control axis
events — `AXIS_ELEVATOR_SET`, `AILERON_SET`, etc. — which X-Plane's own native joystick/axis input
can serve directly, so a MISSING mark there is not necessarily a gap in the emulation). Treat
`MISSING` as "needs a human to check", not as an automatic gap count; the ranked-gap list in the
summary corrects for this by hand.

---

## Layer 1 — Simulation variables (`A:`)
FBW's A380X takes `A:` variables from three independent places that do **not** automatically stay
in sync with each other: the C++ WASM's own SimConnect data definition (feeds the flight-model
math directly, at simulation rate), the Rust `systems_wasm`/`a380_systems_wasm` variable registry
(feeds the TypeScript-ported `systems` crate through `VariableRegistry`), and the JS instruments
(read through the `SimVar` global, independently of both WASM modules). A host must serve all
three paths; serving only `lib.rs`'s `mapping()` table (which covers the Rust registry) is not
enough by itself.

### 1.1 C++ `SimConnectInterface::prepareSimDataSimConnectDataDefinitions` (481 fields, one struct, ID 0)

The FBW A380 C++ WASM (`fbw_a380`) is a from-scratch C++ flight-control-computer simulation (PRIM/
SEC/FCU/FADEC/ADR/IR/RA/LGCIU/SFCC/ILS), distinct from the TypeScript `systems` crate — it computes
control-surface commands and feeds them back into MSFS's own flight model. This is the single
largest, most tightly coupled data definition: it is requested every visual frame
(`SimConnect_RequestDataOnSimObject(..., Period::VisualFrame)`, wired through `Time` in
`systems_wasm/src/lib.rs:596-651` for the Rust side and directly in `SimConnectInterface.cpp` for
this struct) and several of its fields (`ELEVATOR POSITION`, `AILERON POSITION`, `RUDDER POSITION`,
`SPOILERS LEFT/RIGHT POSITION`, etc.) are also **written** back via `SimConnect_SetDataOnSimObject`
(`sendData`, `SimConnectInterface.cpp:sendData`/`sendClientData`) once PRIM/SEC compute them — this
table lists the field only once, not per direction; check the field name in `FlyByWireInterface.cpp`
for which way it actually moves.

| name (`:index` if any) | kind | units | dir | used by | our coverage |
|---|---|---|---|---|---|
| G FORCE | A: | SIMCONNECT_DATATYPE_FLOAT64 / GFORCE | read | SimConnectInterface.cpp:143 | MISSING |
| PLANE PITCH DEGREES | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE | read | SimConnectInterface.cpp:144 | served: lib.rs mapping() |
| PLANE BANK DEGREES | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE | read | SimConnectInterface.cpp:145 | served: lib.rs mapping() |
| STRUCT BODY ROTATION VELOCITY | A: | SIMCONNECT_DATATYPE_XYZ / STRUCT | read | SimConnectInterface.cpp:146 | MISSING |
| STRUCT BODY ROTATION ACCELERATION | A: | SIMCONNECT_DATATYPE_XYZ / STRUCT | read | SimConnectInterface.cpp:147 | MISSING |
| ACCELERATION BODY Z | A: | SIMCONNECT_DATATYPE_FLOAT64 / METER PER SECOND SQUARED | read | SimConnectInterface.cpp:148 | MISSING |
| ACCELERATION BODY X | A: | SIMCONNECT_DATATYPE_FLOAT64 / METER PER SECOND SQUARED | read | SimConnectInterface.cpp:149 | served: lib.rs mapping() |
| ACCELERATION BODY Y | A: | SIMCONNECT_DATATYPE_FLOAT64 / METER PER SECOND SQUARED | read | SimConnectInterface.cpp:150 | served: lib.rs mapping() |
| PLANE HEADING DEGREES MAGNETIC | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:151 | served: src/prim.rs |
| PLANE HEADING DEGREES TRUE | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:152 | served: lib.rs mapping() |
| GPS GROUND MAGNETIC TRACK | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:153 | served: src/sensors.rs |
| ELEVATOR POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / POSITION | read | SimConnectInterface.cpp:154 | MISSING |
| ELEVATOR TRIM POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE | read | SimConnectInterface.cpp:155 | served: src/flight_controls.rs |
| AILERON POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / POSITION | read | SimConnectInterface.cpp:156 | MISSING |
| RUDDER POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / POSITION | read | SimConnectInterface.cpp:157 | MISSING |
| RUDDER TRIM PCT | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:158 | MISSING |
| INCIDENCE ALPHA | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE | read | SimConnectInterface.cpp:159 | served: src/sensors.rs |
| INCIDENCE BETA | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE | read | SimConnectInterface.cpp:160 | MISSING |
| BETA DOT | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE PER SECOND | read | SimConnectInterface.cpp:161 | MISSING |
| AIRSPEED INDICATED | A: | SIMCONNECT_DATATYPE_FLOAT64 / KNOTS | read | SimConnectInterface.cpp:162 | served: lib.rs mapping() |
| AIRSPEED TRUE | A: | SIMCONNECT_DATATYPE_FLOAT64 / KNOTS | read | SimConnectInterface.cpp:163 | served: lib.rs mapping() |
| AIRSPEED MACH | A: | SIMCONNECT_DATATYPE_FLOAT64 / MACH | read | SimConnectInterface.cpp:164 | served: lib.rs mapping() |
| GROUND VELOCITY | A: | SIMCONNECT_DATATYPE_FLOAT64 / KNOTS | read | SimConnectInterface.cpp:165 | MISSING |
| INDICATED ALTITUDE:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / FEET | read | SimConnectInterface.cpp:168 | served: src/js/msfs/environment.js |
| INDICATED ALTITUDE | A: | SIMCONNECT_DATATYPE_FLOAT64 / FEET | read | SimConnectInterface.cpp:169 | served: src/js/msfs/environment.js |
| PLANE ALT ABOVE GROUND MINUS CG | A: | SIMCONNECT_DATATYPE_FLOAT64 / FEET | read | SimConnectInterface.cpp:170 | served: src/js/msfs/environment.js |
| VELOCITY WORLD Y | A: | SIMCONNECT_DATATYPE_FLOAT64 / FEET PER MINUTE | read | SimConnectInterface.cpp:171 | served: lib.rs mapping() |
| CG PERCENT | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:172 | served: src/fuel_network.rs |
| TOTAL WEIGHT | A: | SIMCONNECT_DATATYPE_FLOAT64 / KILOGRAMS | read | SimConnectInterface.cpp:173 | served: lib.rs mapping() |
| GEAR ANIMATION POSITION:0 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:174 | served: src/sensors.rs |
| GEAR ANIMATION POSITION:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:175 | served: src/sensors.rs |
| GEAR ANIMATION POSITION:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:176 | served: src/sensors.rs |
| SPOILERS HANDLE POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / POSITION | read | SimConnectInterface.cpp:177 | served: src/extra_backend_fcdc.rs |
| SPOILERS LEFT POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:178 | MISSING |
| SPOILERS RIGHT POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:179 | MISSING |
| IS SLEW ACTIVE | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:180 | MISSING |
| AUTOPILOT MASTER | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:181 | MISSING |
| AUTOPILOT FLIGHT DIRECTOR ACTIVE:1 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:182 | MISSING |
| AUTOPILOT FLIGHT DIRECTOR ACTIVE:2 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:183 | MISSING |
| AUTOPILOT AIRSPEED HOLD VAR | A: | SIMCONNECT_DATATYPE_FLOAT64 / KNOTS | read | SimConnectInterface.cpp:184 | served: src/js/msfs/environment.js |
| AUTOPILOT ALTITUDE LOCK VAR:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / FEET | read | SimConnectInterface.cpp:185 | served: src/js/msfs/environment.js |
| SIMULATION TIME | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:186 | served: src/js/msfs/tests.rs |
| SIMULATION RATE | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:187 | served: src/js_bridge.rs |
| STRUCTURAL ICE PCT | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:188 | MISSING |
| LINEAR CL ALPHA | A: | SIMCONNECT_DATATYPE_FLOAT64 / PER DEGREE | read | SimConnectInterface.cpp:189 | MISSING |
| STALL ALPHA | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE | read | SimConnectInterface.cpp:190 | MISSING |
| ZERO LIFT ALPHA | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE | read | SimConnectInterface.cpp:191 | MISSING |
| AMBIENT DENSITY | A: | SIMCONNECT_DATATYPE_FLOAT64 / KILOGRAM PER CUBIC METER | read | SimConnectInterface.cpp:192 | served: lib.rs mapping() |
| AMBIENT PRESSURE | A: | SIMCONNECT_DATATYPE_FLOAT64 / MILLIBARS | read | SimConnectInterface.cpp:193 | served: lib.rs mapping() |
| AMBIENT TEMPERATURE | A: | SIMCONNECT_DATATYPE_FLOAT64 / CELSIUS | read | SimConnectInterface.cpp:194 | served: lib.rs mapping() |
| AMBIENT WIND X | A: | SIMCONNECT_DATATYPE_FLOAT64 / KNOTS | read | SimConnectInterface.cpp:195 | served: lib.rs mapping() |
| AMBIENT WIND Y | A: | SIMCONNECT_DATATYPE_FLOAT64 / KNOTS | read | SimConnectInterface.cpp:196 | served: lib.rs mapping() |
| AMBIENT WIND Z | A: | SIMCONNECT_DATATYPE_FLOAT64 / KNOTS | read | SimConnectInterface.cpp:197 | served: lib.rs mapping() |
| AMBIENT WIND VELOCITY | A: | SIMCONNECT_DATATYPE_FLOAT64 / KNOTS | read | SimConnectInterface.cpp:198 | MISSING |
| AMBIENT WIND DIRECTION | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:199 | MISSING |
| TOTAL AIR TEMPERATURE | A: | SIMCONNECT_DATATYPE_FLOAT64 / CELSIUS | read | SimConnectInterface.cpp:200 | served: src/engine_commands.rs |
| PLANE LATITUDE | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:201 | served: lib.rs mapping() |
| PLANE LONGITUDE | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:202 | served: src/js/msfs/simvar.js |
| GENERAL ENG THROTTLE LEVER POSITION:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:203 | served: src/engine_commands.rs |
| GENERAL ENG THROTTLE LEVER POSITION:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:204 | served: src/engine_commands.rs |
| GENERAL ENG THROTTLE LEVER POSITION:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:205 | served: src/engine_commands.rs |
| GENERAL ENG THROTTLE LEVER POSITION:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:206 | served: src/engine_commands.rs |
| TURB ENG JET THRUST:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / POUNDS | read | SimConnectInterface.cpp:207 | served: src/fadec.rs |
| TURB ENG JET THRUST:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / POUNDS | read | SimConnectInterface.cpp:208 | served: src/fadec.rs |
| TURB ENG JET THRUST:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / POUNDS | read | SimConnectInterface.cpp:209 | served: src/fadec.rs |
| TURB ENG JET THRUST:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / POUNDS | read | SimConnectInterface.cpp:210 | served: src/fadec.rs |
| NAV HAS NAV:3 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:211 | served: src/radios.rs |
| NAV LOCALIZER:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:212 | served: src/radios.rs |
| NAV RAW GLIDE SLOPE:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:213 | served: src/prim.rs |
| NAV HAS DME:3 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:214 | served: src/prim.rs |
| NAV DME:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NAUTICAL MILES | read | SimConnectInterface.cpp:215 | served: src/prim.rs |
| NAV HAS LOCALIZER:3 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:216 | served: src/prim.rs |
| NAV RADIAL ERROR:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:217 | served: src/prim.rs |
| NAV HAS GLIDE SLOPE:3 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:218 | served: src/prim.rs |
| NAV GLIDE SLOPE ERROR:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:219 | served: src/prim.rs |
| AUTOTHROTTLE ACTIVE | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:220 | MISSING |
| TURB ENG CORRECTED N1:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:221 | served: src/fadec.rs |
| TURB ENG CORRECTED N1:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:222 | served: src/fadec.rs |
| GPS IS ACTIVE FLIGHT PLAN | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:223 | MISSING |
| GPS WP CROSS TRK | A: | SIMCONNECT_DATATYPE_FLOAT64 / NAUTICAL MILES | read | SimConnectInterface.cpp:224 | MISSING |
| GPS WP TRACK ANGLE ERROR | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:225 | MISSING |
| GPS COURSE TO STEER | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:226 | MISSING |
| TURB ENG COMMANDED N1:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:227 | MISSING |
| TURB ENG COMMANDED N1:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:228 | MISSING |
| TURB ENG COMMANDED N1:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:229 | MISSING |
| TURB ENG COMMANDED N1:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:230 | MISSING |
| TURB ENG N1:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:231 | served: src/sound/triggers.rs |
| TURB ENG N1:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:232 | served: src/sound/triggers.rs |
| TURB ENG N1:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:233 | served: src/sound/triggers.rs |
| TURB ENG N1:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:234 | served: src/sound/triggers.rs |
| TURB ENG CORRECTED N1:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:235 | served: src/fadec.rs |
| TURB ENG CORRECTED N1:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:236 | served: src/fadec.rs |
| TURB ENG CORRECTED N1:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:237 | served: src/fadec.rs |
| TURB ENG CORRECTED N1:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:238 | served: src/fadec.rs |
| ENG COMBUSTION:1 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:239 | served: src/extra_backend_fcdc.rs |
| ENG COMBUSTION:2 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:240 | served: src/extra_backend_fcdc.rs |
| ENG COMBUSTION:3 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:241 | served: src/extra_backend_fcdc.rs |
| ENG COMBUSTION:4 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:242 | served: src/extra_backend_fcdc.rs |
| AUTOPILOT MANAGED SPEED IN MACH | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:243 | MISSING |
| AUTOPILOT SPEED SLOT INDEX | A: | SIMCONNECT_DATATYPE_INT64 / NUMBER | read | SimConnectInterface.cpp:244 | served: src/js/msfs/environment.js |
| ENG ANTI ICE:1 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:245 | served: src/aspects.rs |
| ENG ANTI ICE:2 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:246 | served: src/aspects.rs |
| ENG ANTI ICE:3 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:247 | served: src/aspects.rs |
| ENG ANTI ICE:4 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:248 | served: src/aspects.rs |
| SIM ON GROUND | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:249 | served: lib.rs mapping() |
| KOHLSMAN SETTING MB:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / MBAR | read | SimConnectInterface.cpp:250 | MISSING |
| KOHLSMAN SETTING MB:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / MBAR | read | SimConnectInterface.cpp:251 | MISSING |
| KOHLSMAN SETTING STD:4 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:252 | MISSING |
| CAMERA STATE | A: | SIMCONNECT_DATATYPE_INT64 / NUMBER | read | SimConnectInterface.cpp:253 | MISSING |
| PLANE ALTITUDE | A: | SIMCONNECT_DATATYPE_FLOAT64 / METERS | read | SimConnectInterface.cpp:254 | served: src/js/msfs/simvar.js |
| NAV MAGVAR:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREES | read | SimConnectInterface.cpp:255 | served: src/prim.rs |
| NAV VOR LATLONALT:3 | A: | SIMCONNECT_DATATYPE_LATLONALT / STRUCT | read | SimConnectInterface.cpp:256 | served: src/radios.rs |
| NAV GS LATLONALT:3 | A: | SIMCONNECT_DATATYPE_LATLONALT / STRUCT | read | SimConnectInterface.cpp:257 | served: src/radios.rs |
| BRAKE LEFT POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / POSITION | read | SimConnectInterface.cpp:258 | MISSING |
| BRAKE RIGHT POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / POSITION | read | SimConnectInterface.cpp:259 | MISSING |
| FLAPS HANDLE INDEX | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:260 | MISSING |
| GEAR HANDLE POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / POSITION | read | SimConnectInterface.cpp:261 | MISSING |
| ASSISTANCE TAKEOFF ENABLED | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:262 | MISSING |
| ASSISTANCE LANDING ENABLED | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:263 | MISSING |
| AI AUTOTRIM ACTIVE | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:264 | MISSING |
| AI CONTROLS | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:265 | MISSING |
| WHEEL RPM:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / RPM | read | SimConnectInterface.cpp:268 | served: src/sensors.rs |
| WHEEL RPM:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / RPM | read | SimConnectInterface.cpp:269 | served: src/sensors.rs |
| WHEEL RPM:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / RPM | read | SimConnectInterface.cpp:270 | served: src/sensors.rs |
| WHEEL RPM:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / RPM | read | SimConnectInterface.cpp:271 | served: src/sensors.rs |
| CONTACT POINT COMPRESSION | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:272 | served: src/js_bridge.rs |
| CONTACT POINT COMPRESSION:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:273 | served: src/js_bridge.rs |
| CONTACT POINT COMPRESSION:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:274 | served: src/js_bridge.rs |
| CONTACT POINT COMPRESSION:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:275 | served: src/js_bridge.rs |
| CONTACT POINT COMPRESSION:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:276 | served: src/js_bridge.rs |
| ANTISKID BRAKES ACTIVE | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:277 | served: src/extra_backend_fcdc.rs |
| SEA LEVEL PRESSURE | A: | SIMCONNECT_DATATYPE_FLOAT64 / MBAR | read | SimConnectInterface.cpp:278 | served: src/failures.rs |
| FUELSYSTEM TANK QUANTITY:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:283 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:284 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:285 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:286 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:5 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:287 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:6 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:288 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:7 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:289 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:8 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:290 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:9 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:291 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:10 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:292 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:11 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:293 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:12 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:294 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:13 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:295 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:14 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:296 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:15 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:297 | served: src/fuel.rs |
| FUELSYSTEM TANK QUANTITY:16 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS | read | SimConnectInterface.cpp:298 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:300 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:301 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:302 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:303 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:5 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:304 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:6 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:305 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:7 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:306 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:8 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:307 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:9 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:308 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:10 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:309 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:11 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:310 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:12 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:311 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:13 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:312 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:14 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:313 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:15 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:314 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:16 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:315 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:17 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:316 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:18 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:317 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:19 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:318 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:20 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:319 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:21 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:320 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:22 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:321 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:23 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:322 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:24 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:323 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:25 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:324 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:26 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:325 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:27 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:326 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:28 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:327 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:29 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:328 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:30 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:329 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:31 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:330 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:32 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:331 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:33 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:332 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:34 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:333 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:35 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:334 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:36 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:335 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:37 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:336 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:38 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:337 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:39 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:338 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:40 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:339 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:41 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:340 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:42 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:341 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:43 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:342 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:44 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:343 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:45 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:344 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:46 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:345 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:47 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:346 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:48 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:347 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:49 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:348 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:50 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:349 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:51 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:350 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:52 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:351 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:53 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:352 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:54 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:353 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:55 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:354 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:56 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:355 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:57 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:356 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:58 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:357 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:59 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:358 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:60 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:359 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:61 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:360 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:62 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:361 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:63 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:362 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:64 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:363 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:65 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:364 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:66 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:365 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:67 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:366 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:68 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:367 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:69 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:368 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:70 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:369 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:71 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:370 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:72 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:371 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:73 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:372 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:74 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:373 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:75 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:374 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:76 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:375 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:77 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:376 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:78 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:377 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:79 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:378 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:80 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:379 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:81 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:380 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:82 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:381 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:83 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:382 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:84 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:383 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:85 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:384 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:86 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:385 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:87 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:386 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:88 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:387 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:89 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:388 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:90 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:389 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:91 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:390 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:92 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:391 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:93 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:392 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:94 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:393 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:95 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:394 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:96 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:395 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:97 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:396 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:98 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:397 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:99 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:398 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:100 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:399 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:101 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:400 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:102 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:401 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:103 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:402 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:104 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:403 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:105 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:404 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:106 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:405 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:107 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:406 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:108 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:407 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:109 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:408 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:110 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:409 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:111 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:410 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:112 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:411 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:113 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:412 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:114 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:413 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:115 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:414 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:116 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:415 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:117 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:416 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:118 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:417 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:119 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:418 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:120 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:419 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:121 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:420 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:122 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:421 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:123 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:422 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:124 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:423 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:125 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:424 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:126 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:425 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:127 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:426 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:128 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:427 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:129 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:428 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:130 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:429 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:131 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:430 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:132 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:431 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:133 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:432 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:134 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:433 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:135 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:434 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:136 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:435 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:137 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:436 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:138 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:437 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:139 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:438 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:140 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:439 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:141 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:440 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:142 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:441 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:143 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:442 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:144 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:443 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:145 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:444 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:146 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:445 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:147 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:446 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:148 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:447 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:149 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:448 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:150 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:449 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:151 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:450 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:152 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:451 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:153 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:452 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:154 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:453 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:155 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:454 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:156 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:455 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:157 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:456 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:158 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:457 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:159 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:458 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:160 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:459 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:161 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:460 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:162 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:461 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:163 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:462 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:164 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:463 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:165 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:464 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:166 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:465 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:167 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:466 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:168 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:467 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:169 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:468 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:170 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:469 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:171 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:470 | served: src/fuel.rs |
| FUELSYSTEM LINE FUEL FLOW:172 | A: | SIMCONNECT_DATATYPE_FLOAT64 / GALLONS PER HOUR | read | SimConnectInterface.cpp:471 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:473 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:474 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:475 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:476 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:5 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:477 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:6 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:478 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:7 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:479 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:8 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:480 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:9 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:481 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:10 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:482 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:11 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:483 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:12 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:484 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:13 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:485 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:14 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:486 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:15 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:487 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:16 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:488 | served: src/fuel.rs |
| FUELSYSTEM JUNCTION SETTING:17 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:489 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:491 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:492 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:493 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:494 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:5 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:495 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:6 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:496 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:7 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:497 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:8 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:498 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:9 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:499 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:10 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:500 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:11 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:501 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:12 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:502 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:13 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:503 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:14 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:504 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:15 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:505 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:16 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:506 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:17 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:507 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:18 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:508 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:19 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:509 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:20 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:510 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:21 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:511 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:22 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:512 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:23 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:513 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:24 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:514 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:25 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:515 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:26 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:516 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:27 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:517 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:28 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:518 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:29 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:519 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:30 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:520 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:31 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:521 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:32 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:522 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:33 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:523 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:34 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:524 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:35 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:525 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:36 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:526 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:37 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:527 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:38 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:528 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:39 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:529 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:40 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:530 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:41 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:531 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:42 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:532 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:43 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:533 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:44 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:534 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:45 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:535 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:46 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:536 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:47 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:537 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:48 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:538 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:49 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:539 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:50 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:540 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:51 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:541 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:52 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:542 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:53 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:543 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:54 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:544 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:55 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:545 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:56 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:546 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:57 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:547 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:58 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:548 | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:59 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:549 | served: src/fuel.rs |
| FUELSYSTEM PUMP ACTIVE:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:551 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:552 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:553 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:554 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:5 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:555 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:6 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:556 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:7 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:557 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:8 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:558 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:9 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:559 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:10 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:560 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:11 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:561 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:12 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:562 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:13 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:563 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:14 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:564 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:15 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:565 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:16 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:566 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:17 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:567 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:18 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:568 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:19 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:569 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:20 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:570 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:21 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:571 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:22 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:572 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:23 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:573 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:24 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:574 | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:25 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:575 | served: src/aspects.rs |
| FUELSYSTEM TRIGGER STATUS:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:577 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:578 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:579 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:580 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:5 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:581 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:6 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:582 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:7 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:583 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:8 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:584 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:9 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:585 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:10 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:586 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:11 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:587 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:12 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:588 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:13 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:589 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:14 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:590 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:15 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:591 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:16 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:592 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:17 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:593 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:18 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:594 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:19 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:595 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:20 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:596 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:21 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:597 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:22 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:598 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:23 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:599 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:24 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:600 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:25 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:601 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:26 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:602 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:27 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:603 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:28 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:604 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:29 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:605 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:30 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:606 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:31 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:607 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:32 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:608 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:33 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:609 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:34 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:610 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:35 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:611 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:36 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:612 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:37 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:613 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:38 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:614 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:39 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:615 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:40 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:616 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:41 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:617 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:42 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:618 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:43 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:619 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:44 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:620 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:45 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:621 | served: src/fuel.rs |
| FUELSYSTEM TRIGGER STATUS:46 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:622 | served: src/fuel.rs |
| ELEVATOR TRIM POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / DEGREE | read | SimConnectInterface.cpp:898 | served: src/flight_controls.rs |
| RUDDER TRIM PCT | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT OVER 100 | read | SimConnectInterface.cpp:900 | MISSING |
| GENERAL ENG THROTTLE LEVER POSITION:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:902 | served: src/engine_commands.rs |
| GENERAL ENG THROTTLE LEVER POSITION:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:903 | served: src/engine_commands.rs |
| GENERAL ENG THROTTLE LEVER POSITION:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:904 | served: src/engine_commands.rs |
| GENERAL ENG THROTTLE LEVER POSITION:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / PERCENT | read | SimConnectInterface.cpp:905 | served: src/engine_commands.rs |
| GENERAL ENG THROTTLE MANAGED MODE:1 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:906 | MISSING |
| GENERAL ENG THROTTLE MANAGED MODE:2 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:907 | MISSING |
| GENERAL ENG THROTTLE MANAGED MODE:3 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:908 | MISSING |
| GENERAL ENG THROTTLE MANAGED MODE:4 | A: | SIMCONNECT_DATATYPE_FLOAT64 / NUMBER | read | SimConnectInterface.cpp:909 | MISSING |
| SPOILERS HANDLE POSITION | A: | SIMCONNECT_DATATYPE_FLOAT64 / POSITION | read | SimConnectInterface.cpp:911 | served: src/extra_backend_fcdc.rs |
| KOHLSMAN SETTING STD:4 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:913 | MISSING |
| KOHLSMAN SETTING STD:1 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:915 | MISSING |
| KOHLSMAN SETTING STD:2 | A: | SIMCONNECT_DATATYPE_INT64 / BOOL | read | SimConnectInterface.cpp:916 | MISSING |

### 1.2 Rust `systems_wasm`/`a380_systems_wasm` variable registry — 124 `provides_aircraft_variable` calls

This is what the ported TypeScript `systems` crate (electrical, hydraulics, fuel, flight controls,
etc.) actually reads through `VariableRegistry`/`MsfsVariableRegistry` (`systems_wasm/src/lib.rs:
492-594`). Declared in `a380_systems_wasm/src/lib.rs:429-564` via `.provides_aircraft_variable(name,
unit, index)`; a further 16 are declared inline as `Variable::aircraft(...)` inside the aspect
modules (`anti_ice.rs:22,60`, `electrical.rs:59-60`, `fuel.rs:10`, plus the A380-specific aspects for
payload stations, gear/wheel rotation, spoilers, ADIRS, doors — see `aspects.rs` calls in
`a380_systems_wasm/src`). `sensors.rs`'s own header (fbw-xp-systems) explicitly says the set it
covers is "the one the systems register... not the longer list the MSFS glue declares" — i.e. this
124-row list is a superset of what any single flight actually reads, since not every aspect is
active in every state.

| name (`:index` if any) | kind | units | dir | used by | our coverage |
|---|---|---|---|---|---|
| ACCELERATION BODY X | A: | feet per second squared | read | a380_systems_wasm/src/lib.rs:429 | served: lib.rs mapping() |
| ACCELERATION BODY Y | A: | feet per second squared | read | a380_systems_wasm/src/lib.rs:430 | served: lib.rs mapping() |
| ACCELERATION BODY Z | A: | feet per second squared | read | a380_systems_wasm/src/lib.rs:431 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| AIRSPEED INDICATED | A: | Knots | read | a380_systems_wasm/src/lib.rs:432 | served: lib.rs mapping() |
| AIRSPEED MACH | A: | Mach | read | a380_systems_wasm/src/lib.rs:433 | served: lib.rs mapping() |
| AIRSPEED TRUE | A: | Knots | read | a380_systems_wasm/src/lib.rs:434 | served: lib.rs mapping() |
| AMBIENT DENSITY | A: | Slugs per cubic feet | read | a380_systems_wasm/src/lib.rs:435 | served: lib.rs mapping() |
| AMBIENT IN CLOUD | A: | Bool | read | a380_systems_wasm/src/lib.rs:436 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| AMBIENT PRECIP RATE | A: | millimeters of water | read | a380_systems_wasm/src/lib.rs:437 | served: lib.rs mapping() |
| AMBIENT PRESSURE | A: | inHg | read | a380_systems_wasm/src/lib.rs:438 | served: lib.rs mapping() |
| AMBIENT TEMPERATURE | A: | celsius | read | a380_systems_wasm/src/lib.rs:439 | served: lib.rs mapping() |
| AMBIENT WIND DIRECTION | A: | Degrees | read | a380_systems_wasm/src/lib.rs:440 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| AMBIENT WIND VELOCITY | A: | Knots | read | a380_systems_wasm/src/lib.rs:441 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| AMBIENT WIND X | A: | meter per second | read | a380_systems_wasm/src/lib.rs:442 | served: lib.rs mapping() |
| AMBIENT WIND Y | A: | meter per second | read | a380_systems_wasm/src/lib.rs:443 | served: lib.rs mapping() |
| AMBIENT WIND Z | A: | meter per second | read | a380_systems_wasm/src/lib.rs:444 | served: lib.rs mapping() |
| ANTISKID BRAKES ACTIVE | A: | Bool | read | a380_systems_wasm/src/lib.rs:445 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| CENTER WHEEL ROTATION ANGLE | A: | Degrees | read | a380_systems_wasm/src/lib.rs:446 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| CONTACT POINT COMPRESSION | A: | Percent | read | a380_systems_wasm/src/lib.rs:447 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| CONTACT POINT COMPRESSION:1 | A: | Percent | read | a380_systems_wasm/src/lib.rs:448 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| CONTACT POINT COMPRESSION:2 | A: | Percent | read | a380_systems_wasm/src/lib.rs:449 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| CONTACT POINT COMPRESSION:3 | A: | Percent | read | a380_systems_wasm/src/lib.rs:450 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| CONTACT POINT COMPRESSION:4 | A: | Percent | read | a380_systems_wasm/src/lib.rs:451 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| ENG ON FIRE:1 | A: | Bool | read | a380_systems_wasm/src/lib.rs:452 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| ENG ON FIRE:2 | A: | Bool | read | a380_systems_wasm/src/lib.rs:453 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| ENG ON FIRE:3 | A: | Bool | read | a380_systems_wasm/src/lib.rs:454 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| ENG ON FIRE:4 | A: | Bool | read | a380_systems_wasm/src/lib.rs:455 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:1 | A: | gallons | read | a380_systems_wasm/src/lib.rs:456 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:2 | A: | gallons | read | a380_systems_wasm/src/lib.rs:457 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:3 | A: | gallons | read | a380_systems_wasm/src/lib.rs:458 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:4 | A: | gallons | read | a380_systems_wasm/src/lib.rs:459 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:5 | A: | gallons | read | a380_systems_wasm/src/lib.rs:460 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:6 | A: | gallons | read | a380_systems_wasm/src/lib.rs:461 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:7 | A: | gallons | read | a380_systems_wasm/src/lib.rs:462 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:8 | A: | gallons | read | a380_systems_wasm/src/lib.rs:463 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:9 | A: | gallons | read | a380_systems_wasm/src/lib.rs:464 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:10 | A: | gallons | read | a380_systems_wasm/src/lib.rs:465 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM TANK QUANTITY:11 | A: | gallons | read | a380_systems_wasm/src/lib.rs:466 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| FUELSYSTEM LINE FUEL FLOW:141 | A: | gallons per hour | read | a380_systems_wasm/src/lib.rs:467 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GEAR ANIMATION POSITION | A: | Percent | read | a380_systems_wasm/src/lib.rs:468 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GEAR ANIMATION POSITION:1 | A: | Percent | read | a380_systems_wasm/src/lib.rs:469 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GEAR ANIMATION POSITION:2 | A: | Percent | read | a380_systems_wasm/src/lib.rs:470 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GEAR ANIMATION POSITION:3 | A: | Percent | read | a380_systems_wasm/src/lib.rs:471 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GEAR ANIMATION POSITION:4 | A: | Percent | read | a380_systems_wasm/src/lib.rs:472 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GEAR CENTER POSITION | A: | Percent | read | a380_systems_wasm/src/lib.rs:473 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GEAR LEFT POSITION | A: | Percent | read | a380_systems_wasm/src/lib.rs:474 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GEAR RIGHT POSITION | A: | Percent | read | a380_systems_wasm/src/lib.rs:475 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GENERAL ENG STARTER ACTIVE:1 | A: | Bool | read | a380_systems_wasm/src/lib.rs:476 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GENERAL ENG STARTER ACTIVE:2 | A: | Bool | read | a380_systems_wasm/src/lib.rs:477 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GPS GROUND SPEED | A: | Knots | read | a380_systems_wasm/src/lib.rs:478 | served: lib.rs mapping() |
| GPS GROUND MAGNETIC TRACK | A: | Degrees | read | a380_systems_wasm/src/lib.rs:479 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| GPS GROUND TRUE TRACK | A: | Degrees | read | a380_systems_wasm/src/lib.rs:480 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| INCIDENCE ALPHA | A: | Degrees | read | a380_systems_wasm/src/lib.rs:481 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| INDICATED ALTITUDE | A: | Feet | read | a380_systems_wasm/src/lib.rs:482 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| INTERACTIVE POINT OPEN:0 | A: | Percent | read | a380_systems_wasm/src/lib.rs:483 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| INTERACTIVE POINT OPEN:2 | A: | Percent | read | a380_systems_wasm/src/lib.rs:484 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| INTERACTIVE POINT OPEN:3 | A: | Percent | read | a380_systems_wasm/src/lib.rs:485 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| INTERACTIVE POINT OPEN:10 | A: | Percent | read | a380_systems_wasm/src/lib.rs:486 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| KOHLSMAN SETTING MB:1 | A: | Millibars | read | a380_systems_wasm/src/lib.rs:487 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| LIGHT BEACON | A: | Bool | read | a380_systems_wasm/src/lib.rs:488 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| LIGHT BEACON ON | A: | Bool | read | a380_systems_wasm/src/lib.rs:489 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PLANE ALT ABOVE GROUND | A: | Feet | read | a380_systems_wasm/src/lib.rs:490 | served: lib.rs mapping() |
| PLANE PITCH DEGREES | A: | Degrees | read | a380_systems_wasm/src/lib.rs:491 | served: lib.rs mapping() |
| PLANE BANK DEGREES | A: | Degrees | read | a380_systems_wasm/src/lib.rs:492 | served: lib.rs mapping() |
| PLANE HEADING DEGREES MAGNETIC | A: | Degrees | read | a380_systems_wasm/src/lib.rs:493 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PLANE HEADING DEGREES TRUE | A: | Degrees | read | a380_systems_wasm/src/lib.rs:494 | served: lib.rs mapping() |
| PLANE LATITUDE | A: | degree latitude | read | a380_systems_wasm/src/lib.rs:495 | served: lib.rs mapping() |
| PLANE LONGITUDE | A: | degree longitude | read | a380_systems_wasm/src/lib.rs:496 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PRESSURE ALTITUDE | A: | Feet | read | a380_systems_wasm/src/lib.rs:497 | served: lib.rs mapping() |
| PUSHBACK STATE | A: | Enum | read | a380_systems_wasm/src/lib.rs:498 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PUSHBACK ANGLE | A: | Radians | read | a380_systems_wasm/src/lib.rs:499 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| SEA LEVEL PRESSURE | A: | Millibars | read | a380_systems_wasm/src/lib.rs:500 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| SIM ON GROUND | A: | Bool | read | a380_systems_wasm/src/lib.rs:501 | served: lib.rs mapping() |
| SURFACE TYPE | A: | Enum | read | a380_systems_wasm/src/lib.rs:502 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TOTAL AIR TEMPERATURE | A: | celsius | read | a380_systems_wasm/src/lib.rs:503 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TOTAL WEIGHT | A: | Pounds | read | a380_systems_wasm/src/lib.rs:504 | served: lib.rs mapping() |
| TOTAL WEIGHT YAW MOI | A: | Slugs feet squared | read | a380_systems_wasm/src/lib.rs:505 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TOTAL WEIGHT PITCH MOI | A: | Slugs feet squared | read | a380_systems_wasm/src/lib.rs:506 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TRAILING EDGE FLAPS LEFT PERCENT | A: | Percent | read | a380_systems_wasm/src/lib.rs:507 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TRAILING EDGE FLAPS RIGHT PERCENT | A: | Percent | read | a380_systems_wasm/src/lib.rs:508 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG CORRECTED N1:1 | A: | Percent | read | a380_systems_wasm/src/lib.rs:509 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG CORRECTED N1:2 | A: | Percent | read | a380_systems_wasm/src/lib.rs:510 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG CORRECTED N1:3 | A: | Percent | read | a380_systems_wasm/src/lib.rs:511 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG CORRECTED N1:4 | A: | Percent | read | a380_systems_wasm/src/lib.rs:512 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG CORRECTED N2:1 | A: | Percent | read | a380_systems_wasm/src/lib.rs:513 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG CORRECTED N2:2 | A: | Percent | read | a380_systems_wasm/src/lib.rs:514 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG CORRECTED N2:3 | A: | Percent | read | a380_systems_wasm/src/lib.rs:515 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG CORRECTED N2:4 | A: | Percent | read | a380_systems_wasm/src/lib.rs:516 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG IGNITION SWITCH EX1:1 | A: | Enum | read | a380_systems_wasm/src/lib.rs:517 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG JET THRUST:1 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:518 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG JET THRUST:2 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:519 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG JET THRUST:3 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:520 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| TURB ENG JET THRUST:4 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:521 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| UNLIMITED FUEL | A: | Bool | read | a380_systems_wasm/src/lib.rs:522 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| VELOCITY BODY X | A: | feet per second | read | a380_systems_wasm/src/lib.rs:523 | served: lib.rs mapping() |
| VELOCITY BODY Y | A: | feet per second | read | a380_systems_wasm/src/lib.rs:524 | served: lib.rs mapping() |
| VELOCITY BODY Z | A: | feet per second | read | a380_systems_wasm/src/lib.rs:525 | served: lib.rs mapping() |
| VELOCITY WORLD Y | A: | feet per minute | read | a380_systems_wasm/src/lib.rs:526 | served: lib.rs mapping() |
| WHEEL RPM:1 | A: | RPM | read | a380_systems_wasm/src/lib.rs:527 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| WHEEL RPM:2 | A: | RPM | read | a380_systems_wasm/src/lib.rs:528 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| ROTATION VELOCITY BODY X | A: | degree per second | read | a380_systems_wasm/src/lib.rs:529 | served: lib.rs mapping() |
| ROTATION VELOCITY BODY Y | A: | degree per second | read | a380_systems_wasm/src/lib.rs:530 | served: lib.rs mapping() |
| ROTATION VELOCITY BODY Z | A: | degree per second | read | a380_systems_wasm/src/lib.rs:531 | served: lib.rs mapping() |
| PAYLOAD STATION WEIGHT:1 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:547 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:2 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:548 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:3 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:549 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:4 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:550 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:5 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:551 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:6 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:552 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:7 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:553 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:8 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:554 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:9 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:555 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:10 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:556 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:11 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:557 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:12 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:558 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:13 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:559 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:14 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:560 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:15 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:561 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:16 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:562 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:17 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:563 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |
| PAYLOAD STATION WEIGHT:18 | A: | Pounds | read | a380_systems_wasm/src/lib.rs:564 | check sensors.rs/fadec.rs/prim.rs (own doc tables) or missing |

### 1.3 JS instruments — `SimVar.GetSimVarValue`/`SetSimVarValue`

1,093 call sites across `fbw-a380x/src/systems` and `fbw-common/src/systems`, naming 621 distinct
simvar strings (many indexed, e.g. `CIRCUIT CONNECTION ON:2..81`). Of those, 144 are variables the
`SimVar` helper treats as `A:` (explicit `A:` prefix, 42; or a bare name matching MSFS's known
simvar list, 102 — `SimVar.GetSimVarValue` treats an unprefixed name as an aircraft variable by
default). The rest are `L:` (444, FBW's own named vars — layer-agnostic to MSFS, any host must
support generic local-variable storage but these are not part of "the MSFS contract" per se), `H:`
(7 literal + parameterised), `C:` (3, input controllers), and `K:` (20, covered in Layer 3).

| name | kind | units | dir | used by | our coverage |
|---|---|---|---|---|---|
| CABIN SEATBELTS ALERT SWITCH | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/rpn.rs |
| CIRCUIT CONNECTION ON:2 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:64 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:65 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:66 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:67 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:68 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:69 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:70 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:71 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:72 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:73 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:74 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:75 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:76 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:77 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:78 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:79 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:80 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:81 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT SWITCH ON:151 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/key_events.rs |
| EMPTY WEIGHT | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/efb.rs |
| ENG ANTI ICE:1 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| ENG ANTI ICE:2 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| ENG ANTI ICE:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| ENG ANTI ICE:4 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT GOAL:${index} | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| INTERACTIVE POINT OPEN:0 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT OPEN:1 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT OPEN:10 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT OPEN:2 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT OPEN:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT OPEN:5 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT OPEN:8 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT OPEN:9 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| LIGHT BEACON | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/aircraft_presets.rs |
| ON ANY RUNWAY | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend_fbw.rs |
| PLANE ALTITUDE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/simvar.js |
| PLANE LATITUDE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend_fbw.rs |
| PLANE LONGITUDE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/simvar.js |
| STRUCTURAL DEICE SWITCH | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| ACCELERATION BODY X | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/pushback.rs |
| ACCELERATION BODY Y | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend_fbw.rs |
| ACCELERATION BODY Z | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/lib.rs |
| AIRCRAFT WIND Z | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| AIRSPEED INDICATED | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/engine_commands.rs |
| AMBIENT IN CLOUD | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/sensors.rs |
| ANTISKID BRAKES ACTIVE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend_fcdc.rs |
| ATC FLIGHT NUMBER | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| AUTOPILOT ALTITUDE LOCK VAR:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/environment.js |
| AUTOPILOT FLIGHT DIRECTOR ACTIVE:1 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| AUTOPILOT FLIGHT DIRECTOR ACTIVE:2 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| BUS LOOKUP INDEX | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| CABIN SEATBELTS ALERT SWITCH | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/rpn.rs |
| CAMERA STATE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| CHASE CAMERA HEADLOOK | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| CIRCUIT CONNECTION ON:2 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:64 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:65 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:66 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:67 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:68 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:69 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:70 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:71 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:72 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:73 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:74 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:75 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:76 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:77 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:78 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:79 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:80 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| CIRCUIT CONNECTION ON:81 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/circuits.rs |
| COCKPIT CAMERA HEADLOOK | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| COM ACTIVE FREQUENCY:${this.index} | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/radios.rs |
| COM STANDBY FREQUENCY:${this.index} | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/radios.rs |
| ELEVATOR TRIM POSITION | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/flight_controls.rs |
| ENG N1 RPM:1 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| ENG N1 RPM:2 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| ENG N1 RPM:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| ENG N1 RPM:4 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| FUELSYSTEM PUMP ACTIVE:1 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:2 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:4 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:5 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| FUELSYSTEM PUMP ACTIVE:6 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| FUELSYSTEM TANK WEIGHT:2 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/fuel.rs |
| FUELSYSTEM TANK WEIGHT:5 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/fuel.rs |
| FUELSYSTEM TANK WEIGHT:6 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/fuel.rs |
| FUELSYSTEM TANK WEIGHT:9 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:46 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:47 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:48 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/fuel.rs |
| FUELSYSTEM VALVE OPEN:49 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/fuel.rs |
| GEAR HANDLE POSITION | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| GEAR POSITION:0 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| GEAR STEER ANGLE PCT:0 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| GEAR TOTAL PCT EXTENDED | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| GLASSCOCKPIT AUTOMATIC BRIGHTNESS | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend_fbw.rs |
| GPS GROUND TRUE TRACK | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/sensors.rs |
| GPS POSITION LAT | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| GPS POSITION LON | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| INDICATED ALTITUDE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/environment.js |
| INDICATED ALTITUDE:4 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/environment.js |
| INTERACTIVE POINT OPEN:0 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| INTERACTIVE POINT OPEN:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/aspects.rs |
| IS SLEW ACTIVE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| LIGHT BEACON | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/aircraft_presets.rs |
| LIGHT WING | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| MARKER SOUND | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/radios.rs |
| NAV LOCALIZER:3 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/radios.rs |
| PLANE ALT ABOVE GROUND | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/environment.js |
| PLANE ALTITUDE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/simvar.js |
| PLANE HEADING DEGREES MAGNETIC | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/prim.rs |
| PLANE HEADING DEGREES TRUE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/lib.rs |
| PLANE LATITUDE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend_fbw.rs |
| PLANE LONGITUDE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/simvar.js |
| PLANE PITCH DEGREES | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/lib.rs |
| PUSHBACK AVAILABLE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| RELATIVE WIND VELOCITY BODY Z | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| ROTATION ACCELERATION BODY X | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/pushback.rs |
| ROTATION ACCELERATION BODY Y | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend_fbw.rs |
| ROTATION ACCELERATION BODY Z | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/lib.rs |
| ROTATION VELOCITY BODY X | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/lib.rs |
| ROTATION VELOCITY BODY Y | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/pushback.rs |
| ROTATION VELOCITY BODY Z | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/lib.rs |
| RUDDER PEDAL POSITION | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| RUDDER TRIM PCT | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| SIM ON GROUND | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/efb.rs |
| STEER INPUT CONTROL | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| STRUCTURAL ICE PCT | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| TITLE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| TOTAL AIR TEMPERATURE | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/engine_commands.rs |
| TRANSPONDER STATE:${this.activeXpdr + 1} | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| TRANSPONDER STATE:1 | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | MISSING |
| VELOCITY BODY X | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/lib.rs |
| VELOCITY BODY Y | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/pushback.rs |
| VELOCITY BODY Z | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/extra_backend/pushback.rs |
| VERTICAL SPEED | A: | (per call) | read/write | JS instruments (SimVar.*SimVarValue) | served: src/js/msfs/environment.js |

### 1.4 SimConnect client data areas (C++ PRIM/SEC/FCU/FADEC/ADR/IR/RA/LGCIU/SFCC/ILS bus simulation)

Separate from the 481-field struct above: `SimConnectInterface.h`'s `ClientData` enum has 50
entries — one `SimConnect_CreateClientData`/`SimConnect_MapClientDataNameToID` pair per computer's
discrete-input, analog-input, discrete-output, analog-output and inter-computer "bus" blocks (e.g.
`A32NX_CLIENT_DATA_PRIM_DISCRETE_INPUT` at `SimConnectInterface.cpp:941-1032`, continuing through
`SEC_*`, `FCU_*`, `FADEC_*`, `ADR_*`, `IR_*`, `RA_*`, `LGCIU_*`, `SFCC_*`, `ILS_*`). These are raw
C structs (not named `A:`/`L:` variables) shared between the WASM module and itself across frames
(and, for some, with other gauges) — they exist so PRIM/SEC/FADEC can talk to each other and to the
EFB/ECAM the same way real LRUs would over an ARINC bus, not because MSFS needs them. **Our
coverage: served, by a different design, in `prim.rs`.** `prim.rs`'s own header says it is a direct
port of `FlyByWireInterface.cpp`'s `updateRa`/`updateLgciu`/`updateSfcc`/`updateIls`/`updateAdirs`/
`updateFqms`/`updateTcas`/`updateAesu`/`updateFcu*`/`updatePrim*`/`updateSec*` call sequence, driving
the same compiled C++ computer objects (`fbw_computers.rs`) directly with bus structs assembled from
the Rust systems' variables — bypassing SimConnect client data entirely, since everything now runs
in one process. `prim.rs::UNAVAILABLE` lists each specific input this port has no source for (e.g.
the calculated-radio-receiver localizer distance without DME, some pitch-trim-switch discretes) —
that list is effectively the real gap list for this layer, not the client-data mechanism itself.

---

## Layer 2 — Environment variables (`E:`) and `GAME:` strings

| name | kind | units/type | dir | used by | our coverage |
|---|---|---|---|---|---|
| E:ABSOLUTE TIME | E: | seconds since year 0 | read | JS `environment.js`-equivalent consumers, e.g. `RmpStateController.ts` clock code | served: `js_bridge.rs:259` `Env::AbsoluteTime` |
| E:SIMULATION TIME | E: | seconds | read | `SimulationTime` data def, `systems_wasm/src/lib.rs:596-605`, requested every `Period::VisualFrame` — this **is** the WASM module's delta-time clock | served: `js_bridge.rs:252` `Env::SimulationTime`; `xp.rs`/`lib.rs` flight_loop drives the plugin's own delta time |
| E:LOCAL TIME / E:ZULU TIME | E: | seconds since midnight | read | cockpit clock instruments | served: `js_bridge.rs:281-282` |
| E:ZULU DAY OF MONTH / ZULU MONTH OF YEAR / ZULU YEAR / ZULU DAY OF WEEK | E: | int | read | clock/FMS date | served: `js_bridge.rs:267-269` (day, month, year; day-of-week not directly grepped, check `js_bridge.rs`) |
| E:TIME OF DAY | E: | enum (0-3) | read | lighting logic (day/dusk/night) | served: `js_bridge.rs:285`, `crate::extra_backend_fbw::time_of_day()` |
| E:IS AIRCRAFT | E: | bool | read | sound/AI-traffic filtering code | not directly located in `js_bridge.rs`'s grepped `Env` variants — check |
| GAME:GetGameState / GAME: strings generally | GAME: | string/enum | read | `GameStateProvider` (12 files), flight-load lifecycle gating | served: `js_bridge.rs:231,369,644` `Target::Game`, logs once per unknown name and reads as empty string |

Our own `js_bridge.rs` module doc (lines 18-27) already states the contract precisely: `E:` names
are the simulator's clock, `GAME:` names are MSFS game variables, both resolved through
`Env`/`Target::Game`. This layer is well covered; the only open item is confirming every literal
`E:`/`GAME:` name FBW actually reads (not just the classes) is in `js_bridge.rs`'s match arms —
not fully cross-checked in this pass (small, bounded list — 9 distinct `E:` strings found).

---

## Layer 3 — Key events (`K:`), `H:` events, `B:`/input events

### 3.1 C++ `Events` enum — 243 entries, mapped 1:1 via `SimConnect_MapClientEventToSimEvent`

`SimConnectInterface.cpp` calls `addInputDataDefinition` (which itself calls
`SimConnect_MapClientEventToSimEvent` then `SimConnect_AddClientEventToNotificationGroup`) 233
times against the enum's 243 entries (some entries are handled without going through
`addInputDataDefinition` — e.g. custom `A32NX_*` events sent only via `SimConnect_TransmitClientEvent`/
`_EX1`, used 2 places in the file, `SimConnectInterface.cpp:1494,1513`). These cover: primary flight
control axes/discretes (elevator/aileron/rudder/trim), autopilot master/FD/AP-disengage, the whole
FCU button/knob set (`A32NX_FCU_*`, ~90 entries), EFIS control panel, radios (COM/NAV/ADF/XPNDR/ATC),
sim rate, pushback/ground services, lighting, and fuel/electrical/ignition discretes used by the
aircraft-preset procedures. `processKeyEvent` (registered via `register_key_event_handler_EX1`,
`main.cpp:87`) is the receiving side for events MSFS (or another add-on) sends back at FBW.

| event name | kind | args | dir | used by | our coverage |
|---|---|---|---|---|---|
| AXIS_ELEVATOR_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AXIS_AILERONS_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AXIS_RUDDER_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_LEFT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_AXIS_PLUS | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_CENTER | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_RIGHT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_AXIS_MINUS | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_TRIM_LEFT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_TRIM_RESET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_TRIM_RIGHT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| RUDDER_TRIM_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| RUDDER_TRIM_SET_EX1 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AILERON_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AILERONS_LEFT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AILERONS_RIGHT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| CENTER_AILER_RUDDER | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| ELEVATOR_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| ELEV_DOWN | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| ELEV_UP | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| ELEV_TRIM_DN | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| ELEV_TRIM_UP | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| ELEVATOR_TRIM_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AXIS_ELEV_TRIM_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_MASTER | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| AUTOPILOT_OFF | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| AUTOPILOT_ON | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AUTOPILOT_DISENGAGE_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AUTOPILOT_DISENGAGE_TOGGLE | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| TOGGLE_FLIGHT_DIRECTOR | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| A32NX_AUTOPILOT_DISENGAGE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_AP_1_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_AP_2_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_AP_DISCONNECT_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_ATHR_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_ATHR_DISCONNECT_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_FD_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_SPD_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_SPD_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_SPD_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_SPD_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_SPD_PULL | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_SPD_MACH_TOGGLE_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_HDG_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_HDG_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_HDG_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_HDG_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_HDG_PULL | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_TRK_FPA_TOGGLE_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_TRUE_TOGGLE_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_ALT_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_ALT_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_ALT_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_ALT_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_ALT_PULL | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_ALT_INCREMENT_TOGGLE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_ALT_INCREMENT_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_METRIC_ALT_TOGGLE_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_VS_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_VS_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_VS_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_VS_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_VS_PULL | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_LOC_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_APPR_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_ALT_BUTTON_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_FCU_EFIS_L_RANGE_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_RANGE_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_RANGE_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_MODE_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_MODE_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_MODE_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_VV_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_LS_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_TAXI_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_BARO_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_BARO_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_BARO_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_BARO_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_BARO_PULL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_CSTR_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_WPT_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_VORD_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_NDB_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_ARPT_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_NAVAID_1_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_NAVAID_1_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_NAVAID_2_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_NAVAID_2_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_WX_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_TERR_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_L_TRAF_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_RANGE_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_RANGE_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_RANGE_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_MODE_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_MODE_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_MODE_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_VV_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_LS_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_TAXI_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_BARO_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_BARO_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_BARO_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_BARO_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_BARO_PULL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_CSTR_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_WPT_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_VORD_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_NDB_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_ARPT_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_NAVAID_1_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_NAVAID_1_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_NAVAID_2_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_NAVAID_2_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_WX_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_TERR_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FCU_EFIS_R_TRAF_PUSH | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FMGC_DIR_TO_TRIGGER | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FMGC_PRESET_SPD_ACTIVATE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FMGC_SPD_MODE_ACTIVATE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_FMGC_MACH_MODE_ACTIVATE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_EFIS_L_CHRONO_PUSHED | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_EFIS_R_CHRONO_PUSHED | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_AIRSPEED_ON | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_AIRSPEED_OFF | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_HDG_HOLD_ON | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_HDG_HOLD_OFF | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_ALT_HOLD_ON | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_ALT_HOLD_OFF | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_VS_ON | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_VS_OFF | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_SPEED_SLOT_INDEX_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_SPD_VAR_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_SPD_VAR_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_SPD_VAR_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_MACH_VAR_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_MACH_VAR_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_HEADING_SLOT_INDEX_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| HEADING_BUG_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| HEADING_BUG_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| HEADING_BUG_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_ALTITUDE_SLOT_INDEX_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_ALT_VAR_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_ALT_VAR_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_ALT_VAR_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_VS_SLOT_INDEX_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_VS_VAR_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_VS_VAR_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_APR_HOLD | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_LOC_HOLD | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_ALT_HOLD | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_VS_HOLD | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_ATT_HOLD | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AP_MACH_HOLD | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| KOHLSMAN_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| KOHLSMAN_INC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| KOHLSMAN_DEC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| BAROMETRIC_STD_PRESSURE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| BAROMETRIC | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AUTO_THROTTLE_ARM | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| AUTO_THROTTLE_DISCONNECT | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| A32NX_AUTO_THROTTLE_DISCONNECT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AUTO_THROTTLE_TO_GA | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_ATHR_RESET_DISABLE | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/afs_events.rs |
| A32NX_THROTTLE_MAPPING_SET_DEFAULTS | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_THROTTLE_MAPPING_LOAD_FROM_FILE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_THROTTLE_MAPPING_LOAD_FROM_LOCAL_VARIABLES | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| A32NX_THROTTLE_MAPPING_SAVE_TO_FILE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE1_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE2_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE3_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE4_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_AXIS_SET_EX1 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE1_AXIS_SET_EX1 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE2_AXIS_SET_EX1 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE3_AXIS_SET_EX1 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE4_AXIS_SET_EX1 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_FULL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_CUT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_INCR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_DECR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_INCR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_DECR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_10 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_20 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_30 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_40 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_50 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_60 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_70 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_80 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_90 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE1_FULL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE1_CUT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE1_INCR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE1_DECR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE1_INCR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE1_DECR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE2_FULL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE2_CUT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE2_INCR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE2_DECR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE2_INCR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE2_DECR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE3_FULL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE3_CUT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE3_INCR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE3_DECR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE3_INCR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE3_DECR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE4_FULL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE4_CUT | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE4_INCR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE4_DECR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE4_INCR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE4_DECR_SMALL | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_REVERSE_THRUST_TOGGLE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| THROTTLE_REVERSE_THRUST_HOLD | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| FLAPS_UP | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| FLAPS_1 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| FLAPS_2 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| FLAPS_3 | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| FLAPS_DOWN | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| FLAPS_INCR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| FLAPS_DECR | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| FLAPS_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AXIS_FLAPS_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SPOILERS_ON | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SPOILERS_OFF | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SPOILERS_TOGGLE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SPOILERS_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| AXIS_SPOILER_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SPOILERS_ARM_ON | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SPOILERS_ARM_OFF | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SPOILERS_ARM_TOGGLE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SPOILERS_ARM_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| SIM_RATE_INCR | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| SIM_RATE_DECR | K: | - | send/intercept | SimConnectInterface.h Events enum | served: src/key_events.rs |
| SIM_RATE_SET | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |
| SYSTEM_EVENT_PAUSE | K: | - | send/intercept | SimConnectInterface.h Events enum | MISSING (may be native X-Plane axis/default input) |

**Reading the MISSING marks above:** roughly half of the 243 are MSFS's *stock* flight-control axis
and discrete events (`AXIS_ELEVATOR_SET`, `RUDDER_LEFT`, `AILERONS_RIGHT`, `ELEV_TRIM_DN`, standard
radio `COM_RADIO_SET_HZ` family, etc.) — under `fbw-xp-systems`'s design these are not something to
"emulate as MSFS events" so much as native X-Plane axis/command input that should be wired directly;
a MISSING mark there is a design question, not necessarily an implementation gap. The genuinely
FBW-specific ones worth checking one at a time are the `A32NX_FCU_*`, `A32NX_EFIS_*`, and
`A32NX_ATHR_*` families, since those are FlyByWire's own input surface with no native X-Plane
equivalent — `afs_events.rs` (285 lines) is exactly the module that should own them; confirm each
family against it directly rather than trusting the crude grep above.

### 3.2 `key_events.rs` — our own already-written K: contract (most authoritative source for this layer)

`fbw-xp-systems/src/key_events.rs`'s module doc (lines 1-40+) is itself a hand-built, sourced table
of every non-radio, non-door, non-pushback `K:` event FBW sends, with the MSFS effect and the exact
sender (file:line) for each — `LIGHT_POTENTIOMETER_SET`, `ELECTRICAL_CIRCUIT_TOGGLE`,
`ELECTRICAL_BUS_TO_CIRCUIT_CONNECTION_TOGGLE`, `FUELSYSTEM_VALVE_OPEN/_CLOSE`,
`TURBINE_IGNITION_SWITCH_SETn`, `CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE`, `SPOILERS_ARM_SET`,
`RUDDER_TRIM_SET`, light switches, `XPNDR_SET`/`XPNDR_IDENT_ON`, `SIM_RATE_INCR`/`_DECR`,
`TOGGLE_JETWAY`/`TOGGLE_RAMPTRUCK`/`REQUEST_*` ground services, `REQUEST_POWER_SUPPLY`, and the
`A32NX.FCU_*`/`AP_*`/`AUTO_THROTTLE_*` family routed to `afs_events.rs`. Treat this file as the
ground truth for **served** `K:` writes; it is more precise than the grep-based table above.

### 3.3 JS `K:` writes (`SimVar.SetSimVarValue('K:...')`), 27 distinct names found

`K:A32NX.ATHR_RESET_DISABLE`, `K:A32NX.FCU_ALT_PUSH`, `K:A32NX.FCU_ALT_SET`,
`K:A32NX.FMGC_DIR_TO_TRIGGER`, `K:A32NX.FMS_PRESET_SPD_ACTIVATE`,
`K:A32NX.THROTTLE_MAPPING_LOAD_FROM_FILE/_LOCAL_VARIABLES`, `K:A32NX.THROTTLE_MAPPING_SAVE_TO_FILE`,
`K:A32NX.THROTTLE_MAPPING_SET_DEFAULTS`, `K:AP_MANAGED_SPEED_IN_MACH_OFF/_ON`,
`K:COM_2_RADIO_SET_HZ`, `K:COM_RADIO_SET_HZ`, `K:COM_STBY_RADIO_SET_HZ`,
`K:ELECTRICAL_CIRCUIT_TOGGLE`, `K:REQUEST_CATERING/_FUEL_KEY/_LUGGAGE/_POWER_SUPPLY`,
`K:SIM_RATE_DECR/_INCR`, `K:TOGGLE_AIRCRAFT_EXIT/_JETWAY/_PUSHBACK/_RAMPTRUCK`, `K:TUG_DISABLE`,
`K:XPNDR_IDENT_ON`, `K:XPNDR_SET`. **Our coverage: served** — every one of these is named explicitly
in `key_events.rs`'s doc table (3.2) or `radios.rs`/`afs_events.rs`.

### 3.4 `H:` events — 573 names in our own build tool, only 11 literal names found by grep in FBW's TS

`H:A32NX_ISIS_BUGS_PRESSED/_RELEASED`, `H:A32NX_ISIS_LS_PRESSED`, `H:A32NX_RMP_LEFT_TOGGLE_SWITCH`,
`H:A32NX_SD_STS_NEXT_PAGE`, and the parameterised `` H:RMP_{1,2,3}_VHF_CALL_${vhfIndex}_PRESSED/_RELEASED ``
(`RmpStateController.ts` and similar). Separately, the behaviour XML under
`fbw-a380x/src/base/**/model/behaviour` sends/reads 16 distinct `H:` cockpit-click events (via
`(>H:NAME)`/`(H:NAME)` RPN in `<Component>`/`<CURSOR>` blocks) and 400 total `A:`/`K:` RPN references
across those files combined. **Our coverage: served — `js_bridge.rs`'s doc (line 27) states `H:`
events "reach every instrument, and `take_events` too"**, and `tools/js-build/hevents.txt` already
enumerates 573 `H:` names (larger than what this pass found by direct grep, meaning that build tool
was generated by scanning more than the plain `.ts` source — likely also the compiled XML/instrument
bundles). No further gap expected here beyond keeping that generated list in sync with FBW updates.

### 3.5 `B:` vars / input events

No literal `B:`-prefixed variable strings were found by grep in `fbw-a380x/src/systems` or
`fbw-common/src/systems` TypeScript. `C:` (input controller) strings: 3 found, all
`SimVar`-namespace input state reads, not separately catalogued here for time; low priority.

---

## Layer 4 — MSFS C/C++ gauge API used by the WASM

| function | purpose | caller | our coverage |
|---|---|---|---|
| `PANEL_SERVICE_PRE_INSTALL` | one-time init when the gauge is installed | `main.cpp:12` (`gauge_callback`/`fbw_a380_gauge_callback` switch) | served conceptually: `xp.rs`/`lib.rs` plugin start (`XPluginStart`) is the equivalent one-time init |
| `PANEL_SERVICE_PRE_DRAW` | per-frame update, this is where `FlyByWireInterface::update`/`MsfsHandler::handle(PreDraw)` are driven | `main.cpp:17` | served: `lib.rs:1229` `flight_loop`, registered via `XPLMRegisterFlightLoopCallback` (`xp.rs:263`) |
| `PANEL_SERVICE_PRE_KILL` | teardown | `main.cpp:22` | served: `lib.rs:1325` unregister on plugin stop |
| `register_key_event_handler_EX1` / `unregister_key_event_handler_EX1` | receive MSFS key events sent by other add-ons/cockpit | `main.cpp:87,103`, callback `SimConnectInterface::processKeyEvent` (`SimConnectInterface.cpp:1795`) | served differently: X-Plane commands (`fbw/hevent/...`, custom commands) are the equivalent input path per `js_bridge.rs`'s doc |
| `execute_calculator_code` | run RPN "gauge calculator" strings, mostly to set `H:`/`L:` vars MSFS's own default `A320_Neo_*` gauges watch, or toggle FBW's own bools | 17 call sites, all in `FlyByWireInterface.cpp` and `SimConnectInterface.cpp` (e.g. `SimConnectInterface.cpp:2404` `(L:A32NX_FCU_ALT_INCREMENT_1000, bool) ! (>L:A32NX_FCU_ALT_INCREMENT_1000)`) | partial: our `H:`/`L:` var plumbing exists (`js_bridge.rs`), but there is no general RPN/calculator-code interpreter noted in this pass — each of the 17 call sites' effect would need to be hand-translated, not executed generically |
| `trigger_key_event` / `trigger_key_event_ex1` | send a key event with up to 3 extra args, from Rust (`systems_wasm`) | 12 call sites across `autobrakes.rs`, `brakes.rs`, `flaps.rs`, `gear.rs`, `nose_wheel_steering.rs`, `aspects.rs` (all A380/common wasm) | served: our `key_events.rs`/`xp.rs` model — needs confirming these 12 specific triggers are represented, not done individually in this pass |
| `fsVarsGetAVar` / equivalent low-level `fsVars*` C API | not found used directly — FBW's C++ uses SimConnect's typed data definitions instead of the raw `fsVars` gauge API | — | n/a — not part of FBW's actual contract |
| `fsNetwork*` (HTTP) | none found in `fbw_a380`/`extra-backend*`/`extra-backend-a380x` | — | n/a |
| MSFS file I/O (`\work` folder) | flight data recorder writes/reads `.fdr` files | `FlightDataRecorder.cpp:165,207` | not checked against our repo in this pass — likely low priority (debug/black-box recorder, not required for flight) |
| sound (`fsSound`) | none found | — | n/a |
| render (NanoVG) | none found — FBW's WASM does no direct rendering; all rendering is the JS/Coherent instruments | — | n/a |

FBW's C++ WASM is unusually "clean" here: no sound, no direct rendering, no HTTP — its footprint is
almost entirely SimConnect data/events plus `execute_calculator_code` for a handful of legacy
`A320_Neo_*` gauge interop points and its own `H:`/`L:` toggles.

---

## Layer 5 — Coherent/JS runtime API

### 5.1 `Coherent.call` — 16 distinct names found

`COMM_BUS_WASM_CALLBACK`, `GET_AIR_TRAFFIC`, `GET_METAR_BY_IDENT`, `GET_TAF_BY_IDENT`,
`LOAD_AIRPORT`, `LOAD_AIRPORT_FROM_STRUCT`, `LOAD_INTERSECTION`, `LOAD_NDB`, `LOAD_VOR`,
`OPEN_WEB_BROWSER`, `PLAY_INSTRUMENT_SOUND`, `SEARCH_BY_IDENT`, `SEARCH_NEAREST`,
`START_NEAREST_SEARCH_SESSION`, `TOOLBAR_SET_ACTIVE_PAUSE`, `setValueReg_Number`,
`setValueReg_String`. Most of these are facility/navdata lookups going through msfs-sdk's
`FacilityLoader`/`NearestSearchSession` wrappers (Layer 6) rather than called raw by FBW code.
**Our coverage: not checked directly in this pass** — `js_bridge.rs`/`js/msfs/coherent.js` (225
lines) is presumably the shim for this surface; needs a line-by-line check against this list of 16,
not done here for time.

### 5.2 `Coherent.on` — 11 distinct event names found

`A32NX_FM_DEBUG_LNAV_STATUS`, `A32NX_FM_DEBUG_VNAV_STATUS`, `FBW_NXDATASTORE_UPDATE`,
`` FBW_POP_${id}_NO/_YES `` (EFB popup responses), `NearestSearchCompleted[WithStruct]`,
`OnInteractionEvent`, `SendAirport`/`SendIntersection`/`SendNdb`/`SendVor` (facility load callbacks).

### 5.3 JS globals/classes FBW/msfs-sdk expect from MSFS's core JS

| global/class | files using it | our coverage |
|---|---|---|
| `RegisterViewListener` | 18 files; names used: `JS_LISTENER_FACILITY`, `JS_LISTENER_NOTIFICATIONS`, `JS_LISTENER_POPUP`, `JS_LISTENER_SIMVARS`, `JS_LISTENER_TOOLBAR_PANELS` | check `js/msfs/coherent.js`/`instrument.js` — not verified per-listener in this pass |
| `BaseInstrument` | 26 files (every instrument's base class) | served: `js/msfs/instrument.js` (878 lines) is presumably this shim |
| `GameStateProvider` / `GameState.*` | 12 / 13 files (flight-load gating, e.g. don't run systems until `ingame`) | served: `js_bridge.rs` `GAME:` handling (Layer 2) |
| `GetStoredData` / `SetStoredData` | 3 files each | served: `js/msfs/environment.js` and `js/msfs/mod.rs` implement it (not `persistence.rs`, which is a separate airframe-wear/failures store) |
| `LaunchFlowEvent` | 2 files | not verified |
| `NXDataStore` | 58 files — FBW's own persistence wrapper, built on `GetStoredData`/`SetStoredData` or `DataStore` | served: same as above, plus `efb.rs` references `NXDataStore` directly |
| `VCockpit.` | 1 file | not verified |
| `coui://` asset scheme | 1 file | not verified |
| `/VFS/` asset paths | 6 files | check `js/msfs/dom_standin.js` (839 lines, likely the `fetch`/DOM shim) — not verified per-path in this pass |
| `TemplateElement`, custom-element lifecycle | not separately searched (implied by every Instrument.tsx `customElements.whenDefined` pattern, standard msfs-sdk usage) | check `js/msfs/dom_standin.js` |

`persistence.rs` (in `fbw-xp-systems`) is also a strong candidate for where `GetStoredData`/
`SetStoredData`/`NXDataStore` land — not opened in this pass; flagged for follow-up rather than
guessed at.

---

## Layer 6 — Facilities / nav data API

FBW's own MSFS-backed navdata client lives entirely in
`fbw-common/src/systems/navdata/client/backends/Msfs/` (`FacilityCache.ts`, `FsTypes.ts`,
`Mapping.ts`, `Msfs.ts`, `MsfsNearbyFacilityMonitor.ts`, `NearbyFacilityCache.ts`) — this is the
whole surface, not scattered calls. High-level shape: `FacilityCache.searchByIdent<T>(ident,
IcaoSearchFilter, maxResults)` is the workhorse (used by `Mapping.ts:1783`, `Msfs.ts:239,257,275,294`
for intersections/NDBs/VORs), backed by MSFS's facility loader + `START_NEAREST_SEARCH_SESSION`/
`SEARCH_NEAREST` (Layer 5.1/5.2) for proximity queries and `LOAD_AIRPORT`/`LOAD_INTERSECTION`/
`LOAD_NDB`/`LOAD_VOR` (Coherent calls) for direct ICAO loads, with responses arriving as
`SendAirport`/`SendIntersection`/`SendNdb`/`SendVor`/`NearestSearchCompleted[WithStruct]` events.
`MfdFmsDataAirport.tsx` additionally uses `loadAirport`/`loadAllRunways` from
`@fmgc/flightplanning/DataLoading`, and `OANC/Oanc.tsx` loads airport maps (taxiway/stand layout)
through the same cache for the Onboard Airport Navigation Chart.

**Our coverage: not checked in this pass beyond confirming it is out of scope for a "thinks it's
still in MSFS" runtime emulation in the narrow sense** — FBW's own navdata story already has a
non-MSFS backend option (their navdata server), and `fbw-xp-systems`'s `src/navdata` and `src/oans`
and `src/mapdata` directories (seen in the earlier directory listing) suggest this is handled as its
own workstream, separate from the runtime `A:`/`K:`/gauge contract this brief otherwise covers. Flag
for a dedicated follow-up brief rather than folding in here superficially.

---

## Layer 7 — Config/data files read from the package at runtime

| file | sections/keys actually read by FBW code (not just present in the package) | evidence |
|---|---|---|
| `systems.cfg` | `[ELECTRICAL]` bus definitions, consumed by `with_electrical_buses` (`systems_wasm/src/msfs.rs` wiring, `electrical.rs`) | `MsfsSimulationBuilder::with_electrical_buses` doc, `msfs.rs:96-106`; `electrical.rs:105` comment references `apu_pct_rpm_per_second` from systems.cfg |
| `flight_model.cfg` | contact-point/gear geometry (max compression column 9, per `sensors.rs`'s own table); interactive-point indices for doors (`INTERACTIVE POINT OPEN:n`, doors listed at `flight_model.cfg:826-837` per `sensors.rs`) | `fbw-xp-systems/src/sensors.rs` header |
| `.FLT` files (`apron.FLT`, `hangar.flt`, `taxi.flt`, `runway.FLT`, `cruise.FLT`, `final.FLT`, `approach.FLT`, `Climb.flt`) | `A32NX_START_STATE` value per file (apron=2, hangar=1, taxi=3, runway=4, cruise=6, final=8), `BatterySwitch` (hangar vs. apron) | `fbw-xp-systems/src/start_state.rs` header, with exact line numbers per file already recorded there (e.g. `apron.FLT:398`, `hangar.flt:212,375`) |
| `panel.cfg` | `[VCockpitNN]` sections — 24 found in the interior cockpit's `panel.cfg`, each naming an HTML gauge FBW's build outputs into `html_ui/Pages/VCockpit/Instruments/A380X/...` | `js_bridge.rs` module doc lines 3-13 |
| `engines.cfg`, `cockpit.cfg` | not independently confirmed read by FBW's own Rust/C++ code in this pass (vs. just present for MSFS's own engine/cockpit modelling) — flag for follow-up | — |
| behaviour XML (`model/behaviour/**`, 36 files) | cockpit switch/knob/cursor bindings: `<SIMVAR>`, `<CURSOR>`, `<POTENTIOMETER>`, `<TOOLTIPID>` elements, RPN `(A:...)`/`(>K:...)` — 400 `A:`/`K:` references total | `fbw-a380x/src/base/.../model/behaviour/*.xml`; conversion already owned by `D:\A380\msfs2xp-aircraft` per the task's own framing, not `fbw-xp-systems` |

**Our coverage:** the `.FLT`-driven start-state contract and the `systems.cfg`/`flight_model.cfg`
pieces above are explicitly, precisely ported (`start_state.rs`, `sensors.rs`, `electrical.rs`'s
Rust-side comment). Behaviour-XML conversion is explicitly out of `fbw-xp-systems`'s scope (it is
`msfs2xp-aircraft`'s job per the task brief) — treat that as a different contract, already handled
elsewhere, not a gap in this repo.

---

## Layer 8 — Timing/lifecycle assumptions

| assumption | MSFS side | our coverage |
|---|---|---|
| Per-frame update cadence | `PANEL_SERVICE_PRE_DRAW` (gauge callback, `main.cpp:17`) drives `FlyByWireInterface::update`; the Rust side's `MsfsHandler::handle(MSFSEvent::PreDraw)` (`systems_wasm/src/lib.rs:228-244`) drives `Simulation::tick`. Both run at the sim's visual frame rate (display Hz, not fixed) | served: `flight_loop` (`lib.rs:1229`), registered via `XPLMRegisterFlightLoopCallback` (`xp.rs:263`) — X-Plane's flight loop is the direct equivalent |
| Delta time source | `SimulationTime` data definition, requested at `Period::VisualFrame` (`systems_wasm/src/lib.rs:596-651`); `Time::take()` caps abnormal deltas at 500ms and treats `next_delta == 0` as "sim is pausing" (no PreDraw processing that frame) | needs confirming `lib.rs`'s `flight_loop` reproduces both the 500ms cap and the "delta==0 means paused" skip — not verified line-by-line in this pass |
| Pause / any-pause / active-pause | `SimConnectInterface::isSimInPause()` / `isSimInAnyPause()` / `isSimInActivePause()` (`SimConnectInterface.cpp:128-139`) gate whether systems update at all | served: `lib.rs:684,814,879-881` reads X-Plane's `sim/time/paused` dataref; `lib.rs:1237-1241` skips the systems tick while paused, matching "a paused sim does not move FlyByWire's systems either" (own comment) — does not distinguish MSFS's any-pause vs. active-pause, likely not needed at this granularity |
| Sim rate | `SIM_RATE_INCR`/`_DECR`/`_SET` events, `updateSimulationRateLimits` (`SimConnectInterface.h`) | served: `Env::SimulationRate` (`js_bridge.rs:280`), reads an X-Plane dataref, clamped `.max(0.)` |
| Slew | `IS SLEW ACTIVE` simvar (found among the 481-field struct, marked MISSING by the blunt grep in Layer 1.1) | not confirmed served — worth a direct check, since slew is a distinct flight-model mode X-Plane also has (`sim/operation/override/override_planepath` family) |
| Start-state flow | Flight-file-selected `A32NX_START_STATE` read once at aircraft construction (`systems_wasm/src/lib.rs:58-66`, before `Simulation::new`), never re-read mid-flight | served, by re-derivation rather than a flight file: `start_state.rs`'s documented rule set (on-ground/in-air, engines running, runway lineup or >40kt) infers the same 8 states from live X-Plane state at plugin start |
| Flight load/unload | Implied by `GameStateProvider`/`GameState.*` (12-13 files) gating instrument/system startup until in-game | served: `GAME:` handling (Layer 2/5) |

Overall this layer is one of the better-covered ones — pause is served; the delta-time-cap
reproduction and slew are small, checkable next steps rather than open-ended unknowns.

---

## Summary

**Totals per layer** (counts are of distinct names/entries found by grep, not call sites, unless
noted):

1. Simulation variables (`A:`): 481-field C++ SimConnect data definition (1 struct, exhaustive) +
   124 Rust `provides_aircraft_variable` declarations (+16 more via aspect modules) + 144 distinct
   `A:`-class names read from JS + 50 SimConnect client-data areas (a different kind of object, see
   1.4).
2. `E:`/`GAME:`: 9 distinct `E:` strings + generic `GAME:` handling.
3. `K:`/`H:`/`B:`: 243-entry C++ `Events` enum (exhaustive) + 27 JS `K:` writes + 573 `H:` names
   (our own build tool's count) + 400 `A:`/`K:` RPN references across 36 behaviour-XML files + 0
   literal `B:` vars found.
4. Gauge API: `PANEL_SERVICE_{PRE_INSTALL,PRE_DRAW,PRE_KILL}`, key-event
   register/trigger (12+1 call sites), 17 `execute_calculator_code` call sites, no sound/HTTP/NanoVG
   usage found.
5. Coherent/JS runtime: 16 `Coherent.call` names, 11 `Coherent.on` names, plus the
   `RegisterViewListener`/`BaseInstrument`/`GameStateProvider`/`GetStoredData`/`NXDataStore`/`/VFS/`
   surface (18/26/12/3/58/6 files respectively).
6. Facilities: one concentrated client, `navdata/client/backends/Msfs/*` (6 files); out of scope for
   this runtime brief beyond flagging it.
7. Config/data files: `systems.cfg` `[ELECTRICAL]`, `flight_model.cfg` gear/door geometry, 8 `.FLT`
   files' start-state values, `panel.cfg`'s 24 `[VCockpitNN]` sections, 36 behaviour-XML files
   (owned by `msfs2xp-aircraft`, not this repo).
8. Timing/lifecycle: per-frame `PANEL_SERVICE_PRE_DRAW`/flight-loop cadence, `SimulationTime`
   delta-time protocol with a 500ms cap, three-level pause state, sim rate, slew, 8-state start-state
   flow, game-state-gated load/unload.

**Biggest gaps in our emulation, ranked by impact:**

1. **The C++ SimData struct's non-`lib.rs`-mapped fields (Layer 1.1, ~53 of 481 rows flagged
   MISSING by name, before accounting for `sensors.rs`/`prim.rs`'s own prose coverage).** This is
   the highest-impact layer because it feeds the flight-model math every frame; anything genuinely
   missing here (vs. covered in prose elsewhere) directly affects flight dynamics fidelity. Needs a
   careful manual pass cross-referencing each MISSING row against `sensors.rs`'s and `prim.rs`'s doc
   tables (which this automated pass could only search for as literal strings, undercounting real
   coverage) before trusting the raw MISSING count.
2. **`execute_calculator_code` call sites (17) have no generic interpreter** — each is currently
   either hand-ported or not; since these set `H:`/`L:` vars other MSFS gauges or FBW's own bools
   watch (e.g. the `A32NX_FCU_ALT_INCREMENT_1000` toggle), missing ones are silent behavioural gaps,
   not crashes, making them easy to overlook. Worth enumerating all 17 by hand and confirming each.
3. **Coherent.call/`RegisterViewListener` surface (Layer 5) was not checked line-by-line against
   `js/msfs/coherent.js` (225 lines) and `js/msfs/instrument.js` (878 lines)** in this pass — given
   those files' sizes, coverage is plausible but unverified; a focused pass matching the 16
   `Coherent.call` names and 5 `JS_LISTENER_*` names against those shims would close this out
   quickly.
4. **Slew state and the exact any-pause/active-pause distinction (Layer 8)** — small in scope, but
   slew is exactly the kind of MSFS-only mode (freeze position, ignore physics) that, if unhandled,
   could make FBW's systems do something wrong the moment a host tool repositions the aircraft.
5. **SimConnect client data areas (Layer 1.4)** turned out *not* to be a gap on inspection —
   `prim.rs` ports the computer logic directly rather than replaying the bus protocol — but this was
   only caught by reading `prim.rs`'s own header; the initial grep-only pass would have wrongly
   flagged it as the single largest missing piece. This is a caution about trusting any automated
   MISSING count in this document without checking the target file's own doc comments first, several
   of which (`sensors.rs`, `key_events.rs`, `js_bridge.rs`, `start_state.rs`, `prim.rs`) already
   contain hand-verified, more-precise versions of exactly the tables this brief tried to
   reconstruct by grep.

**Surprising findings:**

- `fbw-xp-systems` already carries extraordinarily thorough doc-comment tables for several of these
  layers (`key_events.rs`'s `K:` table, `sensors.rs`'s simvar table, `start_state.rs`'s `.FLT` rules,
  `js_bridge.rs`'s variable-class contract) — this brief's grep-based tables are less precise than
  what already exists in the repo for those specific layers; future work should treat those files as
  primary sources and use grep mainly to find what is *not yet* in one of those tables.
- FBW's C++ WASM has zero sound, HTTP, or NanoVG usage — its MSFS footprint is unusually narrow and
  concentrated in SimConnect data/events, which simplifies the emulation surface for Layer 4.
- The 481-field SimData struct is bidirectional (read for physics input, written back for
  control-surface output) but declared through one shared `addDataDefinition` helper with no
  direction marker in the source — determining actual direction per field requires searching
  `FlyByWireInterface.cpp` for each name individually, which this pass did not do exhaustively.
- The largest single K:-event family by far is FBW's own `A32NX_FCU_*` (~90 of the 243 `Events`
  enum entries) — MSFS's *own* default events are a minority of what this aircraft actually sends.

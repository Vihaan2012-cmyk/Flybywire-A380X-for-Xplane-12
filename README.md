# FlyByWire A380X for X-Plane 12

A study-level port of the [FlyByWire A380X](https://github.com/flybywiresim/aircraft)
to X-Plane 12. FlyByWire's own A380 systems, fly-by-wire and instruments run inside
X-Plane through this plugin. Around them sits a physical simulation layer where **every
failure is causal**: a fault changes a physical quantity (a valve's authority, a pump's
displacement, a compressor's efficiency), and what the crew sees comes out of the
physics, not out of a scripted symptom.

> Status: in active development, pre-release. The repository is private until the
> first public release.

---

## At a glance

| | |
|---|---|
| Aircraft | Airbus A380-800, 4 × Rolls-Royce Trent 972B-84, PW980 APU |
| Simulator | X-Plane 12 (Windows) |
| Systems core | FlyByWire's A380 Rust systems (electrical, hydraulic, pneumatic, fuel, air conditioning, gear, APU, avionics network), unmodified except for documented patches |
| Fly-by-wire | FlyByWire's C++ PRIM/SEC flight control laws and autothrust, ported (`src/fbw_cpp`, `src/fbw_controllers.rs`, `src/prim.rs`) |
| Displays | FlyByWire's instruments (PFD, ND, EWD/ECAM, MFD, EFB…) run in a bundled browser runtime and are drawn into the cockpit (`src/display`, `app/`) |
| Our physics | Gas-turbine engine model, fuel network, bleed ports, oil system, hot-section thermal model, ADIRS, tyres and brakes, bays, damage and wear |
| Deep systems | **17 live areas** (`src/deep/`): APU, avionics network, breakers, cabin, electrical, engine accessories, environment, fire/ice, flight controls, fuel, gear structure, hydraulics, oxygen, pneumatic ducts, sensors, thermal zones, wiring |
| Failures | **5,555** causal single failures, 5,195 of them from the deep-systems areas |
| Components | **2,013** components with continuous health parameters, persisted between flights |
| Circuit breakers | **399**, each de-powering its real consumer in FlyByWire's electrical simulation |
| MEL | 64 items / 89 sub-items mapped to 120 failures, with per-unit placarding |
| ECAM | **304** alerts raised from the deep areas' own state, alongside FlyByWire's own catalogue |
| Tests | **2,455** unit and integration tests, plus a multi-tier failure battery |

Full reference data, sources and architecture: [docs/SPECIFICATIONS.md](docs/SPECIFICATIONS.md).

## Lines of code

Counted from the files in this repository (`git ls-files`), excluding FlyByWire's own
sources.

| Part | Lines |
|---|---|
| Plugin (Rust, `src/`) | 144,252 |
| … of which the deep-systems areas (`src/deep/`) | 91,612 |
| … of which physics models (`src/physics/`) | 5,820 |
| Test emulator and battery (Rust, `emulator/`) | 8,541 |
| Desktop app (Rust, `app/`) | 3,373 |
| JavaScript / TypeScript (instrument runtime, bridges, worker) | 37,062 |
| C++ (fly-by-wire shims) | 1,164 |
| Patches to FlyByWire's Rust systems (`patches/fbw-rust`, 37 patches) | 11,559 |
| Documentation (`docs/`, Markdown) | 24,219 |

FlyByWire's systems code this builds on adds about 128,700 lines of Rust
(47,032 in `a380_systems`, 81,666 in the shared `systems` crate).

## Failures by ATA chapter

Every failure, from the original catalogue and from the deep-systems areas alike, is
classified by ATA chapter.

| ATA | Chapter | Failures |
|---|---|---|
| 21 | Air conditioning / pressurisation | 465 |
| 22 | Autoflight | 27 |
| 23 | Communications | 48 |
| 24 | Electrical power | 967 |
| 25 | Equipment / furnishings | 48 |
| 26 | Fire protection | 274 |
| 27 | Flight controls | 433 |
| 28 | Fuel | 1,008 |
| 29 | Hydraulic power | 176 |
| 30 | Ice and rain protection | 124 |
| 31 | Indicating / recording | 18 |
| 32 | Landing gear | 405 |
| 33 | Lights | 90 |
| 34 | Navigation | 247 |
| 35 | Oxygen | 49 |
| 36 | Pneumatic | 115 |
| 38 | Water / waste | 19 |
| 42 | Integrated modular avionics | 153 |
| 44 | Cabin systems | 35 |
| 49 | APU | 54 |
| 52 | Doors | 77 |
| 53 | Fuselage | 5 |
| 56 | Windows | 3 |
| 57 | Wings | 2 |
| 71–80 | Power plant / engine (per engine, × 4) | 622 |
| 91 | Wiring | 91 |
| | **Total** | **5,555** |

Every failure is also a component with a continuous `loss` level (0–100 %), so partial
and combined failures interact physically (a restriction that only becomes critical
because an upstream pump has already lost pressure margin).

## Engine model (Trent 972)

A three-spool thermodynamic model calibrated to the public EASA type-certificate data
sheet **TCDS E.012** (Issue 12):

- **Take-off thrust** matches the certificated rating at sea level ISA.
- **Acceleration** from 15 % to 95 % take-off power is 5.56 s (data sheet: 5.6 s), from a
  fuel-air-ratio (Wf/P3) acceleration schedule.
- **TGT trim:** the cockpit shows trimmed TGT and the engine measures untrimmed, per the
  data sheet's Note 16.
- **Limits and exceedances:** TGT (take-off, maximum continuous, overtemperature, start,
  relight), rotor speeds and IP overspeed, oil pressure and temperature, all logged as
  exceedances.
- **Ground limits:** fan keep-out zone and the ground N1 limit.
- **Customer bleed:** IP8 and HP6 ports with the data sheet's 206.8 / 231 kPa switch-over,
  driving FlyByWire's bleed valves. The maximum permissible bleed vs turbine entry
  temperature tables are monitored.
- **Hot-section thermal mass:** heat soak, residual TGT after shutdown, and hotter
  restarts into hot metal.
- **Oil system:**
  - pump, relief valve, filter with bypass, and viscosity from MIL-PRF-23699 data (cold
    oil runs high pressure);
  - three bearing chambers with soak-back after shutdown;
  - fuel-cooled and air-cooled oil coolers (the fuel-cooled one heats the fuel).
- **Loads:** bleed extraction and gearbox loads (generators, hydraulic pumps).
- **Degradation:** compressor and turbine efficiency, flow capacity and bearing wear feed
  the gas path.

Known gap: the off-design cycle runs cool (low turbine entry temperature and pressure
ratio at take-off, cold idle TGT). The gas-path rebuild with spool matching and
component maps is in progress.

## Compared with the Hot Start CL650

The CL650 is the reference for study-level depth in X-Plane. The comparison below is at
the feature level.

A caveat on the left-hand column: it is written from public material and from using the
aircraft, not from its internals, which are encrypted. Where it says something is
"simpler" or gives no mechanism, that is the limit of what can be checked from outside
— not a measured finding. The right-hand column is counted from this repository's own
code and catalogue.

| Area | CL650 | This project today |
|---|---|---|
| Single failures | ~1,375 (≈ 700 electrical) | **5,555** (967 electrical, 1,008 fuel) |
| Electrical | Per-load network, every load can fail or short | Per-load network: **376 loads**, each able to short to ground or open, fault current set by its own feeder resistance; buses solved by Millman/Thevenin; 399 breakers tripping on the real current; 91 wiring failures |
| Engine gas path | Stage-level compressors, maps, surge | Thermodynamic spools + oil + hot section, TCDS-calibrated; fuel system, ignition, starting, VSVs, bleed valves, vibration, reversers, dual-channel EEC |
| Hydraulics | Line/volume network | Line network with pumps, reservoirs and leaks (176 failures) |
| Pneumatic / air conditioning | Simpler | FlyByWire packs + air cycle machine, real engine bleed ports, duct network (115 + 465 failures) |
| Flight control logic | Business jet | Full Airbus fly-by-wire laws (FlyByWire) + actuator and high-lift models (433 failures) |
| Landing gear | Tyres, brakes, retraction | Gear structure with strut loads, fatigue and collapse (405 failures) |
| Avionics network | Not modelled at this depth | ARINC 653 partitions, module and bay faults (153 failures) |
| Wear, MEL, persistence | Full aircraft state persistence — every part of the aircraft's state is restored on reload | Wear, damage creep, MEL with per-unit placarding, persistent airframe |

Where it is still behind: the CL650 has had years of tuning against the real
aircraft, and its documentation, checklists and failure behaviour have been exercised
by a large user base. Almost nothing here has been flown in anger by anyone. A failure
count is not depth, and this table is a feature comparison, not a claim to have
overtaken it.

## Roadmap

The deep-systems push (`src/deep/`, `docs/deep/`) is largely delivered: 17 areas are
wired in and ticking, and the catalogue stands at 5,555 failures against an original
target of about 2,500. What it added:

- **New systems:**
  - per-load electrical network;
  - engine fuel system, ignition, starting, variable stator vanes and bleed valves, vibration, reversers, dual-channel EEC;
  - hydraulic line network;
  - flight-control actuators and high lift;
  - gear structure and collapse;
  - physical sensors and air data probes;
  - APU with a compressor map;
  - fire detection/extinguishing and ice accretion;
  - avionics network faults;
  - cabin water/waste/IFE.
- **Coupling models:** airframe thermal zones, wiring bundles and zones, a pneumatic duct network, bird strikes and environment events (lightning, hail, volcanic ash, ice crystals, runway contamination), and a 6-DOF flight model for the test emulator.
- **Target:** about 2,500 causal single failures, each backed by an emulator test —
  passed, at 5,555.

Still ahead: the aircraft's own converted assets (cockpit interaction for the MCDU and
EFB, ND range and mode), the loadsheet reaching the EFB rather than only the cfg's
defaults, and the engine model's behaviour at high thrust.

## Diagnostics

The plugin reads a few environment variables that switch parts of it off. They exist
for bisecting a fault to the code that causes it — the aircraft keeps flying with any
of them set, so a run with and a run without is a measurement rather than an argument.

| Variable | Effect |
|---|---|
| `FBW_SCREENS=off` | Draws no cockpit display: no upload, no quad, no underlay. The browser views keep running, so the difference in frame time is what the twenty screens cost — which no timer inside the plugin can see. |
| `FBW_XP_EFFECTS=off` | Stops mirroring the plugin's own failure state onto X-Plane's (`sim/operation/failures/rel_*`: fires, seizures, flameouts, hydraulic leaks, tyres, brakes). |
| `FBW_DEEP=off` | Skips the deep-systems areas for the frame. |
| `FBW_XP_WRITES=…` | Stops the plugin driving X-Plane's own physics. `off` for all of it, or a comma list of `surfaces`, `handling`, `weight`, `weight-stations`, `weight-cg`, `weight-fuel`. |

Each one logs that it is active on its first tick, so a run can be checked rather than
assumed. Two cautions learned the hard way: confirm from `Log.txt` that the switch
actually took effect before trusting a result, and do not judge "it did not crash" by
sim uptime — that clock runs while the sim sits in a menu.

## Testing

- **Unit and integration tests:** `cargo +stable-x86_64-pc-windows-gnu test --release --features test-support --lib`
- **Emulator** (`emulator/`): runs the whole aircraft without X-Plane from any start state (cold and dark, ground power, powered, engines running).
- **Failure battery** (`emulator/src/bin/battery.rs`):
  - every single failure;
  - exact group testing of pairs (pairs sharing an effect run individually, the rest in pooled groups);
  - physical invariants checked every tick;
  - a work queue across all CPU cores;
  - a live progress page on `http://127.0.0.1:8790`.
- **Operations battery:** start-up, shutdown and procedures, plus wear accumulation.

## Building

The plugin builds with the GNU toolchain (no Visual Studio needed):

    cargo +stable-x86_64-pc-windows-gnu build --release --features js

`--features js` is not optional for a flyable build: it brings in QuickJS and the
Oxc TypeScript loader that run FlyByWire's instruments. Without it the plugin still
compiles and loads, but every cockpit display is dead — and the only outward sign is
that `win.xpl` comes out around 15 MB instead of about 31 MB.

The built `fbw_a380_systems.dll` is installed as
`Aircraft/<aircraft>/plugins/fbw_a380_systems/64/win.xpl`. X-Plane scans every
sub-folder of `plugins/` for `64/win.xpl`, so renaming the *folder* does not disable
the plugin; rename the `.xpl` itself. Windows will not let you replace it while
X-Plane is running.

FlyByWire's sources are expected at `D:\fbw-aircraft` with the patches in
`patches/fbw-rust` applied. The desktop app (`app/`) builds with the MSVC toolchain
and CEF.

## The aircraft

The X-Plane aircraft (3D models, textures, cockpit, liveries) is converted from
FlyByWire's MSFS package by a separate converter tool. The converted aircraft is about
3.6 GB, so it is distributed through GitHub Releases (split archives with a manifest and
installer, see `distribution/`) rather than stored in git.

## Licence

GPL-3.0, like FlyByWire's A380X which this project builds on. FlyByWire assets and code
remain under their original licence.

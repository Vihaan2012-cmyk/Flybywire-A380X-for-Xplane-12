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
| Failures | **360** causal single failures across 20 ATA chapters (growing — see the roadmap) |
| Components | **352** components with continuous health parameters, persisted between flights |
| Circuit breakers | **265**, each de-powering its real consumer in FlyByWire's electrical simulation |
| MEL | 64 items / 89 sub-items mapped to 120 failures, with per-unit placarding |
| Tests | **831** unit and integration tests, plus a multi-tier failure battery |

## Lines of code

Counted from the files in this repository (`git ls-files`), excluding FlyByWire's own
sources.

| Part | Lines |
|---|---|
| Plugin (Rust, `src/`) | 85,059 |
| … of which physics models (`src/physics/`) | 10,544 |
| Test emulator and battery (Rust, `emulator/`) | 4,978 |
| Desktop app (Rust, `app/`) | 3,425 |
| JavaScript / TypeScript (instrument runtime, bridges, worker) | 33,897 |
| HTML (app and panel UIs) | 2,769 |
| C++ (fly-by-wire shims) | 1,164 |
| Patches to FlyByWire's Rust systems (`patches/fbw-rust`, 37 patches) | 11,559 |
| Documentation (`docs/`, Markdown) | 11,363 |

FlyByWire's systems code this builds on adds about 128,700 lines of Rust
(47,032 in `a380_systems`, 81,666 in the shared `systems` crate).

## Failures by ATA chapter

| ATA | Chapter | Failures |
|---|---|---|
| 21 | Air conditioning / pressurisation | 54 |
| 22 | Autoflight | 3 |
| 24 | Electrical power | 47 |
| 26 | Fire protection | 18 |
| 27 | Flight controls | 9 |
| 28 | Fuel | 49 |
| 29 | Hydraulic power | 24 |
| 30 | Ice and rain protection | 3 |
| 32 | Landing gear | 35 |
| 34 | Navigation | 22 |
| 36 | Pneumatic | 19 |
| 49 | APU | 5 |
| 72–80 | Engine (per engine, × 4) | 72 |
| | **Total** | **360** |

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

| Area | CL650 | This project today | Planned |
|---|---|---|---|
| Single failures | ~1,375 (≈ 700 electrical) | 360 | ~2,500 |
| Electrical | Per-load network, every load can fail or short | FlyByWire buses/contactors + 265 breakers; loads lumped per bus | Per-load network with fault currents and wiring zones |
| Engine gas path | Stage-level compressors, maps, surge | Thermodynamic spools + oil + hot section, TCDS-calibrated | Spool matching, maps, stage damage, surge |
| Hydraulics | Line/volume network | FlyByWire circuits and pumps | Line/volume network with fluid thermal model |
| Pneumatic / air conditioning | Simpler | FlyByWire packs + air cycle machine, real engine bleed ports | Duct network with leaks and overheat loops |
| Flight control logic | Business jet | Full Airbus fly-by-wire laws (FlyByWire) | Actuator hinge moments, jams, runaways |
| Wear, MEL, persistence | Not modelled | Wear, damage creep, MEL, persistent airframe | — |

Ahead today: air conditioning and pressurisation, pneumatics, flight-control logic,
landing gear/tyres/brakes, wear/MEL. Behind today: electrical per-load depth, engine gas
path, hydraulic plumbing.

## Roadmap

A deep-systems push is under way (`src/deep/`, `docs/deep/`):

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
- **Target:** about 2,500 causal single failures, each backed by an emulator test.

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

    cargo +stable-x86_64-pc-windows-gnu build --release

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

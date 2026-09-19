# Technical specifications

Reference data the simulation is built on, where each figure comes from, and how the
plugin is put together. Figures marked **(model)** are what the code uses; where a public
source and the model disagree, both are given.

## 1. Aircraft (Airbus A380-800)

Public, typical values (Airbus *A380 Aircraft Characteristics — Airport and Maintenance
Planning*). Weight variants differ between operators.

| Item | Value |
|---|---|
| Length | 72.72 m |
| Wingspan | 79.75 m |
| Height | 24.09 m |
| Maximum take-off weight | 560–575 t (weight variant) |
| Maximum landing weight | 386–394 t |
| Maximum zero-fuel weight | 361–369 t |
| Fuel capacity | ~320,000 L in 11 tanks (the model uses FlyByWire's tank layout, `src/fuel.rs`) |
| Engines | 4 × Rolls-Royce Trent 900 series |
| APU | Pratt & Whitney PW980A |
| Hydraulics | 2 systems (green, yellow) at 5,000 psi, 8 engine-driven pumps, electric pumps, plus electrical backup actuators (EHA/EBHA) |
| Electrical | 4 variable-frequency generators (VFGs), 2 APU generators, RAT, batteries, TRUs, static inverter |
| Landing gear | Nose, 2 wing (4 wheels each), 2 body (6 wheels each); 22 wheels |

## 2. Engine: Rolls-Royce Trent 900 (EASA TCDS E.012, Issue 12, 16 March 2026)

| Item | TCDS | Model |
|---|---|---|
| Architecture | 3-shaft, high bypass; LP fan single stage, IP compressor 8 stages, HP compressor 6 stages; HP turbine 1 stage, IP turbine 1 stage, LP turbine 5 stages; single annular combustor | same stage counts (gas-path rebuild in progress) |
| Fan diameter | 2.95 m | 2.95 m |
| Overall length / max diameter | 5,477.5 mm / 3,944 mm | — |
| Dry weight | 6,246 kg | 6,246 kg (drives spool inertias and hot-section mass) |
| Take-off thrust, ISA SLS (5 min), 972-84 / 972E-84 | 341.41 kN (76,752 lbf) | **356.8 kN (80,213 lbf)**, FlyByWire's figure for the 972B-84 (not in this TCDS) — to be reconciled |
| Maximum continuous thrust | 319.60 kN (71,850 lbf) | — |
| Flat rating | ISA +15 °C at take-off | — |
| 100 % rotor speeds | LP 2,900 / IP 8,300 / HP 12,200 rpm | same |
| Take-off rotor maxima | LP 97.2 %, IP 98.7 %, HP 97.8 %; IP overspeed 99.5 % for 20 s | monitored as exceedances |
| TGT, trimmed (displayed) | take-off 900 °C, max continuous 850 °C, overtemperature 920 °C (20 s), ground start 700 °C (<50 % HP), relight 850 °C | same; the cockpit shows the trimmed value |
| TGT, untrimmed (measured, Profile 5) | take-off 956 °C, max continuous 939 °C, overtemperature 957 °C | the physical engine's TGT; damage and creep use these |
| Acceleration | 15 % → 95 % rated take-off power in 5.6 s | 5.56 s |
| Oil pressure minimum | 25 psi idle to 70 % HP; 50 psi above 95 % HP | met (56 psi idle, 120 psi take-off) |
| Oil temperature | start ≥ −30 °C (−40 °C special procedure); ≥ 40 °C before take-off power; max 196 °C | monitored |
| Ground operation | fan keep-out zone 64–72 % N1 below 60 kt; N1 capped at 78 % below 32.5 kt | applied to the N1 target |
| Customer bleed ports | IP8 at take-off/climb/cruise, HP6 at idle/descent; switch at 206.8 kPa IP port pressure (231 kPa with 2 bleeds and 1 pack, 237.9 kPa icing) | IP8/HP6 ports from the compressor model; FlyByWire's HP valve switches at the TCDS pressures (absolute) |
| Maximum customer bleed | %W26 (HP6) / %W24 (IP8) vs T41 tables, normal and abnormal | tables implemented; exceedances logged |

Engine model details: `src/physics/engine/` (thermodynamic spools, governor, bleed ports,
TGT trim), `oil.rs` (oil system), `hot_section.rs` (hot-section thermal mass),
`bleed_limits.rs` (TCDS bleed tables). Known gap: the cycle runs cool off-design;
the gas-path rebuild (stage-stacked compressors, volume dynamics, turbine flow capacity)
addresses it.

### Oil system constants (model, GENERIC unless noted)

| Item | Value | Source |
|---|---|---|
| Oil | MIL-PRF-23699; 27.6 cSt @ 40 °C, 5.1 cSt @ 100 °C (Walther fit) | Mobil Jet Oil II data sheet |
| Pump delivery | 150 L/min at 100 % N3 | GENERIC |
| Relief valve | cracks at 145 psi | GENERIC (gauge max 149 psi, package engines.cfg) |
| Filter bypass | 30 psid | GENERIC |
| Bearing chambers | front, HP/IP, tail; soak-back after shutdown | GENERIC sizes |
| Coolers | fuel-cooled (ε 0.8) and air-cooled (ε 0.7, EEC-scheduled valve) | GENERIC |
| Oil heat load | 15 % of spool mechanical loss (~190 kW at take-off) | GENERIC |

### Hot section (model, GENERIC)

12 % of dry weight at 450 J/(kg·K); 60 s warm-up time constant at design flow;
convection ∝ (mass flow)^0.8; radiation (ε 0.7, 6 m²) and natural convection when stopped.

## 3. Failures, components and ECAM

- **Failure ids.** The existing catalogue uses ATA × 1,000 + n (e.g. `36_012`). New deep-system
  failures use area × 1,000,000 + ATA × 1,000 + n (`src/deep/api.rs`), so they never collide.
- **Magnitudes.** Each failure has a continuous magnitude from 0 to 1 acting on one physical model
  field, such as leak area, valve authority, pump displacement or compressor efficiency.
- **Components.** Each component has continuous health parameters, persisted between flights; a
  component's level drives its failures' magnitudes.
- **ECAM alerts** are data (`src/deep/api.rs`):
  - level, which sets colour, aural and master light;
  - trigger condition over published variables, with a confirmation delay;
  - flight-phase inhibits;
  - procedure lines whose completion is a condition on the variable the real cockpit control
    writes, so operating the control completes the line;
  - timed lines, STATUS lines and INOP systems.

## 4. Software architecture

    X-Plane 12 ──datarefs──► plugin (Rust, one flight loop)
                                ├─ FlyByWire A380 systems (Rust, patched)  ◄─ SimulatorReaderWriter
                                ├─ FlyByWire fly-by-wire / PRIM / autothrust (C++ via shims)
                                ├─ FADEC + physical engines (src/engine_commands.rs, src/physics/engine)
                                ├─ physics: fuel network, bleed, ADIRS, tyres, bays, damage/wear
                                ├─ failures, components, breakers, MEL, persistence
                                └─ Vars ──► instruments runtime (FlyByWire TS/JS in a browser engine)
                                            ──► cockpit displays, desktop app, EFB

- **Variables:** everything the systems read or write is a named variable, published to X-Plane
  under `fbw/…`.
- **FlyByWire's sources** are used as-is, plus the patches in `patches/fbw-rust/`. Examples:
  pneumatic valve seizure, bleed ports from the engine model, breakers de-powering real consumers,
  deterministic seeded randomness.
- **Test emulator** (`emulator/`): runs the full plugin without X-Plane from any start state. It
  powers the unit and integration tests, the failure battery (single failures, exact pair group
  testing, per-tick physical invariants) and the operations battery.

## 5. Repository layout

| Path | Contents |
|---|---|
| `src/` | the plugin |
| `src/physics/` | physical models (engine, fuel, air, ADIRS, tyres, bays, damage) |
| `src/deep/` | the deep-systems push: new models per area, registered through `api.rs` |
| `src/display/`, `src/js/` | instrument rendering and the FlyByWire JS/TS runtime bridge |
| `src/fbw_cpp/` | C++ shims for FlyByWire's fly-by-wire and PRIM code |
| `emulator/` | test emulator, failure battery, operations battery |
| `app/` | desktop app (Study pages, components, MEL, failures) |
| `patches/fbw-rust/` | patches to FlyByWire's Rust systems |
| `docs/` | physics notes per system, briefs, this document |
| `distribution/` | release publishing (GitHub Releases, Cloudflare Worker manifest, installer) |

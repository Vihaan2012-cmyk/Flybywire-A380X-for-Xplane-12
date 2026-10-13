# Brief: physical simulation ("hyper-realism")

The user wants the systems to simulate the actual physics: air, fluid, heat and electric current flowing through real components. Values should come out of that flow, not be set as variables or state-machine outputs. The yardstick is docs/cl650-reference.md, for example "generator electrical load imposes mechanical drag on the engine core, which affects fuel consumption."

## Rules (all workstreams)
- **Read first:** D:\A380\fbw-xp-systems\docs\team.md (layout, build) and docs/analysis/systems.md.
- **No sub-agents.** Read lean: grep and targeted reads.
- **Audit before building.** FlyByWire already models some physics (pneumatic containers with mass flow, cabin air mass and pressure, hydraulics). Keep what is already physical. Replace or extend only what is a shortcut: constants, if-chains, lookup states, instant values.
- **Where the code goes:** FlyByWire's Rust systems live in D:\fbw-aircraft (a git repo; the plugin builds them as a path dependency). You may edit them. After each working change, save your diff with `git -C D:\fbw-aircraft diff -- <your paths> > D:\A380\fbw-xp-systems\patches\fbw-rust\<workstream>.patch`, so it survives FBW updates. Keep FBW's code style and tests. New plugin-side physics goes in D:\A380\fbw-xp-systems\src\physics\<area>.rs.
- **Physics must be real:** conservation of mass and energy, ideal gas, orifice and valve flow equations, heat exchangers with effectiveness, Ohm's and Kirchhoff's laws, battery equivalent circuits.
- **No invented numbers:** parameters come from FBW's own code and data, public A380/engine references (cite them), or first-principles derivation from cited geometry. Document every parameter's source in a table in docs/physics/<area>.md. If no source exists, say so and choose the most defensible derived value, clearly marked.
- **Stability and cost:** fixed sub-steps if needed, with no NaNs or explosions at any sim rate or pause. Keep the per-frame cost small and report it.
- **Tests:** conservation checks (mass and energy balance), steady-state values against references, and failure behaviour (a leak, a closed valve, an open breaker).
- **Build:** use `cargo +stable-x86_64-pc-windows-gnu test --release --features js` with CARGO_TARGET_DIR=D:\A380\fbw-build\target-phys-<area>. Keep the whole tree building, and never delete another workstream's files. Back up with /d/A380/fbw-build/backup-plugin.sh before large edits.
- **Study panel:** the lead builds the diagrams. In your report, list the flow and state quantities to show (per duct, valve, bus and so on).

## Removing approximations
The user wants every approximation gone. Examples:
- thrust scaled by pressure ratio;
- X-Plane spool dynamics;
- no generator or bleed drag;
- the 2 h fuel-temperature time constant;
- generic oxygen bottle figures;
- the 500 gal/h jettison rate.

Replace each one with a physical model, and give sourced parameters wherever they exist.

## Shared contract between workstreams (engine loads)
Loads the engines must turn into drag and fuel are exchanged through plugin Vars (`vars.get`/`read`/`write`), per engine n = 1..4:
- `ENGINE_BLEED_EXTRACTION_KG_S:n`: the bleed mass flow the pneumatic model draws (written by pneumatics).
- `ENGINE_GEARBOX_ELEC_LOAD_W:n`: generator shaft power (written by electrical: electrical load divided by generator efficiency).
- `ENGINE_GEARBOX_HYD_LOAD_W:n`: engine-driven hydraulic pump shaft power (written by hydraulics: pressure × flow divided by efficiency).
- `ENGINE_FUEL_DEMAND_KG_S:n`: fuel flow the engine burns (written by the engine model; fuel reads it).
- APU equivalents use n = 0 where applicable.

Until another workstream writes a variable, read it as 0. Never fake it.

## Report
- what was variable-based before and what is physical now;
- equations and parameter sources;
- tests and cost;
- the Study quantities;
- what only X-Plane can verify.

## Second pass: no simplifications (current task)
The user wants **no simplifications**, as realistic as possible, and **done fast**. Earlier engineers were stopped mid-work. Read the existing files and docs/physics/<area>.md, keep what is sound, and finish. First get the tree building: `cargo +stable-x86_64-pc-windows-gnu test --release --features js`. Fix trivial breaks anywhere, and mention them.

- **Engines** (src/physics/engine/):
  - Replace the reduced-order torque scaling with a full component-matching solve (Newton-Raphson with a numerical Jacobian). Unknowns are the map operating points and the burner exit temperature; the residuals are station mass continuity, turbine and nozzle flow capacity, and spool power balance (transient). Warm-start it, damp it and bound it.
  - Scale published NASA/GasTurb standard maps to a Trent 900 design point from EASA TCDS E.012, Rolls-Royce public figures (BPR about 8.5-8.7, OPR about 39, 3.0 m fan, stage counts) and the ICAO emissions databank (fuel flow at 7/30/85/100%). Calibrate to those, and tabulate the error.
- **Electrical** (src/physics/electrical.rs, FBW electrical patch):
  - breaker ratings from FBW's actual per-consumer demands, plus standard CB sizing;
  - feeder wire resistance from AS50881 gauge ampacity and run lengths;
  - the battery as a Thevenin 2-RC/Shepherd model with Peukert effect, the SOC-OCV curve and temperature, for the A380's actual battery chemistry and rating;
  - VFG efficiency and regulation from published 150 kVA VFG data, plus static inverter and emergency generator sag;
  - read `PROBE_HEAT_LOAD_W:n` as a load.
- **Air** (FBW air_cycle_machine.rs and pneumatic, src/physics/air.rs):
  - compressible orifice flow (subsonic and choked) for the HP, PRV, crossbleed, pack and APU valves, with the pneumatic tests fixed properly;
  - heat exchanger effectiveness-NTU from UA derived from published pack data;
  - per-zone solar, avionics, IFE, galley and lighting heat;
  - engine and wing anti-ice bleed consumption and leading-edge heat;
  - fix the 11 failing a380_systems air-conditioning tests (mock plenum pressure).
- **Fluids** (src/physics/fluids.rs, fuel.rs, oxygen.rs, FBW hydraulic patch). First fix oxygen.rs's tests: `step` is called with `&mut TestVars` but takes `&mut Vars`, so make it generic. Then:
  - multi-node stratified tank thermal models with wetted areas from the real A380 wing and tank geometry, and correlation-based heat transfer;
  - jettison from real nozzle and pipe geometry, validated against published dump times;
  - hydraulic fluid temperature per circuit and Skydrol/HyJet viscosity;
  - wire the fuel viscosity derate into fuel_network.rs;
  - per-actuator EHA/EBHA power;
  - the real A380 crew oxygen installation, instead of scaled A320 data.
- **ADIRS** (src/physics/adirs.rs, FBW navigation patch):
  - an RK4 strapdown integrator, plus GPS-lost free-inertial and ATT modes;
  - per-ADIRU lever-arm effects, misalignment and an independent MMR GPS;
  - probe heat switch logic per FBW, writing `PROBE_HEAT_LOAD_W:n`;
  - real A380 bus assignments, heater power and static-port error where published;
  - wire FBW's ADIRU power logic to the buses.

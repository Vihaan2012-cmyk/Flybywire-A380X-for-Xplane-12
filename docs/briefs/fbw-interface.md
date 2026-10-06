# Brief: FlyByWireInterface leftovers (owner of src/extra_backend_fbw.rs)

Read docs/team.md first. A previous engineer started src/extra_backend_fbw.rs; read it and finish.
- **What you may touch:** minimal edits in prim.rs, failures.rs, throttle.rs, engine_commands.rs, extra_backend_fcdc.rs, and lib.rs slot lines.
- **js_bridge.rs:** don't edit it, except the one existing line that reads `crate::extra_backend_fbw::time_of_day()` for `E:TIME OF DAY`. Values the scripts read are resolved on X-Plane's main thread each frame, and must not block.

## Goal
Port the remaining FBW A380X FlyByWireInterface parts marked MISSING in docs/systems-coverage.md. The source is D:\fbw-aircraft\fbw-a380x\src\wasm\fbw_a380\src\FlyByWireInterface.cpp and its helpers. Port exactly, with file:line citations.

1. **C++ computer failures.** Register every failure id FailuresConsumer gives the PRIM/SEC/FCDC/FCU (27xxx, 22xxx, 31xxx, 34xxx...; see the A380X failure definitions and `failuresConsumer.isActive`) in src/failures.rs, with FBW's names. Feed them where the C++ reads them in prim.rs and extra_backend_fcdc.rs, and remove them from `UNAVAILABLE`.
2. **Controls.** `A32NX_SIDESTICK_POSITION_X/Y`, `A32NX_RUDDER_PEDAL_ANIMATION_POSITION`, `FLIGHT_CONTROLS_TRACKING_MODE`.
3. **Reversers.** FBW's reverser aspect (a380_systems_wasm reversers.rs: `REVERSER_DELTA_SPEED`, `REVERSER_ANGULAR_ACCELERATION`) versus X-Plane's own reverse (propmode 3), which the plugin uses today. Pick the faithful approach without double counting, justify it, then implement and test it.
4. **LightSync inputs.**
   - `E:TIME OF DAY` (MSFS enum).
   - `A:ON ANY RUNWAY`, from src/navdata runways.
   - `GLASSCOCKPIT AUTOMATIC BRIGHTNESS`, only with a real basis; otherwise document why not.

   Write A: values as Vars simulator variables.
5. **handleSimulationRate**, onto `sim/time/sim_speed`.
6. **updatePerformanceMonitoring** (`A32NX_PERFORMANCE_WARNING_ACTIVE`).

## Tests
Add tests for each item. Keep the full suite green; use CARGO_TARGET_DIR=D:\A380\fbw-build\target-fbwiface. Fix your own build errors first (for example the ambiguous float `to_radians`). Update docs/systems-coverage.md, and mark the fuel pump `CIRCUIT CONNECTION ON:n` row fixed.

## Report
Per item, with evidence, plus your lib.rs lines.

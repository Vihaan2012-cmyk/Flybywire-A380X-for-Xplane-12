# FlyByWire A380X systems in X-Plane 12

An X-Plane 12 plugin that runs FlyByWire's own A380X systems simulation
(electrics, hydraulics, bleed air, fuel, APU, gear, brakes and the rest of
their Rust code) outside MSFS, against X-Plane's flight model.

It works because FlyByWire's systems never talk to MSFS directly: they read
and write named variables through two small traits,
`SimulatorReaderWriter` and `VariableRegistry`. MSFS's SimConnect glue lives
in a separate crate (`a380_systems_wasm`), which this plugin replaces. The
systems crates themselves are portable Rust and compile for Windows
unchanged.

    FlyByWire a380_systems (unchanged)
              |
        SimulatorReaderWriter          <- this plugin implements it
              |
    X-Plane datarefs (input + output)

## What the plugin does

- Builds FlyByWire's `A380` and ticks it every flight loop with X-Plane's
  frame time.
- Feeds the simulation the state it expects each tick (airspeed, altitude,
  attitude, accelerations, winds, weight and so on) from X-Plane datarefs,
  converted to the units FlyByWire's `UpdateContext` reads: knots, feet,
  feet per second squared, degrees Celsius, pounds, degrees.
- Publishes every variable the systems read or write as an X-Plane dataref
  under `fbw/`, so the cockpit, Lua and other plugins can see and set them.
  A variable FlyByWire calls `A32NX_ELEC_AC_1_BUS_IS_POWERED` becomes
  `fbw/A32NX_ELEC_AC_1_BUS_IS_POWERED`.

## What it does not do

- No displays (PFD, ND, ECAM): those are FlyByWire's TypeScript instruments
  and need a browser runtime.
- No fly-by-wire control laws or autopilot: that is their C++ module, a
  separate port.
- Cockpit switches are not yet wired to the systems' input variables; MSFS
  does that in its model behaviour XML.

## Building

FlyByWire's workspace pins the MSVC toolchain for its wasm build. This
plugin builds with the GNU toolchain, which needs no Visual Studio:

    cargo +stable-x86_64-pc-windows-gnu build --release

Copy `target/release/fbw_a380_systems.dll` to the aircraft as

    Aircraft/<aircraft>/plugins/fbw_a380_systems/64/win.xpl

## Licence

FlyByWire's A380X is GPL-3.0; this plugin links their systems crates, so it
is GPL-3.0 as well. It is a local build for testing, not a redistribution.

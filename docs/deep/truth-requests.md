# What the live areas need that `Truth` does not carry yet

Running list, filled in as each area grows a live system. Nothing here is
invented: each entry is a field an area needs to make a *registered*
failure do what its own `effect` text says. An area that lacks one has
said so rather than substituting a plausible number.

Apply as one pass once every area's live system exists, so the struct
does not change under agents mid-flight.

## Contract gap: an area cannot read another area's output

`live.rs`'s module doc says areas read the previous frame's values of
anything another area publishes. **There is no mechanism for that** —
`Area::tick(&Truth, &Faults)` has no access to published values. This is
a bug in the contract, not in any area.

The fix is for `Deep::tick` to keep the map it collects from `publish`
and hand the previous frame's map to `tick`. It changes the trait
signature, so it lands after the area agents are done.

What it unblocks, concretely: `pneumatic_ducts`' overheat detection loops
watch the bay each duct runs through, and those bay temperatures are
`thermal_zones`' output. Without it, a duct leak publishes its heat and
the loop that should trip on it never sees it — the leak → overheat →
isolation chain the whole area exists for is cut in the middle. Every
zone is currently given recovery temperature, which is the right
baseline for an unheated ram-ventilated bay, so false-trip faults work
and real ones do not.

## Cockpit control state

The largest single gap. Many registered failures are inert not because
the physics is missing but because nothing tells the model what the crew
selected — a valve stuck *closed* is invisible when the valve was never
commanded open.

| Field | Unblocks |
|---|---|
| Fire pushbutton + agent pushbutton, per engine and per cargo bay | 20 bottle squib failures and the whole suppression chain (`fire_ice`). Only the APU's works today, because its ground discharge is the one path needing no crew action. |
| Wing (x2) and nacelle (x4) anti-ice selection | 12 anti-ice failures (`fire_ice`) and 3 wing anti-ice duct failures (`pneumatic_ducts`). Stuck-*open* works today because it floors flow regardless; stuck-closed and duct leak do not. |
| Engine/APU bleed pushbuttons, cross-bleed selector, pack flow control valve positions | `pneumatic_ducts` currently runs on a documented `live::ControlAssumptions` (packs open, WAI off, starters off, cross-bleed open only when the APU is sole source). |
| Starter engagement, per engine | 4 start-duct failures (`pneumatic_ducts`); the start ducts sit at ambient today. |
| Rain removal selection | 2 failures (`fire_ice`); no jet exists to degrade. |
| Commanded gear door position (nose/wing/body) | 3 door jam failures (`thermal_zones`); the jam latch works but has nothing to diverge from. |

## Environment and engine

| Field | Unblocks |
|---|---|
| Solar irradiance (or sun elevation) | `ThermalNetwork::step` takes a solar flux and every zone carries a sun-exposure fraction. Passed as 0 today rather than inventing 800 W/m2. |
| `engine_hp_port_pressure_pa` / `_temp_k`, per engine | The HP6 branch and its stuck-valve failure, and makes the precooler genuinely work — HP6 is the hot source it exists to cool. `Truth` carries one bleed port; the model has a real IP8 tap *and* an HP6 valve. HP6 is fed 0 Pa today, which is below FBW's own 15 psi interlock so the valve correctly stays shut, rather than claiming HP6 = IP8, which is wrong by about 200 K. |
| Cabin / lavatory local temperature | The lavatory fusible link (`fire_ice`). |
| `engine_bypass_mdot_kg_s` from the crate's own engine model | Would replace `pneumatic_ducts`' N1-derived estimate of precooler cooling air. |

## Failure granularity

`pneumatic_ducts`' registry deliberately registers one id per fault
mechanism per component *class* ("x4 engines"), so arming the duct leak
applies it to all four engine ducts at once. Per-engine arming needs
either per-instance failure ids or a per-instance channel in `Faults`.
Worth a decision before the EFB exposes these to the crew.

## Model gaps noted in passing

- Nothing downstream of the pneumatic ducts actually consumes air: a
  pack duct fills and stops, so at steady state every flow is zero and
  the precooler has nothing to exchange. A pack discharge into cabin
  pressure fixes it but needs a cabin pressure input and is a topology
  change.
- `CARGO_BULK_SMOKE_DETECTED` is published by nobody: `fire_ice` has no
  bulk hold and registers no bulk detector, so publishing it would be a
  hardcoded zero. The bulk alert still reaches its trigger through
  `thermal_zones`' contribution on the bay's smoke concentration. If the
  primary trigger should be live, a bulk detector needs registering.

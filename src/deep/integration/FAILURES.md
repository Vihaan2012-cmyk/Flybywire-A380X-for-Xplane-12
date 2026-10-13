# Integration area — failure catalogue

`ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect`.

**Empty by design.** Integration originates no new injectable physical
failures of its own — see `registry.rs`'s module doc for the full
reasoning. Every failure a user can inject (a jammed/runaway/blown-back
flight-control surface, an icing encounter, a bird strike/lightning/hail/
volcanic-ash event, a collapsed gear leg) already belongs to, and is
registered by, the modelling area that owns it:

| Physical fault | Owning area's registry |
|---|---|
| Aileron/elevator/rudder/spoiler/THS/flap/slat jam, runaway, blow-back, disconnect, flutter-damper loss | `Area::FlightControls` (once `deep/flight_controls/registry.rs` exists) |
| Ice accretion on wing/nacelle/probe/windshield | `Area::FireIce` (`deep/fire_ice/registry.rs`) |
| Probe icing/heater/mechanical faults | `Area::Sensors` (`deep/sensors/registry.rs`) |
| Bird strike, lightning, hail, volcanic ash, ice-crystal icing, runway contamination, wind shear/turbulence | `Area::Environment` (`deep/environment/registry.rs`) |
| Collapsed/structurally-failed gear leg | `Area::GearStructure` (`deep/gear_structure/registry.rs`) |

This directory's own `registry.rs` registers only the interface
`ComponentDef`s that carry those areas' outputs onto the real aircraft
(the weather-truth feed, the aggregate applied ice state, the per-leg
X-Plane relay), each with a diagnostic health parameter and no failure of
its own — nothing here is a "renaming" of another area's fault, and
nothing here is padding: it is a genuinely empty section because this
area's job is relaying, not originating.

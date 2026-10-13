# Circuit breakers — failures

Format: `ATA | proposed name | model element it acts on | magnitude meaning
(0..1) | effect`.

Both faults below are genuinely one physical mechanism each, instanced once
per breaker (the same "×4 engines, L/R, green/yellow" expansion convention
`docs/deep/BRIEF.md` describes for every other area's components) —
`registry.rs` registers a real, distinct `FailureDef` per breaker per fault
(373 breakers × 2 = 746 failure ids, spread across ATA 21-36, 44, 49, 52,
73-74 as `catalog.rs`'s own table lists — including the 77 control/
excitation-supply breakers added in session 2), not two lines standing in
for something un-instanced.

| ATA | proposed name | model element | magnitude (0..1) | effect |
|---|---|---|---|---|
| (per breaker's own ATA) | `<BREAKER NAME> nuisance trip (calibration drift)` | `deep::breakers::trip::Breaker.step`'s `BreakerFaults.trip_calibration_drift` | 0 = correctly calibrated bimetal spring / SSPC reference .. 1 = fully drifted low; lowers this breaker's own effective trip threshold by up to 40% of its true rated current | the breaker opens under a load it should carry, de-energising whatever load/equipment it protects (its own `protected_load` -- true for 396 of the 399 as of the gap-closing pass -- or the named real equipment for the 3 remaining battery-output breakers with no load model, see `catalog::ata24_power_sources`) even though nothing downstream actually failed |
| (per breaker's own ATA) | `<BREAKER NAME> fails to trip (contact weld)` | `deep::breakers::trip::Breaker.step`'s `BreakerFaults.contact_resistance` | 0 = clean contacts .. 1 = fully welded/fused by repeated arcing; multiplies the I²t heat threshold and the magnetic instantaneous multiple by `1 / (1 - contact_resistance)` (clamped at 0.999) — diverging as the fault approaches 1.0, not a fixed multiple, so a fully welded breaker's threshold sits far above anything a sustained overload can reach | the breaker does not open on a genuine overload or short, so its protected load/equipment keeps drawing fault current downstream of a breaker that should have isolated it — the dangerous, real aerospace contactor/breaker failure mode this fault represents |

No other distinct physical fault is modelled in this directory. Magnetic
instantaneous trip and SSPC arc-fault detection are correct, healthy
*protective* behaviour (`trip.rs`'s `TripCause::Magnetic`/`ArcFault`), not
failure modes of the breaker itself, and are exercised by `trip.rs`'s own
unit tests rather than listed here.

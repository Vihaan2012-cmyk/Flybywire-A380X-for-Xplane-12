# Failures — src/deep/sensors

`ATA | name | model element | magnitude (0..1 unless noted) | effect`. One row per genuinely
distinct physical fault mechanism (per `docs/deep/BRIEF.md`'s working-style rule: "do not pad
with renamings"); `registry.rs` is where each mechanism is expanded into its real per-instance
components/failures (e.g. the generic "temperature sensor open circuit" row below backs a
separate registered failure for each of the ~10 different physical temperature sensors this
directory places — T25, oil temperature, hydraulic reservoir temperature, brake temperature,
fuel temperature, duct temperature — since it is the exact same open-circuit RTD failure
mechanism at a different installation point, not a new mechanism). Full detail (component id,
exact `model_field` path, sourcing) is in `registry.rs`.

| ATA | name | model element | magnitude | effect |
|---|---|---|---|---|
| 34 | Pitot heater failure | `pitot::PitotProbe` heater | 0 healthy .. 1 no heat | Ice accretes in icing conditions, eventually blocking the tube (see next) |
| 34 | Pitot tube insect/tape blockage | `pitot::PitotProbe` open area | 0 clear .. 1 sealed | Pneumatic lag grows below ~0.97 open area; full blockage above it |
| 34 | Pitot tube mechanical damage | `pitot::PitotProbe` open area | 0 clear .. 1 sealed | Same as above, separate cause |
| 34 | Pitot drain hole blocked | `pitot::PitotProbe` drain | 0 clear .. 1 sealed | With tube blocked: clear drain decays sensed total pressure to static (airspeed sags to zero); blocked drain freezes it (airspeed then tracks altitude changes inversely) |
| 34 | Static port blocked | `static_port::StaticPort` | 0 clear .. 1 (>=0.98 sealed) | Sensed static pressure freezes; altitude/airspeed stop tracking reality |
| 34 | Static line leak to cabin | `static_port::StaticPort` leak | 0 none .. >1 leak dominates | Sensed static pressure biased toward cabin pressure |
| 34 | AoA vane heater failure | `aoa_vane::AoaVane` heater | 0 healthy .. 1 no heat | Vane ices and jams in icing conditions (see next) |
| 34 | AoA vane mechanically stuck | `aoa_vane::AoaVane` hinge | 0 free .. 1 (>=0.98 seized) | Reported AoA frozen at last free angle |
| 34 | AoA resolver wear/drift | `aoa_vane::AoaVane` resolver | 0 none .. 1 max drift rate | Slow random-walk bias growth in reported AoA |
| 34 | AoA vane damage (bent) | `aoa_vane::AoaVane` bias | 0 none .. 1 (8 deg max) | Fixed offset error from the moment of damage |
| 34 | TAT probe heater failure | `tat_probe::TatProbe` heater | 0 healthy .. 1 no heat | Icing grows the element's thermal time constant, slowing/biasing the reading |
| 34 | TAT probe recovery factor degradation | `tat_probe::TatProbe` recovery | 0 (r=0.99) .. 1 (r=0) | Reported TAT reads low relative to true recovery temperature |
| 34 | Radio altimeter transceiver electronics fault | `radio_altimeter::RadioAltimeter` transceiver | 0 healthy .. 1 (>=0.98 failed) | No valid height output |
| 34 | Radio altimeter transmit antenna fault | `radio_altimeter::RadioAltimeter` tx antenna | 0 healthy .. 1 (>=0.98 failed) | No valid height output |
| 34 | Radio altimeter receive antenna fault | `radio_altimeter::RadioAltimeter` rx antenna | 0 healthy .. 1 (>=0.98 failed) | No valid height output |
| 34 | Radio altimeter receive antenna gain loss | `radio_altimeter::RadioAltimeter` rx antenna | 0 healthy .. 1 as severe as tracking-loop degradation | Height reading jitters more (reduced SNR), stays valid — same symptom as tracking-loop degradation, distinct physical cause |
| 34 | Radio altimeter false fixed-offset reading | `radio_altimeter::RadioAltimeter` offset | signed ft, not 0..1 | Constant height bias at every true height (e.g. the historically documented -6 ft ground reading) |
| 34 | Radio altimeter tracking-loop filter degradation | `radio_altimeter::RadioAltimeter` noise | 0 baseline .. 1+ extra | Height reading jitters more, worse near ground/over water |
| 34 | GPS receiver electronics fault | `gps::GpsReceiver` receiver | 0 healthy .. 1 (>=0.98 no fix) | No GPS position at all |
| 34 | GPS antenna fault | `gps::GpsReceiver` antenna | 0 healthy .. 1 (>=0.98 no fix) | No GPS position at all — same effect as receiver fault, distinct physical part |
| 34 | GPS antenna gain loss | `gps::GpsReceiver` antenna | 0 none .. 1 as severe as full jamming | Acts like jamming on the effective satellite count — distinct physical cause (passive gain loss vs. active RF interference) |
| 34 | GPS jamming | `gps::GpsReceiver` effective sats | 0 none .. 1 all unusable | Effective satellites fall, position error grows, fix lost below 4 |
| 34 | GPS spoofing | `gps::GpsReceiver` spoof target | commanded offset (m), not 0..1 | Position walks off gradually toward the spoofed target |
| 30 | Ice detector deice heater failure | `ice_detector::IceDetector` heater | 0 healthy .. 1 no deice | ICE DETECTED latches instead of cycling; ice keeps accumulating |
| 30 | Ice detector frequency sensor drift | `ice_detector::IceDetector` electronics | signed fractional shift, not 0..1 | Can mask real icing (false negative) or fabricate a shift (false positive) |
| 30 | Ice detector probe damage | `ice_detector::IceDetector` probe | signed fractional shift, not 0..1 | Offsets the calibrated baseline; can false-trigger with no ice present |
| 32/52 | Proximity sensor gap out of rigging | `discrete::ProximitySensor` gap | signed mm, not 0..1 | Near/far (or open/closed) switch point shifts; wrong position indication without a hard stuck fault — applies to both landing-gear (uplock/downlock/WOW, ATA 32) and door (open/closed, ATA 52) installations of the same sensor |
| 32/52 | Proximity sensor stuck near | `discrete::ProximitySensor` | 0 healthy .. 1 (>=0.5 stuck) | Always reports target present |
| 32/52 | Proximity sensor stuck far | `discrete::ProximitySensor` | 0 healthy .. 1 (>=0.5 stuck) | Always reports target absent |
| (28, not registered by this directory — see below) | Fuel probe water contamination | `discrete::fuel_probe_indicated_level` | 0 none .. 1 all water | Indicated quantity reads high vs. true liquid volume. The physics function still lives here (also backs `discrete::oil_probe_indicated_level`, ATA 79, which *is* registered), but no ATA 28 tank instance is registered by this directory: a dedicated fuel agent now owns `src/deep/fuel/` end to end, per the lead's instruction. |
| (28, not registered by this directory — see above) | Fuel probe open circuit | `discrete::fuel_probe_indicated_level` | 0 healthy .. 1 (>=0.98 open) | Indicated quantity reads zero |
| 79 | Oil quantity probe water contamination | `discrete::oil_probe_indicated_level` | 0 none .. 1 all water | Indicated oil quantity reads high vs. true volume (failed oil cooler/breached seal) — same capacitance mechanism as the fuel probe, different liquid/permittivity and a physically separate component |
| 77 | Engine speed pickup air gap increase | `engine_sensors::speed_pickup_reading` gap | 0 nominal .. 1 max modelled | Signal amplitude falls; below the EEC's detection floor this raises the minimum speed at which the channel can still detect the pickup |
| 77 | Engine speed pickup open circuit | `engine_sensors::speed_pickup_reading` wiring | 0 healthy .. 1 (>=0.98 open) | No signal at any speed |
| 77 | TGT harness junction open | `engine_sensors::tgt_harness_average_c` junction | 0 healthy .. 1 (>=0.98 open, per junction) | That junction drops out of the average, biasing it toward whichever junctions remain |
| 77 | TGT harness junction drift | `engine_sensors::tgt_harness_average_c` junction | signed K offset, not 0..1 | Biases the average by roughly offset/junction_count |
| 77 | Vibration pickup bias | `engine_sensors::VibrationPickup` | signed, not 0..1 | Constant offset added to the true reading |
| 77 | Vibration pickup stuck output | `engine_sensors::VibrationPickup` | 0 healthy .. 1 fully frozen | Reading stops responding to true vibration |
| 77 | Vibration pickup intermittent dropout | `engine_sensors::VibrationPickup` connector | probability/s, not 0..1 | Momentary signal loss (loose connector), holding the last value |
| 73 | Fuel flow transmitter bearing wear | `engine_sensors::fuel_flow_transmitter_reading` rotor | 0 nominal .. 1 max modelled | Rotor under-spins for the true flow; the meter under-reads |
| 73 | Fuel flow transmitter debris blockage | `engine_sensors::fuel_flow_transmitter_reading` inlet | 0 none .. 1 fully blocked | Less flow reaches the rotor than the engine actually burns; the meter under-reads — distinct cause from bearing wear (acts upstream of the rotor, not on its calibration) |
| 73 | Fuel flow transmitter stuck rotor | `engine_sensors::fuel_flow_transmitter_reading` rotor | 0 healthy .. 1 (>=0.98 seized) | Reads zero/fixed regardless of true flow |
| 26 | Smoke detector desensitised optics | `smoke_detector::SmokeDetector` optics | 0 clean .. 1 fully desensitised | Delayed or missed detection of real smoke |
| 26 | Smoke detector spurious signal | `smoke_detector::SmokeDetector` electronics | %/ft added, not 0..1 | Can alarm with no smoke present |
| 26 | Smoke detector stuck output | `smoke_detector::SmokeDetector` | 0 healthy .. 1 fully frozen | Reading stops responding to true smoke density |
| 28/29/32/36/77/79 | Temperature sensor open circuit | `discrete::temperature_sensor_reading_c` | 0 healthy .. 1 fully open | Reading pegs to top of indicating range — same RTD mechanism reused for fuel/hydraulic-reservoir/duct/T25/oil temperature sensors, a distinct registered failure per installation in `registry.rs` |
| 28/29/32/36/77/79 | Temperature sensor short circuit | `discrete::temperature_sensor_reading_c` | 0 healthy .. 1 fully shorted | Reading pegs to bottom of indicating range — same reuse as above |
| 21/29/32/35/77/79 | Pressure transducer zero drift | `discrete::PressureTransducer` bias | signed Pa/hr, not 0..1 | Indicated pressure slowly diverges from truth — reused for hydraulic/tyre/P30/oil/oxygen/cabin pressure, a distinct registered failure per installation |
| 21/29/32/35/77/79 | Pressure transducer stuck output | `discrete::PressureTransducer` | 0 healthy .. 1 fully frozen | Indicated pressure stops responding to reality — same reuse as above |
| 29 | Float level sensor binding | `float_level::FloatLevelSensor` | 0 free .. 1 (>=0.98 seized) | Indicated quantity freezes regardless of true fluid volume changes |
| 29 | Float level sensor bias | `float_level::FloatLevelSensor` | signed fraction of full scale, not 0..1 | Constant offset added to the indicated quantity |
| 29 | Float level sensor open circuit | `float_level::FloatLevelSensor` | 0 healthy .. 1 (>=0.98 open) | Indicated quantity reads a conservative zero |
| 32 | Brake wear pin/sensor binding | `brake_wear::BrakeWearPin` | 0 free .. 1 fully seized | Indicated remaining brake life stops tracking real wear, overstating remaining life while the real stack keeps wearing — dangerous indicated-vs-real divergence |
| 32 | Brake wear sender bias | `brake_wear::BrakeWearPin` | signed fraction of full scale, not 0..1 | Offsets remaining-life indication in either direction |
| 32 | Brake wear open circuit | `brake_wear::BrakeWearPin` | 0 healthy .. 1 (>=0.98 open) | Indicated remaining life reads a conservative zero |
| 34 | Static averaging line blocked | `static_port::average_pair` | 0 clear .. 1 (>=0.98 blocked) | Isolates the system's left/right static ports from each other: sideslip position-error cancellation is lost, but neither port's own reading is invalidated |
| 77 | TGT junction position vs. a hot streak | `engine_sensors::tgt_harness_average_c` (`HotStreak` input) | not a fault of this component — a plain input from elsewhere (e.g. a coked fuel nozzle group) | A junction near the streak's centre reads locally hot/cold; the harness average rises/falls by less than the peak, in proportion to how many junctions sit near it |

# Progress — engine_accessories

- [done] LP (boost) fuel pump, centrifugal, affinity-law curve + NPSH cavitation — `fuel/lp_pump.rs` — wear + inlet restriction faults, cavitation emerges from NPSH shortfall, not scripted.
- [done] Fuel filter with bypass valve — `fuel/filter.rs` — mirrors `physics::engine::oil`'s filter/bypass pattern; clog fault, impending-bypass threshold.
- [done] HP gear fuel pump, displacement ∝ N3 — `fuel/hp_pump.rs` — wear (internal slip) + inlet-starvation faults.
- [done] Fuel metering unit: metering valve + constant-dP spill valve — `fuel/fmu.rs` — valve sticking (rate-limited actuator), spill stuck-open/stuck-closed faults.
- [done] HP fuel shut-off valve — `fuel/shutoff_valve.rs` — rate-limited travel, stuck fault.
- [done] Fuel flow transmitter, dual-pickoff turbine flowmeter — `fuel/flow_transmitter.rs` — per-channel bias/frozen faults, feeds future EEC voting (`eec.rs`).
- [done] Burner manifold + 8 nozzle groups, coking → hot streak — `fuel/manifold.rs` — per-group blockage fault, common-manifold-pressure bisection conserves total flow, `hot_streak_severity` output documented for a combustor/hot-section consumer.
- [done] Ignition: two exciter/igniter chains per engine, RC charge-time spark rate, plug-erosion breakdown-voltage cutoff — `ignition.rs` — exciter failure + igniter erosion faults (×2 chains).
- [superseded] Moved catalogue/ECAM content into code per the lead's later instruction — deleted `CATALOGUE.md`/`ECAM.md`, added `registry.rs` (`pub fn register(r: &mut Registry)`, `crate::deep::api`), declared in `mod.rs`. Covers all fuel-system and ignition failures/components/alerts above.
- [done] Starting: starter air valve (stuck open/closed/slow, one continuous fault axis), air turbine starter + sprag clutch (fails to engage = hung start, fails to disengage + sustained high N3 = disintegration), starter duty-cycle heating — `starting/air_valve.rs`, `starting/turbine.rs`, `starting/duty_cycle.rs` — registered in `registry.rs` (`register_starting`, ATA 80).
- [fixed] Reviewer blockers: `fuel/hp_pump.rs` `DISPLACEMENT_M3_PER_REV` was ~6.6x too small (2.5e-5 now, design flow ~4.07 kg/s vs the gas path's real ~2.48 kg/s design point, `physics::engine::gas_path`); `fuel/fmu.rs` `AREA_MAX_M2` was ~4.7x too small (1.7e-4 now, full-open ~5.2 kg/s); `fuel/manifold.rs`'s `combustor_pa` was unused, now feeds a new `ManifoldState.manifold_absolute_pa` output plus a test; `fuel/flow_transmitter.rs`'s bare `5.0` is now the named, cited `PICKOFF_BIAS_MAX_KG_S`. All stale "~3.4 kg/s / docs/physics/engine.md" citations replaced with the gas path's real ~2.48 kg/s figure.
- [done] Compressor airflow control: IP VSV actuator + schedule (jam, rigging bias -> stall-margin output), IP/HP handling bleed valves (orifice bleed flow + schedule, jam -> either bleeds core air or loses margin) — `airflow_control/vsv.rs`, `airflow_control/bleed_valve.rs` — registered in `registry.rs` (`register_airflow_control`, ATA 72/75). Both expose a documented `stall_margin_delta_pct` output for the gas path's compressor model to consume; neither touches `physics::engine::compressor`.
- [done] Rotor dynamics: per-spool (N1/N2/N3) imbalance vibration (blade loss/ice/bird strike -> eccentric mass -> forced SDOF response -> tracking-filter-lagged EICAS-style index), bearing defect signature (BPFO/BPFI/BSF/FTF from standard bearing kinematics) — `rotor_dynamics/imbalance.rs`, `rotor_dynamics/bearing.rs` — registered in `registry.rs` (`register_rotor_dynamics`, ATA 77). One representative main-bearing set per engine rather than every individual bearing, noted as a scope simplification.
- [done] Thrust reverser: 3-lock (primary/secondary/tertiary) hydraulic actuation, OR-redundant holding vs AND-required release, uncommanded deployment only from all-three-locks failing, separate actuator-jam fails-to-deploy/stow path — `thrust_reverser.rs` — inboard engines 2/3 only (the real A380 carries no reverser on 1/4, per FBW's own `a380_systems/reverser` model). Registered in `registry.rs` (`register_thrust_reverser`, ATA 78).
- [done] EEC: dual independent channels each with their own N1/N2/N3/TGT/P30 sensors (reusing `fuel::flow_transmitter`'s existing dual fuel-flow pick-offs as the sixth voted parameter rather than re-modelling it), channel-fault-driven A/B selection (both dead -> no valid channel), per-parameter sensor bias/frozen faults with disagree flags — `eec.rs` — registered in `registry.rs` (`register_eec`, ATA 73, `n` counter starts at 500 to stay clear of the fuel system's own ATA-73 range).
- [done] Nacelle: anti-ice valve (orifice bled flow + cowl lip heat balance, stuck fault), ventilation (ram + eductor terms, blockage faults, vapour-accumulation-risk output per CS-25-style ventilation intent), fire/overheat detection (dual-loop per zone x2 zones, sense-only, `confirmed`/`loop_disagree` outputs as the documented hand-off interface to the dedicated fire-system agent -- no suppression logic here) — `nacelle/anti_ice.rs`, `nacelle/ventilation.rs`, `nacelle/fire_detection.rs` — registered in `registry.rs` (`register_nacelle`, ATA 30/71/26). This completes the original backlog; continuing to the next most valuable items.
- [done] Rotor dynamics extended per coordinator request: replaced the one representative main-bearing set with the Trent's real 5-bearing arrangement, matching `physics::engine::oil`'s own three chambers exactly (Front = fan+IP front bearings, HpIp = HP+IP turbine bearings, Tail = LP turbine rear bearing) — `rotor_dynamics/bearings.rs` (new; `rotor_dynamics/bearing.rs`'s pure `signature()` physics unchanged and reused, not duplicated). Each bearing driven by its own spool's speed, with its own 4 spall faults, an `overall_wear` aggregate, and a stateful oil-debris/chip-detector model (debris accumulates from the worst active defect x speed, trips a discrete `chip_detected` past a threshold, only clears via explicit `clear_chip_detector()`).
- New Vars this area's models need published (none exist yet, all proposed `A32NX_ENG_n_*`, see `registry.rs` for exact names/where used): `A32NX_ENG_n_FUEL_FILTER_IMPENDING_BYPASS`, `A32NX_ENG_n_FUEL_FILTER_BYPASSED`, `A32NX_ENG_n_HP_PUMP_LOW_FLOW`, `A32NX_ENG_n_FF_DISAGREE`, `A32NX_ENG_n_FMU_FAULT`, `A32NX_ENG_n_THRUST_ABNORMAL`, `A32NX_ENG_n_HP_SOV_DISAGREE`, `A32NX_ENG_n_FF_CHANNEL_DISAGREE`, `A32NX_ENG_n_NOZZLE_IMBALANCE`, `A32NX_ENG_n_NO_IGNITION_AVAILABLE`, `A32NX_ENG_n_START_VALVE_DISAGREE`, `A32NX_ENG_n_STARTER_DISENGAGE_FAULT`, `A32NX_ENG_n_STARTER_DISINTEGRATED`, `A32NX_ENG_n_STARTER_OVERHEAT`, `A32NX_ENG_n_VSV_SCHEDULE_ERROR_DEG`, `A32NX_ENG_n_IP_HANDLING_BLEED_DISAGREE`, `A32NX_ENG_n_HP_HANDLING_BLEED_DISAGREE`, `A32NX_ENG_n_N1_VIB_INDEX`, `A32NX_ENG_n_N2_VIB_INDEX`, `A32NX_ENG_n_N3_VIB_INDEX`, `A32NX_ENG_n_{SPOOL}_{BEARING}_DEFECT_AMPLITUDE_MM_S` and `A32NX_ENG_n_{BEARING}_CHIP_DETECTED` for each of the 5 bearings (FAN_FRONT, IP_FRONT, HP_TURBINE, IP_TURBINE, LP_TURBINE_REAR; supersedes the earlier single `A32NX_ENG_n_BEARING_DEFECT_AMPLITUDE_MM_S`), `A32NX_ENG_n_REV_UNCOMMANDED`, `A32NX_ENG_n_REV_POSITION_DISAGREE` (n = 2, 3 only), `A32NX_ENG_n_EEC_CHANNEL_FAULT`, `A32NX_ENG_n_EEC_NO_VALID_CHANNEL`, `A32NX_ENG_n_EEC_SENSOR_DISAGREE`, `A32NX_ENG_n_ANTI_ICE_DISAGREE`, `A32NX_ENG_n_NACELLE_VAPOUR_RISK`, `A32NX_ENG_n_CORE_FIRE_LOOP_DISAGREE`, `A32NX_ENG_n_FAN_FIRE_LOOP_DISAGREE`.

## Live system (deep push, `live.rs`)

- [done] `live.rs` — the area's live instance behind `crate::deep::live::Area`
  (`live_system() -> Box<dyn Area>`), declared in this directory's `mod.rs`.
  `tick` drives the real models from `deep::live::Truth` and applies every
  failure `registry.rs` registers by reading `Faults::get(id)` into the exact
  `model_field` that entry names; `publish` emits every variable this area's
  ECAM triggers cite, plus the state behind them for the EFB Study pages.
  Failure ids are resolved at construction by registering into a throw-away
  `Registry` and looking each one up by component + `model_field`, so a
  renumbering in `registry.rs` fails loudly instead of silently unhooking a
  failure. Inputs `Truth` does not carry yet are collected in one documented
  `...Commands` struct per area rather than invented.
- [done] sourced-constants pass — fuel/common.rs — `viscosity_cst` relabelled as an explicit specification-*worst-case* curve rather than a typical batch: its cold anchor is DEF STAN 91-091 / ASTM D1655's -20 C ceiling of 8.0 mm^2/s, which is why it reads ~1.92 mm^2/s at 20 C against a typical batch's ~1.7. Searched CGSB 3.23, ASTM D1655, DEF STAN 91-091, CRC Report 635 and supplier data sheets for a citable *typical* cold-end figure: every public source pins only the maximum, so no re-anchor is possible. Left as is deliberately -- high viscosity is the demanding direction for the filter and pump models.
- [done] **Whole area brought live** — `live.rs` — `EngineChain` now owns, per
  engine, every subsystem this area models, not just the fuel chain and the
  EEC: both ignition chains (ATA 74), the starter air valve + air turbine
  starter + duty-cycle heating (80), the IP VSV and both handling bleed
  valves (72/75), three spools' imbalance vibration and all five bearings'
  defect signatures and chip detectors (77), the thrust reverser on engines
  2/3 (78), and the nacelle anti-ice valve, ventilation and both fire zones'
  detection loops (30/71/26). Every one of this area's registered failures is
  now routed into the `model_field` its entry names, and every variable its
  104 previously-unreachable ECAM alerts trigger on is published.
- [done] **The fuel chain was metering zero.** `EngineAccessoryCommands::
  wf_command_kg_s` defaulted to `[0.0; 4]` and nothing ever set it, so
  `metering_error_fraction` returned 0 unconditionally and the whole chain was
  inert. It reads `Truth::engine_fuel_flow_kg_s` — this crate's own engine
  model's real flow into the combustor — and the override is now an
  `Option`, for tests only.
- New cross-area reads (through `Truth::published`, one frame behind):
  `DEEP_PNEU_ENG_n_START_DUCT_PRESSURE_PA` (`deep::pneumatic_ducts`) drives the
  air turbine starter's supply; `THERMAL_ZONE_NACELLECOWLn_TEMPERATURE_C`
  (`deep::thermal_zones`) is what the fire detection loops sense.
- [fixed, W165] Was inert: the six reverser lock faults and the reverser
  actuator jam need a *deploy command*, and `tick()` built its per-frame
  `EngineAccessoryCommands` from `self.commands` -- a field nothing in
  production ever assigned. `Truth::controls.reverser_deploy_commanded` has
  carried the real lever (the same TLA opening-authorisation angle
  FlyByWire's own `A380ReverserController` uses) since commit e0cd0d5; `tick()`
  now copies it into `commands.reverser_deploy_commanded` every frame. The 16
  fire-loop "fails to detect" faults are correctly invisible until a nacelle
  zone is actually above the loops' trip temperature — that is what the
  redundant loop is for, not a gap.
- Published-name strings are built once per engine at construction
  (`ChainNames`) rather than `format!`-ed every frame, as
  `deep::pneumatic_ducts::live` already does.
- [fixed] `ENG n EEC CHANNEL FAULT` only knew about channel A: `EecState`
  carried `active` and nothing else, so losing the *standby* channel was
  invisible and all four channel-B faults moved nothing. `EecState` now
  reports each channel's serviceability — the same discrete verdict
  `Eec::select` already derives — and the alert means what it says on the
  aircraft: the EEC is running single-channel, whichever channel died.
- Measured: this area's 350 registered failures, swept through
  `integration::failure_audit`'s whole profile set — **320 move something
  published, 30 do not**: the 16 fire-loop `fails_to_detect` (correctly
  invisible until a nacelle zone is above the loops' trip temperature) and
  all 14 reverser faults, before W165's fix wired `Truth::controls.
  reverser_deploy_commanded` (real since e0cd0d5) into this area's own
  `tick()` -- previously read from `self.commands`, which production never
  set.
  All 104 of this area's previously-unreachable ECAM alerts now read
  published variables; `ecam_triggers_that_read_a_variable_no_area_publishes`
  reports 0 alerts that can never fire across the whole registry.
  Frame cost 7.2 us per frame, release, tick + publish, four engines.
- Asked for by the sensors area, and *not* published, because this area does
  not model them and a value would have to be invented: `A32NX_ENG_n_OIL_
  PRESSURE_PA` / `_OIL_TEMPERATURE_C` / `_OIL_QUANTITY_FRACTION` (the oil
  system is `physics::engine::oil`, the lead's, not this directory's),
  `A32NX_ENG_n_TGT_TRUE_C` (TGT is measured downstream of the LP turbine, a
  station nothing in this port computes — the same gap `EngineAccessory
  Commands::engine_tgt_k` documents) and `A32NX_ENG_n_T25_C` (HPC inlet; the
  only compressor station `Truth` carries is the customer bleed tap, which is
  IP *delivery* and port-dependent, not station 2.5). The vibration indices
  they also asked for, `A32NX_ENG_n_N1_VIB_INDEX` and `_N3_VIB_INDEX`, are
  published (with `_N2_`), so those 24 failures are unblocked.

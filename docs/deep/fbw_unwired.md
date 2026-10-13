# FlyByWire's abnormal procedures that nothing triggers

## The finding, re-counted

Counted from FlyByWire's own source by `src/deep/ecam/fbw_tests.rs`'s
`the_defined_and_wired_counts_are_what_this_module_was_built_against`, which
parses both files every run rather than trusting a number in prose:

| | count | where |
|---|---|---|
| abnormal-sensed procedures **defined** | **1004** | `fbw-a380x/src/systems/instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/*.ts` |
| ids **FlyByWire triggers** | **273** (272 of which are defined) | `systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts`, `ewdAbnormalSensed` |
| **unwired** | **732** | the difference |
| **triggered by this port** | **46** | `src/deep/ecam/fbw/` |

**686 remain.** The second pass added 7: `240800021` ELEC C/B TRIPPED,
`260800065`/`260800066` MAIN/UPPER DECK LAVATORY SMOKE, and
`701800093`..`701800096` ENG n OIL TEMP HI. All seven came out of category
(c) below — each needed a *modelling* change first, not just a trigger.

The one wired-but-not-defined id is a stale entry in FlyByWire's own map; it
is why the test intersects the two sets instead of comparing sizes.

By file, the 732 break down as

```
153  ata70.ts         (engines)              97  ata28.ts    (fuel)
 93  ata31-32-33.ts   (indicating/gear)      86  ata24.ts    (electrical)
 73  ata21-22-23.ts   (air cond/AFS/comms)   69  ata26.ts    (fire & smoke)
 68  ata27.ts         (flight controls)      58  ata34.ts    (navigation)
 18  ata29-30.ts      (hydraulics)           11  ata80-rest.ts
  6  ata46-49-52-56.ts
```

This is not only 732 missing EWD warnings. The ECL's ABN PROC page
(`instruments/src/EWD/elements/WdAbnormalSensedProcedures.tsx`) renders the
`fws_abn_sensed_procedures` bus topic, which `FwsAbnormalSensed` publishes
from `FwsCore.presentedAbnormalProceduresList`, and the only way into that
map is `FwsCore.ts:5561`'s loop over `this.ewdAbnormal`. An unwired id is
also an **electronic checklist the crew can never be shown**.

## The mechanism

`src/deep/ecam/fbw/` holds one `FbwProc` per procedure this port triggers:
FlyByWire's own nine-digit id, a `deep::api::Cond` trigger over variables
our own areas publish, the flight-phase inhibit, the SD page, the
confirmation delay, `notActiveWhenItemActive` suppressors, and per-item
`show`/`checked` predicates.

`src/deep/ecam/fbw_codegen.rs` turns those into the `__deepFbwAlerts` array,
and `deep_ecam_bridge.js`'s `installDeepEcamFbw(fws, defs, procs)` builds an
`EwdAbnormalItem` for each and puts it into `fws.ewdAbnormal`,
`fws.allSuppressableItems` and `fws.abnormalSensed.ewdAbnormalSensed` --
FlyByWire's own three live dicts, read by `Object.entries` every tick. Both
are installed from the same `FwsCore.update()` `SourcePatch` the existing
bridge already uses (`patches.rs`, patch 5), so this costs no new anchor.

Three things it deliberately does **not** do:

* **It emits no text.** No title, no item name, no INOP SYS line, no STATUS
  line. FlyByWire's catalogue already carries all of it for these ids;
  emitting a second copy under a second id is the duplication this whole
  module exists to avoid. (Contrast `codegen.rs`, which does emit text,
  because the alerts it serves are ours and FlyByWire has never heard of
  them.) `fbw_codegen`'s own test asserts no title string reaches the JS.
* **It sizes every item vector from the live procedure.** `installDeepEcamFbw`
  reads `EcamAbnormalProcedures[id].items.length` at install time and builds
  `whichItemsChecked()`/`whichItemsToShow()` to that length, so an entry can
  never be the wrong size for the checklist it belongs to
  (`FwsCore.ts:5594-5607`). `FbwProc::item_count` records the same number
  from FlyByWire's source and a test fails if the two disagree.
* **It refuses an id something else already drives.** `installDeepEcamFbw`
  skips any id already in `fws.ewdAbnormal`, and
  `no_entry_takes_an_id_flybywire_already_triggers` re-reads
  `FwsAbnormalSensed.ts` every run. Two triggers on one procedure is worse
  than none.

## Per-item wiring: what "sensed" means here

A FlyByWire checklist item is either `sensed: true` (the FWS ticks it) or
`sensed: false` (the crew ticks it, and `FwsCore.ts:5650`'s `fusedChecked`
leaves it to them entirely). For an item this port genuinely computes, the
`FbwItem` carries a `checked` condition and the line ticks itself. For one it
does not, the item carries no condition and stays crew-actioned — **that is
the correct representation, not a gap**, and it is the rule that stopped
several tempting-but-wrong tickings:

* "GEN 1+2 … OFF THEN ON" is a *cycle*; `A32NX_OVHD_ELEC_ENG_GEN_n_PB_IS_ON`
  shows a *state*. Ticking it on the state would claim a reset that may not
  have happened.
* "COMMERCIAL 1 … OFF" is an overhead pushbutton;
  `ELEC_COMMERCIAL_SHED_ACTIVE` is the *automatic* load shed. Ticking the
  line from it would claim the crew had acted when the aircraft had.

Items that do sense, today: "GEN n … OFF" on
`A32NX_OVHD_ELEC_ENG_GEN_n_PB_IS_ON` (`src/aspects.rs:612-618` writes it
every frame), and "L/G LEVER … DOWN"/"… UP" on
`GEAR_LEVER_POSITION_REQUEST`, the same variable
`deep::gear_structure::registry`'s own procedures read back.

## What is wired (46)

| ATA | ids | procedures | trigger source |
|---|---|---|---|
| 24 | `240800021` | ELEC C/B TRIPPED | `BREAKERS_TRIPPED_NOT_COMMANDED_COUNT >= 1` (2nd pass) |
| 26 | `260800065/066` | MAIN / UPPER DECK LAVATORY SMOKE | any of that deck's four `DEEP_SMOKE_LAV_n_ALARM` (2nd pass) |
| 70 | `701800093`–`701800096` | ENG n OIL TEMP HI | oil above 177 °C with the HP spool turning (2nd pass) |
| 24 | `240800004/007/009/010/012` | AC BUS 2/3/4, AC EMER, AC ESS FAULT | `ELEC_<bus>_BUS_POTENTIAL < 90 V` **and** `…_IS_POWERED` off **and** the AC network alive |
| 24 | `240800005/006/008` | AC BUS 2+3 & DC 1+2, AC 2+4, AC 3+4 & DC 2 | conjunction of the same single-bus conditions |
| 24 | `240800026/027/028/029/030/031` | DC BUS 1, 1+2, 1+ESS, 2, ESS, ESS PART | `ELEC_DC_*_BUS_IS_POWERED` |
| 24 | `240800016/018` | APU TR FAULT, BAT 2 (ESS) FAULT | `ELEC_TR_APU_FAULT`, `ELEC_BAT_2_FAULT` |
| 24 | `240800062/063/064` | GEN 2/3/4 FAULT | `ELEC_GEN_n_FAULT` |
| 24 | `240800082/083` | TR 2 FAULT, TR ESS FAULT | `ELEC_TR_2_FAULT`, `ELEC_TR_ESS_FAULT` |
| 28 | `281800089/090/091` | TRIM TK L / R / L+R PMP FAULT | `FUEL_TRIM_PUMP_DEGRADATION:1/:2` |
| 32 | `320800036` | L/G DOORS NOT CLOSED | lever up, all five legs up-locked, a `GEAR_DOOR_POSITION:n` still open |
| 32 | `320800040` | L/G GEAR NOT LOCKED UP | lever up, a `GEAR_UPLOCKED:n` still false after 30 s |
| 32 | `320800063` | WHEEL TIRE PRESS LO | any published `DEEP_TYRE_PRESSURE_SENSED_PA:n` below 90 % of `physics::tyre::COLD_PRESSURE_PA` |
| 70 | `701800057`–`701800060` | ENG n IGN A FAULT | igniters energised and chain A at 0 Hz |
| 70 | `701800061`–`701800064` | ENG n IGN B FAULT | igniters energised and chain B at 0 Hz |
| 70 | `701800085`–`701800088` | ENG n OIL PRESS LO | oil pressure below the 25 psi EASA TCDS E.012 minimum with the HP spool turning |

The **network-alive gate** on every ATA 24 bus procedure (at least one main
AC bus powered) is not a device to make an alert behave: an FWS with no
electrical power annunciates nothing on the real aircraft, and this port's JS
runs whatever the aircraft's state, so without it all eight bus procedures
would be lit at a cold and dark gate. `an_elec_bus_fault_needs_the_network_to_be_alive`
holds that.

The **core-running gate** on ENG n OIL PRESS LO is the same idea: a shut-down
engine has no oil pressure and that is not a fault
(`eng_oil_press_lo_stays_quiet_on_a_shut_down_engine`).

WHEEL TIRE PRESS LO builds its condition from the *published names*
(`ata32::tyre_pressure_vars`) rather than from a written-down wheel count.
`deep::sensors`' transducer set changed twice during this pass, and a
hard-coded range is wrong in both directions: too long and the trigger reads
a variable nobody publishes, which `Cond::eval` reads as 0 Pa -- an
instantly-true "tyre flat"; too short and a wheel added later sits silently
outside the alert. This is the one place `wirings()` is not pure data, and
it is why.

## Why the other 693 are not wired

Grouped by cause, because this list is what says where the aircraft is still
thin. Each ATA module's own doc comment carries the per-id detail; this is
the summary.

### (a) Our own registry already announces it — wiring theirs would double

Roughly **80** of the unwired ids. `deep::registry()` carries 304 alerts of
our own, and where one already has the same title and the same trigger
variable, FlyByWire's id is left alone: a second entry would put the same
warning on the EWD twice under two ids, and `notActiveWhenItemActive` cannot
suppress across the two systems.

The large clusters: ATA 70/72/73/74/77/78/80 (`ENG n EEC FAULT`, `FUEL
FILTER CLOG/BYPASS`, `HP SOV FAULT`, `FADEC FUEL METERING FAULT`, `EEC
SENSOR DISAGREE`, per-bearing `BRG CHIP DET` and `VIB HI`, `REVERSER
FAULT/UNLOCKED`, `START VALVE FAULT`); ATA 26 cargo smoke and every engine,
APU, avionics and MLG fire-detection procedure; ATA 28 `FUEL LEAK`,
`JETTISON FAULT`, `TRIM TK XFR FAULT`, `WING XFEED FAULT`, `QTY INDICATION
FAULT`, `FOB LO TEMP`, `AUTO CG XFR FAULT`; ATA 29 both `RSVR` families;
ATA 32 `GEAR NOT DOWNLOCKED`, `GEAR DISAGREE`, `ANTISKID N/U`, `PARK BRAKE
LO PR`; ATA 24 `AC BUS 1`, `BAT 1`, `GEN 1`, `TR 1`, `EMER CONFIG`, `APU GEN`.

This is why ATA 24 wires GEN **2/3/4** and TR **2/ESS** but not GEN 1 or
TR 1, and why ATA 29 and ATA 26 end up with nothing at all.

One instructive case: **HYD G/Y SYS TEMP HI** (`290800037/038`).
`HYD_{G,Y}_FLUID_TEMP_C` is published and the limit is properly sourced
(Skydrol LD-4's 107 °C maximum continuous, Eastman Pub. No. 7249153C), but
`HYD_{G,Y}_RESERVOIR_OVHT` is published as *that same comparison*
(`thermal.rs:118`), and our `HYD G RSVR OVHT` already fires on it. Splitting
the two needs a separate *manifold* temperature the hydraulics area does not
publish.

### (b) The system is not modelled at this resolution

The largest group, roughly **430** ids.

* **ATA 21/22/23** (73): cabin-fan, trim-air, zone-controller, FMC, FCU,
  RMP, HF/VHF and SATCOM procedures. No area models the AFS or the comms
  radios.
* **ATA 27** (68): every unwired one names a *computer* (PRIM, SEC, FCDC) or
  a control-law degradation. `deep::flight_controls` models surfaces,
  actuators and jams, and raises its own `F/CTL … FAULT` alerts from them;
  it has no PRIM/SEC.
* **ATA 34** (58): GPS, ILS, MMR, radio-altimeter, ADR and IR procedures.
  `deep::sensors` models the probes and the ADRs and raises its own alerts;
  the navigation receivers are not modelled.
* **ATA 31** (30): CDS display units, EFIS control panels, cursor-control
  devices, keyboards, recorders, HUD, video multiplexer — none modelled.
* **ATA 32** (30 of the 62): the braking and steering *control* system —
  normal/alternate/emergency brake selection, the two BSCU channels, the
  selector valves, the tiller and pedal transducers, gravity extension, WOW
  voting, the LGCIS channels.
* **ATA 28** (~40): the FQMS itself — auto ground transfer, refuel/defuel,
  CG computation and prediction, weight-and-balance backup. `deep::fuel`
  models tanks, transfers and leaks, not a fuel management computer.
* **ATA 26** (33): every `… DET FAULT` and `… BOTTLES FAULT`.
  `deep::sensors`' smoke detectors publish an alarm and a reading, not a
  detector-fault discrete, and the cabin extinguisher bottles are not
  modelled.
* **ATA 24** (~30): the generator control units (`GEN n OFF`), the
  integrated drives and their oil (`DRIVE n …`, 16 ids), the external power
  receptacles, the primary and secondary supply centres, the static
  inverter, the TR monitors.
* **ATA 70** (~40): engine fuel leak, strainer, fuel/oil contamination,
  overthrust protection, the start-sequence supervisor, thrust-lever
  transducers, engine type disagree.
* **ATA 52** (6): the six upper-deck doors. `deep::sensors` publishes
  proximity sensors for eight doors under their own names; none is an
  upper-deck door.
* **ATA 99** (11): `MISC BOMB ON BOARD`, `DITCHING`, `FORCED LANDING`,
  `EMER EVAC`, `SEVERE TURBULENCE` — crew-declared situations, not sensed
  conditions, and marked WIP in FlyByWire's own catalogue.

### (c) Modelled, but the condition is not detectable from what we publish

Roughly **55** ids. These are the near misses, and the cheapest ones for a
next pass.

* ~~**ELEC C/B TRIPPED** (`240800021`)~~ — **WIRED (2nd pass).**
  `deep::breakers` now publishes `BREAKERS_TRIPPED_NOT_COMMANDED_COUNT`,
  counted from `trip::Breaker::status()` so a `Tripped(Thermal|Magnetic|
  ArcFault)` or `LockedOut` unit counts and an `OpenCommanded` one does
  not. The procedure reads `>= 1`. The original finding, kept because it
  is why the obvious variable could not be used: `deep::breakers` publishes 399 breakers
  and `BREAKERS_OPEN_COUNT`, but that count is `!u.breaker.closed`
  (`breakers/live.rs:246`), which includes a crew-commanded open — so wiring
  it would annunciate C/B TRIPPED whenever the crew pulled a breaker. The
  per-breaker `BKR_<id>_STATUS` does separate a thermal/magnetic/arc trip
  (2/3/4) from a commanded open, but both a commanded open and a
  cause-less trip encode as 1, and an `any(...)` over 399 breakers is 399
  `SimVar` reads per FWS per tick, twice over. **One aggregate
  "tripped, not commanded" count from `deep::breakers` wires this in one
  line.**
* ~~**Lavatory smoke** (`260800065/066`)~~ — **WIRED (2nd pass).** Closed by
  adding the missing *source*, not by writing a trigger:
  `deep::thermal_zones` now registers two ATA 26 failures, a main-deck and
  an upper-deck **lavatory waste-bin fire** (`26_thermal.cabin_{main,upper}
  _deck_lavatory_fire_load`, failures 26/9 and 26/10), injecting
  20 kW / 0.002 kg/s at full severity into `zones.cabin_{main,upper}_deck`
  exactly as the three cargo fires do into their bays. That is the one
  cabin fire the aircraft is certified to detect by itself (CS/FAR 25.854
  requires a lavatory smoke detector and a waste-receptacle extinguisher),
  which is why these two procedures exist. No lavatory *zone* was invented:
  the lavatory extract draws the deck's own air past the detector, which is
  what `sensors::live_discrete` already says. The triggers read the
  detectors (`any(DEEP_SMOKE_LAV_1..4_ALARM)` / `5..8`), not the zone, so
  they carry the sensors' own faults. The original finding follows.
  Eight lavatory detectors exist and
  the deck split is documented, but each samples
  `THERMAL_ZONE_CABIN{MAIN,UPPER}DECK_SMOKE_CONCENTRATION` and **no
  registered failure injects smoke into a cabin deck** —
  `thermal_zones::live` injects only into the cargo bays, the nacelle cowls
  and the APU compartment (`live.rs:363`, `:371`, `:377`). The trigger would
  read only published variables and would still never fire. Smoke does
  advect between zones along the ventilation links
  (`thermal_zones/network.rs:462-475`), so a cargo fire *might* reach a
  deck; that was not verified and a procedure is not wired on "might".
  **A cabin-deck smoke source (a galley or IFE fire) makes these two live.**
* **SMOKE L/R MAIN and L/R UPPER AVNCS SMOKE** (`260800030/038`–`041`):
  `deep::fire_ice` models one avionics zone, not five. Announcing the wrong
  bay is worse than announcing none.
* **ENG n OIL TEMP HI** (`701800093`–`701800096`) — **WIRED (2nd pass)**, at
  **177 °C**, sourced from FlyByWire's own A380X ENGINE system-display page
  (`fbw-a380x/src/systems/instruments/src/SD/Pages/Engine/elements/
  EngineColumn.tsx:74`, `engineOilTemperature > 177 ? 'Amber' : 'Green'`).
  That is the number this aeroplane's own gauge calls out of limits, so it
  is the number the caution beside it has to agree with. Gated on the core
  turning, because `deep::sensors` pegs an open-circuit oil-temperature
  channel at the top of its range and a broken wire on a parked aeroplane
  is not an engine caution.
  **ENG n OIL TEMP LO** (`701800097`–`701800100`) stays unwired: the A380X
  SD page has no low threshold at all, and the A32NX page's
  `OIL_TEMP_LOW_TAKEOFF = 38` is a CFM56/V2500 figure on another aeroplane.
  A low-oil-temperature caution also needs a "thrust about to be increased
  above idle" concept this port does not have, or it is simply on at every
  cold-soaked gate.
  The original finding: the oil
  temperature is genuinely modelled and published
  (`DEEP_ENG_n_OIL_TEMP_SENSED_C`, from `physics::engine::oil`), but no oil
  temperature limit for the Trent 900 family is cited anywhere in this
  repository or in FlyByWire's, and unlike the oil *pressure* limit there is
  no public certification figure to anchor one to. **One sourced number
  wires eight procedures.**
* **ENG n EGT OVER LIMIT / N1-N2 OVER LIMIT** (8 ids): no area publishes
  EGT, and the shaft speeds are published only as fractions of rated speed
  with no red line recorded to compare against.
* **ENG n OIL FILTER CLOGGED** (4): `physics::engine::oil` models the filter
  and its bypass valve; nothing publishes the differential or the bypass
  state out of it.
* **ENG n THRUST LOSS** (4): `A32NX_ENG_n_THRUST_ABNORMAL` is published but
  is the *same metering error* as `A32NX_ENG_n_FMU_FAULT` past a larger
  threshold (`engine_accessories/live.rs:1396-1397`, 0.33 against 0.10), so
  it can never be true without our own FADEC FUEL METERING FAULT already
  being up. It annunciates a metering fault, not an independent thrust loss.
* **FUEL FEED TK n TEMP HI** (4): `FUEL_TANK_TEMP_C:n` is published for all
  eleven tanks, but nothing records which indices are the four feed tanks
  and no high-temperature limit is sourced. **A tank-index table in
  `deep::fuel` plus one limit wires these four.**
* **FUEL WING/TRIM TK OVERFLOW, COLLECTOR CELL n NOT FULL, INR TKs QTY LO**
  (7): all compare a quantity against a per-tank capacity that is not
  published.
* **L/G ABNORM OLEO PRESS** (`320800031`, and its monitoring):
  `GEAR_STRUT_GAS_CHARGE_FRACTION:n` is published but is a charge *fraction*
  with no sourced servicing band.
* **ELEC AC BUS 1+2 & DC BUS 1 FAULT** (`240800003`): wirable, but it
  includes AC BUS 1, which our own single-bus alert also announces and which
  nothing here can suppress. It becomes wirable the moment
  `deep::electrical`'s own `ELEC AC BUS 1 FAULT` is either withdrawn in
  favour of FlyByWire's `240800002` or given a suppression hook.
* **ENG n STALL** (`701800113`–`701800116`):
  `A32NX_ENG_n_{HP,IP}_HANDLING_BLEED_STALL_MARGIN_PCT` and
  `A32NX_ENG_n_VSV_STALL_MARGIN_DELTA_PCT` are published and would make a
  genuinely computed surge trigger, but our own `ENG 1 STALL` (on
  `ENV_HAIL_FLAMEOUT_RISK:1`) carries the identical title for engine 1, so
  wiring all four would duplicate one of them and wiring three would be
  arbitrary.

### (d) Not examined id by id — still true, and unchanged by the 2nd pass

The second pass spent its whole budget on category (c) and **opened none of
the seven unexamined chapters**: ATA 21/22/23, 27, 31, 33 and 34 are exactly
as the first pass left them. The `AVNCS_*` / `314800003`/`314800004` lead
below is untouched and is still the most promising place to start.



**ATA 21/22/23, 27, 31, 33 and 34 were not examined id by id** — the
groupings above for those chapters come from reading their titles and the
areas' published-variable sets, not from the per-id pass ATA 24, 26, 28, 29,
32 and 70 got. The four richest sources of published variables that were
never mined are:

* `AVNCS_*` (≈200 names: module, switch, cable, virtual-link and function
  availability) against **ATA 31/42/46** — `314800003/004` FWS 1+2 (&
  FCDC 1+2) FAULT in particular looks reachable from the CPIOM-C partition
  availability our own `NETWORK CPIOM-C1 FAULT` already reads.
* `DEEP_PNEU_*` (102) and `THERMAL_ZONE_*` (78) against **ATA 21 and 36**.
* `ELEC_LOAD_*_POWERED` (376) against the many "… LOST" and "… REDUNDANCY
  LOST" procedures across every chapter.
* `FCTL_*` (≈80) against **ATA 27**, where the obstacle is the missing
  PRIM/SEC rather than the missing variables.

## Tooling

`src/deep/ecam/dump_published.rs` writes the two lists this work was done
against:

```
CARGO_TARGET_DIR=D:/A380/fbw-xp-systems/target-j1 \
  cargo test --lib deep::ecam::dump_published -- --nocapture
```

* `%TEMP%\deep_published_vars.txt` — every name `Deep::published_names()`
  reports (4228 today), sorted. Authoritative: a trigger naming anything
  outside this set reads 0 for ever.
* `%TEMP%\deep_own_alerts.txt` — every alert `deep::registry()` carries
  (304 today) as `ATA \t key \t level \t title \t trigger variables`. This
  is the list to check a FlyByWire procedure against before wiring it.

## Tests

`src/deep/ecam/fbw_tests.rs`, all passing, none ignored:

* `no_wired_trigger_reads_a_variable_nobody_publishes` — the standing guard.
* `no_wired_trigger_is_permanently_true_or_permanently_false` — the same
  three-valued `failure_audit::reachability` check our own 304 alerts face.
* `every_item_condition_reads_a_published_or_a_known_plugin_owned_variable`
  — with the cockpit-control allow-list spelled out and each entry's writer
  cited.
* `no_entry_takes_an_id_flybywire_already_triggers`,
  `every_entry_names_a_procedure_flybywire_actually_defines`,
  `every_item_vector_matches_the_procedures_own_item_count`,
  `no_entry_is_louder_than_flybywires_own_title_colour` (FlyByWire states the
  level in the title: `\x1b<2m` red, `<4m` amber, `<3m` green — every one of
  the 39 is amber, which is why none is wired as a red warning),
  `the_defined_and_wired_counts_are_what_this_module_was_built_against` —
  all four re-read FlyByWire's source and skip when the tree is absent.
* `every_entry_has_a_distinct_nine_digit_flybywire_id`,
  `every_suppressor_is_itself_a_defined_procedure`.
* Per-family reachability, arming the real failure and ticking the real
  areas: `eng_oil_press_lo_fires_when_the_oil_pressure_falls_with_the_core_turning`,
  `eng_oil_press_lo_stays_quiet_on_a_shut_down_engine`,
  `eng_ign_a_fault_fires_when_that_chains_exciter_dies_and_leaves_the_other_alone`,
  `eng_ign_fault_stays_quiet_when_the_igniters_are_not_energised`,
  `elec_gen_fault_fires_when_that_generator_fails`,
  `an_elec_bus_fault_needs_the_network_to_be_alive`,
  `wheel_tire_press_lo_fires_on_a_soft_tyre_and_not_on_a_serviced_one`,
  `fuel_trim_pump_fault_separates_one_failed_pump_from_both`,
  `lg_gear_not_locked_up_and_doors_not_closed_separate_a_jam_from_a_clean_retraction`.

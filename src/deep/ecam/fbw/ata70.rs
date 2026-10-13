//! ATA 70 -- engines. 153 of FlyByWire's 163 procedures here are unwired,
//! the largest single bucket of the 732.
//!
//! It is also the chapter where this port's own alerts are deepest:
//! `deep::engine_accessories`' `registry.rs` alone registers around a
//! hundred, covering the EEC, the fuel filter, the HP pump, the HP SOV, the
//! metering unit, the flow transmitter, the burner manifold, the igniters,
//! the starter and the start valve, and `deep::sensors` adds per-bearing
//! chip and vibration alerts for all four engines. Most of FlyByWire's
//! unwired ATA 70 procedures name a condition one of those already
//! announces, and wiring them too would put the same warning on the EWD
//! twice.
//!
//! What is left is the oil system -- which no area has an alert for -- and
//! the *per-chain* ignition faults, which our own `ENG n IGNITION FAULT`
//! cannot cover because it is raised from
//! `A32NX_ENG_n_NO_IGNITION_AVAILABLE`, and that is true only when
//! *neither* chain can spark (`engine_accessories/ignition.rs:109`,
//! `powered && a_hz <= 0.0 && b_hz <= 0.0`). A single dead exciter or a
//! single eroded igniter -- the commonest ignition fault there is -- is
//! invisible to it and is exactly what FlyByWire's `IGN A FAULT`/`IGN B
//! FAULT` procedures are for.
//!
//! # What is deliberately left unwired
//!
//! * Already raised by one of our own alerts, so wiring FlyByWire's id
//!   would double-annunciate: `701800013`..`701800028` FADEC FAULT / IDENT /
//!   SYS / TEMP HI and `701800001`..`701800004` CTL SYS FAULT (our `ENG n
//!   EEC FAULT` / `EEC CHANNEL FAULT`), `701800005`..`701800008` CTL VLV
//!   FAULT (`ENG n FADEC FUEL METERING FAULT`), `701800033`..`701800036`
//!   FUEL FILTER CLOGGED (`ENG n FUEL FILTER CLOG`), `701800053`..
//!   `701800056` HP FUEL VLV FAULT (`ENG n HP SOV FAULT`),
//!   `701800065`..`701800068` IGN A+B FAULT (`ENG n IGNITION FAULT`),
//!   `701800077`..`701800080` OIL CHIP DETECTED (`deep::sensors`' five
//!   per-bearing `ENG n ... BRG CHIP DET` alerts), `701800105`..`701800108`
//!   SENSOR FAULT (`ENG n EEC SENSOR DISAGREE`), `701800113`..`701800116`
//!   STALL (`ENG 1 STALL`), `701800121`..`701800128` START VLV FAULT
//!   (`ENG n START VALVE FAULT`), `701800137`..`701800150` REVERSER
//!   (`ENG n REVERSER FAULT` / `REVERSER UNLOCKED`), `701800152` HI
//!   VIBRATIONS (`ENG n N1/N2/N3 VIB HI`).
//! * `701800133`..`701800136` THRUST LOSS: `A32NX_ENG_n_THRUST_ABNORMAL` is
//!   published, but it is the *same metering error* as
//!   `A32NX_ENG_n_FMU_FAULT` past a larger threshold
//!   (`engine_accessories/live.rs:1396-1397`, 0.33 against 0.10), so it can
//!   never be true without our own `ENG n FADEC FUEL METERING FAULT`
//!   already being up. It annunciates a metering fault, not an independent
//!   loss of thrust.
//! * `701800097`..`701800100` OIL TEMP **LO**: `701800093`..`701800096` OIL
//!   TEMP HI are wired below on a limit that *is* sourced, but nothing in
//!   either repository states a *minimum* oil temperature for the Trent
//!   900. FlyByWire's own A380X ENGINE SD page draws oil temperature amber
//!   above 177 C and has no low threshold at all
//!   (`instruments/src/SD/Pages/Engine/elements/EngineColumn.tsx:74`). Its
//!   A32NX page does carry one -- `OIL_TEMP_LOW_TAKEOFF = 38`
//!   (`fbw-a32nx/.../SD/Pages/Eng/Eng.tsx:328`) -- but that is a
//!   CFM56/V2500 figure on a different aeroplane, and borrowing it for a
//!   Trent 900 would be exactly the fabricated value the no-guessing rule
//!   forbids. A low-oil-temperature caution also needs a "thrust is about
//!   to be increased above idle" concept this port does not have, or it
//!   would simply be on at every cold-soaked gate. Left unwired.
//! * `701800009`..`701800012` EGT OVER LIMIT, `701800073`..`701800076`
//!   N1/N2 OVER LIMIT: no area publishes EGT, and the shaft speeds are
//!   published only as pick-up fractions of rated speed with no
//!   certification red line recorded to compare them against.
//! * `701800081`..`701800084` OIL FILTER CLOGGED: `physics::engine::oil`
//!   models the filter and its bypass valve, but nothing publishes the
//!   filter differential or the bypass state out of it.
//! * `701800041`..`701800052` FUEL LEAK / STRAINER CLOGGED / SYS
//!   CONTAMINATION, `701800101`..`701800104` OVTHR PROT LOST,
//!   `701800117`..`701800120` START FAULT, `701800129`..`701800132` THR
//!   LEVER FAULT, `701800159`..`701800161` TWO ENG OUT / TYPE DISAGREE:
//!   not modelled -- there is no engine-level fuel leak detector, no
//!   overthrust protection, no start-sequence supervisor, no thrust lever
//!   transducer health and no published engine-out discrete.
//!
//! # Re-checked in a later pass, still unwired
//!
//! * `701800029`..`701800032` ENG n FAIL, `701800109`..`701800112` ENG n
//!   SHUTDOWN, `701800151` ALL ENGINES FAILURE and `701800158` ENGINE
//!   THRUST LOCKED are the only ATA 70 ids FlyByWire itself triggers
//!   (`FwsAbnormalSensed.ts` lines 4564-4739, checked with a grep that
//!   matches the file's real four-space indent -- a two-space anchor finds
//!   nothing and reads as "FlyByWire drives none of ATA 70," which is
//!   false). Confirmed disjoint from every id wired in this file.
//! * `701800081`..`701800084` OIL FILTER CLOGGED is not unwired any more --
//!   `ata_cheap_wins.rs` wires it from `A32NX_ENG_n_OIL_FILTER_BYPASSED`,
//!   which now exists. Left out of this file's `wire()` because that is
//!   where it already lives; noted here so a later pass does not
//!   re-diagnose it as open.
//! * `701800009`..`701800012` EGT OVER LIMIT: still unwired, and for a
//!   sharper reason than "nothing publishes EGT". This engine publishes
//!   *TGT* (`DEEP_ENG_n_TGT_SENSED_C`, the turbine-gas-temperature figure a
//!   Trent-family EEC actually limits on), not EGT. Those are different
//!   stations in the gas path; wiring FlyByWire's EGT procedure off a TGT
//!   reading would be answering the wrong question with a number that
//!   merely sounds similar, which is worse than the quantity being absent.
//! * `701800073`..`701800076` N1/N2 OVER LIMIT: re-checked against
//!   `physics::engine::governor` and the rest of `physics::engine` for an
//!   overspeed trip or red-line fraction. None exists -- shaft speeds are
//!   published only as pick-up fractions of *rated* speed, with no
//!   certified maximum recorded anywhere in either repository to compare
//!   them against. Still unwired.
//! * `701800114`..`701800116` ENG 2/3/4 STALL: re-examined past the
//!   "duplicates our own ENG 1 STALL" reasoning that covers only id
//!   `701800113`. `A32NX_ENG_n_{HP,IP}_HANDLING_BLEED_STALL_MARGIN_PCT`
//!   turns out to be the *same* value as
//!   `A32NX_ENG_n_VSV_STALL_MARGIN_DELTA_PCT`
//!   (`engine_accessories/live.rs:1478`, both read from one
//!   `stall_margin_delta_pct`), which `airflow_control::vsv` computes as
//!   `-STALL_MARGIN_COEFF_PCT_PER_DEG2 * schedule_error_deg^2`
//!   (`vsv.rs:96`) -- a continuous, always-non-positive penalty for a VSV
//!   schedule error, not a margin measured from a 100 %-at-nominal
//!   baseline down to a real 0 %-at-surge floor. There is no zero-crossing
//!   in it that means "this engine has actually surged"; any cutoff picked
//!   on it (e.g. "below -5 %") would be an invented threshold on a
//!   quantity that was never built to be compared against one. Left
//!   unwired for all four engines, not just the one our own alert already
//!   covers.
//! * Starter health (`A32NX_ENG_n_STARTER_DISENGAGE_FAULT`,
//!   `_DISINTEGRATED`, `_OVERHEAT`, `_HOUSING_RISE_K`) is published and
//!   genuinely modelled, but `ata70.ts` has no procedure that names the
//!   starter itself -- `701800117`-`701800120` START FAULT and
//!   `701800121`-`701800128` START VLV FAULT are the only start-sequence
//!   ids, and both are the "not modelled" start-sequence-supervisor /
//!   start-valve case above, not a starter-health case these variables
//!   would answer. No id to wire them to.
//!
//! # Phase 2 (2026-09-27): `E-ENG-DESIGN.md` Revision 3, then the FCOM pass
//!
//! Several of the "not modelled" ids above are modelled now, and the FCOM
//! (`E:/fbw-debug/ecam/refs/A380-FCOM.txt`, decoded in
//! `E:/fbw-debug/ecam/fcom_alerts.json`) supplied real thresholds and
//! inhibit lists the paragraphs above did not have. Notably:
//! * `701800009`-`701800012` EGT OVER LIMIT and `701800073`-`701800076`
//!   N1/N2 OVER LIMIT are wired below. The EGT case is resolved by reading
//!   EASA TCDS E.012 Note 16 (trimmed/untrimmed pairs), not by a new
//!   publish. The N1/N2 case needed no new sensor at all -- `eec.rs`'s own
//!   5-parameter dual-channel model (`PARAMS`) already tracks N1 alongside
//!   N2/N3/TGT/P30 and already publishes `A32NX_ENG_n_EEC_{N1,N2}_SELECTED`
//!   (percent); the FCOM's own red-line thresholds (96.1 %/97.8 %,
//!   PRO-ABN-ECAM p.5794) are used directly, in preference to the earlier
//!   plan of extending `deep::sensors`' separate N2/N3 "pickup" abstraction
//!   with a new N1 channel it does not need.
//! * `701800041`-`701800052` (FUEL LEAK / STRAINER CLOGGED / SYS
//!   CONTAMINATION), `701800089`-`701800092` OIL SYS CONTAMINATION,
//!   `701800097`-`701800100` OIL TEMP LO, `701800101`-`701800104` OVTHR
//!   PROT LOST, `701800129`-`701800132` THR LEVER FAULT, `701800137`-
//!   `701800140`/`701800145`-`701800148`/`701800154` (reverser control/
//!   energized/locked/minor-fault/selected), `701800157` THR LEVERS NOT SET,
//!   `701800159`-`701800160` TWO ENG OUT and `701800153` RELIGHT IN FLIGHT
//!   are all wired below -- see `E-ENG-DESIGN.md` Patterns 16-36 and
//!   `E-ENG-FCOM.json` for the source of each threshold and inhibit list.
//! * `701800143`-`701800144` REVERSER INHIBITED moved from MODEL back to
//!   UNSOURCED this pass: FCOM PRO-ABN-ECAM p.5827 ("ENG 2(3) REVERSER
//!   INHIBITED") reads "One thrust reverser has been electrically and
//!   mechanically inhibited by maintenance action" -- a maintenance-
//!   selected dispatch lockout, not the in-flight lever-position inhibit
//!   Revision 3 designed. No maintenance-inhibit selector exists anywhere
//!   in this model, so this stays unwired; see `E-ENG-DESIGN.md`'s own
//!   UNSOURCED section.
//! * `701800151` ALL ENGINES FAILURE and `701800109`-`701800112` ENG n
//!   SHUTDOWN both have real FCOM procedures (PRO-ABN-ECAM p.5833 and
//!   p.5805) with genuine, sourceable triggers ("all engines are failed";
//!   engine shutdown) -- but both remain on the "FlyByWire triggers this
//!   itself" list above (`FwsAbnormalSensed.ts`), so wiring them here would
//!   violate `no_entry_takes_an_id_flybywire_already_triggers`. Confirmed,
//!   not re-opened.
//! * `701800032` ENG n START FAULT (Pattern 32) and `701800156`/`700900002`
//!   TAIL PIPE FIRE (Pattern 40) were checked directly against the FCOM's
//!   own page (p.5810, p.5858) and stay unwired: the FCOM lists causes for
//!   START FAULT but no duration figure to time a hung start against, and
//!   TAIL PIPE FIRE is a crew-summoned reference display in the FCOM with
//!   no sourceable tailpipe-temperature data anywhere in either repository.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, not, var, Cond, Level};

/// At least one main AC bus is live -- the same cold-and-dark gate every
/// other chapter's own `network_alive` uses (`ata24.rs:54`,
/// `ata31_33.rs:81`), needed here for the causes that are not already
/// gated by `core_running` (a fuel-line leak/contamination discrete, for
/// instance, is not itself derived from a spinning spool).
fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

/// This engine's thrust lever is at or above take-off power. FlyByWire's
/// own MCT/TOGA thrust-lever-angle detents (`FwsFlightPhases.ts:215-247`):
/// MCT is 33.3-36.7 degrees, full/TOGA power is above 43.3 degrees -- either
/// band counts (`E-ENG-DESIGN.md` Pattern 18, reused by Patterns 34 and 35
/// below exactly as the design sheet says to). Reads
/// `A32NX_ENG_n_TLA_DEG`, this area's own mirror of FlyByWire's single
/// (channel A) `AUTOTHRUST_TLA:n` (`engine_accessories/live.rs`'s own
/// `tla_deg` field) -- a trigger `Cond` may only read a deep area's own
/// published var, never FlyByWire's raw Var directly, however well
/// precedented reading it elsewhere in this file would be
/// (`no_wired_trigger_reads_a_variable_nobody_publishes`).
/// This engine's thrust lever is at or above MCT (`FwsFlightPhases.ts:220`,
/// `eng1SupMCT = !(eng1TLA < 36.7)`).
fn at_or_above_mct(eng: u32) -> Cond {
    var(&format!("A32NX_ENG_{eng}_TLA_DEG")).ge(36.7)
}

/// FlyByWire's own exact take-off-power detent (`FwsFlightPhases.ts:214-
/// 244`, `eng1TOPowerSignal = (eng1TLAFTO && eng1MCT) || eng1TLAFullPwr ||
/// eng1SupMCT`): at or above MCT on its own, OR in the MCT band with a
/// flex/derated take-off temperature entered (`eng1TLAFTO`, this area's own
/// `A32NX_TO_FLEX_TEMP_SET` mirror of `AIRLINER_TO_FLEX_TEMP`) -- with a
/// flex take-off, MCT-band thrust *is* the commanded take-off thrust, not
/// merely close to it. `eng1TLAFullPwr` (`TLA > 43.3`) is already a subset
/// of `eng1SupMCT` (`TLA >= 36.7`), so it does not need its own term here.
fn eng_to_power_signal(eng: u32) -> Cond {
    let tla = format!("A32NX_ENG_{eng}_TLA_DEG");
    let mct_band = all(vec![var(&tla).gt(33.3), var(&tla).lt(36.7)]);
    any(vec![at_or_above_mct(eng), all(vec![var("A32NX_TO_FLEX_TEMP_SET").on(), mct_band])])
}

/// Minimum oil temperature to accelerate to take-off power, deg C. EASA
/// TCDS E.012 SS IV.1.4 gives a *certification* figure of 40 C, but the
/// FCOM's own procedure for this exact alert (PRO-ABN-ECAM p.5802, "ENG
/// 1(2)(3)(4) OIL TEMP LO") states the real ECAM threshold directly: "the
/// engine oil temperature is less than 50 C" -- a more specific, more
/// directly applicable source for this specific id than the TCDS's general
/// certification minimum, so the FCOM's own number is used, per the rule
/// that the FCOM wins when it gives one. (The FCOM's own trigger is also
/// ground-only and time/T.O-CONFIG-gated, not a phase-3/4-TLA composition;
/// that refinement needs an "on ground" and a "time since start" signal
/// this chapter does not yet have wired into a `Cond`, so the phase-3/4
/// composition below is kept as a documented approximation of *when*, with
/// the FCOM's own *threshold* corrected.)
const OIL_TEMP_MIN_TO_C: f64 = 50.0;

/// `ENG 1(2)(3)(4) N1/N2 OVER LIMIT` red-line thresholds, percent of rated
/// speed. FCOM PRO-ABN-ECAM p.5794: "One of the following engine parameter
/// is above red limit: N1 above 96.1 %, N2 above 97.8 %." This supersedes
/// the EASA TCDS's own certification-limit figures (97.2 %/98.7 % 5-minute
/// take-off, 99.5 % 20-second IP overspeed) for *this* alert specifically --
/// the TCDS figures are the certified maxima with their allowed transient
/// durations, but this FCOM procedure's own annunciation threshold is the
/// lower, continuously-monitored red line the real ECAM computer actually
/// compares against, which is the more directly applicable source for the
/// alert this id names.
const N1_RED_LIMIT_PCT: f64 = 96.1;
const N2_RED_LIMIT_PCT: f64 = 97.8;

/// EASA TCDS E.012, Note 16: trimmed/untrimmed TGT pairs are TO 900/983,
/// MCT 850/963, over-temperature 920/991. This model's own
/// `DEEP_ENG_n_TGT_SENSED_C` reads the untrimmed value (`deep/plugin.rs`'s
/// own doc), so it has to be compared against the untrimmed figures, not
/// the cockpit-displayed trimmed ones (`E-ENG-DESIGN.md` Pattern 33). The
/// take-off tier's own untrimmed figure (983) is not used by any pattern
/// below -- Pattern 33 now uses the over-temperature and MCT pairs, keyed
/// by regime, and Pattern 36 reuses the MCT pair as its own analogue -- but
/// is left here as a documented, sourced constant for whoever needs it.
#[allow(dead_code)]
const TGT_TO_LIMIT_UNTRIMMED_C: f64 = 983.0;
const TGT_MCT_LIMIT_UNTRIMMED_C: f64 = 963.0;
const TGT_OVERTEMP_LIMIT_UNTRIMMED_C: f64 = 991.0;

/// Minimum oil pressure, Pa. EASA TCDS E.012 (Trent 900 family, public)
/// gives 25 psi from idle to 70 % HP and 50 psi above 95 % HP;
/// `physics::engine::oil`'s own certification test cites the same figures
/// (`oil.rs:436-439`). 25 psi is the floor that holds at every power
/// setting, so it is the one a single threshold can use without
/// annunciating a healthy idle.
const OIL_PRESS_MIN_PA: f64 = 25.0 * crate::physics::engine::oil::PSI_PA;

/// Maximum oil temperature, deg C. **Sourced, and specific to this
/// aeroplane**: FlyByWire's own A380X ENGINE system-display page draws the
/// oil temperature amber above 177 C and green below it
/// (`fbw-a380x/src/systems/instruments/src/SD/Pages/Engine/elements/
/// EngineColumn.tsx:74`, `engineOilTemperature > 177 ? 'Amber' : 'Green'`).
/// That is the number the aircraft's own display calls out of limits, so it
/// is the number an OIL TEMP HI caution has to agree with -- a threshold
/// that disagreed with the gauge beside it would be worse than none. It is
/// also the Trent 900 maximum continuous figure, which is why the SD page
/// uses it; the deliberately *not* used alternative is FlyByWire's A32NX
/// page, whose 140/155 C pair is a CFM56/V2500 limit on another aeroplane.
const OIL_TEMP_MAX_C: f64 = 177.0;

/// The HP spool turning fast enough that the oil pump is delivering, as a
/// fraction of rated N3. **GENERIC**: a Trent idles around 55-60 % N3, and
/// half of rated speed is below any running condition and far above a
/// windmilling or motoring one, so this separates "the engine is running
/// and its oil pressure should be up" from "the engine is shut down and has
/// no oil pressure, which is not a fault". Both pick-up channels are
/// offered, so one failed speed pick-up does not silence the oil warning.
const CORE_RUNNING_FRACTION: f64 = 0.5;

fn core_running(eng: u32) -> Cond {
    any(vec![
        var(&format!("DEEP_ENG_{eng}_N3_PICKUP_A_FRAC")).gt(CORE_RUNNING_FRACTION),
        var(&format!("DEEP_ENG_{eng}_N3_PICKUP_B_FRAC")).gt(CORE_RUNNING_FRACTION),
    ])
}

pub fn wire(v: &mut Vec<FbwProc>) {
    for eng in 1..=4u32 {
        let i = u64::from(eng) - 1;

        // ---- IGN A / IGN B FAULT.
        //
        // The exciters are energised and one chain is producing no spark at
        // all. `spark_rate_hz` returns exactly 0 when that chain's exciter
        // has failed or its igniter gap has eroded past the exciter's reach
        // (`engine_accessories/ignition.rs:61-69`), and returns a positive,
        // merely slower rate when it is only degraded -- so this is a dead
        // chain, not a tired one, and it can only read true while ignition
        // is actually selected.
        for (chain, base) in [("A", 701_800_057u64), ("B", 701_800_061u64)] {
            v.push(
                proc(
                    base + i,
                    // FlyByWire's own title, for the reader; the flight deck
                    // shows its own copy out of its own table.
                    if chain == "A" { "ENG n IGN A FAULT" } else { "ENG n IGN B FAULT" },
                    // Amber in FlyByWire's own title (`\x1b<4m`), and a
                    // single-chain loss with a healthy twin, so advisory
                    // rather than caution -- the same level FlyByWire gives
                    // its own single-channel losses (`211800001` PACK 1 CTL
                    // 1 FAULT, `failure: 1`).
                    Level::Advisory,
                    sd_page::ENG,
                    all(vec![
                        var(&format!("A32NX_ENG_{eng}_IGN_POWERED")).on(),
                        var(&format!("A32NX_ENG_{eng}_IGN_{chain}_SPARK_RATE_HZ")).le(0.0),
                    ]),
                    "the igniters are energised and this chain is producing no spark, while the other chain may still be firing",
                )
                // 2 s: long enough that the first frames of an ignition
                // selection, before the exciter charges, are not a fault.
                .confirm(2.0)
                .inhibit(phase::ENG_56),
            );
        }

        // ---- OIL PRESS LO.
        v.push(
            proc(
                701_800_085 + i,
                "ENG n OIL PRESS LO",
                // Amber in FlyByWire's own title (`\x1b<4m`); a caution,
                // not an advisory -- the crew acts on it.
                Level::Caution,
                sd_page::ENG,
                all(vec![core_running(eng), var(&format!("DEEP_ENG_{eng}_OIL_PRESSURE_SENSED_PA")).lt(OIL_PRESS_MIN_PA)]),
                "the engine's own oil pressure transducer reads below the 25 psi EASA TCDS E.012 minimum while the HP spool is turning at running speed",
            )
            // 2 s, so a step in oil pressure through a transient is not a
            // caution; the same order as the confirmation
            // `deep::api`'s own worked oil-pressure example uses.
            .confirm(2.0)
            .inhibit(phase::ENG_56),
        );

        // ---- OIL TEMP HI.
        //
        // The core-running gate is here for a different reason than on OIL
        // PRESS LO. A shut-down engine's oil is cold, so this threshold
        // could not be crossed by a cold aeroplane -- but it *can* be
        // crossed by a failed transducer, and `deep::sensors` pegs an
        // open-circuit oil-temperature channel at the top of its range by
        // design. Requiring the spool to be turning keeps a broken wire on
        // a parked aeroplane from annunciating an engine caution, without
        // suppressing the real case: oil only gets hot with the engine
        // running.
        v.push(
            proc(
                701_800_093 + i,
                "ENG n OIL TEMP HI",
                // Amber in FlyByWire's own title (`\x1b<4m`).
                Level::Caution,
                sd_page::ENG,
                all(vec![core_running(eng), var(&format!("DEEP_ENG_{eng}_OIL_TEMP_SENSED_C")).gt(OIL_TEMP_MAX_C)]),
                "the engine's own oil temperature transducer reads above the 177 C that FlyByWire's own A380X ENGINE SD page draws amber, with the HP spool turning",
            )
            // 15 s. Oil temperature is a slow quantity with a large
            // thermal mass behind it (`physics::engine::oil`'s 20 kg
            // tank), so a crossing that does not persist for seconds is
            // the transducer talking, not the oil.
            .confirm(15.0)
            .inhibit(phase::ENG_56),
        );

        // ---- FUEL FILTER MONITORING FAULT (Pattern 16). The
        // differential-pressure monitor's own electronics/switch reads
        // faulted, independent of the element's real clog state
        // (`engine_accessories/fuel/filter.rs`'s new `monitor_fault`).
        // FCOM PRO-ABN-ECAM p.5784: "The fuel filter is no longer
        // monitored" -- a crew-awareness item with no aural or master
        // light shown (confirmed by rendering the page), the same quiet
        // tier FlyByWire's own single-channel losses use.
        v.push(
            proc(
                701_800_037 + i,
                "ENG n FUEL FILTER MONITORING FAULT",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_FUEL_FILTER_MONITOR_FAULT")).on(),
                "the fuel filter's own bypass-warning monitor reads faulted, independent of the element's real clog state",
            )
            .confirm(5.0)
            .inhibit(&[phase::ELEC_PWR, phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        // ---- FUEL LEAK (Pattern 20, cross-team E-FUEL, confirmed against
        // `E-FUEL-DESIGN.md` D11). `deep::fuel`'s own per-engine feed-line
        // leak discrete, resolved to this specific engine rather than a
        // wing side; its own settling delay already lives inside the
        // published boolean (`live.rs:140-142`'s `LEAK_WINDOW_S`/
        // `LEAK_CONFIRM_WINDOWS`), so no second confirm is stacked here.
        // FCOM PRO-ABN-ECAM p.5785 gives the real inhibit: this engine's
        // fuel-flow-vs-average-of-three comparison is only meaningful once
        // thrust has stabilised in the climb, so the FCOM inhibits every
        // phase except 1500 ft (phase 8) -- a much narrower window than a
        // guess would have produced, and the FCOM wins over this design's
        // own draft `phase::ENG_56`.
        v.push(
            proc(
                701_800_041 + i,
                "ENG n FUEL LEAK",
                Level::Caution,
                sd_page::ENG,
                all(vec![var(&format!("FUEL_ENG_LEAK_DETECTED:{eng}")).on(), network_alive()]),
                "deep::fuel's per-engine feed-line leak discrete, resolved to this specific engine rather than a wing side",
            )
            .confirm(0.0)
            .inhibit(&[phase::ELEC_PWR, phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN, phase::AT_OR_BELOW_80_KT, phase::ENGINES_SHUTDOWN]),
        );

        // ---- FUEL STRAINER CLOGGED (Pattern 21). `fuel::strainer`'s own
        // bypass bit; the bypass-crack differential is now FCOM-sourced (12
        // psi, p.5787), not an approximation. FCOM's own inhibit list.
        v.push(
            proc(
                701_800_045 + i,
                "ENG n FUEL STRAINER CLOGGED",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_FUEL_STRAINER_CLOGGED")).on(),
                "the fuel strainer's own bypass valve has cracked open at the FCOM's 12 psi differential (PRO-ABN-ECAM p.5787), upstream of the fine filter",
            )
            .confirm(5.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        // ---- FUEL SYS CONTAMINATION (Pattern 22, cross-team E-FUEL,
        // confirmed against `E-FUEL-DESIGN.md` D16). No new failure id --
        // reuses the already-registered per-engine filter free-water
        // fraction discrete.
        v.push(
            proc(
                701_800_049 + i,
                "ENG n FUEL SYS CONTAMINATION",
                Level::Caution,
                sd_page::ENG,
                all(vec![var(&format!("FUEL_ENG_CONTAMINATION_DETECTED:{eng}")).on(), network_alive()]),
                "deep::fuel's per-engine filter free-water fraction above zero, resolved to this specific engine's own feed line",
            )
            .confirm(0.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::TOUCH_DOWN, phase::AT_OR_BELOW_80_KT]),
        );

        // ---- OIL SYS CONTAMINATION (Pattern 19). Derived aggregate: two
        // or more of the five already-published per-bearing chip discretes
        // active at once, distinguishing widespread metal contamination
        // from one bearing's own chip light (which fires its own,
        // already-wired alert instead).
        v.push(
            proc(
                701_800_089 + i,
                "ENG n OIL SYS CONTAMINATION",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_OIL_SYSTEM_CONTAMINATION")).on(),
                "two or more of this engine's five per-bearing chip detectors are active at once, distinct from any one bearing's own chip-light alert",
            )
            .confirm(5.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::TOUCH_DOWN]),
        );

        // ---- MINOR FAULT (Pattern 23). FCOM PRO-ABN-ECAM p.5793: "Some
        // internal engine sensors or one FADEC channel are failed" -- a
        // deferred, crew-awareness item (no aural or master light shown,
        // confirmed by rendering the page), distinct from the controlling
        // `EEC_CHANNEL_FAULT` this port already wires under its own id.
        // This model's own EEC backup oil-temperature probe (a
        // non-controlling trend/logging channel) stands in for "some
        // internal sensor"; the FCOM's own text does not name which one.
        v.push(
            proc(
                701_800_069 + i,
                "ENG n MINOR FAULT",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_EEC_MAINTENANCE_FAULT")).on(),
                "the EEC's own backup (non-controlling) oil-temperature probe disagrees with the primary reading -- a real, deferred internal-sensor fault, standing in for the FCOM's own unspecified 'some internal engine sensor'",
            )
            .confirm(10.0)
            .inhibit(&[phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        // ---- OVTHR PROT LOST (Pattern 30). Both independent N3 (HP)
        // pickup channels invalid while oil pressure proves the engine is
        // actually turning. FCOM PRO-ABN-ECAM p.5803 confirms this is a
        // deferred BITE-style display ("displayed after flight if only one
        // engine is affected... at aircraft power-up if more than two
        // engines are affected"), not a real-time in-flight caution -- its
        // own phase bar is inhibited for the entire flight and shown only
        // on the ground, exactly matching this design's own real-time
        // cause wired to the FCOM's own ground-only display window; no
        // aural or master light either (confirmed by rendering the page),
        // so Advisory, not the Caution this design first assumed.
        v.push(
            proc(
                701_800_101 + i,
                "ENG n OVTHR PROT LOST",
                Level::Advisory,
                sd_page::ENG,
                all(vec![
                    var(&format!("DEEP_ENG_{eng}_N3_PICKUP_A_VALID")).eq(0.0),
                    var(&format!("DEEP_ENG_{eng}_N3_PICKUP_B_VALID")).eq(0.0),
                    var(&format!("DEEP_ENG_{eng}_OIL_PRESSURE_SENSED_PA")).gt(OIL_PRESS_MIN_PA),
                ]),
                "both independent N3 (HP) speed-pickup channels read invalid while oil pressure proves the engine is actually turning -- the overthrust-protection function's own monitoring input lost",
            )
            .confirm(5.0)
            .inhibit(&[phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        // ---- THR LEVER FAULT (Pattern 31). Dual-channel thrust-lever
        // position transducer disagree; the comparison itself is computed
        // in `live.rs` (a `Cond` cannot difference two Vars) and published
        // as `A32NX_ENG_n_THR_LEVER_DISAGREE`.
        v.push(
            proc(
                701_800_129 + i,
                "ENG n THR LEVER FAULT",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_THR_LEVER_DISAGREE")).on(),
                "the thrust lever's second (channel B) position transducer disagrees with FlyByWire's own single TLA channel by more than the tolerance",
            )
            .confirm(2.0)
            .inhibit(phase::ENG_56),
        );

        // ---- EGT OVER LIMIT (Pattern 33, regime-split). FCOM
        // PRO-ABN-ECAM p.5769's own triggering text has a real, two-part
        // structure keyed by regime -- "above the red line (1,002 C) during
        // T/O and go-around, when the reversers are selected, or when
        // alpha-floor is active" and, separately, "above the amber line
        // (970 C) when the thrust lever is at or below MCT". Those two
        // conditions are mutually exclusive by regime, which this wiring
        // now mirrors exactly using `at_or_above_mct` (the same detent
        // `eng_to_power_signal`/Pattern 18 already reads) as the split, in
        // place of Revision 4's single flat threshold. The FCOM's own
        // 1,002/970 figures are not used as the *numbers*: "EGT" is the
        // trimmed reading by definition (TCDS Note 6), this model's own
        // `DEEP_ENG_n_TGT_SENSED_C` is untrimmed, and the trim offset is
        // regime-dependent (TCDS Note 16: TO 900/983, MCT 850/963,
        // over-temperature 920/991) -- converting 1,002/970 without a
        // stated pair for either would be a guess. Each regime instead
        // uses the TCDS's own untrimmed figure for the matching condition:
        // above-MCT uses the 991 C over-temperature limit (the FCOM's own
        // "T/O and go-around" red-line regime), at-or-below-MCT uses the
        // 963 C MCT continuous limit (the FCOM's own "at or below MCT"
        // amber-line regime, matched exactly by name this time, not
        // borrowed as Pattern 36's own analogue was). `FbwProc` still
        // carries one `confirm_s` for the whole trigger, not a per-branch
        // timer, so both tiers share the TCDS's own 20 s over-temperature
        // allowance -- tight enough to still protect the "continuous"
        // 963 C tier promptly, and exactly the TCDS's own number for the
        // 991 C tier, so neither tier is stretched past what it was
        // sourced for.
        v.push(
            proc(
                701_800_009 + i,
                "ENG n EGT OVER LIMIT",
                Level::Caution,
                sd_page::ENG,
                any(vec![
                    all(vec![core_running(eng), at_or_above_mct(eng), var(&format!("DEEP_ENG_{eng}_TGT_SENSED_C")).gt(TGT_OVERTEMP_LIMIT_UNTRIMMED_C)]),
                    all(vec![core_running(eng), not(at_or_above_mct(eng)), var(&format!("DEEP_ENG_{eng}_TGT_SENSED_C")).gt(TGT_MCT_LIMIT_UNTRIMMED_C)]),
                ]),
                "at or above MCT, the engine's own untrimmed TGT is past the EASA TCDS E.012 20 s over-temperature limit (991 C, FCOM's own T/O-GA/reverser/alpha-floor red-line regime); at or below MCT, it is past the TCDS's own MCT continuous limit (963 C, the FCOM's own at-or-below-MCT amber-line regime)",
            )
            .confirm(20.0)
            .inhibit(&[phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF]),
        );

        // ---- N1/N2 OVER LIMIT (Pattern 34). FCOM PRO-ABN-ECAM p.5794's
        // own red-line thresholds, sourced directly against this model's
        // already-published, already dual-channel-validated EEC selected
        // readings (`A32NX_ENG_n_EEC_{N1,N2}_SELECTED`, percent) -- no new
        // sensor is needed: the EEC's own 5-parameter dual-channel model
        // (`eec.rs`'s `PARAMS`) already tracks N1 alongside N2/N3/TGT/P30,
        // which this design's Revision 3 missed in favour of the separate
        // `sensors::live_discrete` N2/N3 "pickup" abstraction (a different,
        // additional dual-channel layer feeding the independent overthrust
        // -protection circuit, not the EEC's own primary sensing this alert
        // needs). FlyByWire's own title colour is Amber (Caution); the FCOM
        // shows CRC/MASTER WARN (Warning) for this same procedure, so the
        // quieter of the two is kept, per the rule.
        v.push(
            proc(
                701_800_073 + i,
                "ENG n N1/N2 OVER LIMIT",
                Level::Caution,
                sd_page::ENG,
                any(vec![
                    var(&format!("A32NX_ENG_{eng}_EEC_N1_SELECTED")).gt(N1_RED_LIMIT_PCT),
                    var(&format!("A32NX_ENG_{eng}_EEC_N2_SELECTED")).gt(N2_RED_LIMIT_PCT),
                ]),
                "the EEC's own selected N1 or N2 reading is above the FCOM's red-line limit (96.1%/97.8%, PRO-ABN-ECAM p.5794)",
            )
            .confirm(0.0)
            .inhibit(phase::ENG_56),
        );

        // ---- OIL TEMP LO (Pattern 35, composed). Pattern 3's own oil
        // temperature sensor with Pattern 18's own take-off-power signal;
        // no new component. FCOM's own inhibit list.
        v.push(
            proc(
                701_800_097 + i,
                "ENG n OIL TEMP LO",
                Level::Caution,
                sd_page::ENG,
                all(vec![eng_to_power_signal(eng), var(&format!("DEEP_ENG_{eng}_OIL_TEMP_SENSED_C")).lt(OIL_TEMP_MIN_TO_C)]),
                "this engine's thrust lever is at or above take-off power while its own oil temperature reads below the FCOM's own 50 C threshold (PRO-ABN-ECAM p.5802)",
            )
            .confirm(0.0)
            .inhibit(&[phase::ELEC_PWR, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN, phase::ENGINES_SHUTDOWN]),
        );

        // ---- START FAULT (Pattern 32, one real OR-branch only). FCOM
        // PRO-ABN-ECAM p.5810/5811 lists ten OR'd causes for this single id
        // ("low starter air pressure", "no starter air pressure", "low N1",
        // "low N2", "hung start", "EGT overlimit", "no light up", "engine
        // stall", "starter time exceeded", "thr levers not at idle") but
        // gives no numeric duration for any of the time-based ones (hung
        // start / starter time exceeded / no light up) -- rendering both
        // pages confirms this stays exactly the gap Revision 3 already
        // found, not newly resolved. One cause needs no duration at all and
        // this model can drive it honestly: "THR LEVERS NOT AT IDLE" during
        // an active start. `A32NX_ENG_n_START_VALVE_POSITION > 0.5` stands
        // in for "a start is in progress" (the valve is open because a
        // start was commanded -- `truth.controls.starter_engaged`,
        // `live.rs:1291`), `!core_running` keeps this to the start attempt
        // itself, and `A32NX_ENG_n_TLA_DEG` away from idle is read directly
        // -- "idle" is 0 degrees by definition, not an invented limit, and
        // a small dead-band (2 degrees) absorbs lever-position noise the
        // same way other tolerances in this area do. The other nine causes
        // stay unmodelled for this id: no starter-air-pressure transducer
        // is published anywhere in this area, and the three time-based
        // causes still have no sourced duration.
        v.push(
            proc(
                701_800_117 + i,
                "ENG n START FAULT",
                Level::Advisory,
                sd_page::ENG,
                all(vec![
                    var(&format!("A32NX_ENG_{eng}_START_VALVE_POSITION")).gt(0.5),
                    not(core_running(eng)),
                    var(&format!("A32NX_ENG_{eng}_TLA_DEG")).gt(2.0),
                ]),
                "a start is commanded (the start valve is open) and the core is not yet running while this engine's own thrust lever reads away from idle -- the FCOM's 'THR LEVERS NOT AT IDLE' start-fault cause; the other nine OR'd FCOM causes for this id have no sourced number to trigger on (no starter-air-pressure publish; no hung-start/no-light-up/starter-time duration in either the FCOM or the TCDS)",
            )
            .confirm(2.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::TOUCH_DOWN]),
        );
    }

    // ---- RELIGHT IN FLIGHT, sensed (Pattern 36). A single FBW id (not one
    // per engine, unlike the patterns above): `any` across all four
    // engines' own relight-attempt condition -- ignition energised while
    // that engine's core is not running and the aircraft is genuinely
    // airborne (all five gear legs unloaded -- `GEAR_LEG_COMPRESSION`, the
    // same weight-on-wheels signal `deep::sensors`' own proximity switches
    // read), the real distinction between a relight attempt and a normal
    // ground start. `phase::NONE`: the FCOM shows no phase bar at all for
    // this procedure (it is a crew-summoned reference display,
    // PRO-ABN-ECAM p.5852), so no phase-based inhibit concept applies --
    // the airborne check in the trigger itself is what keeps a normal
    // ground start silent instead.
    let airborne = all(vec![
        var("GEAR_LEG_COMPRESSION:1").lt(0.05),
        var("GEAR_LEG_COMPRESSION:2").lt(0.05),
        var("GEAR_LEG_COMPRESSION:3").lt(0.05),
        var("GEAR_LEG_COMPRESSION:4").lt(0.05),
        var("GEAR_LEG_COMPRESSION:5").lt(0.05),
    ]);
    v.push(
        proc(
            701_800_153,
            "ENG RELIGHT IN FLIGHT",
            Level::Caution,
            sd_page::ENG,
            any((1..=4u32)
                .map(|eng| {
                    all(vec![
                        var(&format!("A32NX_ENG_{eng}_IGN_POWERED")).on(),
                        not(core_running(eng)),
                        airborne.clone(),
                        var(&format!("DEEP_ENG_{eng}_TGT_SENSED_C")).gt(TGT_MCT_LIMIT_UNTRIMMED_C),
                    ])
                })
                .collect()),
            "ignition is energised on at least one engine, its core is not running and the aircraft is genuinely airborne (all five gear legs unloaded) -- a relight attempt, with the TGT above MCT's own untrimmed 963 C trim offset reused as the best-available analogue for the FCOM's 850 C trimmed relight limit",
        )
        .confirm(2.0)
        .inhibit(phase::NONE),
    );

    // ---- TAIL PIPE FIRE, sensed (Pattern 40, re-sourced). Rendering the
    // FCOM's own procedure past its title page (PRO-ABN-ECAM p.5853, "ENG
    // TAIL PIPE FIRE (Cont'd)") turns up a real, sensed recognition
    // criterion that the earlier pass, reading only the title page, missed:
    // "Internal engine fire may be encountered during engine start or
    // engine shutdown. It may be seen by the ground crew, or **the EGT may
    // fail to decrease after the MASTER LEVER is turned off**." This port's
    // own `DEEP_ENG_n_TGT_SENSED_C` is a pass-through of `physics::engine`'s
    // own combustion simulation, not something `deep::engine_accessories`
    // can independently keep elevated after shutdown -- perturbing that
    // real gas-temperature trajectory would mean changing shared physics
    // code well outside this chapter's own files. This area already owns a
    // physically equivalent cause, though: a stuck HP shut-off valve
    // (`fuel::shutoff_valve::ShutoffValveFaults.stuck`, already registered,
    // `ENG_{eng}_HP_SOV_FAULT`'s own cause) that freezes in whatever
    // position it was in when the fault armed -- "a stuck-open valve cannot
    // be used to shut an engine down" (that component's own doc). A valve
    // stuck open when the crew has commanded shutdown is unmetered fuel
    // continuing to reach the hot section after the master lever is turned
    // off -- the real physical mechanism a post-shutdown tailpipe fire
    // actually needs, not an invented substitute, and it needs no new
    // failure or publish: `A32NX_ENG_n_HP_SOV_POSITION` (already published)
    // reading substantially open while `core_running` (already used
    // throughout this file) reads false is exactly that state. Reusing the
    // same underlying cause `ENG_{eng}_HP_SOV_FAULT` (this area's own
    // internal alert) already raises is not a duplicate annunciation --
    // that alert is a generic valve-disagree fault; this one is the
    // specific fire-risk escalation of the same root cause, the same
    // sourced-reuse shape Pattern 19 (`OIL SYS CONTAMINATION`, reusing the
    // five per-bearing chip detectors) already uses in this design.
    v.push(
        proc(
            701_800_156,
            "ENG TAIL PIPE FIRE",
            Level::Caution,
            sd_page::ENG,
            any((1..=4u32)
                .map(|eng| all(vec![not(core_running(eng)), var(&format!("A32NX_ENG_{eng}_HP_SOV_POSITION")).gt(0.05)]))
                .collect()),
            "the core has stopped turning (shutdown) while this engine's own HP fuel shut-off valve still reads substantially open -- unmetered fuel continuing to reach the hot section after the master lever was turned off, the FCOM's own recognition criterion (PRO-ABN-ECAM p.5853, 'the EGT may fail to decrease after the MASTER LEVER is turned off')",
        )
        .confirm(10.0)
        .inhibit(phase::NONE),
    );

    // ---- TWO ENG OUT ON SAME/OPPOSITE SIDE (Pattern 17, composed).
    // `core_running(eng)` split left (1, 2) vs. right (3, 4) wing, gated so
    // a fully cold aircraft (every engine's core not running) does not read
    // as a permanent two-engine-out. FCOM PRO-ABN-ECAM p.5862/5868 confirm
    // both the level (SC/MASTER CAUT, Caution) and the inhibit ([5, 6],
    // `phase::ENG_56`) by rendering the page.
    let any_running = any(vec![core_running(1), core_running(2), core_running(3), core_running(4)]);
    v.push(
        proc(
            701_800_159,
            "ENG TWO ENG OUT ON SAME SIDE",
            Level::Caution,
            sd_page::ENG,
            all(vec![
                any_running.clone(),
                any(vec![
                    all(vec![not(core_running(1)), not(core_running(2))]),
                    all(vec![not(core_running(3)), not(core_running(4))]),
                ]),
            ]),
            "both engines on one wing (1+2 or 3+4) are not running while at least one engine elsewhere is, ruling out a fully cold aircraft",
        )
        .confirm(2.0)
        .inhibit(phase::ENG_56),
    );
    v.push(
        proc(
            701_800_160,
            "ENG TWO ENG OUT ON OPPOSITE SIDE",
            Level::Caution,
            sd_page::ENG,
            all(vec![
                any_running,
                any(vec![
                    all(vec![not(core_running(1)), not(core_running(3))]),
                    all(vec![not(core_running(1)), not(core_running(4))]),
                    all(vec![not(core_running(2)), not(core_running(3))]),
                    all(vec![not(core_running(2)), not(core_running(4))]),
                ]),
            ]),
            "one engine on each wing is not running while at least one engine elsewhere is, ruling out a fully cold aircraft",
        )
        .confirm(2.0)
        .inhibit(phase::ENG_56),
    );

    // ---- THR LEVERS NOT SET (Pattern 18, now exact on its first clause).
    // FCOM PRO-ABN-ECAM p.5860: "Displayed during takeoff when either: one
    // throttle lever is set between CL and FLEX/MCT, or in case of
    // disagreement between the thrust lever position and the takeoff
    // thrust mode selected by the FADECs." The first clause is now built
    // from FlyByWire's own two named detents exactly, not a coarser
    // approximation: `eng1MCL` (`FwsFlightPhases.ts:220`, `TLA > 22.9`, the
    // CL/climb detent) and the same MCT lower bound (`TLA > 33.3`)
    // `eng_to_power_signal`'s own `mct_band` already reads -- "between CL
    // and FLEX/MCT" is exactly the gap `22.9 < TLA <= 33.3`. The second
    // clause (a FADEC-selected take-off mode discrete to compare the lever
    // against) has no equivalent published anywhere in this model and stays
    // unmodelled -- flagged, not silently dropped. The FCOM's own inhibit
    // list (phases 2-3 active) is used, which wins over this design's own
    // earlier phases-3-4 draft.
    let lever_between_cl_and_mct = |eng: u32| {
        let tla = format!("A32NX_ENG_{eng}_TLA_DEG");
        all(vec![var(&tla).gt(22.9), var(&tla).le(33.3)])
    };
    v.push(
        proc(
            701_800_157,
            "ENG THR LEVERS NOT SET",
            Level::Caution,
            sd_page::ENG,
            any(vec![lever_between_cl_and_mct(1), lever_between_cl_and_mct(2), lever_between_cl_and_mct(3), lever_between_cl_and_mct(4)]),
            "at least one thrust lever is set between the CL/climb detent (22.9 deg) and the FLEX/MCT band (33.3 deg), FlyByWire's own detents (FwsFlightPhases.ts) and the FCOM's own first triggering clause (PRO-ABN-ECAM p.5860); the FCOM's second clause (a FADEC-selected take-off mode discrete) has no equivalent in this model and is not wired",
        )
        .confirm(0.0)
        .inhibit(&[phase::ELEC_PWR, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN, phase::AT_OR_BELOW_80_KT, phase::ENGINES_SHUTDOWN]),
    );

    // ---- Thrust reverser patterns (engines 2 and 3 only -- the real A380
    // carries no reverser on 1/4). Patterns 24, 25, 27, 28: FCOM's own
    // inhibit lists throughout; `Truth::controls.reverser_deploy_commanded`
    // is already wired to the real reverse-thrust lever by the integration
    // baseline this worktree carries (`AUTOTHRUST_TLA <= -4.3 deg`,
    // `deep/plugin.rs:846`) -- Revision 3's own "the day a lever reaches
    // Truth" premise is out of date; the plumbing already exists, so these
    // are EXISTS-composed, not new input-wiring work.
    let mut energized_conds = Vec::new();
    for eng in [2u32, 3u32] {
        energized_conds.push(var(&format!("A32NX_ENG_{eng}_REV_ENERGIZED")).on());

        // ---- REVERSER CTL FAULT (Pattern 24).
        v.push(
            proc(
                if eng == 2 { 701_800_137 } else { 701_800_138 },
                "ENG n REVERSER CTL FAULT",
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_REV_CTL_FAULT")).on(),
                "the reverser's own EEC-side control loop reads faulted, holding the sleeve at its last position",
            )
            .confirm(2.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT]),
        );

        // ---- REVERSER ENERGIZED (Pattern 25).
        v.push(
            proc(
                if eng == 2 { 701_800_139 } else { 701_800_140 },
                "ENG n REVERSER ENERGIZED",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_REV_ENERGIZED")).on(),
                "the reverse-thrust lever is selected for this engine, independent of the sleeve's actual position",
            )
            .confirm(0.0)
            .inhibit(&[phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::TOUCH_DOWN]),
        );

        // ---- REVERSER INHIBITED (Pattern 26, re-sourced). FCOM
        // PRO-ABN-ECAM p.5827: "One thrust reverser has been electrically
        // and mechanically inhibited by maintenance action" -- a
        // maintenance-selected dispatch lockout, not the in-flight
        // lever-position condition first designed. This port models it
        // from the MEL: `crate::mel::deferred_state` of `crate::failures`'
        // own "thrust reverser lock fault" item (`78_000+i`, MEL 78-30-04,
        // `mel_catalog.rs:194`) -- exactly the real-world action ("the
        // reverser fails to lock stowed, or fails to deploy on command...
        // deactivated and secured" per that item's own dispatch condition)
        // the FCOM's text describes, published as
        // `A32NX_ENG_n_REV_MEL_INOP` (`engine_accessories/live.rs`, next to
        // this engine's other reverser publishes). No aural or master light
        // (confirmed by rendering the page), so Advisory.
        v.push(
            proc(
                if eng == 2 { 701_800_143 } else { 701_800_144 },
                "ENG n REVERSER INHIBITED",
                Level::Advisory,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_REV_MEL_INOP")).on(),
                "this reverser's thrust-reverser-lock-fault item is currently deferred under the MEL -- a maintenance action, not an in-flight condition",
            )
            .confirm(0.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );

        // ---- REV LOCKED (Pattern 27). Normal, healthy stowed+locked memo:
        // not commanded, sleeve within the same disagree tolerance
        // `REV_POSITION_DISAGREE` itself uses (`live.rs`'s own
        // `REVERSER_DISAGREE_TOLERANCE`, 0.05), and no lock degraded.
        v.push(
            proc(
                if eng == 2 { 701_800_145 } else { 701_800_146 },
                "ENG n REV LOCKED",
                Level::Advisory,
                sd_page::ENG,
                all(vec![
                    var(&format!("A32NX_ENG_{eng}_REV_ENERGIZED")).off(),
                    var(&format!("A32NX_ENG_{eng}_REV_POSITION")).le(0.05),
                    var(&format!("A32NX_ENG_{eng}_REV_LOCK_DEGRADED_COUNT")).eq(0.0),
                    var(&format!("A32NX_ENG_{eng}_REV_CTL_FAULT")).off(),
                ]),
                "the reverser is not commanded, its sleeve reads stowed within tolerance and none of its three locks are degraded",
            )
            .confirm(1.0)
            .inhibit(&[phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT]),
        );

        // ---- REVERSER MINOR FAULT (Pattern 28). Exactly one of the three
        // independent locks degraded while position still reads nominal --
        // degraded redundancy, distinct from Pattern 13's own duplicate-of
        // targets (`REVERSER_FAULT`/`REVERSER_UNLOCKED`), which take over
        // once position actually disagrees.
        v.push(
            proc(
                if eng == 2 { 701_800_147 } else { 701_800_148 },
                "ENG n REVERSER MINOR FAULT",
                Level::Advisory,
                sd_page::ENG,
                all(vec![
                    var(&format!("A32NX_ENG_{eng}_REV_LOCK_DEGRADED_COUNT")).eq(1.0),
                    var(&format!("A32NX_ENG_{eng}_REV_POSITION_DISAGREE")).off(),
                ]),
                "exactly one of this reverser's three independent locks is degraded while the sleeve position still agrees with its command",
            )
            .confirm(5.0)
            .inhibit(&[phase::ELEC_PWR, phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::AT_OR_ABOVE_400_FT, phase::AT_OR_ABOVE_1500_FT, phase::AT_OR_BELOW_800_FT, phase::TOUCH_DOWN]),
        );
    }

    // ---- ENG REVERSER SELECTED (Pattern 29). Reuses Pattern 25's own
    // publishes directly -- no new publish needed. FCOM PRO-ABN-ECAM
    // p.5856's own triggering text is explicit that this is an *in-flight*
    // condition ("Any thrust reverser is selected in flight. Thrust
    // reverser system protection prohibits in-flight deployment") -- a
    // reverser selected on the ground after landing is the normal case,
    // not this alert, so the airborne check this design's Pattern 36
    // already built (`airborne`, all five gear legs unloaded) is reused
    // here too rather than the design sheet's un-gated `any(lever2,
    // lever3)`.
    v.push(
        proc(
            701_800_154,
            "ENG REVERSER SELECTED",
            Level::Advisory,
            sd_page::ENG,
            all(vec![any(energized_conds), airborne]),
            "either reverser-carrying engine's lever is selected to reverse thrust while the aircraft is genuinely airborne (all five gear legs unloaded), matching the FCOM's own 'in flight' wording",
        )
        .confirm(0.0)
        .inhibit(&[phase::ELEC_PWR, phase::FIRST_ENG_STARTED, phase::SECOND_ENG_TO_POWER, phase::AT_OR_ABOVE_80_KT, phase::AT_OR_ABOVE_V1, phase::LIFT_OFF, phase::TOUCH_DOWN, phase::AT_OR_BELOW_80_KT, phase::ENGINES_SHUTDOWN]),
    );
}

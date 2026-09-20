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

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

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
    }
}

//! ATA 29 -- hydraulics. All 18 unwired procedures in `ata29-30.ts` belong
//! to this chapter (every ATA 30 ice-and-rain procedure FlyByWire defines is
//! already wired in `FwsAbnormalSensed.ts`), and **none of them is wired
//! here**. This module exists to record why, in code, next to the ones that
//! are -- the list of what the aircraft is still thin on is as much of the
//! deliverable as the triggers.
//!
//! * `290800037`/`290800038` HYD G/Y SYS TEMP HI: `HYD_{G,Y}_FLUID_TEMP_C`
//!   is published and `deep::hydraulics::thermal`'s `OVERHEAT_K` is a
//!   properly sourced limit (Skydrol LD-4's 107 C maximum continuous
//!   operating temperature, Eastman Pub. No. 7249153C). But
//!   `HYD_{G,Y}_RESERVOIR_OVHT` is published as *that same comparison*
//!   (`thermal.rs:118`, `overheat: self.temp_k > OVERHEAT_K`), and
//!   `deep::hydraulics::registry` already raises `HYD G RSVR OVHT` /
//!   `HYD Y RSVR OVHT` from it. The two would be true in exactly the same
//!   frames, so wiring FlyByWire's procedure would put two titles on the
//!   EWD for one temperature crossing. Splitting them needs a separate
//!   *system* temperature -- the manifold rather than the reservoir --
//!   which `deep::hydraulics` does not publish.
//! * `290800013`..`290800018` FUEL HEAT EXCHANGER VLV FAULT / AIR LEAK and
//!   their detection faults, `290800027`/`290800028` SYS COOLING FAULT:
//!   `deep::hydraulics::thermal` models the fuel/hydraulic heat exchanger
//!   as a conductance, with no valve, no air side and no leak path, so
//!   there is nothing that could be faulted.
//! * `290800023`..`290800026`, `290800033`/`290800034` OVHT DET FAULT (per
//!   channel), `290800029`/`290800030` SYS MONITORING FAULT: the overheat
//!   detection *loops* and the monitoring computers are not modelled --
//!   `fluid_overheat` is the model's own verdict, not a sensor with a
//!   health of its own, so a detector fault has nothing to act on.

use super::FbwProc;

pub fn wire(_v: &mut [FbwProc]) {}

//! The FlyByWire consumers a protection unit takes power away from.
//!
//! Each is a consumer the port's FlyByWire patches give a
//! `ELEC_<X>_BREAKER_OPEN` input (`patches/fbw-rust/power-path-*.patch`,
//! `breakers.patch`): 1 cuts its power, 0 -- including never written --
//! leaves it powered. The X-Plane plugin drives them from its older
//! `src/breakers.rs` table; a host running this crate drives them from the
//! units here, so the unit a fault trips is the one that cuts the consumer.
//!
//! The table is `src/breakers.rs`'s gated entries, each keyed to the deep
//! unit(s) of the same id, and the plugin's
//! `deep::electrical_crate_parity::the_crate_gates_are_this_plugins_gated_consumers`
//! derives it from that file and checks it equals this one. Not here: the
//! five AC bus feeders (`ELEC_AC_n_FEED_BREAKER_OPEN`,
//! `ELEC_AC_ESS_SHED_BREAKER_OPEN`), which the deep network models as its
//! own contactors rather than as protection units.

/// One gated consumer.
pub struct Gate {
    /// The FlyByWire input, without the `A32NX_` prefix.
    pub variable: &'static str,
    /// The units feeding it. The consumer loses power only when all of them
    /// are open: a dual-fed computer keeps running on either feed.
    pub units: &'static [&'static str],
}

pub const GATES: &[Gate] = &[
    Gate { variable: "ELEC_CABIN_FAN_1_BREAKER_OPEN", units: &["cab-fan-1"] },
    Gate { variable: "ELEC_CABIN_FAN_2_BREAKER_OPEN", units: &["cab-fan-2"] },
    Gate { variable: "ELEC_CABIN_FAN_3_BREAKER_OPEN", units: &["cab-fan-3"] },
    Gate { variable: "ELEC_CABIN_FAN_4_BREAKER_OPEN", units: &["cab-fan-4"] },
    Gate { variable: "ELEC_FDAC_1_1_BREAKER_OPEN", units: &["fdac-1a"] },
    Gate { variable: "ELEC_FDAC_1_2_BREAKER_OPEN", units: &["fdac-1b"] },
    Gate { variable: "ELEC_FDAC_2_1_BREAKER_OPEN", units: &["fdac-2a"] },
    Gate { variable: "ELEC_FDAC_2_2_BREAKER_OPEN", units: &["fdac-2b"] },
    Gate { variable: "ELEC_TADD_1_BREAKER_OPEN", units: &["tadd-1"] },
    Gate { variable: "ELEC_TADD_2_BREAKER_OPEN", units: &["tadd-2"] },
    Gate { variable: "ELEC_VCM_FWD_1_BREAKER_OPEN", units: &["vcm-fwd-1"] },
    Gate { variable: "ELEC_VCM_FWD_2_BREAKER_OPEN", units: &["vcm-fwd-2"] },
    Gate { variable: "ELEC_VCM_AFT_1_BREAKER_OPEN", units: &["vcm-aft-1"] },
    Gate { variable: "ELEC_VCM_AFT_2_BREAKER_OPEN", units: &["vcm-aft-2"] },
    Gate { variable: "ELEC_OCSM_1_1_BREAKER_OPEN", units: &["ocsm-1a"] },
    Gate { variable: "ELEC_OCSM_1_2_BREAKER_OPEN", units: &["ocsm-1b"] },
    Gate { variable: "ELEC_OCSM_2_1_BREAKER_OPEN", units: &["ocsm-2a"] },
    Gate { variable: "ELEC_OCSM_2_2_BREAKER_OPEN", units: &["ocsm-2b"] },
    Gate { variable: "ELEC_OCSM_3_1_BREAKER_OPEN", units: &["ocsm-3a"] },
    Gate { variable: "ELEC_OCSM_3_2_BREAKER_OPEN", units: &["ocsm-3b"] },
    Gate { variable: "ELEC_OCSM_4_1_BREAKER_OPEN", units: &["ocsm-4a"] },
    Gate { variable: "ELEC_OCSM_4_2_BREAKER_OPEN", units: &["ocsm-4b"] },
    Gate { variable: "ELEC_PACK_1_FLOW_VALVE_1_BREAKER_OPEN", units: &["pack-1-flow-valve-1"] },
    Gate { variable: "ELEC_PACK_1_FLOW_VALVE_2_BREAKER_OPEN", units: &["pack-1-flow-valve-2"] },
    Gate { variable: "ELEC_PACK_2_FLOW_VALVE_1_BREAKER_OPEN", units: &["pack-2-flow-valve-1"] },
    Gate { variable: "ELEC_PACK_2_FLOW_VALVE_2_BREAKER_OPEN", units: &["pack-2-flow-valve-2"] },
    Gate { variable: "ELEC_FIRE_LOOP_A_1_BREAKER_OPEN", units: &["fire-loop-eng1-A"] },
    Gate { variable: "ELEC_FIRE_LOOP_B_1_BREAKER_OPEN", units: &["fire-loop-eng1-B"] },
    Gate { variable: "ELEC_FIRE_LOOP_A_2_BREAKER_OPEN", units: &["fire-loop-eng2-A"] },
    Gate { variable: "ELEC_FIRE_LOOP_B_2_BREAKER_OPEN", units: &["fire-loop-eng2-B"] },
    Gate { variable: "ELEC_FIRE_LOOP_A_3_BREAKER_OPEN", units: &["fire-loop-eng3-A"] },
    Gate { variable: "ELEC_FIRE_LOOP_B_3_BREAKER_OPEN", units: &["fire-loop-eng3-B"] },
    Gate { variable: "ELEC_FIRE_LOOP_A_4_BREAKER_OPEN", units: &["fire-loop-eng4-A"] },
    Gate { variable: "ELEC_FIRE_LOOP_B_4_BREAKER_OPEN", units: &["fire-loop-eng4-B"] },
    Gate { variable: "ELEC_FIRE_LOOP_A_APU_BREAKER_OPEN", units: &["fire-loop-apu-A"] },
    Gate { variable: "ELEC_FIRE_LOOP_B_APU_BREAKER_OPEN", units: &["fire-loop-apu-B"] },
    Gate { variable: "ELEC_FIRE_LOOP_A_MLG_BREAKER_OPEN", units: &["fire-loop-mlgbay-A"] },
    Gate { variable: "ELEC_FIRE_LOOP_B_MLG_BREAKER_OPEN", units: &["fire-loop-mlgbay-B"] },
    Gate { variable: "ELEC_LGCIU_1_BREAKER_OPEN", units: &["lgciu-1-normal-bkr", "lgciu-1-2nd-bkr"] },
    Gate { variable: "ELEC_LGCIU_2_BREAKER_OPEN", units: &["lgciu-2-normal-bkr", "lgciu-2-2nd-bkr"] },
    Gate { variable: "ELEC_PUMP_GA_BREAKER_OPEN", units: &["hyd-epump-ga"] },
    Gate { variable: "ELEC_PUMP_GB_BREAKER_OPEN", units: &["hyd-epump-gb"] },
    Gate { variable: "ELEC_PUMP_YA_BREAKER_OPEN", units: &["hyd-epump-ya"] },
    Gate { variable: "ELEC_PUMP_YB_BREAKER_OPEN", units: &["hyd-epump-yb"] },
    Gate { variable: "ELEC_AUTOBRAKE_DISARM_SOLENOID_BREAKER_OPEN", units: &["autobrake-disarm-sol"] },
    Gate { variable: "ELEC_RA_1_BREAKER_OPEN", units: &["ra-sys-a"] },
    Gate { variable: "ELEC_RA_2_BREAKER_OPEN", units: &["ra-sys-b"] },
    Gate { variable: "ELEC_RA_3_BREAKER_OPEN", units: &["ra-sys-c"] },
    Gate { variable: "ELEC_EGPWC_BREAKER_OPEN", units: &["egpwc"] },
    Gate { variable: "ELEC_BLEED_ENG_1_BREAKER_OPEN", units: &["bleed-eng-1"] },
    Gate { variable: "ELEC_BLEED_ENG_2_BREAKER_OPEN", units: &["bleed-eng-2"] },
    Gate { variable: "ELEC_BLEED_ENG_3_BREAKER_OPEN", units: &["bleed-eng-3"] },
    Gate { variable: "ELEC_BLEED_ENG_4_BREAKER_OPEN", units: &["bleed-eng-4"] },
    // Each cross-feed fire-extinguisher bottle has 2 pyrotechnic squibs (its
    // 2 real power feeds); both must open before the bottle it fires -- and
    // every engine's own shot-1/shot-2 bottle pair, since the bottles
    // cross-feed to all 4 engines -- loses power (fire_and_smoke_protection.rs).
    Gate { variable: "ELEC_FIRE_BOTTLE_1_BREAKER_OPEN", units: &["eng-fire-bottle-1-squib-1", "eng-fire-bottle-1-squib-2"] },
    Gate { variable: "ELEC_FIRE_BOTTLE_2_BREAKER_OPEN", units: &["eng-fire-bottle-2-squib-1", "eng-fire-bottle-2-squib-2"] },
    Gate { variable: "ELEC_FIRE_BOTTLE_APU_BREAKER_OPEN", units: &["apu-fire-bottle-squib-1", "apu-fire-bottle-squib-2"] },
    // RAT deployment solenoid (electrical/mod.rs A380RamAirTurbineController).
    Gate { variable: "ELEC_RAT_DEPLOY_SOLENOID_BREAKER_OPEN", units: &["rat-deploy-solenoid"] },
    // APU starter-generator start contactor control coil (apu/pw980.rs Pw980StartMotor).
    Gate { variable: "ELEC_APU_STARTER_CONTACTOR_BREAKER_OPEN", units: &["apu-start-contactor"] },
];

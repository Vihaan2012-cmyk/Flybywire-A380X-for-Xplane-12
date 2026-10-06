use super::adr;
use super::aoa_vane::{AoaVane, AoaVaneFaults};
use super::gps::{GpsFaults, GpsReceiver};
use super::ice_detector::{IceDetector, IceDetectorFaults};
use super::pitot::{PitotFaults, PitotProbe};
use super::radio_altimeter::{RadioAltimeter, RadioAltimeterFaults};
use super::static_port::{
    average_pair, StaticAveragingLineFaults, StaticPort, StaticPortFaults,
};
use super::tat_probe::{TatProbe, TatProbeFaults};
use super::discrete::{self, TemperatureSensorFaults};
use super::live_discrete::DiscreteSensors;
use super::{engine_sensors, registry};
use crate::deep::api::Registry;
use crate::deep::integration::weather_truth;
use crate::deep::live::{Area, DerivedFailure, Faults, Truth};
use std::collections::BTreeMap;

pub fn live_system() -> Box<dyn Area> {
    Box::new(LiveSensors::new())
}

const AC_BUS_ALIVE_V: f64 = 90.0;

const HEATER_FAILED_FRACTION: f64 = 0.5;

const NOMINAL_SATELLITES_VISIBLE: u32 = 10;

const OAT_RANGE_C: (f64, f64) = (-99.0, 99.0);

const ADR_DISAGREE_CAS_MS: f64 = 16.0 * 0.514_444;
const ADR_DISAGREE_ALT_M: f64 = 250.0 * 0.304_8;

mod full_scale {
    pub const RA_FALSE_OFFSET_FT: f64 = -20.0;
    pub const ICE_DETECTOR_SHIFT: f64 = 0.05;
    pub const GPS_SPOOF_OFFSET_M: f64 = 1_000.0;
}

pub struct FaultIndex(BTreeMap<(String, String), u64>);

impl FaultIndex {
    pub fn build() -> Self {
        let mut r = Registry::default();
        registry::register(&mut r);
        let mut map = BTreeMap::new();
        for f in &r.failures {
            if let Some(start) = f.model_field.find("(faults.") {
                let field = &f.model_field[start + "(faults.".len()..];
                if let Some(field) = field.strip_suffix(')') {
                    map.insert((f.component.clone(), field.to_string()), f.id);
                }
            }
        }
        Self(map)
    }

    pub fn id(&self, component: &str, field: &str) -> u64 {
        self.0
            .get(&(component.to_string(), field.to_string()))
            .copied()
            .unwrap_or(0)
    }

    pub fn ids<const N: usize>(&self, component: &str, fields: [&str; N]) -> [u64; N] {
        fields.map(|f| self.id(component, f))
    }
}

struct AirDataChannel {
    pitot: PitotProbe,
    static_left: StaticPort,
    static_right: StaticPort,
    pitot_faults: [u64; 4],
    static_left_faults: [u64; 2],
    static_right_faults: [u64; 2],
    averaging_line_fault: u64,
    ac_bus: usize,
}

struct LiveVane {
    vane: AoaVane,
    faults: [u64; 4],
    ac_bus: usize,
}

struct LiveTat {
    probe: TatProbe,
    faults: [u64; 2],
    ac_bus: usize,
}

struct LiveIceDetector {
    detector: IceDetector,
    faults: [u64; 3],
}

struct LiveRadioAltimeter {
    unit: RadioAltimeter,
    transceiver_faults: [u64; 3],
    tx_antenna_fault: u64,
    rx_antenna_faults: [u64; 2],
    direct_coupling_fault: u64,
}

struct LiveGps {
    receiver: GpsReceiver,
    receiver_faults: [u64; 3],
    antenna_faults: [u64; 2],
}

#[derive(Clone, Copy, Debug, Default)]
struct Snapshot {
    pitot_heater_failed: [bool; 4],
    pitot_blocked: [bool; 4],
    pitot_ice_kg: [f64; 4],
    static_degraded: [bool; 4],
    adr: [adr::AdrOutputs; 3],
    standby: adr::AdrOutputs,
    adr_disagree: bool,
    standby_disagree: bool,
    adr_cas_vote: adr::VoteResult,
    adr_alt_vote: adr::VoteResult,
    adr_cas_error_ms: [f64; 3],
    adr_alt_error_m: [f64; 3],
    adr_sat_error_c: [f64; 3],
    aoa_jammed: [bool; 3],
    aoa_deg: [f64; 3],
    aoa_error_deg: [f64; 3],
    aoa_heater_failed: [bool; 3],
    tat_heater_failed: [bool; 2],
    tat_c: [f64; 2],
    ice_detected: [bool; 2],
    ice_kg: [f64; 2],
    ra_valid: [bool; 3],
    ra_in_range: [bool; 3],
    ra_agl_ft: [f64; 3],
    ra_transceiver_fault: [f64; 3],
    ra_antenna_fault: [f64; 3],
    ra_direct_coupling_fault: [f64; 3],
    gps_valid: [bool; 3],
    gps_degraded: [bool; 3],
    gps_error_m: [f64; 3],
    gps_offset_m: [[f64; 2]; 3],
    standby_oat_c: f64,
    n1_pickup_valid: [[bool; 2]; 4],
    n1_pickup_frac: [[f64; 2]; 4],
    total_heater_power_w: f64,
    oat_1_2_heater_failed: [bool; 2],
    sideslip_jammed: [bool; 3],
    sideslip_heater_failed: [bool; 3],
    tat3_heater_failed: bool,
    tat3_recovery_degraded: bool,
    adr_capt_fo_alt_diff_ft: f64,
    ir_capt_fo_pitch_diff_deg: f64,
    ir_capt_fo_roll_diff_deg: f64,
    ir_capt_fo_hdg_diff_deg: f64,
    ir_capt_fo_fpa_diff_deg: f64,
    ir_gyro_drift_deg_hr: [f64; 3],
    kccu_part_failed: [[bool; 2]; 2],
    du_display_monitor_disagree: [bool; 5],
    baro_ref_disagree: bool,
}

pub const CDS_MONITORED_DUS: [&str; 5] = ["capt-pfd-du", "capt-nd-du", "capt-ewd-du", "fo-pfd-du", "fo-nd-du"];

pub fn cds_monitor_var(du: &str) -> String {
    format!("DEEP_CDS_{}_MONITOR_DISAGREE", du.to_ascii_uppercase().replace('-', "_"))
}

pub const IR_GYRO_DRIFT_AT_FULL_FAULT_DEG_HR: f64 = 100.0;

fn ir_disagreement_deg(a: Option<f64>, b: Option<f64>, wrap: bool) -> f64 {
    match (a, b) {
        (Some(a), Some(b)) if wrap => ((a - b + 540.0).rem_euclid(360.0) - 180.0).abs(),
        (Some(a), Some(b)) => (a - b).abs(),
        _ => 0.0,
    }
}

pub struct LiveSensors {
    channels: [AirDataChannel; 4],
    vanes: [LiveVane; 3],
    tat: [LiveTat; 2],
    ice: [LiveIceDetector; 2],
    ra: [LiveRadioAltimeter; 3],
    gps: [LiveGps; 3],
    n1_pickup_faults: [[[u64; 2]; 2]; 4],
    oat_faults: [u64; 2],
    oat_1_2_faults: [u64; 2],
    sideslip_faults: [[u64; 2]; 3],
    tat3_faults: [u64; 2],
    ir_faults: [u64; 3],
    kccu_faults: [[u64; 2]; 2],
    du_faults: [u64; 5],
    discrete: DiscreteSensors,
    snapshot: Snapshot,
}

impl Default for LiveSensors {
    fn default() -> Self {
        Self::new()
    }
}

impl LiveSensors {
    pub fn new() -> Self {
        let index = FaultIndex::build();
        const ISA_SEA_LEVEL_PA: f64 = 101_325.0;

        let channels = core::array::from_fn(|i| {
            let n = i + 1;
            AirDataChannel {
                pitot: PitotProbe::new(ISA_SEA_LEVEL_PA),
                static_left: StaticPort::new(ISA_SEA_LEVEL_PA),
                static_right: StaticPort::new(ISA_SEA_LEVEL_PA),
                pitot_faults: index.ids(
                    &format!("34_nav.pitot_{n}"),
                    [
                        "heater_failure",
                        "insect_or_tape_blockage",
                        "mechanical_damage",
                        "drain_blocked",
                    ],
                ),
                static_left_faults: index
                    .ids(&format!("34_nav.static_{n}_1"), ["blocked", "leak_to_cabin"]),
                static_right_faults: index
                    .ids(&format!("34_nav.static_{n}_2"), ["blocked", "leak_to_cabin"]),
                averaging_line_fault: index
                    .id(&format!("34_nav.static_avg_line_{n}"), "line_blocked"),
                ac_bus: i,
            }
        });

        let vanes = core::array::from_fn(|i| LiveVane {
            vane: AoaVane::new(0x5A0A_0001 + i as u64, 0.0),
            faults: index.ids(
                &format!("34_nav.aoa_{}", i + 1),
                ["heater_failure", "mechanically_stuck", "resolver_wear", "damage"],
            ),
            ac_bus: i,
        });

        let tat = core::array::from_fn(|i| LiveTat {
            probe: TatProbe::new(15.0),
            faults: index.ids(
                &format!("34_nav.tat_{}", i + 1),
                ["heater_failure", "recovery_degradation"],
            ),
            ac_bus: i,
        });

        let ice = core::array::from_fn(|i| LiveIceDetector {
            detector: IceDetector::new(),
            faults: index.ids(
                &format!("30_ice.detector_{}", i + 1),
                ["heater_failure", "frequency_sensor_bias", "probe_damage_bias"],
            ),
        });

        let ra = core::array::from_fn(|i| {
            let n = i + 1;
            LiveRadioAltimeter {
                unit: RadioAltimeter::new(0x4A17_0001 + i as u64),
                transceiver_faults: index.ids(
                    &format!("34_nav.ra_transceiver_{n}"),
                    ["transceiver_fault", "false_offset_ft", "tracking_loop_degradation"],
                ),
                tx_antenna_fault: index
                    .id(&format!("34_nav.ra_tx_antenna_{n}"), "tx_antenna_fault"),
                rx_antenna_faults: index.ids(
                    &format!("34_nav.ra_rx_antenna_{n}"),
                    ["rx_antenna_fault", "rx_antenna_degradation"],
                ),
                direct_coupling_fault: index
                    .id(&format!("34_nav.ra_ant_coupling_{n}"), "direct_coupling_fault"),
            }
        });

        let gps = core::array::from_fn(|i| {
            let n = i + 1;
            LiveGps {
                receiver: GpsReceiver::new(0x6950_0001u64 + i as u64),
                receiver_faults: index.ids(
                    &format!("34_nav.gps_receiver_{n}"),
                    ["receiver_fault", "jamming", "spoof_target_offset_m"],
                ),
                antenna_faults: index.ids(
                    &format!("34_nav.gps_antenna_{n}"),
                    ["antenna_fault", "antenna_degradation"],
                ),
            }
        });

        let n1_pickup_faults = core::array::from_fn(|e| {
            core::array::from_fn(|ch| {
                let channel = if ch == 0 { "a" } else { "b" };
                index.ids(
                    &format!("77_eng.speed_n1_{}_{channel}", e + 1),
                    ["air_gap_increase", "open_circuit"],
                )
            })
        });

        let oat_faults = index.ids("34_nav.oat_standby", ["open_circuit", "short_circuit"]);

        let oat_1_2_faults = core::array::from_fn(|i| index.id(&format!("34_nav.oat_{}", i + 1), "heater_failure"));
        let sideslip_faults = core::array::from_fn(|i| index.ids(&format!("34_nav.sideslip_{}", i + 1), ["heater_failure", "mechanically_stuck"]));
        let tat3_faults = index.ids("34_nav.tat_3", ["heater_failure", "recovery_degradation"]);
        let ir_faults = core::array::from_fn(|i| index.id(&format!("34_nav.ir_{}", i + 1), "alignment_drift"));
        let kccu_faults = ["capt", "fo"].map(|side| index.ids(&format!("31_elec.kccu-{side}"), ["ccd_failed", "keyboard_failed"]));
        let du_faults = CDS_MONITORED_DUS.map(|du| index.id(&format!("31_elec.{du}"), "display_monitor_disagree"));

        let discrete = DiscreteSensors::new(&index);

        Self {
            channels,
            vanes,
            tat,
            ice,
            ra,
            gps,
            n1_pickup_faults,
            oat_faults,
            oat_1_2_faults,
            sideslip_faults,
            tat3_faults,
            ir_faults,
            kccu_faults,
            du_faults,
            discrete,
            snapshot: Snapshot::default(),
        }
    }

    fn bus_alive(truth: &Truth, bus: usize) -> bool {
        truth.ac_bus_volts.get(bus).copied().unwrap_or(0.0) > AC_BUS_ALIVE_V
    }

    fn true_total_pressure_pa(static_pa: f64, mach: f64) -> f64 {
        static_pa.max(1.0) * (1.0 + 0.2 * mach.max(0.0).powi(2)).powf(3.5)
    }

    fn lwc_gm3(truth: &Truth) -> f64 {
        let Some(weather) = truth.environment.weather.as_ref() else { return 0.0 };
        let cloud = weather_truth::dominant_cloud(weather);
        weather_truth::lwc_kg_m3_from_conditions(truth.environment.sat_c, cloud) * 1_000.0
    }
}

impl Area for LiveSensors {
    fn name(&self) -> &'static str {
        "sensors"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s;
        let env = &truth.environment;
        let static_pa = env.ambient_pressure_pa.max(1.0);
        let mach = env.mach();
        let total_pa = Self::true_total_pressure_pa(static_pa, mach);
        let lwc_gm3 = Self::lwc_gm3(truth);
        let true_aoa_deg = 0.0;
        let static_port_aoa_deg = super::static_port::REFERENCE_AOA_DEG;
        let cabin_pa = truth.cabin_pressure_pa;

        let mut snap = Snapshot::default();
        let mut heater_w = 0.0;

        let mut sensed_total = [0.0; 4];
        let mut sensed_static = [0.0; 4];
        for (i, ch) in self.channels.iter_mut().enumerate() {
            let powered = Self::bus_alive(truth, ch.ac_bus);
            let pitot_faults = PitotFaults {
                heater_failure: faults.get(ch.pitot_faults[0]),
                insect_or_tape_blockage: faults.get(ch.pitot_faults[1]),
                mechanical_damage: faults.get(ch.pitot_faults[2]),
                drain_blocked: faults.get(ch.pitot_faults[3]),
            };
            let pitot = ch.pitot.step(
                total_pa,
                static_pa,
                env.tas_ms,
                env.sat_c,
                lwc_gm3,
                powered,
                &pitot_faults,
                dt,
            );

            let left = ch.static_left.step(
                static_pa,
                cabin_pa,
                static_port_aoa_deg,
                mach,
                &StaticPortFaults {
                    blocked: faults.get(ch.static_left_faults[0]),
                    leak_to_cabin: faults.get(ch.static_left_faults[1]),
                },
                dt,
            );
            let right = ch.static_right.step(
                static_pa,
                cabin_pa,
                static_port_aoa_deg,
                mach,
                &StaticPortFaults {
                    blocked: faults.get(ch.static_right_faults[0]),
                    leak_to_cabin: faults.get(ch.static_right_faults[1]),
                },
                dt,
            );
            let pair = average_pair(
                left,
                right,
                &StaticAveragingLineFaults {
                    line_blocked: faults.get(ch.averaging_line_fault),
                },
            );

            sensed_total[i] = pitot.sensed_total_pressure_pa;
            sensed_static[i] = pair.averaged_pa;
            snap.pitot_blocked[i] = pitot.blocked;
            snap.pitot_ice_kg[i] = pitot.ice_kg;
            snap.static_degraded[i] = pair.degraded;
            snap.pitot_heater_failed[i] = powered
                && pitot.heater_power_w < HEATER_FAILED_FRACTION * super::pitot::RATED_HEATER_W;
            heater_w += pitot.heater_power_w;
        }

        for (i, t) in self.tat.iter_mut().enumerate() {
            let powered = Self::bus_alive(truth, t.ac_bus);
            let out = t.probe.step(
                env.sat_c,
                mach,
                env.tas_ms,
                lwc_gm3,
                powered,
                &TatProbeFaults {
                    heater_failure: faults.get(t.faults[0]),
                    recovery_degradation: faults.get(t.faults[1]),
                },
                dt,
            );
            snap.tat_c[i] = out.sensed_tat_c;
            snap.tat_heater_failed[i] = powered
                && out.heater_power_w < HEATER_FAILED_FRACTION * super::tat_probe::RATED_HEATER_W;
            heater_w += out.heater_power_w;
        }

        let tat_for_adr = [snap.tat_c[0], snap.tat_c[1], snap.tat_c[0]];
        let true_ref = adr::compute(total_pa, static_pa, adr::tat_from_sat_c(env.sat_c, mach));
        for i in 0..3 {
            snap.adr[i] = adr::compute(sensed_total[i], sensed_static[i], tat_for_adr[i]);
            snap.adr_cas_error_ms[i] = snap.adr[i].cas_ms - true_ref.cas_ms;
            snap.adr_alt_error_m[i] = snap.adr[i].pressure_altitude_m - true_ref.pressure_altitude_m;
            snap.adr_sat_error_c[i] = snap.adr[i].sat_c - true_ref.sat_c;
        }
        snap.standby = adr::compute(sensed_total[3], sensed_static[3], snap.tat_c[0]);
        snap.adr_cas_vote = adr::vote3(
            snap.adr[0].cas_ms,
            snap.adr[1].cas_ms,
            snap.adr[2].cas_ms,
            ADR_DISAGREE_CAS_MS,
        );
        snap.adr_alt_vote = adr::vote3(
            snap.adr[0].pressure_altitude_m,
            snap.adr[1].pressure_altitude_m,
            snap.adr[2].pressure_altitude_m,
            ADR_DISAGREE_ALT_M,
        );
        snap.adr_disagree = snap.adr_cas_vote.disagree || snap.adr_alt_vote.disagree;
        snap.standby_disagree = (snap.standby.cas_ms - snap.adr_cas_vote.value).abs() > ADR_DISAGREE_CAS_MS
            || (snap.standby.pressure_altitude_m - snap.adr_alt_vote.value).abs() > ADR_DISAGREE_ALT_M;

        for (i, v) in self.vanes.iter_mut().enumerate() {
            let powered = Self::bus_alive(truth, v.ac_bus);
            let stuck = faults.get(v.faults[1]);
            let out = v.vane.step(
                true_aoa_deg,
                env.tas_ms,
                env.sat_c,
                lwc_gm3,
                powered,
                &AoaVaneFaults {
                    heater_failure: faults.get(v.faults[0]),
                    mechanically_stuck: stuck,
                    resolver_wear: faults.get(v.faults[2]),
                    damage: faults.get(v.faults[3]),
                },
                dt,
            );
            snap.aoa_deg[i] = out.sensed_aoa_deg;
            snap.aoa_error_deg[i] = out.sensed_aoa_deg - true_aoa_deg * super::aoa_vane::UPWASH_FACTOR;
            snap.aoa_jammed[i] = out.jammed_by_ice || stuck >= 0.98;
            snap.aoa_heater_failed[i] = powered
                && out.heater_power_w < HEATER_FAILED_FRACTION * super::aoa_vane::RATED_HEATER_W;
            heater_w += out.heater_power_w;
        }

        for (i, d) in self.ice.iter_mut().enumerate() {
            let out = d.detector.step(
                env.sat_c,
                env.tas_ms,
                lwc_gm3,
                &IceDetectorFaults {
                    heater_failure: faults.get(d.faults[0]),
                    frequency_sensor_bias: faults.get(d.faults[1])
                        * full_scale::ICE_DETECTOR_SHIFT,
                    probe_damage_bias: faults.get(d.faults[2]) * full_scale::ICE_DETECTOR_SHIFT,
                },
                dt,
            );
            snap.ice_detected[i] = out.ice_detected;
            snap.ice_kg[i] = out.ice_kg;
        }

        let agl_ft = if truth.on_ground { 0.0 } else { truth.altitude_ft.max(0.0) };
        let in_range = (0.0..=super::radio_altimeter::MAX_RANGE_FT).contains(&agl_ft);
        for (i, r) in self.ra.iter_mut().enumerate() {
            let transceiver_fault = faults.get(r.transceiver_faults[0]);
            let tx_antenna_fault = faults.get(r.tx_antenna_fault);
            let rx_antenna_fault = faults.get(r.rx_antenna_faults[0]);
            let out = r.unit.step(
                agl_ft,
                false,
                &RadioAltimeterFaults {
                    transceiver_fault,
                    tx_antenna_fault,
                    rx_antenna_fault,
                    rx_antenna_degradation: faults.get(r.rx_antenna_faults[1]),
                    false_offset_ft: faults.get(r.transceiver_faults[1])
                        * full_scale::RA_FALSE_OFFSET_FT,
                    tracking_loop_degradation: faults.get(r.transceiver_faults[2]),
                    direct_coupling_fault: faults.get(r.direct_coupling_fault),
                },
            );
            snap.ra_valid[i] = out.valid;
            snap.ra_in_range[i] = in_range;
            snap.ra_agl_ft[i] = out.agl_ft;
            snap.ra_transceiver_fault[i] = transceiver_fault;
            snap.ra_antenna_fault[i] = tx_antenna_fault.max(rx_antenna_fault);
            snap.ra_direct_coupling_fault[i] = faults.get(r.direct_coupling_fault);
        }

        for (i, g) in self.gps.iter_mut().enumerate() {
            let spoof = faults.get(g.receiver_faults[2]) * full_scale::GPS_SPOOF_OFFSET_M;
            let out = g.receiver.step(
                NOMINAL_SATELLITES_VISIBLE,
                &GpsFaults {
                    receiver_fault: faults.get(g.receiver_faults[0]),
                    jamming: faults.get(g.receiver_faults[1]),
                    antenna_fault: faults.get(g.antenna_faults[0]),
                    antenna_degradation: faults.get(g.antenna_faults[1]),
                    spoof_target_offset_m: [spoof, spoof],
                },
                dt,
            );
            snap.gps_valid[i] = out.valid;
            snap.gps_error_m[i] = out.position_error_1sigma_m;
            snap.gps_offset_m[i] = out.position_offset_m;
            snap.gps_degraded[i] = faults.get(g.receiver_faults[1]) > 0.0 && out.valid;
        }

        for engine in 0..4 {
            let true_frac = truth.engine_n1_frac[engine].max(0.0);
            for channel in 0..2 {
                let ids = self.n1_pickup_faults[engine][channel];
                let out = engine_sensors::speed_pickup_reading(
                    true_frac,
                    &engine_sensors::SpeedPickupFaults {
                        air_gap_increase: faults.get(ids[0]),
                        open_circuit: faults.get(ids[1]),
                    },
                );
                snap.n1_pickup_valid[engine][channel] = out.valid;
                snap.n1_pickup_frac[engine][channel] = out.speed_frac;
            }
        }

        snap.standby_oat_c = discrete::temperature_sensor_reading_c(
            env.sat_c,
            OAT_RANGE_C,
            &TemperatureSensorFaults {
                open_circuit: faults.get(self.oat_faults[0]),
                short_circuit: faults.get(self.oat_faults[1]),
            },
        );

        for i in 0..2 {
            snap.oat_1_2_heater_failed[i] = faults.get(self.oat_1_2_faults[i]) > 0.0;
        }
        for i in 0..3 {
            snap.sideslip_heater_failed[i] = faults.get(self.sideslip_faults[i][0]) > 0.0;
            snap.sideslip_jammed[i] = faults.get(self.sideslip_faults[i][1]) > 0.0;
        }
        snap.tat3_heater_failed = faults.get(self.tat3_faults[0]) > 0.0;
        snap.tat3_recovery_degraded = faults.get(self.tat3_faults[1]) > 0.0;

        snap.adr_capt_fo_alt_diff_ft = (snap.adr[0].pressure_altitude_m - snap.adr[1].pressure_altitude_m).abs() * crate::M_TO_FT;

        let (capt, fo) = crate::deep::live::capt_fo_ir(truth.att_hdg_switching_knob);
        let (a, b) = (&truth.ir[capt], &truth.ir[fo]);
        snap.ir_capt_fo_pitch_diff_deg = ir_disagreement_deg(a.pitch_deg, b.pitch_deg, false);
        snap.ir_capt_fo_roll_diff_deg = ir_disagreement_deg(a.roll_deg, b.roll_deg, true);
        snap.ir_capt_fo_hdg_diff_deg = ir_disagreement_deg(a.true_heading_deg, b.true_heading_deg, true);
        snap.ir_capt_fo_fpa_diff_deg = ir_disagreement_deg(a.flight_path_angle_deg, b.flight_path_angle_deg, false);
        snap.ir_gyro_drift_deg_hr = core::array::from_fn(|i| faults.get(self.ir_faults[i]) * IR_GYRO_DRIFT_AT_FULL_FAULT_DEG_HR);

        snap.kccu_part_failed = core::array::from_fn(|side| core::array::from_fn(|part| faults.get(self.kccu_faults[side][part]) > 0.0));
        snap.du_display_monitor_disagree = core::array::from_fn(|i| faults.get(self.du_faults[i]) > 0.0);

        snap.baro_ref_disagree = truth.controls.baro_mode[0] != truth.controls.baro_mode[1];

        snap.total_heater_power_w = heater_w;
        self.snapshot = snap;

        self.discrete.tick(truth, faults);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let s = &self.snapshot;

        out("DEEP_ADR_VOTE_DISAGREE", f64::from(s.adr_disagree));
        for i in 0..4 {
            out(
                &format!("DEEP_PITOT_{}_HEATER_FAILED", i + 1),
                f64::from(s.pitot_heater_failed[i]),
            );
        }
        for i in 0..3 {
            out(&format!("DEEP_AOA_{}_JAMMED", i + 1), f64::from(s.aoa_jammed[i]));
        }
        for i in 0..2 {
            out(
                &format!("DEEP_TAT_{}_HEATER_FAILED", i + 1),
                f64::from(s.tat_heater_failed[i]),
            );
        }
        for i in 0..3 {
            out(&format!("DEEP_RA_{}_VALID", i + 1), f64::from(s.ra_valid[i]));
            out(&format!("DEEP_GPS_{}_VALID", i + 1), f64::from(s.gps_valid[i]));
            out(&format!("DEEP_GPS_{}_DEGRADED", i + 1), f64::from(s.gps_degraded[i]));
        }

        for i in 0..3 {
            let n = i + 1;
            out(&format!("DEEP_ADR_{n}_CAS_MS"), s.adr[i].cas_ms);
            out(&format!("DEEP_ADR_{n}_MACH"), s.adr[i].mach);
            out(&format!("DEEP_ADR_{n}_ALT_M"), s.adr[i].pressure_altitude_m);
            out(&format!("DEEP_ADR_{n}_TAS_MS"), s.adr[i].tas_ms);
            out(&format!("DEEP_ADR_{n}_SAT_C"), s.adr[i].sat_c);
            out(&format!("DEEP_ADR_{n}_OUTLIER"), f64::from(s.adr_cas_vote.outlier[i] || s.adr_alt_vote.outlier[i]));
            out(&format!("DEEP_ADR_{n}_CAS_ERROR_MS"), s.adr_cas_error_ms[i]);
            out(&format!("DEEP_ADR_{n}_ALT_ERROR_M"), s.adr_alt_error_m[i]);
            out(&format!("DEEP_ADR_{n}_SAT_ERROR_C"), s.adr_sat_error_c[i]);
            out(&format!("DEEP_AOA_{n}_DEG"), s.aoa_deg[i]);
            out(&format!("DEEP_AOA_{n}_ERROR_DEG"), s.aoa_error_deg[i]);
            out(&format!("DEEP_AOA_{n}_HEATER_FAILED"), f64::from(s.aoa_heater_failed[i]));
            out(&format!("DEEP_RA_{n}_AGL_FT"), s.ra_agl_ft[i]);
            out(&format!("DEEP_RA_{n}_IN_RANGE"), f64::from(s.ra_in_range[i]));
            out(&format!("DEEP_GPS_{n}_ERROR_M"), s.gps_error_m[i]);
            out(&format!("DEEP_GPS_{n}_OFFSET_N_M"), s.gps_offset_m[i][0]);
            out(&format!("DEEP_GPS_{n}_OFFSET_E_M"), s.gps_offset_m[i][1]);
        }
        out("DEEP_ADR_VOTED_CAS_MS", s.adr_cas_vote.value);
        out("DEEP_ADR_VOTED_ALT_M", s.adr_alt_vote.value);
        out("DEEP_STANDBY_CAS_MS", s.standby.cas_ms);
        out("DEEP_STANDBY_ALT_M", s.standby.pressure_altitude_m);
        out("DEEP_STANDBY_ADR_DISAGREE", f64::from(s.standby_disagree));
        for i in 0..4 {
            let n = i + 1;
            out(&format!("DEEP_PITOT_{n}_BLOCKED"), f64::from(s.pitot_blocked[i]));
            out(&format!("DEEP_PITOT_{n}_ICE_KG"), s.pitot_ice_kg[i]);
            out(&format!("DEEP_STATIC_{n}_DEGRADED"), f64::from(s.static_degraded[i]));
        }
        for i in 0..2 {
            let n = i + 1;
            out(&format!("DEEP_TAT_{n}_C"), s.tat_c[i]);
            out(&format!("DEEP_ICE_DETECTOR_{n}"), f64::from(s.ice_detected[i]));
            out(&format!("DEEP_ICE_DETECTOR_{n}_ICE_KG"), s.ice_kg[i]);
        }
        for engine in 0..4 {
            for (channel, label) in ["A", "B"].iter().enumerate() {
                out(
                    &format!("DEEP_ENG_{}_N1_PICKUP_{label}_VALID", engine + 1),
                    f64::from(s.n1_pickup_valid[engine][channel]),
                );
                out(
                    &format!("DEEP_ENG_{}_N1_PICKUP_{label}_FRAC", engine + 1),
                    s.n1_pickup_frac[engine][channel],
                );
            }
        }
        out("DEEP_STANDBY_OAT_C", s.standby_oat_c);
        out("DEEP_PROBE_HEAT_TOTAL_W", s.total_heater_power_w);

        for i in 0..2 {
            out(&format!("DEEP_OAT_{}_HEATER_FAILED", i + 1), f64::from(s.oat_1_2_heater_failed[i]));
        }
        for i in 0..3 {
            out(&format!("DEEP_SIDESLIP_{}_HEATER_FAILED", i + 1), f64::from(s.sideslip_heater_failed[i]));
            out(&format!("DEEP_SIDESLIP_{}_JAMMED", i + 1), f64::from(s.sideslip_jammed[i]));
        }
        out("DEEP_TAT_3_HEATER_FAILED", f64::from(s.tat3_heater_failed));
        out("DEEP_TAT_3_RECOVERY_DEGRADED", f64::from(s.tat3_recovery_degraded));
        out("DEEP_ADR_CAPT_FO_ALT_DIFF_FT", s.adr_capt_fo_alt_diff_ft);
        for i in 0..3 {
            out(&format!("DEEP_IR_{}_GYRO_DRIFT_DEG_HR", i + 1), s.ir_gyro_drift_deg_hr[i]);
        }
        out("DEEP_IR_CAPT_FO_PITCH_DIFF_DEG", s.ir_capt_fo_pitch_diff_deg);
        out("DEEP_IR_CAPT_FO_ROLL_DIFF_DEG", s.ir_capt_fo_roll_diff_deg);
        out("DEEP_IR_CAPT_FO_HDG_DIFF_DEG", s.ir_capt_fo_hdg_diff_deg);
        out("DEEP_IR_CAPT_FO_FPA_DIFF_DEG", s.ir_capt_fo_fpa_diff_deg);
        for (side, name) in ["CAPT", "FO"].iter().enumerate() {
            out(&format!("DEEP_KCCU_{name}_CCD_FAILED"), f64::from(s.kccu_part_failed[side][0]));
            out(&format!("DEEP_KCCU_{name}_KEYBOARD_FAILED"), f64::from(s.kccu_part_failed[side][1]));
        }
        for (i, du) in CDS_MONITORED_DUS.iter().enumerate() {
            out(&cds_monitor_var(du), f64::from(s.du_display_monitor_disagree[i]));
        }
        out("DEEP_BARO_REF_DISAGREE", f64::from(s.baro_ref_disagree));

        self.discrete.publish(out);

        self.derived_failures(&mut |d| {
            out(&format!("DEEP_DERIVED_FBW_FAILURE_{}", d.fbw_id), d.magnitude);
        });
    }

    fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
        const RADIO_ALTIMETER: [u64; 3] = [34_000, 34_001, 34_002];
        const RADIO_ANTENNA_INTERRUPTED: [u64; 3] = [34_010, 34_011, 34_012];
        const RADIO_ANTENNA_DIRECT_COUPLING: [u64; 3] = [34_020, 34_021, 34_022];
        const TRANSCEIVER_COMPONENT: [&str; 3] =
            ["34_nav.ra_transceiver_1", "34_nav.ra_transceiver_2", "34_nav.ra_transceiver_3"];
        const ANTENNA_COMPONENT: [&str; 3] =
            ["34_nav.ra_tx_antenna_1", "34_nav.ra_tx_antenna_2", "34_nav.ra_tx_antenna_3"];
        const COUPLING_COMPONENT: [&str; 3] =
            ["34_nav.ra_ant_coupling_1", "34_nav.ra_ant_coupling_2", "34_nav.ra_ant_coupling_3"];
        for i in 0..3 {
            out(DerivedFailure {
                fbw_id: RADIO_ALTIMETER[i],
                magnitude: self.snapshot.ra_transceiver_fault[i],
                deep_component: TRANSCEIVER_COMPONENT[i],
                reason: "radio altimeter transceiver failed",
            });
            out(DerivedFailure {
                fbw_id: RADIO_ANTENNA_INTERRUPTED[i],
                magnitude: self.snapshot.ra_antenna_fault[i],
                deep_component: ANTENNA_COMPONENT[i],
                reason: "radio altimeter antenna path broken",
            });
            out(DerivedFailure {
                fbw_id: RADIO_ANTENNA_DIRECT_COUPLING[i],
                magnitude: self.snapshot.ra_direct_coupling_fault[i],
                deep_component: COUPLING_COMPONENT[i],
                reason: "radio altimeter antenna direct coupling",
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::integration::weather_truth::EnvironmentTruth;

    fn icing_cruise() -> Truth {
        Truth {
            dt_s: 1.0 / 30.0,
            environment: EnvironmentTruth {
                sat_c: -10.0,
                leading_edge_c: -5.0,
                ambient_pressure_pa: 46_500.0,
                tas_ms: 200.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 20_000.0,
            on_ground: false,
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            engine_n1_frac: [0.85; 4],
            engine_running: [true; 4],
            ..Truth::default()
        }
    }

    fn vars(area: &LiveSensors) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(area: &mut LiveSensors, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let ticks = (seconds / truth.dt_s).round().max(1.0) as u32;
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
        vars(area)
    }

    #[test]
    fn every_live_sensor_resolved_a_real_failure_id() {
        let area = LiveSensors::new();
        let mut ids: Vec<u64> = Vec::new();
        for ch in &area.channels {
            ids.extend_from_slice(&ch.pitot_faults);
            ids.extend_from_slice(&ch.static_left_faults);
            ids.extend_from_slice(&ch.static_right_faults);
            ids.push(ch.averaging_line_fault);
        }
        for v in &area.vanes {
            ids.extend_from_slice(&v.faults);
        }
        for t in &area.tat {
            ids.extend_from_slice(&t.faults);
        }
        for d in &area.ice {
            ids.extend_from_slice(&d.faults);
        }
        for r in &area.ra {
            ids.extend_from_slice(&r.transceiver_faults);
            ids.push(r.tx_antenna_fault);
            ids.extend_from_slice(&r.rx_antenna_faults);
        }
        for g in &area.gps {
            ids.extend_from_slice(&g.receiver_faults);
            ids.extend_from_slice(&g.antenna_faults);
        }
        for engine in &area.n1_pickup_faults {
            for channel in engine {
                ids.extend_from_slice(channel);
            }
        }
        ids.extend_from_slice(&area.oat_faults);
        assert!(ids.iter().all(|&id| id != 0), "a live sensor has no catalogue failure behind it");
        let mut sorted = ids.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), ids.len(), "two live sensors resolved to the same failure id");
    }

    #[test]
    fn a_healthy_aircraft_agrees_with_itself_and_publishes_real_air_data() {
        let mut area = LiveSensors::new();
        let truth = icing_cruise();
        let published = run(&mut area, &truth, &Faults::default(), 30.0);

        assert_eq!(published["DEEP_ADR_VOTE_DISAGREE"], 0.0);
        let mach = truth.environment.mach();
        assert!((published["DEEP_ADR_1_MACH"] - mach).abs() < 0.02, "{} vs {mach}", published["DEEP_ADR_1_MACH"]);
        for i in 1..=3 {
            assert!(published[&format!("DEEP_ADR_{i}_CAS_MS")] > 100.0);
            assert_eq!(published[&format!("DEEP_GPS_{i}_VALID")], 1.0);
        }
        for (name, value) in &published {
            assert!(value.is_finite(), "{name} is not finite");
        }
        for i in 1..=3 {
            assert_eq!(published[&format!("DEEP_ADR_{i}_CAS_ERROR_MS")], 0.0);
            assert_eq!(published[&format!("DEEP_ADR_{i}_ALT_ERROR_M")], 0.0);
            let sat_error = published[&format!("DEEP_ADR_{i}_SAT_ERROR_C")];
            assert!(
                sat_error > 0.0 && sat_error < 2.0,
                "a healthy, powered TAT probe's own self-heating (tat_probe.rs's self_heating_error_k, exercised by self_heating_error_is_larger_at_low_tas_than_high_tas) biases derived SAT warm by a known, bounded amount at this probe's 200 m/s cruise TAS -- it is not a fault and should not be exactly 0, got {sat_error}"
            );
            assert_eq!(published[&format!("DEEP_AOA_{i}_ERROR_DEG")], 0.0);
        }
    }

    #[test]
    fn an_aoa_vane_damage_moves_only_its_own_channels_error() {
        let mut area = LiveSensors::new();
        let index = FaultIndex::build();
        let id = index.id("34_nav.aoa_2", "damage");
        assert_ne!(id, 0);
        let truth = icing_cruise();
        let published = run(&mut area, &truth, &Faults::from_pairs([(id, 1.0)]), 5.0);

        assert!(
            (published["DEEP_AOA_2_ERROR_DEG"] - 8.0).abs() < 0.05,
            "a bent vane must move its own channel's published AoA error by its full bias: {}",
            published["DEEP_AOA_2_ERROR_DEG"]
        );
        assert_eq!(published["DEEP_AOA_1_ERROR_DEG"], 0.0, "ADIRU 1's vane is its own, healthy, one");
        assert_eq!(published["DEEP_AOA_3_ERROR_DEG"], 0.0, "ADIRU 3's vane is its own, healthy, one");
    }

    #[test]
    fn a_blocked_pitot_moves_only_its_own_channels_cas_error() {
        let mut area = LiveSensors::new();
        let index = FaultIndex::build();
        let id = index.id("34_nav.pitot_2", "mechanical_damage");
        assert_ne!(id, 0);
        let truth = icing_cruise();
        let published = run(&mut area, &truth, &Faults::from_pairs([(id, 1.0)]), 20.0);

        assert!(
            published["DEEP_ADR_2_CAS_ERROR_MS"].abs() > 1.0,
            "a blocked pitot must move its own ADR's published CAS error: {}",
            published["DEEP_ADR_2_CAS_ERROR_MS"]
        );
        assert_eq!(published["DEEP_ADR_1_CAS_ERROR_MS"], 0.0, "ADR 1 has its own, healthy, probe");
        assert_eq!(published["DEEP_ADR_3_CAS_ERROR_MS"], 0.0, "ADR 3 has its own, healthy, probe");
    }

    #[test]
    fn an_armed_pitot_heater_failure_raises_its_own_published_flag_and_ices_the_probe() {
        let mut area = LiveSensors::new();
        let index = FaultIndex::build();
        let id = index.id("34_nav.pitot_2", "heater_failure");
        assert_ne!(id, 0);

        let truth = Truth {
            environment: EnvironmentTruth {
                weather: None,
                ..icing_cruise().environment
            },
            ..icing_cruise()
        };
        let published = run(&mut area, &truth, &Faults::from_pairs([(id, 1.0)]), 10.0);

        assert_eq!(published["DEEP_PITOT_2_HEATER_FAILED"], 1.0, "the alert could never fire");
        assert_eq!(published["DEEP_PITOT_1_HEATER_FAILED"], 0.0, "it took out the wrong probe");
        assert_eq!(published["DEEP_PITOT_3_HEATER_FAILED"], 0.0);
        assert_eq!(published["DEEP_PITOT_4_HEATER_FAILED"], 0.0);
    }

    #[test]
    fn losing_one_ac_bus_fails_only_that_systems_probe_heaters() {
        let mut area = LiveSensors::new();
        let mut truth = icing_cruise();
        truth.ac_bus_volts = [115.0, 0.0, 115.0, 115.0];
        let published = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(published["DEEP_PITOT_2_HEATER_FAILED"], 0.0);
        assert!(published["DEEP_PITOT_2_ICE_KG"] >= published["DEEP_PITOT_1_ICE_KG"]);
    }

    #[test]
    fn blocking_one_systems_static_ports_makes_the_published_adr_voter_disagree() {
        let index = FaultIndex::build();
        let left = index.id("34_nav.static_3_1", "blocked");
        let right = index.id("34_nav.static_3_2", "blocked");

        let mut area = LiveSensors::new();
        let mut truth = icing_cruise();
        run(&mut area, &truth, &Faults::default(), 20.0);
        let faults = Faults::from_pairs([(left, 1.0), (right, 1.0)]);
        area.tick(&truth, &faults);

        truth.environment.ambient_pressure_pa = 30_000.0;
        truth.altitude_ft = 30_000.0;
        let published = run(&mut area, &truth, &faults, 30.0);

        assert_eq!(published["DEEP_ADR_VOTE_DISAGREE"], 1.0, "the blocked system voted with the others");
        assert_eq!(published["DEEP_ADR_3_OUTLIER"], 1.0, "the wrong channel was flagged");
        assert!(
            published["DEEP_ADR_3_ALT_M"] < published["DEEP_ADR_1_ALT_M"] - 1_000.0,
            "the frozen port kept reporting the climb"
        );
    }

    #[test]
    fn an_armed_aoa_vane_seizure_raises_its_own_published_jam_flag() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.aoa_2", "mechanically_stuck");
        let mut area = LiveSensors::new();
        let published = run(&mut area, &icing_cruise(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(published["DEEP_AOA_2_JAMMED"], 1.0);
        assert_eq!(published["DEEP_AOA_1_JAMMED"], 0.0);
        assert_eq!(published["DEEP_AOA_3_JAMMED"], 0.0);
    }

    #[test]
    fn an_armed_ra_transceiver_fault_invalidates_only_that_unit() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.ra_transceiver_1", "transceiver_fault");
        let mut area = LiveSensors::new();
        let truth = Truth { altitude_ft: 1_000.0, on_ground: false, ..icing_cruise() };

        let healthy = run(&mut area, &truth, &Faults::default(), 2.0);
        assert_eq!(healthy["DEEP_RA_1_VALID"], 1.0);
        assert_eq!(healthy["DEEP_RA_1_IN_RANGE"], 1.0);

        let mut failed_area = LiveSensors::new();
        let published = run(&mut failed_area, &truth, &Faults::from_pairs([(id, 1.0)]), 2.0);
        assert_eq!(published["DEEP_RA_1_VALID"], 0.0);
        assert_eq!(published["DEEP_RA_2_VALID"], 1.0);
        assert_eq!(published["DEEP_RA_3_VALID"], 1.0);
    }

    #[test]
    fn armed_gps_jamming_costs_that_receiver_its_fix() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.gps_receiver_1", "jamming");
        let mut area = LiveSensors::new();
        let truth = icing_cruise();

        let mild = run(&mut area, &truth, &Faults::from_pairs([(id, 0.3)]), 2.0);
        assert_eq!(mild["DEEP_GPS_1_VALID"], 1.0, "mild jamming must not drop the fix outright");

        let mut area = LiveSensors::new();
        let heavy = run(&mut area, &truth, &Faults::from_pairs([(id, 0.9)]), 2.0);
        assert_eq!(heavy["DEEP_GPS_1_VALID"], 0.0);
        assert_eq!(heavy["DEEP_GPS_2_VALID"], 1.0);
        assert!(mild["DEEP_GPS_1_ERROR_M"] > 0.0);
    }

    #[test]
    fn an_armed_speed_pickup_open_circuit_invalidates_one_eec_channel_only() {
        let index = FaultIndex::build();
        let id = index.id("77_eng.speed_n1_3_b", "open_circuit");
        let mut area = LiveSensors::new();
        let published = run(&mut area, &icing_cruise(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(published["DEEP_ENG_3_N1_PICKUP_B_VALID"], 0.0);
        assert_eq!(published["DEEP_ENG_3_N1_PICKUP_A_VALID"], 1.0);
        assert_eq!(published["DEEP_ENG_4_N1_PICKUP_B_VALID"], 1.0);
        assert!((published["DEEP_ENG_3_N1_PICKUP_A_FRAC"] - 0.85).abs() < 1e-9);
    }

    #[test]
    fn an_armed_ice_detector_probe_damage_false_triggers_in_clear_warm_air() {
        let index = FaultIndex::build();
        let id = index.id("30_ice.detector_1", "probe_damage_bias");
        let mut area = LiveSensors::new();
        let truth = Truth {
            environment: EnvironmentTruth { sat_c: 20.0, ..Truth::default().environment },
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        };
        let healthy = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(healthy["DEEP_ICE_DETECTOR_1"], 0.0);

        let mut damaged = LiveSensors::new();
        let published = run(&mut damaged, &truth, &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(published["DEEP_ICE_DETECTOR_1"], 1.0);
        assert_eq!(published["DEEP_ICE_DETECTOR_2"], 0.0);
        assert_eq!(published["DEEP_ICE_DETECTOR_1_ICE_KG"], 0.0, "it must be a false alarm, not real ice");
    }

    #[test]
    fn an_armed_standby_oat_open_circuit_pegs_the_published_reading() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.oat_standby", "open_circuit");
        let truth = icing_cruise();

        let mut healthy = LiveSensors::new();
        let published = run(&mut healthy, &truth, &Faults::default(), 1.0);
        assert!((published["DEEP_STANDBY_OAT_C"] - truth.environment.sat_c).abs() < 0.5);

        let mut open = LiveSensors::new();
        let published = run(&mut open, &truth, &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(published["DEEP_STANDBY_OAT_C"], OAT_RANGE_C.1);
    }

    #[test]
    fn every_variable_an_ecam_trigger_names_is_published() {
        let mut area = LiveSensors::new();
        area.tick(&icing_cruise(), &Faults::default());
        let published = vars(&area);
        let mut required = vec![
            "DEEP_ADR_VOTE_DISAGREE".to_string(),
            "DEEP_RA_1_VALID".to_string(),
            "DEEP_GPS_1_VALID".to_string(),
        ];
        required.extend((1..=4).map(|i| format!("DEEP_PITOT_{i}_HEATER_FAILED")));
        required.extend((1..=3).map(|i| format!("DEEP_AOA_{i}_JAMMED")));
        required.extend((1..=2).map(|i| format!("DEEP_TAT_{i}_HEATER_FAILED")));
        for name in required {
            assert!(published.contains_key(&name), "{name} is never published");
        }
        assert_eq!(area.name(), "sensors");
    }

    #[test]
    fn a_cold_dark_aircraft_on_the_ground_produces_nothing_but_finite_numbers() {
        let mut area = LiveSensors::new();
        let published = run(&mut area, &Truth::default(), &Faults::default(), 60.0);
        for (name, value) in &published {
            assert!(value.is_finite(), "{name} = {value}");
        }
        assert_eq!(published["DEEP_ADR_VOTE_DISAGREE"], 0.0);
        assert!(published["DEEP_ADR_1_CAS_MS"] < 1.0);
        assert!(published["DEEP_ADR_1_ALT_M"].abs() < 5.0);
    }
}

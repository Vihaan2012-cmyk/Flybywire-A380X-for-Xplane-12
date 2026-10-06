use crate::deep::api::*;

struct Counter(u16);
impl Counter {
    fn next(&mut self) -> u16 {
        self.0 += 1;
        self.0
    }
}

struct FaultSpec {
    field: &'static str,
    name: &'static str,
    magnitude: &'static str,
    effect: &'static str,
    healthy: f64,
    meaning: &'static str,
}

fn register_failures_only(r: &mut Registry, counter: &mut Counter, ata: u16, id: &str, name: &str, model_path: &str, faults: &[FaultSpec]) -> Vec<u64> {
    let mut ids = Vec::with_capacity(faults.len());
    for f in faults {
        let fid = failure_id(Area::Sensors, ata, counter.next());
        r.failure(FailureDef {
            id: fid,
            area: Area::Sensors,
            ata,
            name: format!("{name}: {}", f.name),
            component: id.to_string(),
            model_field: format!("{model_path}(faults.{})", f.field),
            magnitude: f.magnitude.to_string(),
            effect: f.effect.to_string(),
        });
        ids.push(fid);
    }
    ids
}

fn register_instance(
    r: &mut Registry,
    counter: &mut Counter,
    ata: u16,
    id: &str,
    name: &str,
    model_path: &str,
    faults: &[FaultSpec],
    extra_params: &[ParamDef],
) -> Vec<u64> {
    let ids = register_failures_only(r, counter, ata, id, name, model_path, faults);
    let mut params: Vec<ParamDef> = faults
        .iter()
        .map(|f| ParamDef { name: f.field.to_string(), meaning: f.meaning.to_string(), healthy: f.healthy })
        .collect();
    params.extend(extra_params.iter().cloned());
    r.component(ComponentDef { id: id.to_string(), area: Area::Sensors, ata, name: name.to_string(), params, failures: ids.clone() });
    ids
}

pub fn register(r: &mut Registry) {
    let mut c34 = Counter(0);
    let mut c30 = Counter(0);
    let mut c32 = Counter(0);
    let mut c52 = Counter(0);
    let mut c29 = Counter(0);
    let mut c77 = Counter(0);
    let mut c73 = Counter(0);
    let mut c79 = Counter(0);
    let mut c26 = Counter(0);
    let mut c36 = Counter(0);
    let mut c35 = Counter(0);
    let mut c21 = Counter(0);
    let mut c31 = Counter(0);

    let pitot_heater_ids = register_pitot(r, &mut c34);
    let static_blocked_ids = register_static_port(r, &mut c34);
    let aoa_jam_ids = register_aoa_vane(r, &mut c34);
    let tat_heater_ids = register_tat_probe(r, &mut c34);
    register_ice_detector(r, &mut c30);
    let (ra_transceiver_ids, _ra_tx_ids, _ra_rx_ids) = register_radio_altimeter(r, &mut c34);
    let (gps_receiver_ids, gps_antenna_ids) = register_gps(r, &mut c34);
    register_gear_proximity(r, &mut c32);
    register_door_proximity(r, &mut c52);
    register_hydraulic_sensors(r, &mut c29);
    register_engine_speed_pickups(r, &mut c77);
    register_engine_tgt_harness(r, &mut c77);
    register_engine_vibration(r, &mut c77);
    register_engine_p30_t25(r, &mut c77);
    register_engine_fuel_flow(r, &mut c73);
    register_engine_oil_sensors(r, &mut c79);
    register_brake_temperature(r, &mut c32);
    register_brake_wear(r, &mut c32);
    register_tyre_pressure(r, &mut c32);
    register_smoke_detectors(r, &mut c26);
    register_duct_temperature(r, &mut c36);
    register_oxygen_sensors(r, &mut c35);
    register_cabin_pressure_sensors(r, &mut c21);
    register_oat_probe(r, &mut c34);
    register_static_averaging_lines(r, &mut c34);
    register_oat_probe_1_2(r, &mut c34);
    register_sideslip_vane(r, &mut c34);
    register_tat_probe_3(r, &mut c34);
    register_inertial_reference(r, &mut c34);
    register_kccu_parts(r, &mut c31);
    register_cds_display_monitors(r, &mut c31);
    register_radio_altimeter_direct_coupling(r, &mut c34);

    register_ecam(r, &pitot_heater_ids, &static_blocked_ids, &aoa_jam_ids, &tat_heater_ids, &ra_transceiver_ids, &gps_receiver_ids, &gps_antenna_ids);
}

fn register_pitot(r: &mut Registry, c: &mut Counter) -> Vec<u64> {
    let faults = [
        FaultSpec { field: "heater_failure", name: "heater failure", magnitude: "0 healthy .. 1 no heat", effect: "ice accretes in icing conditions, eventually blocking the tube", healthy: 0.0, meaning: "Heater power loss fraction" },
        FaultSpec { field: "insect_or_tape_blockage", name: "insect/tape blockage", magnitude: "0 clear .. 1 sealed", effect: "pneumatic lag grows below ~97% open area; full blockage above it", healthy: 0.0, meaning: "Fixed non-ice open-area restriction" },
        FaultSpec { field: "mechanical_damage", name: "mechanical damage", magnitude: "0 clear .. 1 sealed", effect: "same as insect/tape blockage, separate cause", healthy: 0.0, meaning: "Bent/crushed-tube open-area restriction" },
        FaultSpec { field: "drain_blocked", name: "drain hole blocked", magnitude: "0 clear .. 1 sealed", effect: "with tube blocked: clear drain decays to static (airspeed sags to zero), blocked drain freezes (airspeed then tracks altitude inversely)", healthy: 0.0, meaning: "Drain conductance loss" },
    ];
    let names = ["ADR 1", "ADR 2", "ADR 3", "Standby"];
    let mut heater_ids = Vec::new();
    for (i, n) in names.iter().enumerate() {
        let id = format!("34_nav.pitot_{}", i + 1);
        let name = format!("Pitot probe ({n})");
        let ids = register_instance(r, c, 34, &id, &name, "pitot::PitotProbe.step", &faults, &[]);
        heater_ids.push(ids[0]);
    }
    heater_ids
}

fn register_static_port(r: &mut Registry, c: &mut Counter) -> Vec<u64> {
    let faults = [
        FaultSpec { field: "blocked", name: "blocked", magnitude: "0 clear .. 1 (>=0.98 sealed)", effect: "sensed static pressure freezes; altitude/airspeed stop tracking reality", healthy: 0.0, meaning: "Port sealed fraction (ice/tape/debris)" },
        FaultSpec { field: "leak_to_cabin", name: "leak into pressurised fuselage", magnitude: "0 none .. >1 leak dominates", effect: "sensed static pressure biased toward cabin pressure", healthy: 0.0, meaning: "Leak conductance as a multiple of the port's own" },
    ];
    let systems = ["ADR 1", "ADR 2", "ADR 3", "Standby"];
    let sides = ["Left", "Right"];
    let mut blocked_ids = Vec::new();
    for (i, sys) in systems.iter().enumerate() {
        for (j, side) in sides.iter().enumerate() {
            let id = format!("34_nav.static_{}_{}", i + 1, j + 1);
            let name = format!("Static port ({sys} {side})");
            let ids = register_instance(r, c, 34, &id, &name, "static_port::StaticPort.step", &faults, &[]);
            blocked_ids.push(ids[0]);
        }
    }
    blocked_ids
}

fn register_aoa_vane(r: &mut Registry, c: &mut Counter) -> Vec<u64> {
    let faults = [
        FaultSpec { field: "heater_failure", name: "heater failure", magnitude: "0 healthy .. 1 no heat", effect: "vane ices and jams in icing conditions (see mechanically_stuck)", healthy: 0.0, meaning: "Heater power loss fraction" },
        FaultSpec { field: "mechanically_stuck", name: "mechanically stuck", magnitude: "0 free .. 1 (>=0.98 seized)", effect: "reported AoA frozen at last free angle", healthy: 0.0, meaning: "Hinge/bearing seizure fraction" },
        FaultSpec { field: "resolver_wear", name: "resolver wear/drift", magnitude: "0 none .. 1 max drift rate", effect: "slow random-walk bias growth in reported AoA", healthy: 0.0, meaning: "Resolver wear scaling the bias drift rate" },
        FaultSpec { field: "damage", name: "damage (bent vane)", magnitude: "0 none .. 1 (8 deg max)", effect: "fixed offset error from the moment of damage", healthy: 0.0, meaning: "Bend fraction of the max modelled bias" },
    ];
    let names = ["ADIRU 1", "ADIRU 2", "ADIRU 3"];
    let mut jam_related_ids = Vec::new();
    for (i, n) in names.iter().enumerate() {
        let id = format!("34_nav.aoa_{}", i + 1);
        let name = format!("AoA vane ({n})");
        let ids = register_instance(r, c, 34, &id, &name, "aoa_vane::AoaVane.step", &faults, &[]);
        jam_related_ids.push(ids[0]);
        jam_related_ids.push(ids[1]);
    }
    jam_related_ids
}

fn register_tat_probe(r: &mut Registry, c: &mut Counter) -> Vec<u64> {
    let faults = [
        FaultSpec { field: "heater_failure", name: "heater failure", magnitude: "0 healthy .. 1 no heat", effect: "icing grows the element's thermal time constant, slowing/biasing the reading", healthy: 0.0, meaning: "Heater power loss fraction" },
        FaultSpec { field: "recovery_degradation", name: "recovery factor degradation", magnitude: "0 (r=0.99) .. 1 (r=0)", effect: "reported TAT reads low relative to true recovery temperature", healthy: 0.0, meaning: "Recovery-factor loss fraction" },
    ];
    let names = ["Captain", "First Officer"];
    let mut heater_ids = Vec::new();
    for (i, n) in names.iter().enumerate() {
        let id = format!("34_nav.tat_{}", i + 1);
        let name = format!("TAT probe ({n})");
        let ids = register_instance(r, c, 34, &id, &name, "tat_probe::TatProbe.step", &faults, &[]);
        heater_ids.push(ids[0]);
    }
    heater_ids
}

fn register_oat_probe_1_2(r: &mut Registry, c: &mut Counter) -> Vec<u64> {
    let faults = [FaultSpec { field: "heater_failure", name: "heater failure", magnitude: "0 healthy .. 1 no heat", effect: "probe ices over and its reading is no longer trustworthy", healthy: 0.0, meaning: "Heater power loss fraction" }];
    let names = ["1", "2"];
    let mut ids = Vec::new();
    for n in names {
        let id = format!("34_nav.oat_{n}");
        let name = format!("OAT probe {n}");
        ids.push(register_instance(r, c, 34, &id, &name, "oat_probe.heater_failure", &faults, &[])[0]);
    }
    ids
}

fn register_sideslip_vane(r: &mut Registry, c: &mut Counter) -> Vec<u64> {
    let faults = [
        FaultSpec { field: "heater_failure", name: "heater failure", magnitude: "0 healthy .. 1 no heat", effect: "vane ices and jams in icing conditions", healthy: 0.0, meaning: "Heater power loss fraction" },
        FaultSpec { field: "mechanically_stuck", name: "mechanically stuck", magnitude: "0 free .. 1 seized", effect: "reported sideslip frozen at last free angle", healthy: 0.0, meaning: "Hinge/bearing seizure fraction" },
    ];
    let mut ids = Vec::new();
    for n in 1..=3 {
        let id = format!("34_nav.sideslip_{n}");
        let name = format!("Sideslip vane {n}");
        let unit_ids = register_instance(r, c, 34, &id, &name, "sideslip_vane.jam_or_heater", &faults, &[]);
        ids.push(unit_ids[0]);
        ids.push(unit_ids[1]);
    }
    ids
}

fn register_tat_probe_3(r: &mut Registry, c: &mut Counter) -> u64 {
    let faults = [
        FaultSpec { field: "heater_failure", name: "heater failure", magnitude: "0 healthy .. 1 no heat", effect: "icing grows the element's thermal time constant, slowing/biasing the reading", healthy: 0.0, meaning: "Heater power loss fraction" },
        FaultSpec { field: "recovery_degradation", name: "recovery factor degradation", magnitude: "0 (r=0.99) .. 1 (r=0)", effect: "reported TAT reads low relative to true recovery temperature", healthy: 0.0, meaning: "Recovery-factor loss fraction" },
    ];
    let ids = register_instance(r, c, 34, "34_nav.tat_3", "TAT probe 3", "tat_probe::TatProbe.step", &faults, &[]);
    ids[0]
}

fn register_inertial_reference(r: &mut Registry, c: &mut Counter) -> Vec<u64> {
    let faults = [FaultSpec {
        field: "alignment_drift",
        name: "gyro drift",
        magnitude: "0 healthy .. 1 = 100 deg/hr added to all three of this IR's gyros (GENERIC, see sensors::live::IR_GYRO_DRIFT_AT_FULL_FAULT_DEG_HR)",
        effect: "this IR's strapdown attitude, heading and flight path angle drift away from the other two; past the FCOM thresholds the CAPT/F.O ATT, HDG or HUD FPV disagree alerts fire",
        healthy: 0.0,
        meaning: "Gyro drift severity",
    }];
    let names = ["1 (CAPT)", "2 (F.O)", "3 (standby)"];
    let mut ids = Vec::new();
    for (i, n) in names.iter().enumerate() {
        let id = format!("34_nav.ir_{}", i + 1);
        let name = format!("Inertial Reference {n}");
        ids.push(register_instance(r, c, 34, &id, &name, "inertial_reference.alignment_drift", &faults, &[])[0]);
    }
    ids
}

fn register_kccu_parts(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "ccd_failed", name: "cursor control device failed", magnitude: "0 healthy .. >0 failed (BITE)", effect: "the KCCU reports its trackball/validation part failed: CDS CURSOR CTL FAULT", healthy: 0.0, meaning: "Cursor control device failure" },
        FaultSpec { field: "keyboard_failed", name: "keyboard failed", magnitude: "0 healthy .. >0 failed (BITE)", effect: "the KCCU reports its keyboard failed: CDS KEYBOARD FAULT", healthy: 0.0, meaning: "Keyboard failure" },
    ];
    for (side, name) in [("capt", "CAPT KCCU"), ("fo", "F/O KCCU")] {
        let id = format!("31_elec.kccu-{side}");
        let ids = register_failures_only(r, c, 31, &id, name, "sensors::live::LiveSensors (KCCU BITE)", &faults);
        let params: Vec<ParamDef> = faults.iter().map(|f| ParamDef { name: f.field.to_string(), meaning: f.meaning.to_string(), healthy: f.healthy }).collect();
        r.extend_component(&id).params(&params).failures(&ids);
    }
}

fn register_cds_display_monitors(r: &mut Registry, c: &mut Counter) {
    let faults = [FaultSpec {
        field: "display_monitor_disagree",
        name: "displayed image disagrees with its monitor",
        magnitude: "0 healthy .. >0 failed",
        effect: "the CDS's display/monitor comparison fails for this unit: CDS DISPLAY DISAGREE",
        healthy: 0.0,
        meaning: "Display/monitor discrepancy",
    }];
    for du in super::live::CDS_MONITORED_DUS {
        let id = format!("31_elec.{du}");
        let name = du.to_ascii_uppercase().replace('-', " ");
        let ids = register_failures_only(r, c, 31, &id, &name, "sensors::live::LiveSensors (CDS display monitor)", &faults);
        let params: Vec<ParamDef> = faults.iter().map(|f| ParamDef { name: f.field.to_string(), meaning: f.meaning.to_string(), healthy: f.healthy }).collect();
        r.extend_component(&id).params(&params).failures(&ids);
    }
}

fn register_radio_altimeter(r: &mut Registry, c: &mut Counter) -> (Vec<u64>, Vec<u64>, Vec<u64>) {
    let (mut transceiver_ids, mut tx_ids, mut rx_ids) = (Vec::new(), Vec::new(), Vec::new());
    for n in 1..=3u16 {
        let transceiver_faults = [
            FaultSpec { field: "transceiver_fault", name: "transceiver electronics fault", magnitude: "0 healthy .. 1 (>=0.98 failed)", effect: "no valid height output at all", healthy: 0.0, meaning: "Transceiver failure fraction" },
            FaultSpec { field: "false_offset_ft", name: "false fixed-offset reading", magnitude: "signed ft, not 0..1", effect: "constant height bias at every true height (e.g. the historically documented -6 ft ground reading)", healthy: 0.0, meaning: "Installation/near-field calibration offset, ft" },
            FaultSpec { field: "tracking_loop_degradation", name: "tracking-loop filter degradation", magnitude: "0 baseline .. 1+ extra", effect: "height reading jitters more than the terrain alone would cause", healthy: 0.0, meaning: "Extra multipath susceptibility from the receiver's own filter" },
        ];
        let id = format!("34_nav.ra_transceiver_{n}");
        let name = format!("Radio altimeter transceiver ({n})");
        let ids = register_instance(r, c, 34, &id, &name, "radio_altimeter::RadioAltimeter.step", &transceiver_faults, &[]);
        transceiver_ids.push(ids[0]);

        let tx_faults = [FaultSpec { field: "tx_antenna_fault", name: "transmit antenna fault", magnitude: "0 healthy .. 1 (>=0.98 failed)", effect: "no valid height output at all", healthy: 0.0, meaning: "Transmit antenna failure fraction" }];
        let tx_id = format!("34_nav.ra_tx_antenna_{n}");
        let tx_name = format!("Radio altimeter transmit antenna ({n})");
        let ids = register_instance(r, c, 34, &tx_id, &tx_name, "radio_altimeter::RadioAltimeter.step", &tx_faults, &[]);
        tx_ids.push(ids[0]);

        let rx_faults = [
            FaultSpec { field: "rx_antenna_fault", name: "receive antenna fault", magnitude: "0 healthy .. 1 (>=0.98 failed)", effect: "no valid height output at all", healthy: 0.0, meaning: "Receive antenna failure fraction" },
            FaultSpec { field: "rx_antenna_degradation", name: "receive antenna gain loss", magnitude: "0 healthy .. 1 as severe as tracking-loop degradation", effect: "height reading jitters more (reduced SNR), stays valid", healthy: 0.0, meaning: "Antenna gain loss short of outright failure" },
        ];
        let rx_id = format!("34_nav.ra_rx_antenna_{n}");
        let rx_name = format!("Radio altimeter receive antenna ({n})");
        let ids = register_instance(r, c, 34, &rx_id, &rx_name, "radio_altimeter::RadioAltimeter.step", &rx_faults, &[]);
        rx_ids.push(ids[0]);
    }
    (transceiver_ids, tx_ids, rx_ids)
}

fn register_radio_altimeter_direct_coupling(r: &mut Registry, c: &mut Counter) {
    for n in 1..=3u16 {
        let coupling_faults = [FaultSpec { field: "direct_coupling_fault", name: "antenna direct coupling", magnitude: "0 healthy .. 1 (>=0.5 coupled)", effect: "the receiver locks onto the direct TX-to-RX leakage path instead of the ground reflection: an erroneous low/unstable height is output, flagged invalid by the RA's own monitor whenever it disagrees with a plausible height (i.e. away from the ground)", healthy: 0.0, meaning: "Direct antenna-to-antenna coupling fraction" }];
        let coupling_id = format!("34_nav.ra_ant_coupling_{n}");
        let coupling_name = format!("Radio altimeter antenna direct coupling ({n})");
        register_instance(r, c, 34, &coupling_id, &coupling_name, "radio_altimeter::RadioAltimeter.step", &coupling_faults, &[]);
    }
}

fn register_gps(r: &mut Registry, c: &mut Counter) -> (Vec<u64>, Vec<u64>) {
    let (mut receiver_ids, mut antenna_ids) = (Vec::new(), Vec::new());
    for n in 1..=3u16 {
        let receiver_faults = [
            FaultSpec { field: "receiver_fault", name: "receiver electronics fault", magnitude: "0 healthy .. 1 (>=0.98 no fix)", effect: "no GPS position at all", healthy: 0.0, meaning: "Receiver hardware/processing failure fraction" },
            FaultSpec { field: "jamming", name: "RF jamming", magnitude: "0 none .. 1 all satellites unusable", effect: "effective satellites fall (weakest first), position error grows, fix lost below 4", healthy: 0.0, meaning: "Jamming strength" },
            FaultSpec { field: "spoof_target_offset_m", name: "spoofing", magnitude: "commanded offset (m), not 0..1", effect: "position walks off gradually toward the spoofed target", healthy: 0.0, meaning: "Commanded spoof target north/east offset" },
        ];
        let id = format!("34_nav.gps_receiver_{n}");
        let name = format!("GPS receiver / MMR ({n})");
        let ids = register_instance(r, c, 34, &id, &name, "gps::GpsReceiver.step", &receiver_faults, &[]);
        receiver_ids.push(ids[0]);

        let antenna_faults = [
            FaultSpec { field: "antenna_fault", name: "antenna fault", magnitude: "0 healthy .. 1 (>=0.98 no fix)", effect: "no GPS position at all", healthy: 0.0, meaning: "Antenna failure fraction" },
            FaultSpec { field: "antenna_degradation", name: "antenna gain loss", magnitude: "0 none .. 1 as severe as full jamming", effect: "acts like jamming on the effective satellite count", healthy: 0.0, meaning: "Antenna gain loss short of outright failure" },
        ];
        let aid = format!("34_nav.gps_antenna_{n}");
        let aname = format!("GPS antenna ({n})");
        let ids = register_instance(r, c, 34, &aid, &aname, "gps::GpsReceiver.step", &antenna_faults, &[]);
        antenna_ids.push(ids[0]);
    }
    (receiver_ids, antenna_ids)
}

fn register_ice_detector(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "heater_failure", name: "deice heater failure", magnitude: "0 healthy .. 1 no deice", effect: "ICE DETECTED latches instead of cycling; ice keeps accumulating", healthy: 0.0, meaning: "Deice heater power loss fraction" },
        FaultSpec { field: "frequency_sensor_bias", name: "frequency sensor drift", magnitude: "signed fractional shift, not 0..1", effect: "can mask real icing (false negative) or fabricate a shift (false positive)", healthy: 0.0, meaning: "Signed bias on the sensed resonant-frequency shift" },
        FaultSpec { field: "probe_damage_bias", name: "probe damage", magnitude: "signed fractional shift, not 0..1", effect: "offsets the calibrated baseline, can false-trigger with no ice present", healthy: 0.0, meaning: "Baseline offset from physical probe damage" },
    ];
    for n in 1..=2 {
        let id = format!("30_ice.detector_{n}");
        let name = format!("Ice detector ({n})");
        register_instance(r, c, 30, &id, &name, "ice_detector::IceDetector.step", &faults, &[]);
    }
}

fn register_gear_proximity(r: &mut Registry, c: &mut Counter) {
    let legs = ["Nose", "Left Body", "Right Body", "Left Wing", "Right Wing"];
    let roles = ["Uplock", "Downlock", "WOW"];
    let faults = [
        FaultSpec { field: "gap_error_mm", name: "gap out of rigging", magnitude: "signed mm, not 0..1", effect: "near/far switch point shifts; wrong position indication without a hard stuck fault", healthy: 0.0, meaning: "Rigging/wear error on the nominal sensing gap" },
        FaultSpec { field: "stuck_near", name: "stuck near", magnitude: "0 healthy .. 1 (>=0.5 stuck)", effect: "always reports target present regardless of true position", healthy: 0.0, meaning: "Stuck-near fraction" },
        FaultSpec { field: "stuck_far", name: "stuck far", magnitude: "0 healthy .. 1 (>=0.5 stuck)", effect: "always reports target absent regardless of true position", healthy: 0.0, meaning: "Stuck-far fraction" },
    ];
    for leg in legs {
        for role in roles {
            let id = format!("32_gear.prox_{}_{}", leg.to_lowercase().replace(' ', "_"), role.to_lowercase());
            let name = format!("{leg} gear {role} proximity sensor");
            register_instance(r, c, 32, &id, &name, "discrete::ProximitySensor.sense", &faults, &[]);
        }
    }
}

fn register_brake_temperature(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 fully open", effect: "reading pegs to top of indicating range", healthy: 0.0, meaning: "Open-circuit fraction" },
        FaultSpec { field: "short_circuit", name: "short circuit", magnitude: "0 healthy .. 1 fully shorted", effect: "reading pegs to bottom of indicating range", healthy: 0.0, meaning: "Short-circuit fraction" },
    ];
    for wheel in braked_wheels() {
        let id = format!("32_gear.brake_temp_{}", wheel.to_lowercase().replace(' ', "_"));
        let name = format!("Brake temperature sensor ({wheel})");
        register_instance(r, c, 32, &id, &name, "discrete::temperature_sensor_reading_c", &faults, &[]);
    }
}

pub(super) fn braked_wheels() -> Vec<String> {
    let mut wheels = Vec::new();
    for leg in ["Left Wing", "Right Wing", "Left Body", "Right Body"] {
        for i in 1..=4 {
            wheels.push(format!("{leg} {i}"));
        }
    }
    wheels
}

fn register_brake_wear(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "pin_binding", name: "pin/sensor binding", magnitude: "0 free .. 1 fully seized", effect: "indicated remaining life stops tracking real wear (freezes at full seizure), overstating remaining brake life while the real stack keeps wearing -- the dangerous indicated-vs-real divergence", healthy: 0.0, meaning: "Mechanical binding fraction" },
        FaultSpec { field: "sender_bias", name: "sender bias", magnitude: "signed fraction of full scale, not 0..1", effect: "constant offset in either direction: optimistic bias overstates remaining life, pessimistic bias triggers early replacement", healthy: 0.0, meaning: "Signed calibration bias" },
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 (>=0.98 open)", effect: "indicated remaining life reads a conservative zero", healthy: 0.0, meaning: "Open-circuit fraction" },
    ];
    for wheel in braked_wheels() {
        let id = format!("32_gear.brake_wear_{}", wheel.to_lowercase().replace(' ', "_"));
        let name = format!("Brake wear indicator ({wheel})");
        register_instance(r, c, 32, &id, &name, "brake_wear::BrakeWearPin.step", &faults, &[]);
    }
}

fn register_tyre_pressure(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "drift_rate_pa_per_hr", name: "zero drift", magnitude: "signed Pa/hr, not 0..1", effect: "indicated pressure slowly diverges from truth", healthy: 0.0, meaning: "Zero-drift rate" },
        FaultSpec { field: "stuck", name: "stuck output", magnitude: "0 healthy .. 1 fully frozen", effect: "indicated pressure stops responding to reality", healthy: 0.0, meaning: "Stuck fraction" },
    ];
    let mut wheels = vec!["Nose 1".to_string(), "Nose 2".to_string()];
    wheels.extend(braked_wheels());
    for wheel in &wheels {
        let id = format!("32_gear.tyre_pressure_{}", wheel.to_lowercase().replace(' ', "_"));
        let name = format!("Tyre pressure sensor ({wheel})");
        register_instance(r, c, 32, &id, &name, "discrete::PressureTransducer.step", &faults, &[]);
    }
}

fn register_door_proximity(r: &mut Registry, c: &mut Counter) {
    let doors = ["M1L", "M2L", "M2R", "M4L", "M5L", "U1L", "Cargo :16", "Cargo :17"];
    let roles = ["Open", "Closed"];
    let faults = [
        FaultSpec { field: "gap_error_mm", name: "gap out of rigging", magnitude: "signed mm, not 0..1", effect: "open/closed switch point shifts; wrong door-position indication without a hard stuck fault", healthy: 0.0, meaning: "Rigging/wear error on the nominal sensing gap" },
        FaultSpec { field: "stuck_near", name: "stuck near", magnitude: "0 healthy .. 1 (>=0.5 stuck)", effect: "always reports the sensed position (open or closed per the sensor's role) regardless of the true door state", healthy: 0.0, meaning: "Stuck-near fraction" },
        FaultSpec { field: "stuck_far", name: "stuck far", magnitude: "0 healthy .. 1 (>=0.5 stuck)", effect: "always reports the opposite of the sensed position regardless of the true door state", healthy: 0.0, meaning: "Stuck-far fraction" },
    ];
    for door in doors {
        for role in roles {
            let slug = door.to_lowercase().replace(' ', "_").replace(':', "");
            let id = format!("52_doors.prox_{slug}_{}", role.to_lowercase());
            let name = format!("Door {door} {role} proximity sensor");
            register_instance(r, c, 52, &id, &name, "discrete::ProximitySensor.sense", &faults, &[]);
        }
    }
}

fn register_hydraulic_sensors(r: &mut Registry, c: &mut Counter) {
    let pressure_faults = [
        FaultSpec { field: "drift_rate_pa_per_hr", name: "zero drift", magnitude: "signed Pa/hr, not 0..1", effect: "indicated pressure slowly diverges from truth", healthy: 0.0, meaning: "Zero-drift rate" },
        FaultSpec { field: "stuck", name: "stuck output", magnitude: "0 healthy .. 1 fully frozen", effect: "indicated pressure stops responding to reality", healthy: 0.0, meaning: "Stuck fraction" },
    ];
    let temp_faults = [
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 fully open", effect: "reading pegs to top of indicating range", healthy: 0.0, meaning: "Open-circuit fraction" },
        FaultSpec { field: "short_circuit", name: "short circuit", magnitude: "0 healthy .. 1 fully shorted", effect: "reading pegs to bottom of indicating range", healthy: 0.0, meaning: "Short-circuit fraction" },
    ];
    let quantity_faults = [
        FaultSpec { field: "float_stuck", name: "float binding", magnitude: "0 free .. 1 (>=0.98 seized)", effect: "indicated quantity freezes regardless of true fluid volume changes", healthy: 0.0, meaning: "Float mechanical binding fraction" },
        FaultSpec { field: "sender_bias", name: "sender bias", magnitude: "signed fraction of full scale, not 0..1", effect: "constant offset added to the indicated quantity", healthy: 0.0, meaning: "Signed potentiometer calibration bias" },
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 (>=0.98 open)", effect: "indicated quantity reads a conservative zero", healthy: 0.0, meaning: "Open-circuit fraction" },
    ];
    for system in ["Green", "Yellow"] {
        let pid = format!("29_hyd.pressure_{}", system.to_lowercase());
        let pname = format!("Hydraulic system pressure transducer ({system})");
        register_instance(r, c, 29, &pid, &pname, "discrete::PressureTransducer.step", &pressure_faults, &[]);

        let tid = format!("29_hyd.reservoir_temp_{}", system.to_lowercase());
        let tname = format!("Hydraulic reservoir temperature sensor ({system})");
        register_instance(r, c, 29, &tid, &tname, "discrete::temperature_sensor_reading_c", &temp_faults, &[]);

        let qid = format!("29_hyd.reservoir_quantity_{}", system.to_lowercase());
        let qname = format!("Hydraulic reservoir quantity transmitter ({system})");
        register_instance(r, c, 29, &qid, &qname, "float_level::FloatLevelSensor.step", &quantity_faults, &[]);
    }
}

fn register_engine_speed_pickups(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "air_gap_increase", name: "air gap increase", magnitude: "0 nominal .. 1 max modelled increase", effect: "signal amplitude falls; below the EEC's detection floor this raises the minimum speed at which the channel can still detect the pickup", healthy: 0.0, meaning: "Mounting/rigging gap increase fraction" },
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 (>=0.98 open)", effect: "no signal at any speed", healthy: 0.0, meaning: "Broken wire/connector fraction" },
    ];
    for engine in 1..=4 {
        for shaft in ["N1", "N2", "N3"] {
            for channel in ["A", "B"] {
                let id = format!("77_eng.speed_{}_{engine}_{}", shaft.to_lowercase(), channel.to_lowercase());
                let name = format!("Engine {engine} {shaft} speed pickup, EEC channel {channel}");
                register_instance(r, c, 77, &id, &name, "engine_sensors::speed_pickup_reading", &faults, &[]);
            }
        }
    }
}

fn register_engine_tgt_harness(r: &mut Registry, c: &mut Counter) {
    const JUNCTION_COUNT: f64 = 8.0;
    let faults = [
        FaultSpec { field: "open_circuit", name: "a junction goes open", magnitude: "0 healthy .. 1 (>=0.98 open, per junction)", effect: "that junction drops out of the average, biasing it toward whichever junctions remain", healthy: 0.0, meaning: "Per-junction open-circuit fraction (representative)" },
        FaultSpec { field: "drift_k", name: "a junction drifts", magnitude: "signed K offset, not 0..1", effect: "biases the average by roughly offset/junction_count", healthy: 0.0, meaning: "Per-junction temperature offset before averaging (representative)" },
    ];
    let extra = [ParamDef {
        name: "junction_count".into(),
        meaning: "GENERIC circumferential thermocouple junction count for this harness. Junctions are evenly spaced (angle_deg = i*360/count) and each senses `engine_sensors::HotStreak` (a plain input from e.g. a fuel-nozzle-coking model elsewhere) according to its own position -- not itself a fault of this component.".into(),
        healthy: JUNCTION_COUNT,
    }];
    for engine in 1..=4 {
        let id = format!("77_eng.tgt_harness_{engine}");
        let name = format!("Engine {engine} TGT thermocouple harness");
        register_instance(r, c, 77, &id, &name, "engine_sensors::tgt_harness_average_c", &faults, &extra);
    }
}

fn register_engine_vibration(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "bias", name: "bias", magnitude: "signed, generic amplitude units, not 0..1", effect: "constant offset added to the true reading", healthy: 0.0, meaning: "Signed reading bias" },
        FaultSpec { field: "stuck", name: "stuck output", magnitude: "0 healthy .. 1 fully frozen", effect: "reading stops responding to true vibration", healthy: 0.0, meaning: "Stuck fraction" },
        FaultSpec { field: "intermittent_dropout_rate_per_s", name: "intermittent dropout", magnitude: "probability/s, not 0..1", effect: "momentary signal loss (loose connector), holding the last value", healthy: 0.0, meaning: "Per-second dropout probability" },
    ];
    for engine in 1..=4 {
        for location in ["Fan", "Core"] {
            let id = format!("77_eng.vibration_{engine}_{}", location.to_lowercase());
            let name = format!("Engine {engine} vibration pickup ({location})");
            register_instance(r, c, 77, &id, &name, "engine_sensors::VibrationPickup.step", &faults, &[]);
        }
    }
}

fn register_engine_p30_t25(r: &mut Registry, c: &mut Counter) {
    let pressure_faults = [
        FaultSpec { field: "drift_rate_pa_per_hr", name: "zero drift", magnitude: "signed Pa/hr, not 0..1", effect: "indicated P30 slowly diverges from truth, affecting EEC fuel scheduling", healthy: 0.0, meaning: "Zero-drift rate" },
        FaultSpec { field: "stuck", name: "stuck output", magnitude: "0 healthy .. 1 fully frozen", effect: "indicated P30 stops responding to reality", healthy: 0.0, meaning: "Stuck fraction" },
    ];
    let temp_faults = [
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 fully open", effect: "T25 reading pegs to top of indicating range", healthy: 0.0, meaning: "Open-circuit fraction" },
        FaultSpec { field: "short_circuit", name: "short circuit", magnitude: "0 healthy .. 1 fully shorted", effect: "T25 reading pegs to bottom of indicating range", healthy: 0.0, meaning: "Short-circuit fraction" },
    ];
    for engine in 1..=4 {
        let pid = format!("77_eng.p30_{engine}");
        let pname = format!("Engine {engine} P30 (HP compressor delivery pressure) probe");
        register_instance(r, c, 77, &pid, &pname, "discrete::PressureTransducer.step", &pressure_faults, &[]);

        let tid = format!("77_eng.t25_{engine}");
        let tname = format!("Engine {engine} T25 (HP compressor inlet temperature) probe");
        register_instance(r, c, 77, &tid, &tname, "discrete::temperature_sensor_reading_c", &temp_faults, &[]);
    }
}

fn register_engine_fuel_flow(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "bearing_wear", name: "bearing wear/drag", magnitude: "0 nominal .. 1 max modelled wear", effect: "rotor under-spins for the true flow, the meter under-reads", healthy: 0.0, meaning: "Rotor bearing wear fraction" },
        FaultSpec { field: "debris_blockage", name: "debris partial blockage", magnitude: "0 none .. 1 fully blocked", effect: "less flow reaches the rotor than the engine actually burns, the meter under-reads (distinct cause from bearing wear)", healthy: 0.0, meaning: "Flow blockage fraction upstream of the rotor" },
        FaultSpec { field: "stuck_rotor", name: "stuck rotor", magnitude: "0 healthy .. 1 (>=0.98 seized)", effect: "reads zero/fixed regardless of true flow", healthy: 0.0, meaning: "Rotor seizure fraction" },
    ];
    for engine in 1..=4 {
        let id = format!("73_fuel.flow_transmitter_{engine}");
        let name = format!("Engine {engine} fuel flow transmitter");
        let ids = register_failures_only(r, c, 73, &id, &name, "engine_sensors::fuel_flow_transmitter_reading", &faults);
        let params: Vec<ParamDef> =
            faults.iter().map(|f| ParamDef { name: f.field.to_string(), meaning: f.meaning.to_string(), healthy: f.healthy }).collect();
        r.extend_component(&id).params(&params).failures(&ids);
    }
}

fn register_engine_oil_sensors(r: &mut Registry, c: &mut Counter) {
    let pressure_faults = [
        FaultSpec { field: "drift_rate_pa_per_hr", name: "zero drift", magnitude: "signed Pa/hr, not 0..1", effect: "indicated oil pressure slowly diverges from truth", healthy: 0.0, meaning: "Zero-drift rate" },
        FaultSpec { field: "stuck", name: "stuck output", magnitude: "0 healthy .. 1 fully frozen", effect: "indicated oil pressure stops responding to reality", healthy: 0.0, meaning: "Stuck fraction" },
    ];
    let temp_faults = [
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 fully open", effect: "reading pegs to top of indicating range", healthy: 0.0, meaning: "Open-circuit fraction" },
        FaultSpec { field: "short_circuit", name: "short circuit", magnitude: "0 healthy .. 1 fully shorted", effect: "reading pegs to bottom of indicating range", healthy: 0.0, meaning: "Short-circuit fraction" },
    ];
    let qty_faults = [
        FaultSpec { field: "contamination_frac", name: "water contamination", magnitude: "0 none .. 1 all water", effect: "indicated quantity reads high vs. true oil volume (a failed oil cooler/breached seal letting water in)", healthy: 0.0, meaning: "Fraction of the wetted column that is water" },
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 (>=0.98 open)", effect: "indicated quantity reads zero", healthy: 0.0, meaning: "Open-circuit fraction" },
    ];
    for engine in 1..=4 {
        let pid = format!("79_oil.pressure_{engine}");
        register_instance(r, c, 79, &pid, &format!("Engine {engine} oil pressure transducer"), "discrete::PressureTransducer.step", &pressure_faults, &[]);

        let tid = format!("79_oil.temperature_{engine}");
        register_instance(r, c, 79, &tid, &format!("Engine {engine} oil temperature sensor"), "discrete::temperature_sensor_reading_c", &temp_faults, &[]);

        let qid = format!("79_oil.quantity_{engine}");
        register_instance(r, c, 79, &qid, &format!("Engine {engine} oil quantity probe"), "discrete::oil_probe_indicated_level", &qty_faults, &[]);
    }
}

fn register_smoke_detectors(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "sensitivity_loss", name: "desensitised optics", magnitude: "0 clean .. 1 fully desensitised", effect: "delayed or missed detection of real smoke", healthy: 0.0, meaning: "Optics contamination reducing sensitivity" },
        FaultSpec { field: "false_bias_pct_per_ft", name: "spurious signal", magnitude: "percent/ft added, not 0..1", effect: "can alarm with no smoke present", healthy: 0.0, meaning: "Spurious added obscuration signal" },
        FaultSpec { field: "stuck", name: "stuck output", magnitude: "0 healthy .. 1 fully frozen", effect: "reading stops responding to true smoke density", healthy: 0.0, meaning: "Stuck fraction" },
        FaultSpec { field: "circuit_fault", name: "circuit/self-test fault", magnitude: "0 healthy .. 1 fully faulted", effect: "the detector's own monitored circuit reports faulted, independent of whether smoke is present", healthy: 0.0, meaning: "Detector circuit/self-test health" },
    ];
    let cargo = ["Fwd Cargo A", "Fwd Cargo B", "Aft Cargo A", "Aft Cargo B"];
    for loc in cargo {
        let id = format!("26_fire.smoke_{}", loc.to_lowercase().replace(' ', "_"));
        let name = format!("Smoke detector ({loc})");
        register_instance(r, c, 26, &id, &name, "smoke_detector::SmokeDetector.step", &faults, &[]);
    }
    for n in 1..=8 {
        let id = format!("26_fire.smoke_lav_{n}");
        let name = format!("Lavatory smoke detector ({n}) [GENERIC count, configuration-dependent]");
        register_instance(r, c, 26, &id, &name, "smoke_detector::SmokeDetector.step", &faults, &[]);
    }

    let new_instances = [
        ("bulk_cargo", "Bulk cargo smoke detector"),
        ("avncs_main_l", "L Main avionics bay smoke detector"),
        ("avncs_main_r", "R Main avionics bay smoke detector"),
        ("avncs_upper_l", "L Upper avionics bay smoke detector"),
        ("avncs_upper_r", "R Upper avionics bay smoke detector"),
        ("avncs_aft", "Aft avionics bay smoke detector"),
        ("main5l_fltrest", "MAIN 5L Flight Rest smoke detector"),
        ("main5l_cabrest", "MAIN 5L Cabin Rest smoke detector"),
        ("main_1l_cws", "MAIN 1L CWS smoke detector"),
        ("main_1l_rcc", "MAIN 1L RCC smoke detector"),
        ("upper_1l_cws", "UPPER 1L CWS smoke detector"),
        ("upper_1l_rcc", "UPPER 1L RCC smoke detector"),
        ("main_2l_cws", "MAIN 2L CWS smoke detector"),
        ("main_2l_rcc", "MAIN 2L RCC smoke detector"),
        ("upper_2l_cws", "UPPER 2L CWS smoke detector"),
        ("upper_2l_rcc", "UPPER 2L RCC smoke detector"),
        ("main_3r_cws", "MAIN 3R CWS smoke detector"),
        ("main_3r_rcc", "MAIN 3R RCC smoke detector"),
        ("upper_3r_cws", "UPPER 3R CWS smoke detector"),
        ("upper_3r_rcc", "UPPER 3R RCC smoke detector"),
        ("upper_1l_shower", "UPPER 1L Shower smoke detector"),
        ("upper_1r_shower", "UPPER 1R Shower smoke detector"),
        ("fwdlowercrewrest", "FWD Lower Crew Rest (LDCR) smoke detector"),
    ];
    for (slug, name) in new_instances {
        let id = format!("26_fire.smoke_{slug}");
        register_instance(r, c, 26, &id, name, "smoke_detector::SmokeDetector.step", &faults, &[]);
    }

    let sdf_faults = [
        FaultSpec {
            field: "configuration_fault",
            name: "cabin-configuration mismatch",
            magnitude: "0 healthy .. 1 (any nonzero) the SDF reports a mismatch",
            effect: "the SDF fails to reconcile its fitted smoke detectors against the aircraft's own cabin configuration (FCOM PRO-ABN-ECAM p.4997, `260800042` SMOKE FACILITIES DET FAULT)",
            healthy: 0.0,
            meaning: "SDF cabin-configuration mismatch, a pass-through discrete",
        },
        FaultSpec {
            field: "safety_test_overdue",
            name: "automatic safety test overdue",
            magnitude: "0 healthy (tested) .. 1 (any nonzero) overdue",
            effect: "the SDF's own automatic safety test (run every 10 h on ground) has not completed successfully within the last 50 h (FCOM PRO-ABN-ECAM p.5012, `260800092` SMOKE SAFETY TEST REQUIRED); this failure arms the BITE 'overdue' state directly rather than driving a literal elapsed-hours clock -- this area has no persisted operating-hours counter to drive one from, the same class of simplification `hydraulics::thermal`'s own monitored-switch discretes already use",
            healthy: 0.0,
            meaning: "SDF automatic safety-test-overdue BITE flag",
        },
    ];
    register_instance(r, c, 26, "26_fire.smoke_detection_function", "Smoke Detection Function (SDF)", "live_discrete::DiscreteSensors.tick (aggregate SDF discretes)", &sdf_faults, &[]);
}

fn register_duct_temperature(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 fully open", effect: "reading pegs to top of indicating range (system-dependent whether that is read as an overheat or an invalid signal)", healthy: 0.0, meaning: "Open-circuit fraction" },
        FaultSpec { field: "short_circuit", name: "short circuit", magnitude: "0 healthy .. 1 fully shorted", effect: "reading pegs to bottom of indicating range", healthy: 0.0, meaning: "Short-circuit fraction" },
    ];
    let locations = ["Pack 1 Supply Duct", "Pack 2 Supply Duct", "Trim Air Duct", "Wing Bleed Left", "Wing Bleed Right", "APU Bleed Duct"];
    for loc in locations {
        let id = format!("36_pneu.duct_temp_{}", loc.to_lowercase().replace(' ', "_"));
        let name = format!("Duct temperature sensor ({loc}) [GENERIC location set]");
        register_instance(r, c, 36, &id, &name, "discrete::temperature_sensor_reading_c", &faults, &[]);
    }
}

fn register_oxygen_sensors(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "drift_rate_pa_per_hr", name: "zero drift", magnitude: "signed Pa/hr, not 0..1", effect: "indicated bottle pressure (and so computed quantity) slowly diverges from truth", healthy: 0.0, meaning: "Zero-drift rate" },
        FaultSpec { field: "stuck", name: "stuck output", magnitude: "0 healthy .. 1 fully frozen", effect: "indicated pressure stops responding to reality -- a stuck-high reading can mask a real slow leak until the bottle runs dry", healthy: 0.0, meaning: "Stuck fraction" },
    ];
    let systems = ["Crew", "Therapeutic (first aid)"];
    for (i, system) in systems.iter().enumerate() {
        let id = format!("35_oxy.pressure_{}", i + 1);
        let name = format!("Oxygen system pressure transducer ({system})");
        register_instance(r, c, 35, &id, &name, "discrete::PressureTransducer.step", &faults, &[]);
    }
}

fn register_cabin_pressure_sensors(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "drift_rate_pa_per_hr", name: "zero drift", magnitude: "signed Pa/hr, not 0..1", effect: "indicated cabin/differential pressure slowly diverges from truth, biasing the CPC's cabin altitude schedule", healthy: 0.0, meaning: "Zero-drift rate" },
        FaultSpec { field: "stuck", name: "stuck output", magnitude: "0 healthy .. 1 fully frozen", effect: "the CPC's own pressure feedback stops responding to reality, so its outflow-valve control acts on stale data", healthy: 0.0, meaning: "Stuck fraction" },
    ];
    for cpc in 1..=2 {
        let aid = format!("21_cab.cpc_{cpc}_absolute_pressure");
        let aname = format!("CPC {cpc} cabin absolute pressure transducer");
        register_instance(r, c, 21, &aid, &aname, "discrete::PressureTransducer.step", &faults, &[]);

        let did = format!("21_cab.cpc_{cpc}_differential_pressure");
        let dname = format!("CPC {cpc} cabin/ambient differential pressure transducer");
        register_instance(r, c, 21, &did, &dname, "discrete::PressureTransducer.step", &faults, &[]);
    }
}

fn register_oat_probe(r: &mut Registry, c: &mut Counter) {
    let faults = [
        FaultSpec { field: "open_circuit", name: "open circuit", magnitude: "0 healthy .. 1 fully open", effect: "reading pegs to top of indicating range", healthy: 0.0, meaning: "Open-circuit fraction" },
        FaultSpec { field: "short_circuit", name: "short circuit", magnitude: "0 healthy .. 1 fully shorted", effect: "reading pegs to bottom of indicating range", healthy: 0.0, meaning: "Short-circuit fraction" },
    ];
    let id = "34_nav.oat_standby";
    let name = "Standby outside air temperature probe [GENERIC: dedicated-probe assumption]";
    register_instance(r, c, 34, id, name, "discrete::temperature_sensor_reading_c", &faults, &[]);
}

fn register_static_averaging_lines(r: &mut Registry, c: &mut Counter) -> Vec<u64> {
    let faults = [FaultSpec {
        field: "line_blocked",
        name: "averaging line blocked",
        magnitude: "0 clear .. 1 fully blocked",
        effect: "the two sides' static readings are no longer pneumatically tied together: sideslip-driven left/right position error no longer cancels, but neither side's own reading is invalidated",
        healthy: 0.0,
        meaning: "Averaging line blockage fraction",
    }];
    let systems = ["ADR 1", "ADR 2", "ADR 3", "Standby"];
    let mut ids = Vec::new();
    for (i, sys) in systems.iter().enumerate() {
        let id = format!("34_nav.static_avg_line_{}", i + 1);
        let name = format!("Static port averaging line ({sys})");
        let fids = register_instance(r, c, 34, &id, &name, "static_port::average_pair", &faults, &[]);
        ids.push(fids[0]);
    }
    ids
}

#[allow(clippy::too_many_arguments)]
fn register_ecam(
    r: &mut Registry,
    pitot_heater_ids: &[u64],
    static_blocked_ids: &[u64],
    aoa_jam_ids: &[u64],
    tat_heater_ids: &[u64],
    ra_transceiver_ids: &[u64],
    gps_receiver_ids: &[u64],
    gps_antenna_ids: &[u64],
) {
    let mut adr_related: Vec<u64> = Vec::new();
    adr_related.extend_from_slice(pitot_heater_ids);
    adr_related.extend_from_slice(static_blocked_ids);
    let _ = adr_related;
    let _ = tat_heater_ids;
    let _ = &ra_transceiver_ids;
}

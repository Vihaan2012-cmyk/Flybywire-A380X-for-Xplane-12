use super::brake_wear::{BrakeWearPin, BrakeWearPinFaults};
use super::discrete::{
    oil_probe_indicated_level, temperature_sensor_reading_c, CapacitanceProbeFaults, PressureTransducer,
    PressureTransducerFaults, ProximitySensor, ProximitySensorFaults, TemperatureSensorFaults,
};
use super::engine_sensors::{
    fuel_flow_transmitter_fault, fuel_flow_transmitter_reading, speed_pickup_reading, tgt_harness_average_streaks_c,
    FuelFlowTransmitterFaults, HotStreak, SpeedPickupFaults, TgtJunction, TgtJunctionFaults, VibrationPickup, VibrationPickupFaults,
};
use super::float_level::{FloatLevelFaults, FloatLevelSensor};
use super::live::FaultIndex;
use super::registry;
use super::smoke_detector::{SmokeDetector, SmokeDetectorFaults};
use crate::deep::engine_accessories::fuel::manifold::NUM_GROUPS as NOZZLE_GROUPS;
use crate::deep::live::{Faults, Truth, DOOR_NAMES};

pub const BLOCKED: &[(&str, &str, &str)] = &[];

const PROX_GAP_ERROR_FULL_SCALE_MM: f64 = -25.0;

const PROX_HOME_STANDOFF_MM: f64 = 2.0;

const PROX_TARGET_TRAVEL_MM: f64 = 60.0;

const TRANSDUCER_DRIFT_FULL_SCALE_FRACTION_PER_HR: f64 = 0.05;

const SENDER_BIAS_FULL_SCALE: f64 = 0.2;

const SMOKE_FALSE_BIAS_FULL_SCALE_PCT_PER_FT: f64 = 4.0;

const VIBRATION_BIAS_FULL_SCALE: f64 = 5.0;

const VIBRATION_DROPOUT_FULL_SCALE_PER_S: f64 = 1.0;

const OIL_TEMP_RANGE_C: (f64, f64) = (-70.0, 250.0);

const OIL_PRESSURE_FULL_SCALE_PA: f64 = 149.0 * PSI_TO_PA;

const TYRE_PRESSURE_FULL_SCALE_PA: f64 = 2_000_000.0;

const BRAKE_TEMP_RANGE_C: (f64, f64) = (-70.0, 1200.0);
const HYD_TEMP_RANGE_C: (f64, f64) = (-70.0, 200.0);
const DUCT_TEMP_RANGE_C: (f64, f64) = (-70.0, 500.0);
const T25_RANGE_C: (f64, f64) = (-70.0, 500.0);
const TGT_JUNCTION_COUNT: usize = 8;
const TGT_DRIFT_FULL_SCALE_K: f64 = 100.0;

const HYD_FULL_SCALE_PA: f64 = 34_474_000.0;

const OXYGEN_BOTTLE_FULL_SCALE_PA: f64 = 3000.0 * PSI_TO_PA + 101_325.0;
const P30_FULL_SCALE_PA: f64 = 5_000_000.0;
fn oxygen_charge_absolute_pa(bottle: usize) -> f64 {
    let charge_psi = if bottle == 1 {
        crate::deep::oxygen::crew::CYLINDER_CHARGE_PSI
    } else {
        crate::deep::oxygen::therapeutic::CHARGE_PSI
    };
    charge_psi * PSI_TO_PA + 101_325.0
}

const CABIN_ABSOLUTE_FULL_SCALE_PA: f64 = 120_000.0;
const CABIN_ALT_T0_K: f64 = 288.15;
const CABIN_ALT_L_K_PER_M: f64 = 0.0065;
const CABIN_ALT_P0_PA: f64 = 101_325.0;
const CABIN_ALT_R_AIR: f64 = 287.052_87;
const CABIN_ALT_G0: f64 = 9.80665;
const M_TO_FT: f64 = 3.280_839_9;
const CABIN_DIFFERENTIAL_FULL_SCALE_PA: f64 = 70_000.0;

const PSI_TO_PA: f64 = 6_894.757_293_168_361;

const SMOKE_SPECIFIC_EXTINCTION_M2_PER_KG: f64 = 8_700.0;

const FOOT_M: f64 = 0.304_8;

const R_AIR_J_KG_K: f64 = 287.052_874;

fn obscuration_pct_per_ft(smoke_kg_m3: f64) -> f64 {
    let k = SMOKE_SPECIFIC_EXTINCTION_M2_PER_KG * smoke_kg_m3.max(0.0) * FOOT_M;
    100.0 * (1.0 - (-k).exp())
}

const REGISTRY_LEG_TO_GEAR_INDEX: [usize; 5] = [
    0,
    3,
    4,
    1,
    2,
];

const REGISTRY_LEGS: [&str; 5] = ["Nose", "Left Body", "Right Body", "Left Wing", "Right Wing"];

const BRAKE_WHEEL_INDEX: [usize; 16] = [0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15];

fn slug(name: &str) -> String {
    name.to_lowercase().replace(' ', "_").replace(':', "")
}

fn upper_slug(name: &str) -> String {
    slug(name).to_uppercase()
}

enum ProxSource {
    Lock(String),
    WeightOnWheels(usize),
    DoorTravel { var: String, home_open: bool },
    TruthDoor { index: usize, home_open: bool },
}

impl ProxSource {
    fn var(&self) -> Option<&str> {
        match self {
            ProxSource::Lock(v) => Some(v),
            ProxSource::DoorTravel { var, .. } => Some(var),
            ProxSource::WeightOnWheels(_) | ProxSource::TruthDoor { .. } => None,
        }
    }
}

struct LiveProximity {
    var_near: String,
    var_gap: String,
    source: ProxSource,
    sensor: ProximitySensor,
    faults: [u64; 3],
    near: bool,
    gap_mm: f64,
}

struct LiveTemperature {
    var: String,
    source: String,
    range_c: (f64, f64),
    faults: [u64; 2],
    reading_c: f64,
}

struct LiveOilTemperature {
    var: String,
    engine: usize,
    faults: [u64; 2],
    reading_c: f64,
}

struct LiveT25 {
    var: String,
    engine: usize,
    faults: [u64; 2],
    reading_c: f64,
}

struct LiveOilQuantity {
    var: String,
    engine: usize,
    faults: [u64; 2],
    indicated_frac: f64,
}

struct LiveTgtHarness {
    var: String,
    engine: usize,
    faults: [u64; 2],
    reading_c: f64,
    group_dev_vars: [String; NOZZLE_GROUPS],
    rise_var: String,
}

struct LiveBrakeWear {
    var: String,
    source: String,
    pin: BrakeWearPin,
    faults: [u64; 3],
    indicated_remaining: f64,
}

enum SmokeSource {
    DensityKgM3(String),
    MassFractionOfCabinAir(String),
    LavatoryAndDeck(String, String),
}

impl SmokeSource {
    fn vars(&self) -> Vec<&str> {
        match self {
            SmokeSource::DensityKgM3(v) | SmokeSource::MassFractionOfCabinAir(v) => vec![v],
            SmokeSource::LavatoryAndDeck(lav, deck) => vec![lav, deck],
        }
    }
}

struct LiveSmoke {
    var_reading: String,
    var_alarm: String,
    var_fault: String,
    source: SmokeSource,
    detector: SmokeDetector,
    faults: [u64; 4],
    reading_pct_per_ft: f64,
    alarm: bool,
    fault: bool,
}

enum PressureSource {
    Published { var: String, to_pa: f64 },
    FromTruth,
}

struct LivePressure {
    var: String,
    source: PressureSource,
    transducer: PressureTransducer,
    full_scale_pa: f64,
    faults: [u64; 2],
    reading_pa: f64,
    stuck_frac: f64,
}

impl LivePressure {
    fn step(&mut self, true_pa: f64, faults: &Faults, dt: f64) {
        let drift_pa_per_hr =
            faults.get(self.faults[0]) * TRANSDUCER_DRIFT_FULL_SCALE_FRACTION_PER_HR * self.full_scale_pa;
        self.stuck_frac = faults.get(self.faults[1]);
        self.reading_pa = self.transducer.step(
            true_pa,
            &PressureTransducerFaults { drift_rate_pa_per_hr: drift_pa_per_hr, stuck: self.stuck_frac },
            dt,
        );
    }
}

struct LiveFloat {
    var_indicated: String,
    var_stuck: String,
    source_level: String,
    source_temp_c: String,
    sensor: FloatLevelSensor,
    faults: [u64; 3],
    indicated_frac: f64,
    stuck: bool,
}

struct LiveSpeedPickup {
    var_valid: String,
    var_frac: String,
    engine: usize,
    is_n3: bool,
    faults: [u64; 2],
    valid: bool,
    frac: f64,
}

enum VibrationSource {
    Fan(String),
    Core(String, String),
}

struct LiveVibration {
    var_reading: String,
    var_valid: String,
    source: VibrationSource,
    pickup: VibrationPickup,
    faults: [u64; 3],
    reading: f64,
    valid: bool,
}

struct LiveFuelFlow {
    var: String,
    var_fault: String,
    source: String,
    engine: usize,
    faults: [u64; 3],
    indicated_kg_s: f64,
    fault: bool,
}

pub struct DiscreteSensors {
    proximity: Vec<LiveProximity>,
    temperature: Vec<LiveTemperature>,
    brake_wear: Vec<LiveBrakeWear>,
    smoke: Vec<LiveSmoke>,
    hyd_pressure: Vec<LivePressure>,
    p30: Vec<LivePressure>,
    cabin_absolute: Vec<LivePressure>,
    cabin_differential: Vec<LivePressure>,
    oil_pressure: Vec<LivePressure>,
    oil_temperature: Vec<LiveOilTemperature>,
    oil_quantity: Vec<LiveOilQuantity>,
    tgt_harness: Vec<LiveTgtHarness>,
    t25: Vec<LiveT25>,
    tyre_pressure: Vec<LivePressure>,
    nose_tyre_pressure: Vec<LivePressure>,
    tyre_gas_temp_k: Vec<f64>,
    hyd_quantity: Vec<LiveFloat>,
    core_pickups: Vec<LiveSpeedPickup>,
    vibration: Vec<LiveVibration>,
    fuel_flow: Vec<LiveFuelFlow>,
    cabin_alt_above_landing_elev_ft: [f64; 2],
    vertical_speed_fpm: f64,
    sdf_ids: [u64; 2],
    sdf_configuration_fault: bool,
    sdf_safety_test_overdue: bool,
}

impl DiscreteSensors {
    pub fn new(index: &FaultIndex) -> Self {
        let mut proximity = Vec::new();
        for (r, leg) in REGISTRY_LEGS.iter().enumerate() {
            let gear_index = REGISTRY_LEG_TO_GEAR_INDEX[r];
            let n = gear_index + 1;
            for role in ["Uplock", "Downlock", "WOW"] {
                let id = format!("32_gear.prox_{}_{}", slug(leg), role.to_lowercase());
                let source = match role {
                    "Uplock" => ProxSource::Lock(format!("GEAR_UPLOCKED:{n}")),
                    "Downlock" => ProxSource::Lock(format!("GEAR_DOWNLOCKED:{n}")),
                    _ => ProxSource::WeightOnWheels(gear_index),
                };
                proximity.push(LiveProximity {
                    var_near: format!("DEEP_PROX_GEAR_{}_{}_NEAR", upper_slug(leg), role.to_uppercase()),
                    var_gap: format!("DEEP_PROX_GEAR_{}_{}_GAP_MM", upper_slug(leg), role.to_uppercase()),
                    source,
                    sensor: ProximitySensor::new(),
                    faults: index.ids(&id, ["gap_error_mm", "stuck_near", "stuck_far"]),
                    near: false,
                    gap_mm: 0.0,
                });
            }
        }
        for role in ["Open", "Closed"] {
            let id = format!("52_doors.prox_cargo_16_{}", role.to_lowercase());
            proximity.push(LiveProximity {
                var_near: format!("DEEP_PROX_DOOR_CARGO_16_{}_NEAR", role.to_uppercase()),
                var_gap: format!("DEEP_PROX_DOOR_CARGO_16_{}_GAP_MM", role.to_uppercase()),
                source: ProxSource::DoorTravel {
                    var: "CABIN_CARGO_DOOR_PERCENT:1".into(),
                    home_open: role == "Open",
                },
                sensor: ProximitySensor::new(),
                faults: index.ids(&id, ["gap_error_mm", "stuck_near", "stuck_far"]),
                near: false,
                gap_mm: 0.0,
            });
        }
        let door_registry_slug = |name: &str| -> Option<&'static str> {
            match name {
                "M1L" => Some("m1l"),
                "M2L" => Some("m2l"),
                "M2R" => Some("m2r"),
                "M4L" => Some("m4l"),
                "M5L" => Some("m5l"),
                "U1L" => Some("u1l"),
                "CARGO_AFT" => Some("cargo_17"),
                _ => None,
            }
        };
        for (i, name) in DOOR_NAMES.iter().enumerate() {
            let Some(door_slug) = door_registry_slug(name) else { continue };
            for role in ["Open", "Closed"] {
                let id = format!("52_doors.prox_{door_slug}_{}", role.to_lowercase());
                proximity.push(LiveProximity {
                    var_near: format!("DEEP_PROX_DOOR_{}_{}_NEAR", upper_slug(name), role.to_uppercase()),
                    var_gap: format!("DEEP_PROX_DOOR_{}_{}_GAP_MM", upper_slug(name), role.to_uppercase()),
                    source: ProxSource::TruthDoor { index: i, home_open: role == "Open" },
                    sensor: ProximitySensor::new(),
                    faults: index.ids(&id, ["gap_error_mm", "stuck_near", "stuck_far"]),
                    near: false,
                    gap_mm: 0.0,
                });
            }
        }

        let wheels = registry::braked_wheels();
        let mut temperature = Vec::new();
        let mut brake_wear = Vec::with_capacity(wheels.len());
        for (w, wheel) in wheels.iter().enumerate() {
            let n = BRAKE_WHEEL_INDEX[w] + 1;
            temperature.push(LiveTemperature {
                var: format!("DEEP_BRAKE_TEMP_SENSED_C:{}", w + 1),
                source: format!("BRAKE_STACK_TEMP_C:{n}"),
                range_c: BRAKE_TEMP_RANGE_C,
                faults: index.ids(&format!("32_gear.brake_temp_{}", slug(wheel)), ["open_circuit", "short_circuit"]),
                reading_c: 0.0,
            });
            brake_wear.push(LiveBrakeWear {
                var: format!("DEEP_BRAKE_WEAR_REMAINING_INDICATED:{}", w + 1),
                source: format!("BRAKE_WEAR_FRACTION:{n}"),
                pin: BrakeWearPin::new(1.0),
                faults: index.ids(
                    &format!("32_gear.brake_wear_{}", slug(wheel)),
                    ["pin_binding", "sender_bias", "open_circuit"],
                ),
                indicated_remaining: 1.0,
            });
        }

        let mut tyre_pressure = Vec::with_capacity(16);
        for (w, wheel) in wheels.iter().enumerate() {
            tyre_pressure.push(LivePressure {
                var: format!("DEEP_TYRE_PRESSURE_SENSED_PA:{}", w + 1),
                source: PressureSource::FromTruth,
                transducer: PressureTransducer::new(crate::physics::tyre::COLD_PRESSURE_PA),
                full_scale_pa: TYRE_PRESSURE_FULL_SCALE_PA,
                faults: index.ids(&format!("32_gear.tyre_pressure_{}", slug(wheel)), ["drift_rate_pa_per_hr", "stuck"]),
                reading_pa: crate::physics::tyre::COLD_PRESSURE_PA,
                stuck_frac: 0.0,
            });
        }

        let mut nose_tyre_pressure = Vec::with_capacity(2);
        for n in 1..=2usize {
            nose_tyre_pressure.push(LivePressure {
                var: format!("DEEP_TYRE_PRESSURE_SENSED_PA:NOSE_{n}"),
                source: PressureSource::FromTruth,
                transducer: PressureTransducer::new(crate::physics::tyre::COLD_PRESSURE_PA),
                full_scale_pa: TYRE_PRESSURE_FULL_SCALE_PA,
                faults: index.ids(&format!("32_gear.tyre_pressure_nose_{n}"), ["drift_rate_pa_per_hr", "stuck"]),
                reading_pa: crate::physics::tyre::COLD_PRESSURE_PA,
                stuck_frac: 0.0,
            });
        }

        for (loc, source) in [
            ("Pack 1 Supply Duct", "DEEP_PNEU_PACK_1_SUPPLY_TEMPERATURE_C"),
            ("Pack 2 Supply Duct", "DEEP_PNEU_PACK_2_SUPPLY_TEMPERATURE_C"),
            ("Trim Air Duct", "DEEP_THERM_TRIM_AIR_DUCT_TEMPERATURE_C"),
            ("Wing Bleed Left", "DEEP_PNEU_WAI_L_DUCT_TEMPERATURE_C"),
            ("Wing Bleed Right", "DEEP_PNEU_WAI_R_DUCT_TEMPERATURE_C"),
            ("APU Bleed Duct", "DEEP_PNEU_APU_DUCT_TEMPERATURE_C"),
        ] {
            temperature.push(LiveTemperature {
                var: format!("DEEP_DUCT_TEMP_{}_C", upper_slug(loc)),
                source: source.to_string(),
                range_c: DUCT_TEMP_RANGE_C,
                faults: index.ids(&format!("36_pneu.duct_temp_{}", slug(loc)), ["open_circuit", "short_circuit"]),
                reading_c: 0.0,
            });
        }

        let mut hyd_pressure = Vec::with_capacity(2);
        let mut hyd_quantity = Vec::with_capacity(2);
        for colour in ["Green", "Yellow"] {
            let up = colour.to_uppercase();
            let low = colour.to_lowercase();
            hyd_pressure.push(LivePressure {
                var: format!("DEEP_HYD_{up}_PRESSURE_SENSED_PA"),
                source: PressureSource::Published {
                    var: format!("HYD_{up}_MANIFOLD_PRESSURE_PSI"),
                    to_pa: PSI_TO_PA,
                },
                transducer: PressureTransducer::new(0.0),
                full_scale_pa: HYD_FULL_SCALE_PA,
                faults: index.ids(&format!("29_hyd.pressure_{low}"), ["drift_rate_pa_per_hr", "stuck"]),
                reading_pa: 0.0,
                stuck_frac: 0.0,
            });
            temperature.push(LiveTemperature {
                var: format!("DEEP_HYD_{up}_RESERVOIR_TEMP_SENSED_C"),
                source: format!("HYD_{up}_FLUID_TEMP_C"),
                range_c: HYD_TEMP_RANGE_C,
                faults: index.ids(&format!("29_hyd.reservoir_temp_{low}"), ["open_circuit", "short_circuit"]),
                reading_c: 0.0,
            });
            hyd_quantity.push(LiveFloat {
                var_indicated: format!("DEEP_HYD_{up}_RESERVOIR_QTY_INDICATED"),
                var_stuck: format!("DEEP_HYD_{up}_RESERVOIR_QTY_STUCK"),
                source_level: format!("HYD_{up}_RESERVOIR_LEVEL_FRACTION"),
                source_temp_c: format!("HYD_{up}_FLUID_TEMP_C"),
                sensor: FloatLevelSensor::new(1.0),
                faults: index.ids(
                    &format!("29_hyd.reservoir_quantity_{low}"),
                    ["float_stuck", "sender_bias", "open_circuit"],
                ),
                indicated_frac: 1.0,
                stuck: false,
            });
        }

        for (n, _system) in [(1usize, "crew"), (2, "therapeutic")] {
            hyd_pressure.push(LivePressure {
                var: format!("DEEP_OXYGEN_BOTTLE_PRESSURE_SENSED_PA:{n}"),
                source: PressureSource::Published { var: format!("OXYGEN_BOTTLE_PRESSURE_PA:{n}"), to_pa: 1.0 },
                transducer: PressureTransducer::new(oxygen_charge_absolute_pa(n)),
                full_scale_pa: OXYGEN_BOTTLE_FULL_SCALE_PA,
                faults: index.ids(&format!("35_oxy.pressure_{n}"), ["drift_rate_pa_per_hr", "stuck"]),
                reading_pa: oxygen_charge_absolute_pa(n),
                stuck_frac: 0.0,
            });
        }

        let mut oil_pressure = Vec::with_capacity(4);
        let mut oil_temperature = Vec::with_capacity(4);
        let mut oil_quantity = Vec::with_capacity(4);
        let mut tgt_harness = Vec::with_capacity(4);
        let mut t25 = Vec::with_capacity(4);
        let mut p30 = Vec::with_capacity(4);
        let mut core_pickups = Vec::with_capacity(16);
        let mut vibration = Vec::with_capacity(8);
        let mut fuel_flow = Vec::with_capacity(4);
        for engine in 1..=4usize {
            p30.push(LivePressure {
                var: format!("DEEP_ENG_{engine}_P30_SENSED_PA"),
                source: PressureSource::FromTruth,
                transducer: PressureTransducer::new(101_325.0),
                full_scale_pa: P30_FULL_SCALE_PA,
                faults: index.ids(&format!("77_eng.p30_{engine}"), ["drift_rate_pa_per_hr", "stuck"]),
                reading_pa: 101_325.0,
                stuck_frac: 0.0,
            });
            oil_pressure.push(LivePressure {
                var: format!("DEEP_ENG_{engine}_OIL_PRESSURE_SENSED_PA"),
                source: PressureSource::FromTruth,
                transducer: PressureTransducer::new(0.0),
                full_scale_pa: OIL_PRESSURE_FULL_SCALE_PA,
                faults: index.ids(&format!("79_oil.pressure_{engine}"), ["drift_rate_pa_per_hr", "stuck"]),
                reading_pa: 0.0,
                stuck_frac: 0.0,
            });
            oil_temperature.push(LiveOilTemperature {
                var: format!("DEEP_ENG_{engine}_OIL_TEMP_SENSED_C"),
                engine: engine - 1,
                faults: index.ids(&format!("79_oil.temperature_{engine}"), ["open_circuit", "short_circuit"]),
                reading_c: 0.0,
            });
            oil_quantity.push(LiveOilQuantity {
                var: format!("DEEP_ENG_{engine}_OIL_QTY_SENSED_FRAC"),
                engine: engine - 1,
                faults: index.ids(&format!("79_oil.quantity_{engine}"), ["contamination_frac", "open_circuit"]),
                indicated_frac: 1.0,
            });
            tgt_harness.push(LiveTgtHarness {
                var: format!("DEEP_ENG_{engine}_TGT_SENSED_C"),
                engine: engine - 1,
                faults: index.ids(&format!("77_eng.tgt_harness_{engine}"), ["open_circuit", "drift_k"]),
                reading_c: 15.0,
                group_dev_vars: std::array::from_fn(|k| format!("A32NX_ENG_{engine}_NOZZLE_GROUP_{}_FLOW_DEV", k + 1)),
                rise_var: format!("A32NX_ENG_{engine}_COMBUSTOR_RISE_FRACTION"),
            });
            t25.push(LiveT25 {
                var: format!("DEEP_ENG_{engine}_T25_SENSED_C"),
                engine: engine - 1,
                faults: index.ids(&format!("77_eng.t25_{engine}"), ["open_circuit", "short_circuit"]),
                reading_c: 0.0,
            });
            for shaft in ["n2", "n3"] {
                for channel in ["a", "b"] {
                    let up_shaft = shaft.to_uppercase();
                    let up_channel = channel.to_uppercase();
                    core_pickups.push(LiveSpeedPickup {
                        var_valid: format!("DEEP_ENG_{engine}_{up_shaft}_PICKUP_{up_channel}_VALID"),
                        var_frac: format!("DEEP_ENG_{engine}_{up_shaft}_PICKUP_{up_channel}_FRAC"),
                        engine: engine - 1,
                        is_n3: shaft == "n3",
                        faults: index.ids(
                            &format!("77_eng.speed_{shaft}_{engine}_{channel}"),
                            ["air_gap_increase", "open_circuit"],
                        ),
                        valid: false,
                        frac: 0.0,
                    });
                }
            }
            for location in ["Fan", "Core"] {
                let source = if location == "Fan" {
                    VibrationSource::Fan(format!("A32NX_ENG_{engine}_N1_VIB_INDEX"))
                } else {
                    VibrationSource::Core(
                        format!("A32NX_ENG_{engine}_N2_VIB_INDEX"),
                        format!("A32NX_ENG_{engine}_N3_VIB_INDEX"),
                    )
                };
                vibration.push(LiveVibration {
                    var_reading: format!("DEEP_ENG_{engine}_VIB_{}_INDEX", location.to_uppercase()),
                    var_valid: format!("DEEP_ENG_{engine}_VIB_{}_VALID", location.to_uppercase()),
                    source,
                    pickup: VibrationPickup::new(
                        0x7715_0000 + (engine as u64) * 2 + u64::from(location == "Core"),
                    ),
                    faults: index.ids(
                        &format!("77_eng.vibration_{engine}_{}", location.to_lowercase()),
                        ["bias", "stuck", "intermittent_dropout_rate_per_s"],
                    ),
                    reading: 0.0,
                    valid: true,
                });
            }
            fuel_flow.push(LiveFuelFlow {
                var: format!("DEEP_FF_XMTR_{engine}_INDICATED_KG_S"),
                var_fault: format!("DEEP_FF_XMTR_{engine}_FAULT"),
                source: format!("A32NX_ENG_{engine}_FF_TRUE_KG_S"),
                engine: engine - 1,
                faults: index.ids(
                    &format!("73_fuel.flow_transmitter_{engine}"),
                    ["bearing_wear", "debris_blockage", "stuck_rotor"],
                ),
                indicated_kg_s: 0.0,
                fault: false,
            });
        }

        let mut cabin_absolute = Vec::with_capacity(2);
        let mut cabin_differential = Vec::with_capacity(2);
        for cpc in 1..=2usize {
            cabin_absolute.push(LivePressure {
                var: format!("DEEP_CPC_{cpc}_CABIN_PRESSURE_SENSED_PA"),
                source: PressureSource::FromTruth,
                transducer: PressureTransducer::new(101_325.0),
                full_scale_pa: CABIN_ABSOLUTE_FULL_SCALE_PA,
                faults: index.ids(&format!("21_cab.cpc_{cpc}_absolute_pressure"), ["drift_rate_pa_per_hr", "stuck"]),
                reading_pa: 101_325.0,
                stuck_frac: 0.0,
            });
            cabin_differential.push(LivePressure {
                var: format!("DEEP_CPC_{cpc}_DIFF_PRESSURE_SENSED_PA"),
                source: PressureSource::FromTruth,
                transducer: PressureTransducer::new(0.0),
                full_scale_pa: CABIN_DIFFERENTIAL_FULL_SCALE_PA,
                faults: index
                    .ids(&format!("21_cab.cpc_{cpc}_differential_pressure"), ["drift_rate_pa_per_hr", "stuck"]),
                reading_pa: 0.0,
                stuck_frac: 0.0,
            });
        }

        let mut smoke = Vec::new();
        for (loc, bay) in
            [("Fwd Cargo A", "FWD"), ("Fwd Cargo B", "FWD"), ("Aft Cargo A", "AFT"), ("Aft Cargo B", "AFT")]
        {
            smoke.push(LiveSmoke {
                var_reading: format!("DEEP_SMOKE_{}_PCT_PER_FT", upper_slug(loc)),
                var_alarm: format!("DEEP_SMOKE_{}_ALARM", upper_slug(loc)),
                var_fault: format!("DEEP_SMOKE_{}_FAULT", upper_slug(loc)),
                source: SmokeSource::DensityKgM3(format!("CARGO_{bay}_SMOKE_DENSITY_KG_M3")),
                detector: SmokeDetector::new(),
                faults: index.ids(
                    &format!("26_fire.smoke_{}", slug(loc)),
                    ["sensitivity_loss", "false_bias_pct_per_ft", "stuck", "circuit_fault"],
                ),
                reading_pct_per_ft: 0.0,
                alarm: false,
                fault: false,
            });
        }
        for n in 1..=8 {
            let zone = if n <= 4 { "CABINMAINDECK" } else { "CABINUPPERDECK" };
            smoke.push(LiveSmoke {
                var_reading: format!("DEEP_SMOKE_LAV_{n}_PCT_PER_FT"),
                var_alarm: format!("DEEP_SMOKE_LAV_{n}_ALARM"),
                var_fault: format!("DEEP_SMOKE_LAV_{n}_FAULT"),
                source: if n == 1 {
                    SmokeSource::LavatoryAndDeck("LAVATORY_1_SMOKE_DENSITY_KG_M3".into(), format!("THERMAL_ZONE_{zone}_SMOKE_CONCENTRATION"))
                } else {
                    SmokeSource::MassFractionOfCabinAir(format!("THERMAL_ZONE_{zone}_SMOKE_CONCENTRATION"))
                },
                detector: SmokeDetector::new(),
                faults: index.ids(
                    &format!("26_fire.smoke_lav_{n}"),
                    ["sensitivity_loss", "false_bias_pct_per_ft", "stuck", "circuit_fault"],
                ),
                reading_pct_per_ft: 0.0,
                alarm: false,
                fault: false,
            });
        }

        let new_smoke_instances = [
            ("bulk_cargo", "DEEP_SMOKE_CARGO_BULK", "CARGOBULK"),
            ("avncs_main_l", "DEEP_SMOKE_AVNCS_MAIN_L", "MAINAVIONICS"),
            ("avncs_main_r", "DEEP_SMOKE_AVNCS_MAIN_R", "MAINAVIONICS"),
            ("avncs_upper_l", "DEEP_SMOKE_AVNCS_UPPER_L", "UPPERAVIONICS"),
            ("avncs_upper_r", "DEEP_SMOKE_AVNCS_UPPER_R", "UPPERAVIONICS"),
            ("avncs_aft", "DEEP_SMOKE_AVNCS_AFT", "AFTAVIONICS"),
            ("main5l_fltrest", "DEEP_SMOKE_MAIN5L_FLTREST", "CABINMAINDECK"),
            ("main5l_cabrest", "DEEP_SMOKE_MAIN5L_CABREST", "CABINMAINDECK"),
            ("main_1l_cws", "DEEP_SMOKE_MAIN_1L_CWS", "CABINMAINDECK"),
            ("main_1l_rcc", "DEEP_SMOKE_MAIN_1L_RCC", "CABINMAINDECK"),
            ("upper_1l_cws", "DEEP_SMOKE_UPPER_1L_CWS", "CABINUPPERDECK"),
            ("upper_1l_rcc", "DEEP_SMOKE_UPPER_1L_RCC", "CABINUPPERDECK"),
            ("main_2l_cws", "DEEP_SMOKE_MAIN_2L_CWS", "CABINMAINDECK"),
            ("main_2l_rcc", "DEEP_SMOKE_MAIN_2L_RCC", "CABINMAINDECK"),
            ("upper_2l_cws", "DEEP_SMOKE_UPPER_2L_CWS", "CABINUPPERDECK"),
            ("upper_2l_rcc", "DEEP_SMOKE_UPPER_2L_RCC", "CABINUPPERDECK"),
            ("main_3r_cws", "DEEP_SMOKE_MAIN_3R_CWS", "CABINMAINDECK"),
            ("main_3r_rcc", "DEEP_SMOKE_MAIN_3R_RCC", "CABINMAINDECK"),
            ("upper_3r_cws", "DEEP_SMOKE_UPPER_3R_CWS", "CABINUPPERDECK"),
            ("upper_3r_rcc", "DEEP_SMOKE_UPPER_3R_RCC", "CABINUPPERDECK"),
            ("upper_1l_shower", "DEEP_SMOKE_UPPER_1L_SHOWER", "CABINUPPERDECK"),
            ("upper_1r_shower", "DEEP_SMOKE_UPPER_1R_SHOWER", "CABINUPPERDECK"),
            ("fwdlowercrewrest", "DEEP_SMOKE_FWDLOWERCREWREST", "FWDLOWERCREWREST"),
        ];
        for (slug, var_base, zone) in new_smoke_instances {
            smoke.push(LiveSmoke {
                var_reading: format!("{var_base}_PCT_PER_FT"),
                var_alarm: format!("{var_base}_ALARM"),
                var_fault: format!("{var_base}_FAULT"),
                source: SmokeSource::MassFractionOfCabinAir(format!("THERMAL_ZONE_{zone}_SMOKE_CONCENTRATION")),
                detector: SmokeDetector::new(),
                faults: index.ids(&format!("26_fire.smoke_{slug}"), ["sensitivity_loss", "false_bias_pct_per_ft", "stuck", "circuit_fault"]),
                reading_pct_per_ft: 0.0,
                alarm: false,
                fault: false,
            });
        }

        let sdf_ids = index.ids("26_fire.smoke_detection_function", ["configuration_fault", "safety_test_overdue"]);

        Self {
            proximity,
            temperature,
            brake_wear,
            smoke,
            hyd_pressure,
            p30,
            cabin_absolute,
            cabin_differential,
            oil_pressure,
            oil_temperature,
            oil_quantity,
            tgt_harness,
            t25,
            tyre_pressure,
            nose_tyre_pressure,
            tyre_gas_temp_k: vec![crate::physics::tyre::COLD_TEMP_K; 18],
            hyd_quantity,
            core_pickups,
            vibration,
            fuel_flow,
            cabin_alt_above_landing_elev_ft: [0.0; 2],
            vertical_speed_fpm: 0.0,
            sdf_ids,
            sdf_configuration_fault: false,
            sdf_safety_test_overdue: false,
        }
    }

    pub fn resolved_ids(&self) -> Vec<u64> {
        let mut ids = Vec::new();
        for p in &self.proximity {
            ids.extend_from_slice(&p.faults);
        }
        for s in &self.temperature {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.brake_wear {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.smoke {
            ids.extend_from_slice(&s.faults);
        }
        for s in self
            .hyd_pressure
            .iter()
            .chain(&self.p30)
            .chain(&self.cabin_absolute)
            .chain(&self.cabin_differential)
            .chain(&self.oil_pressure)
            .chain(&self.tyre_pressure)
            .chain(&self.nose_tyre_pressure)
        {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.oil_temperature {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.oil_quantity {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.tgt_harness {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.t25 {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.hyd_quantity {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.core_pickups {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.vibration {
            ids.extend_from_slice(&s.faults);
        }
        for s in &self.fuel_flow {
            ids.extend_from_slice(&s.faults);
        }
        ids.extend_from_slice(&self.sdf_ids);
        ids
    }

    pub fn sources(&self) -> Vec<String> {
        let mut v: Vec<String> = Vec::new();
        v.extend(self.proximity.iter().filter_map(|p| p.source.var().map(|s| s.to_owned())));
        v.extend(self.temperature.iter().map(|s| s.source.clone()));
        v.extend(self.brake_wear.iter().map(|s| s.source.clone()));
        v.extend(self.smoke.iter().flat_map(|s| s.source.vars().into_iter().map(str::to_owned)));
        for p in &self.hyd_pressure {
            if let PressureSource::Published { var, .. } = &p.source {
                v.push(var.clone());
            }
        }
        for q in &self.hyd_quantity {
            v.push(q.source_level.clone());
            v.push(q.source_temp_c.clone());
        }
        for s in &self.vibration {
            match &s.source {
                VibrationSource::Fan(a) => v.push(a.clone()),
                VibrationSource::Core(a, b) => {
                    v.push(a.clone());
                    v.push(b.clone());
                }
            }
        }
        v.extend(self.fuel_flow.iter().map(|s| s.source.clone()));
        v
    }

    pub fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s;
        let published = &truth.published;

        for p in &mut self.proximity {
            let home = match &p.source {
                ProxSource::Lock(var) => published.get_or(var, 0.0),
                ProxSource::WeightOnWheels(leg_index) => {
                    if truth.leg_on_ground[*leg_index] {
                        1.0
                    } else {
                        0.0
                    }
                }
                ProxSource::DoorTravel { var, home_open } => {
                    let open = (published.get_or(var, 0.0) * 0.01).clamp(0.0, 1.0);
                    if *home_open {
                        open
                    } else {
                        1.0 - open
                    }
                }
                ProxSource::TruthDoor { index, home_open } => {
                    let open = truth.door_open_fraction[*index].clamp(0.0, 1.0);
                    if *home_open {
                        open
                    } else {
                        1.0 - open
                    }
                }
            };
            p.gap_mm = PROX_HOME_STANDOFF_MM + PROX_TARGET_TRAVEL_MM * (1.0 - home.clamp(0.0, 1.0));
            p.near = p.sensor.sense(
                p.gap_mm,
                &ProximitySensorFaults {
                    gap_error_mm: faults.get(p.faults[0]) * PROX_GAP_ERROR_FULL_SCALE_MM,
                    stuck_near: faults.get(p.faults[1]),
                    stuck_far: faults.get(p.faults[2]),
                },
            );
        }

        for s in &mut self.temperature {
            let true_c = published.get_or(&s.source, truth.environment.sat_c);
            s.reading_c = temperature_sensor_reading_c(
                true_c,
                s.range_c,
                &TemperatureSensorFaults {
                    open_circuit: faults.get(s.faults[0]),
                    short_circuit: faults.get(s.faults[1]),
                },
            );
        }

        for s in &mut self.oil_temperature {
            s.reading_c = temperature_sensor_reading_c(
                truth.engine_oil_temp_c[s.engine],
                OIL_TEMP_RANGE_C,
                &TemperatureSensorFaults {
                    open_circuit: faults.get(s.faults[0]),
                    short_circuit: faults.get(s.faults[1]),
                },
            );
        }

        for s in &mut self.oil_quantity {
            s.indicated_frac = oil_probe_indicated_level(
                truth.engine_oil_quantity_fraction[s.engine],
                &CapacitanceProbeFaults {
                    contamination_frac: faults.get(s.faults[0]),
                    open_circuit: faults.get(s.faults[1]),
                },
            );
        }

        for s in &mut self.tgt_harness {
            let rise = published.get_or(&s.rise_var, 0.0).clamp(0.0, 1.0);
            let tgt_k = truth.engine_tgt_c[s.engine] + 273.15;
            let sector_deg = 360.0 / NOZZLE_GROUPS as f64;
            let mut streaks = [HotStreak::default(); NOZZLE_GROUPS];
            for (k, streak) in streaks.iter_mut().enumerate() {
                let dev = published.get_or(&s.group_dev_vars[k], 0.0);
                *streak = HotStreak { center_deg: k as f64 * sector_deg, width_deg: sector_deg / 2.355, peak_delta_k: dev * tgt_k.max(0.0) * rise };
            }
            let junctions: Vec<TgtJunction> = (0..TGT_JUNCTION_COUNT)
                .map(|j| TgtJunction {
                    angle_deg: j as f64 * 360.0 / TGT_JUNCTION_COUNT as f64,
                    faults: if j == 0 {
                        TgtJunctionFaults {
                            open_circuit: faults.get(s.faults[0]),
                            drift_k: faults.get(s.faults[1]) * TGT_DRIFT_FULL_SCALE_K,
                        }
                    } else {
                        TgtJunctionFaults::default()
                    },
                })
                .collect();
            let out = tgt_harness_average_streaks_c(truth.engine_tgt_c[s.engine], &streaks, &junctions);
            if out.valid {
                s.reading_c = out.average_c;
            }
        }

        for s in &mut self.t25 {
            s.reading_c = temperature_sensor_reading_c(
                truth.engine_t25_c[s.engine],
                T25_RANGE_C,
                &TemperatureSensorFaults {
                    open_circuit: faults.get(s.faults[0]),
                    short_circuit: faults.get(s.faults[1]),
                },
            );
        }

        for s in &mut self.brake_wear {
            let remaining = 1.0 - published.get_or(&s.source, 0.0).clamp(0.0, 1.0);
            s.indicated_remaining = s.pin.step(
                remaining,
                &BrakeWearPinFaults {
                    pin_binding: faults.get(s.faults[0]),
                    sender_bias: faults.get(s.faults[1]) * SENDER_BIAS_FULL_SCALE,
                    open_circuit: faults.get(s.faults[2]),
                },
                dt,
            );
        }

        let cabin_air_density_kg_m3 =
            (truth.cabin_pressure_pa / (R_AIR_J_KG_K * truth.cabin_temp_k.max(1.0))).max(0.0);
        for s in &mut self.smoke {
            let smoke_kg_m3 = match &s.source {
                SmokeSource::DensityKgM3(var) => published.get_or(var, 0.0),
                SmokeSource::MassFractionOfCabinAir(var) => {
                    published.get_or(var, 0.0) * cabin_air_density_kg_m3
                }
                SmokeSource::LavatoryAndDeck(lav, deck) => {
                    published.get_or(lav, 0.0) + published.get_or(deck, 0.0) * cabin_air_density_kg_m3
                }
            };
            let out = s.detector.step(
                obscuration_pct_per_ft(smoke_kg_m3),
                &SmokeDetectorFaults {
                    sensitivity_loss: faults.get(s.faults[0]),
                    false_bias_pct_per_ft: faults.get(s.faults[1]) * SMOKE_FALSE_BIAS_FULL_SCALE_PCT_PER_FT,
                    stuck: faults.get(s.faults[2]),
                    circuit_fault: faults.get(s.faults[3]),
                },
            );
            s.reading_pct_per_ft = out.reading_pct_per_ft;
            s.alarm = out.alarm;
            s.fault = out.circuit_fault;
        }

        self.sdf_configuration_fault = faults.get(self.sdf_ids[0]) > 0.0;
        self.sdf_safety_test_overdue = faults.get(self.sdf_ids[1]) > 0.0;

        for s in &mut self.hyd_pressure {
            let true_pa = match &s.source {
                PressureSource::Published { var, to_pa } => published.get_or(var, 0.0) * to_pa,
                PressureSource::FromTruth => 0.0,
            };
            s.step(true_pa, faults, dt);
        }
        for (e, s) in self.p30.iter_mut().enumerate() {
            s.step(truth.engine_hp_port_pressure_pa[e], faults, dt);
        }
        for (e, s) in self.oil_pressure.iter_mut().enumerate() {
            s.step(truth.engine_oil_pressure_pa[e], faults, dt);
        }
        for (w, s) in self.tyre_pressure.iter_mut().enumerate() {
            s.step(truth.tyre_pressure_pa[BRAKE_WHEEL_INDEX[w]], faults, dt);
            self.tyre_gas_temp_k[w] = truth.tyre_temp_c[BRAKE_WHEEL_INDEX[w]] + 273.15;
        }
        for (n, s) in self.nose_tyre_pressure.iter_mut().enumerate() {
            s.step(truth.tyre_pressure_pa[crate::physics::tyre::BRAKED_WHEELS + n], faults, dt);
            self.tyre_gas_temp_k[self.tyre_pressure.len() + n] = truth.tyre_temp_c[crate::physics::tyre::BRAKED_WHEELS + n] + 273.15;
        }
        for s in &mut self.cabin_absolute {
            s.step(truth.cabin_pressure_pa, faults, dt);
        }
        let differential_pa = truth.cabin_pressure_pa - truth.environment.ambient_pressure_pa;
        for s in &mut self.cabin_differential {
            s.step(differential_pa, faults, dt);
        }
        for (i, s) in self.cabin_absolute.iter().enumerate() {
            let p_pa = s.reading_pa.max(1.0);
            let alt_m = CABIN_ALT_T0_K / CABIN_ALT_L_K_PER_M * (1.0 - (p_pa / CABIN_ALT_P0_PA).powf(CABIN_ALT_R_AIR * CABIN_ALT_L_K_PER_M / CABIN_ALT_G0));
            self.cabin_alt_above_landing_elev_ft[i] = alt_m * M_TO_FT - truth.landing_elevation_ft;
        }
        self.vertical_speed_fpm = truth.vertical_speed_fpm;

        for s in &mut self.hyd_quantity {
            let level = published.get_or(&s.source_level, 1.0);
            let temp_c = published.get_or(&s.source_temp_c, 15.0);
            let out = s.sensor.step(
                level,
                temp_c,
                &FloatLevelFaults {
                    float_stuck: faults.get(s.faults[0]),
                    sender_bias: faults.get(s.faults[1]) * SENDER_BIAS_FULL_SCALE,
                    open_circuit: faults.get(s.faults[2]),
                },
                dt,
            );
            s.indicated_frac = out.indicated_frac;
            s.stuck = out.stuck;
        }

        for s in &mut self.core_pickups {
            let true_frac =
                if s.is_n3 { truth.engine_n3_frac[s.engine] } else { truth.engine_n2_frac[s.engine] }.max(0.0);
            let out = speed_pickup_reading(
                true_frac,
                &SpeedPickupFaults {
                    air_gap_increase: faults.get(s.faults[0]),
                    open_circuit: faults.get(s.faults[1]),
                },
            );
            s.valid = out.valid;
            s.frac = out.speed_frac;
        }

        for s in &mut self.vibration {
            let true_amplitude = match &s.source {
                VibrationSource::Fan(var) => published.get_or(var, 0.0).max(0.0),
                VibrationSource::Core(a, b) => {
                    let (a, b) = (published.get_or(a, 0.0).max(0.0), published.get_or(b, 0.0).max(0.0));
                    (a * a + b * b).sqrt()
                }
            };
            let (reading, valid) = s.pickup.step(
                true_amplitude,
                &VibrationPickupFaults {
                    bias: faults.get(s.faults[0]) * VIBRATION_BIAS_FULL_SCALE,
                    stuck: faults.get(s.faults[1]),
                    intermittent_dropout_rate_per_s: faults.get(s.faults[2])
                        * VIBRATION_DROPOUT_FULL_SCALE_PER_S,
                },
                dt,
            );
            s.reading = reading;
            s.valid = valid;
        }

        for s in &mut self.fuel_flow {
            let true_kg_s = published.get_or(&s.source, truth.engine_fuel_flow_kg_s[s.engine]);
            let transmitter_faults = FuelFlowTransmitterFaults {
                bearing_wear: faults.get(s.faults[0]),
                debris_blockage: faults.get(s.faults[1]),
                stuck_rotor: faults.get(s.faults[2]),
            };
            s.indicated_kg_s = fuel_flow_transmitter_reading(true_kg_s, &transmitter_faults);
            s.fault = fuel_flow_transmitter_fault(true_kg_s, &transmitter_faults);
        }
    }

    pub fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        for p in &self.proximity {
            out(&p.var_near, b(p.near));
            out(&p.var_gap, p.gap_mm);
        }
        for s in &self.temperature {
            out(&s.var, s.reading_c);
        }
        for s in &self.brake_wear {
            out(&s.var, s.indicated_remaining);
        }
        for s in &self.smoke {
            out(&s.var_reading, s.reading_pct_per_ft);
            out(&s.var_alarm, b(s.alarm));
            out(&s.var_fault, b(s.fault));
        }
        out("DEEP_SMOKE_ANY_DETECTOR_FAULT", b(self.smoke.iter().any(|s| s.fault)));
        out("DEEP_SDF_CONFIGURATION_FAULT", b(self.sdf_configuration_fault));
        out("DEEP_SDF_SAFETY_TEST_OVERDUE", b(self.sdf_safety_test_overdue));
        for s in self
            .hyd_pressure
            .iter()
            .chain(&self.p30)
            .chain(&self.cabin_absolute)
            .chain(&self.cabin_differential)
            .chain(&self.oil_pressure)
            .chain(&self.tyre_pressure)
            .chain(&self.nose_tyre_pressure)
        {
            out(&s.var, s.reading_pa);
        }
        for (s, &temp_k) in self.tyre_pressure.iter().chain(&self.nose_tyre_pressure).zip(&self.tyre_gas_temp_k) {
            let name = s.var.replace("DEEP_TYRE_PRESSURE_SENSED_PA:", "DEEP_TYRE_PRESSURE_COLD_EQUIV_PA:");
            out(&name, s.reading_pa * crate::physics::tyre::COLD_TEMP_K / temp_k.max(1.0));
        }
        for i in 0..2 {
            let fault = self.cabin_absolute[i].stuck_frac.max(self.cabin_differential[i].stuck_frac) >= 1.0;
            out(&format!("DEEP_CPC_{}_SENSOR_FAULT", i + 1), b(fault));
        }
        for i in 0..2 {
            let p_pa = self.cabin_absolute[i].reading_pa.max(1.0);
            let alt_m = CABIN_ALT_T0_K / CABIN_ALT_L_K_PER_M * (1.0 - (p_pa / CABIN_ALT_P0_PA).powf(CABIN_ALT_R_AIR * CABIN_ALT_L_K_PER_M / CABIN_ALT_G0));
            out(&format!("DEEP_CPC_{}_CABIN_ALTITUDE_FT", i + 1), alt_m * M_TO_FT);
            out(&format!("DEEP_CPC_{}_CABIN_ALT_ABOVE_LANDING_ELEV_FT", i + 1), self.cabin_alt_above_landing_elev_ft[i]);
        }
        out("DEEP_ADIRS_VERTICAL_SPEED_FPM", self.vertical_speed_fpm);
        for s in &self.oil_temperature {
            out(&s.var, s.reading_c);
        }
        for s in &self.oil_quantity {
            out(&s.var, s.indicated_frac);
        }
        for s in &self.tgt_harness {
            out(&s.var, s.reading_c);
        }
        for s in &self.t25 {
            out(&s.var, s.reading_c);
        }
        for s in &self.hyd_quantity {
            out(&s.var_indicated, s.indicated_frac);
            out(&s.var_stuck, b(s.stuck));
        }
        for s in &self.core_pickups {
            out(&s.var_valid, b(s.valid));
            out(&s.var_frac, s.frac);
        }
        for s in &self.vibration {
            out(&s.var_reading, s.reading);
            out(&s.var_valid, b(s.valid));
        }
        for s in &self.fuel_flow {
            out(&s.var, s.indicated_kg_s);
            out(&s.var_fault, b(s.fault));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::live::{all_areas, PublishedFrame};
    use std::collections::{BTreeMap, BTreeSet};

    fn index() -> FaultIndex {
        FaultIndex::build()
    }

    fn truth_with(pairs: &[(&str, f64)]) -> Truth {
        let mut frame = PublishedFrame::default();
        for (name, value) in pairs {
            frame.insert((*name).to_string(), *value);
        }
        Truth { dt_s: 0.5, published: frame, ..Truth::default() }
    }

    fn run(s: &mut DiscreteSensors, truth: &Truth, faults: &Faults, ticks: usize) -> BTreeMap<String, f64> {
        for _ in 0..ticks {
            s.tick(truth, faults);
        }
        let mut map = BTreeMap::new();
        s.publish(&mut |n, v| {
            map.insert(n.to_string(), v);
        });
        map
    }

    fn fresh() -> DiscreteSensors {
        DiscreteSensors::new(&index())
    }

    fn brake_truth(temp_c: f64) -> Truth {
        let mut frame = PublishedFrame::default();
        for n in 1..=16 {
            frame.insert(format!("BRAKE_STACK_TEMP_C:{n}"), temp_c);
        }
        Truth { dt_s: 0.5, published: frame, ..Truth::default() }
    }

    #[test]
    fn every_discrete_sensor_resolved_a_real_failure_id() {
        let sensors = fresh();
        let ids = sensors.resolved_ids();
        assert!(!ids.is_empty());
        assert!(ids.iter().all(|&id| id != 0), "a live discrete sensor has no catalogue failure behind it");
        let unique: BTreeSet<u64> = ids.iter().copied().collect();
        assert_eq!(unique.len(), ids.len(), "two live sensors resolved to the same failure id");
    }

    fn everything_published() -> BTreeSet<String> {
        let mut deep = all_areas();
        let mut names = BTreeSet::new();
        deep.tick(Truth::default(), &Faults::default(), &mut |name, _| {
            names.insert(name.to_string());
        });
        names
    }

    #[test]
    fn every_quantity_these_sensors_sense_is_published_by_somebody() {
        let published = everything_published();
        let sensors = fresh();
        let missing: Vec<String> = sensors.sources().into_iter().filter(|s| !published.contains(s)).collect();
        assert!(missing.is_empty(), "these sensors read a variable no area publishes: {missing:?}");
    }

    #[test]
    fn a_blocked_sensors_source_is_still_missing() {
        let published = everything_published();
        for (component, quantity, var) in BLOCKED {
            if var.contains('<') || var.starts_with("Truth::") || var.ends_with(":n") || var.contains("_n_") {
                continue;
            }
            assert!(!published.contains(*var), "{var} ({quantity}) is published now, so {component} can be instantiated");
        }
    }

    #[test]
    fn a_healthy_brake_temperature_sensor_reads_the_brake_it_is_bolted_to() {
        let mut frame = PublishedFrame::default();
        for n in 1..=16 {
            frame.insert(format!("BRAKE_STACK_TEMP_C:{n}"), 100.0 + n as f64 * 10.0);
        }
        let truth = Truth { dt_s: 0.5, published: frame, ..Truth::default() };
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        for (w, &gear_index) in BRAKE_WHEEL_INDEX.iter().enumerate() {
            let expected = 100.0 + (gear_index + 1) as f64 * 10.0;
            let got = out[&format!("DEEP_BRAKE_TEMP_SENSED_C:{}", w + 1)];
            assert!((got - expected).abs() < 1e-6, "sensor {} read {got}, brake {} is at {expected}", w + 1, gear_index + 1);
        }
    }

    #[test]
    fn a_healthy_downlock_proximity_sensor_follows_the_lock_it_watches() {
        let truth = truth_with(&[("GEAR_DOWNLOCKED:2", 1.0)]);
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert_eq!(out["DEEP_PROX_GEAR_LEFT_WING_DOWNLOCK_NEAR"], 1.0);
        assert_eq!(out["DEEP_PROX_GEAR_RIGHT_WING_DOWNLOCK_NEAR"], 0.0);
        assert_eq!(out["DEEP_PROX_GEAR_NOSE_DOWNLOCK_NEAR"], 0.0);
        assert!((out["DEEP_PROX_GEAR_LEFT_WING_DOWNLOCK_GAP_MM"] - PROX_HOME_STANDOFF_MM).abs() < 1e-9);
        assert!(out["DEEP_PROX_GEAR_NOSE_DOWNLOCK_GAP_MM"] > PROX_TARGET_TRAVEL_MM);
    }

    #[test]
    fn a_healthy_weight_on_wheels_sensor_follows_the_real_gear_state() {
        let airborne = Truth { dt_s: 0.5, leg_on_ground: [false; 5], ..Truth::default() };
        let grounded = Truth { dt_s: 0.5, leg_on_ground: [true; 5], ..Truth::default() };
        assert_eq!(run(&mut fresh(), &airborne, &Faults::default(), 2)["DEEP_PROX_GEAR_NOSE_WOW_NEAR"], 0.0);
        assert_eq!(run(&mut fresh(), &grounded, &Faults::default(), 2)["DEEP_PROX_GEAR_NOSE_WOW_NEAR"], 1.0);
    }

    #[test]
    fn a_stuck_far_wow_proximity_sensor_lies_about_being_on_the_ground() {
        let grounded = Truth { dt_s: 0.5, leg_on_ground: [true; 5], ..Truth::default() };
        let mut sensors = fresh();
        let stuck_far_id = sensors
            .proximity
            .iter()
            .find(|p| p.var_near == "DEEP_PROX_GEAR_NOSE_WOW_NEAR")
            .expect("nose WOW sensor exists")
            .faults[2];
        let out = run(&mut sensors, &grounded, &Faults::from_pairs([(stuck_far_id, 1.0)]), 2);
        assert_eq!(out["DEEP_PROX_GEAR_NOSE_WOW_NEAR"], 0.0, "stuck_far must be able to move the reading away from a correctly-near baseline");
    }

    #[test]
    fn a_healthy_reservoir_sender_settles_on_the_level_the_hydraulic_area_computes() {
        let truth = truth_with(&[
            ("HYD_GREEN_RESERVOIR_LEVEL_FRACTION", 0.62),
            ("HYD_GREEN_FLUID_TEMP_C", 15.0),
            ("HYD_YELLOW_RESERVOIR_LEVEL_FRACTION", 0.94),
            ("HYD_YELLOW_FLUID_TEMP_C", 15.0),
        ]);
        let out = run(&mut fresh(), &truth, &Faults::default(), 200);
        assert!((out["DEEP_HYD_GREEN_RESERVOIR_QTY_INDICATED"] - 0.62).abs() < 0.01);
        assert!((out["DEEP_HYD_YELLOW_RESERVOIR_QTY_INDICATED"] - 0.94).abs() < 0.01);
    }

    #[test]
    fn a_healthy_core_speed_pickup_reads_its_own_engines_own_spool() {
        let truth = Truth {
            engine_n2_frac: [0.10, 0.20, 0.30, 0.40],
            engine_n3_frac: [0.50, 0.60, 0.70, 0.80],
            ..Truth::default()
        };
        let out = run(&mut fresh(), &truth, &Faults::default(), 1);
        for e in 1..=4 {
            assert!((out[&format!("DEEP_ENG_{e}_N2_PICKUP_A_FRAC")] - e as f64 * 0.10).abs() < 1e-9);
            assert!((out[&format!("DEEP_ENG_{e}_N3_PICKUP_B_FRAC")] - (0.40 + e as f64 * 0.10)).abs() < 1e-9);
        }
    }

    #[test]
    fn a_healthy_p30_probe_reads_the_hp_compressor_delivery_it_is_tapped_into() {
        let truth = Truth {
            engine_hp_port_pressure_pa: [800_000.0, 900_000.0, 1_000_000.0, 1_100_000.0],
            ..Truth::default()
        };
        let out = run(&mut fresh(), &truth, &Faults::default(), 1);
        for e in 1..=4 {
            let expected = 700_000.0 + e as f64 * 100_000.0;
            assert!((out[&format!("DEEP_ENG_{e}_P30_SENSED_PA")] - expected).abs() < 1e-6);
        }
    }

    #[test]
    fn a_healthy_cpc_transducer_pair_reads_the_cabin_and_the_differential() {
        let mut truth = Truth { cabin_pressure_pa: 84_000.0, ..Truth::default() };
        truth.environment.ambient_pressure_pa = 38_000.0;
        let out = run(&mut fresh(), &truth, &Faults::default(), 1);
        assert!((out["DEEP_CPC_1_CABIN_PRESSURE_SENSED_PA"] - 84_000.0).abs() < 1e-6);
        assert!((out["DEEP_CPC_2_DIFF_PRESSURE_SENSED_PA"] - 46_000.0).abs() < 1e-6);
    }

    #[test]
    fn the_oxygen_transducers_read_their_own_bottle_and_a_stuck_one_hides_it_draining() {
        let charged = |n: usize| oxygen_charge_absolute_pa(n);
        let mut frame = PublishedFrame::default();
        frame.insert("OXYGEN_BOTTLE_PRESSURE_PA:1".to_string(), charged(1));
        frame.insert("OXYGEN_BOTTLE_PRESSURE_PA:2".to_string(), charged(2));
        assert!(charged(1) > charged(2), "the crew bottle is charged higher than the therapeutic one");
        let full = Truth { dt_s: 1.0, published: frame, ..Truth::default() };
        let out = run(&mut fresh(), &full, &Faults::default(), 2);
        assert!((out["DEEP_OXYGEN_BOTTLE_PRESSURE_SENSED_PA:1"] - charged(1)).abs() < 1e-6);
        assert!((out["DEEP_OXYGEN_BOTTLE_PRESSURE_SENSED_PA:2"] - charged(2)).abs() < 1e-6);

        let mut drained_frame = PublishedFrame::default();
        drained_frame.insert("OXYGEN_BOTTLE_PRESSURE_PA:1".to_string(), 0.25 * charged(1));
        drained_frame.insert("OXYGEN_BOTTLE_PRESSURE_PA:2".to_string(), charged(2));
        let drained = Truth { dt_s: 1.0, published: drained_frame, ..Truth::default() };

        let healthy = run(&mut fresh(), &drained, &Faults::default(), 2);
        assert!((healthy["DEEP_OXYGEN_BOTTLE_PRESSURE_SENSED_PA:1"] - 0.25 * charged(1)).abs() < 1e-6);

        let i = index();
        let stuck = i.id("35_oxy.pressure_1", "stuck");
        assert_ne!(stuck, 0, "the crew bottle transducer is registered and instantiated");
        let mut broken = fresh();
        run(&mut broken, &full, &Faults::default(), 2);
        let out = run(&mut broken, &drained, &Faults::from_pairs([(stuck, 1.0)]), 60);
        assert!(
            (out["DEEP_OXYGEN_BOTTLE_PRESSURE_SENSED_PA:1"] - charged(1)).abs() < 1.0,
            "a stuck transducer must still show a full bottle: {}",
            out["DEEP_OXYGEN_BOTTLE_PRESSURE_SENSED_PA:1"]
        );
        assert!((out["DEEP_OXYGEN_BOTTLE_PRESSURE_SENSED_PA:2"] - charged(2)).abs() < 1e-6);
    }

    #[test]
    fn an_armed_brake_temperature_open_or_short_pegs_only_that_wheels_reading() {
        let i = index();
        let open = i.id("32_gear.brake_temp_left_wing_1", "open_circuit");
        let short = i.id("32_gear.brake_temp_right_body_4", "short_circuit");
        assert!(open != 0 && short != 0);
        let truth = brake_truth(250.0);

        let healthy = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert!((healthy["DEEP_BRAKE_TEMP_SENSED_C:1"] - 250.0).abs() < 1e-6);

        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(open, 1.0), (short, 1.0)]), 2);
        assert_eq!(out["DEEP_BRAKE_TEMP_SENSED_C:1"], BRAKE_TEMP_RANGE_C.1, "an open circuit must peg high");
        assert_eq!(out["DEEP_BRAKE_TEMP_SENSED_C:16"], BRAKE_TEMP_RANGE_C.0, "a short must peg low");
        assert!((out["DEEP_BRAKE_TEMP_SENSED_C:2"] - 250.0).abs() < 1e-6, "it took out a neighbouring wheel");
    }

    #[test]
    fn an_armed_brake_wear_pin_binding_overstates_remaining_life_as_the_brake_wears() {
        let id = index().id("32_gear.brake_wear_left_wing_1", "pin_binding");
        assert_ne!(id, 0);
        let fresh_stack = truth_with(&[("BRAKE_WEAR_FRACTION:1", 0.0), ("BRAKE_WEAR_FRACTION:2", 0.0)]);
        let worn = truth_with(&[("BRAKE_WEAR_FRACTION:1", 0.95), ("BRAKE_WEAR_FRACTION:2", 0.95)]);

        let mut healthy = fresh();
        run(&mut healthy, &fresh_stack, &Faults::default(), 20);
        let healthy_out = run(&mut healthy, &worn, &Faults::default(), 200);
        assert!(
            (healthy_out["DEEP_BRAKE_WEAR_REMAINING_INDICATED:1"] - 0.05).abs() < 0.02,
            "a healthy pickoff must follow the real stack: {}",
            healthy_out["DEEP_BRAKE_WEAR_REMAINING_INDICATED:1"]
        );

        let mut bound = fresh();
        run(&mut bound, &fresh_stack, &Faults::default(), 20);
        let armed = Faults::from_pairs([(id, 1.0)]);
        let bound_out = run(&mut bound, &worn, &armed, 200);
        assert!(
            bound_out["DEEP_BRAKE_WEAR_REMAINING_INDICATED:1"] > 0.9,
            "the dangerous case: a seized pin should still be claiming a fresh stack, got {}",
            bound_out["DEEP_BRAKE_WEAR_REMAINING_INDICATED:1"]
        );
        assert!(
            bound_out["DEEP_BRAKE_WEAR_REMAINING_INDICATED:2"] < 0.2,
            "the neighbouring wheel's healthy pickoff must still be telling the truth"
        );
    }

    #[test]
    fn an_armed_brake_wear_open_circuit_reads_zero_and_a_bias_reads_high() {
        let i = index();
        let open = i.id("32_gear.brake_wear_left_wing_2", "open_circuit");
        let bias = i.id("32_gear.brake_wear_left_wing_3", "sender_bias");
        assert!(open != 0 && bias != 0);
        let truth = truth_with(&[("BRAKE_WEAR_FRACTION:2", 0.5), ("BRAKE_WEAR_FRACTION:5", 0.5)]);
        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(open, 1.0), (bias, 1.0)]), 100);
        assert_eq!(out["DEEP_BRAKE_WEAR_REMAINING_INDICATED:2"], 0.0);
        assert!(
            (out["DEEP_BRAKE_WEAR_REMAINING_INDICATED:3"] - (0.5 + SENDER_BIAS_FULL_SCALE)).abs() < 0.02,
            "{}",
            out["DEEP_BRAKE_WEAR_REMAINING_INDICATED:3"]
        );
    }

    #[test]
    fn an_armed_downlock_proximity_fault_reports_a_locked_leg_as_unlocked() {
        let i = index();
        let stuck_far = i.id("32_gear.prox_left_wing_downlock", "stuck_far");
        let rigging = i.id("32_gear.prox_right_wing_downlock", "gap_error_mm");
        let stuck_near = i.id("32_gear.prox_nose_downlock", "stuck_near");
        assert!(stuck_far != 0 && rigging != 0 && stuck_near != 0);
        let truth = truth_with(&[
            ("GEAR_DOWNLOCKED:2", 1.0),
            ("GEAR_DOWNLOCKED:3", 1.0),
            ("GEAR_DOWNLOCKED:4", 1.0),
            ("GEAR_DOWNLOCKED:5", 1.0),
        ]);

        let healthy = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert_eq!(healthy["DEEP_PROX_GEAR_LEFT_WING_DOWNLOCK_NEAR"], 1.0);
        assert_eq!(healthy["DEEP_PROX_GEAR_RIGHT_WING_DOWNLOCK_NEAR"], 1.0);
        assert_eq!(healthy["DEEP_PROX_GEAR_NOSE_DOWNLOCK_NEAR"], 0.0);

        let armed = Faults::from_pairs([(stuck_far, 1.0), (rigging, 1.0), (stuck_near, 1.0)]);
        let out = run(&mut fresh(), &truth, &armed, 2);
        assert_eq!(out["DEEP_PROX_GEAR_LEFT_WING_DOWNLOCK_NEAR"], 0.0, "a stuck-far sensor must miss a made lock");
        assert_eq!(
            out["DEEP_PROX_GEAR_RIGHT_WING_DOWNLOCK_NEAR"], 0.0,
            "a rigging error big enough to miss the target must miss it"
        );
        assert_eq!(out["DEEP_PROX_GEAR_NOSE_DOWNLOCK_NEAR"], 1.0, "a stuck-near sensor must claim a lock that is not made");
        assert_eq!(out["DEEP_PROX_GEAR_LEFT_BODY_DOWNLOCK_NEAR"], 1.0, "it reached a leg it was not armed on");
    }

    #[test]
    fn an_armed_smoke_detector_fault_alarms_on_clean_air_or_misses_a_real_fire() {
        let i = index();
        let false_bias = i.id("26_fire.smoke_fwd_cargo_a", "false_bias_pct_per_ft");
        let deaf = i.id("26_fire.smoke_aft_cargo_a", "sensitivity_loss");
        assert!(false_bias != 0 && deaf != 0);

        let truth = truth_with(&[("CARGO_AFT_SMOKE_DENSITY_KG_M3", 5.0e-5)]);
        let healthy = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert_eq!(healthy["DEEP_SMOKE_FWD_CARGO_A_ALARM"], 0.0, "clean air must not alarm");
        assert_eq!(healthy["DEEP_SMOKE_AFT_CARGO_A_ALARM"], 1.0, "a real fire must alarm");
        assert!(healthy["DEEP_SMOKE_AFT_CARGO_A_PCT_PER_FT"] > 2.0);

        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(false_bias, 1.0), (deaf, 0.95)]), 2);
        assert_eq!(out["DEEP_SMOKE_FWD_CARGO_A_ALARM"], 1.0, "a spurious signal must be able to alarm on clean air");
        assert_eq!(out["DEEP_SMOKE_AFT_CARGO_A_ALARM"], 0.0, "desensitised optics must be able to miss a real fire");
        assert_eq!(out["DEEP_SMOKE_AFT_CARGO_B_ALARM"], 1.0, "the redundant detector in the same bay must still see it");
    }

    #[test]
    fn a_lavatory_detector_senses_the_smoke_in_the_deck_air_it_samples() {
        let truth = truth_with(&[("THERMAL_ZONE_CABINUPPERDECK_SMOKE_CONCENTRATION", 1.0e-4)]);
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert_eq!(out["DEEP_SMOKE_LAV_5_ALARM"], 1.0, "an upper-deck lavatory detector must see upper-deck smoke");
        assert_eq!(out["DEEP_SMOKE_LAV_1_ALARM"], 0.0, "a main-deck one must not");
    }

    #[test]
    fn an_armed_hydraulic_transducer_fault_freezes_or_drifts_its_published_pressure() {
        let i = index();
        let stuck = i.id("29_hyd.pressure_green", "stuck");
        let drift = i.id("29_hyd.pressure_yellow", "drift_rate_pa_per_hr");
        assert!(stuck != 0 && drift != 0);
        let pressurised = truth_with(&[
            ("HYD_GREEN_MANIFOLD_PRESSURE_PSI", 5000.0),
            ("HYD_YELLOW_MANIFOLD_PRESSURE_PSI", 5000.0),
        ]);
        let lost =
            truth_with(&[("HYD_GREEN_MANIFOLD_PRESSURE_PSI", 0.0), ("HYD_YELLOW_MANIFOLD_PRESSURE_PSI", 5000.0)]);

        let mut healthy = fresh();
        run(&mut healthy, &pressurised, &Faults::default(), 2);
        let healthy_out = run(&mut healthy, &lost, &Faults::default(), 2);
        assert!(healthy_out["DEEP_HYD_GREEN_PRESSURE_SENSED_PA"] < 1.0, "a healthy transducer follows the loss");

        let armed = Faults::from_pairs([(stuck, 1.0), (drift, 1.0)]);
        let mut frozen = fresh();
        run(&mut frozen, &pressurised, &Faults::default(), 2);
        let out = run(&mut frozen, &lost, &armed, 2);
        assert!(
            (out["DEEP_HYD_GREEN_PRESSURE_SENSED_PA"] - 5000.0 * PSI_TO_PA).abs() < 1.0,
            "a stuck transducer must go on reporting the pressure that is gone: {}",
            out["DEEP_HYD_GREEN_PRESSURE_SENSED_PA"]
        );
        assert!(
            out["DEEP_HYD_YELLOW_PRESSURE_SENSED_PA"] > 5000.0 * PSI_TO_PA,
            "a drifting transducer must read high, masking a falling system"
        );
    }

    #[test]
    fn an_armed_reservoir_sender_fault_hides_a_real_leak_or_reads_zero() {
        let i = index();
        let seized = i.id("29_hyd.reservoir_quantity_green", "float_stuck");
        let open = i.id("29_hyd.reservoir_quantity_yellow", "open_circuit");
        assert!(seized != 0 && open != 0);
        let full = truth_with(&[
            ("HYD_GREEN_RESERVOIR_LEVEL_FRACTION", 0.95),
            ("HYD_GREEN_FLUID_TEMP_C", 15.0),
            ("HYD_YELLOW_RESERVOIR_LEVEL_FRACTION", 0.95),
            ("HYD_YELLOW_FLUID_TEMP_C", 15.0),
        ]);
        let leaked = truth_with(&[
            ("HYD_GREEN_RESERVOIR_LEVEL_FRACTION", 0.05),
            ("HYD_GREEN_FLUID_TEMP_C", 15.0),
            ("HYD_YELLOW_RESERVOIR_LEVEL_FRACTION", 0.95),
            ("HYD_YELLOW_FLUID_TEMP_C", 15.0),
        ]);

        let mut healthy = fresh();
        run(&mut healthy, &full, &Faults::default(), 100);
        let healthy_out = run(&mut healthy, &leaked, &Faults::default(), 200);
        assert!(healthy_out["DEEP_HYD_GREEN_RESERVOIR_QTY_INDICATED"] < 0.1);

        let armed = Faults::from_pairs([(seized, 1.0), (open, 1.0)]);
        let mut broken = fresh();
        run(&mut broken, &full, &armed, 100);
        let out = run(&mut broken, &leaked, &armed, 200);
        assert!(
            out["DEEP_HYD_GREEN_RESERVOIR_QTY_INDICATED"] > 0.9,
            "a seized float must go on showing the fluid that has drained away: {}",
            out["DEEP_HYD_GREEN_RESERVOIR_QTY_INDICATED"]
        );
        assert_eq!(out["DEEP_HYD_YELLOW_RESERVOIR_QTY_INDICATED"], 0.0, "an open sender must fail safe to zero");
        assert_eq!(out["DEEP_HYD_GREEN_RESERVOIR_QTY_STUCK"], 1.0);
    }

    #[test]
    fn an_armed_duct_temperature_open_circuit_pegs_only_its_own_duct() {
        let id = index().id("36_pneu.duct_temp_wing_bleed_left", "open_circuit");
        assert_ne!(id, 0);
        let truth = truth_with(&[
            ("DEEP_PNEU_WAI_L_DUCT_TEMPERATURE_C", 180.0),
            ("DEEP_PNEU_WAI_R_DUCT_TEMPERATURE_C", 180.0),
        ]);
        let healthy = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert!((healthy["DEEP_DUCT_TEMP_WING_BLEED_LEFT_C"] - 180.0).abs() < 1e-6);

        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(id, 1.0)]), 2);
        assert_eq!(out["DEEP_DUCT_TEMP_WING_BLEED_LEFT_C"], DUCT_TEMP_RANGE_C.1);
        assert!((out["DEEP_DUCT_TEMP_WING_BLEED_RIGHT_C"] - 180.0).abs() < 1e-6);
    }

    #[test]
    fn a_healthy_trim_air_duct_temperature_sensor_reads_the_duct_it_is_bolted_to() {
        let truth = truth_with(&[("DEEP_THERM_TRIM_AIR_DUCT_TEMPERATURE_C", 70.0)]);
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert!((out["DEEP_DUCT_TEMP_TRIM_AIR_DUCT_C"] - 70.0).abs() < 1e-6);

        let short_id = index().id("36_pneu.duct_temp_trim_air_duct", "short_circuit");
        assert_ne!(short_id, 0);
        let shorted = run(&mut fresh(), &truth, &Faults::from_pairs([(short_id, 1.0)]), 2);
        assert_eq!(shorted["DEEP_DUCT_TEMP_TRIM_AIR_DUCT_C"], DUCT_TEMP_RANGE_C.0);
    }

    #[test]
    fn an_armed_core_speed_pickup_fault_costs_one_eec_channel_its_signal() {
        let i = index();
        let open = i.id("77_eng.speed_n2_3_b", "open_circuit");
        let gap = i.id("77_eng.speed_n3_1_a", "air_gap_increase");
        assert!(open != 0 && gap != 0);
        let truth = Truth { engine_n2_frac: [0.9; 4], engine_n3_frac: [0.9; 4], ..Truth::default() };

        let healthy = run(&mut fresh(), &truth, &Faults::default(), 1);
        assert_eq!(healthy["DEEP_ENG_3_N2_PICKUP_B_VALID"], 1.0);
        assert_eq!(healthy["DEEP_ENG_1_N3_PICKUP_A_VALID"], 1.0);

        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(open, 1.0), (gap, 1.0)]), 1);
        assert_eq!(out["DEEP_ENG_3_N2_PICKUP_B_VALID"], 0.0);
        assert_eq!(out["DEEP_ENG_3_N2_PICKUP_A_VALID"], 1.0, "the other EEC channel has its own pickup");
        assert_eq!(out["DEEP_ENG_3_N3_PICKUP_B_VALID"], 1.0, "and the other spool has its own too");
        assert_eq!(out["DEEP_ENG_1_N3_PICKUP_A_VALID"], 0.0, "a fully open air gap cannot reach the detection floor");
        assert_eq!(out["DEEP_ENG_2_N3_PICKUP_A_VALID"], 1.0);
    }

    #[test]
    fn a_healthy_vibration_pickup_reads_the_case_motion_at_its_own_location() {
        let truth = truth_with(&[
            ("A32NX_ENG_1_N1_VIB_INDEX", 2.5),
            ("A32NX_ENG_1_N2_VIB_INDEX", 3.0),
            ("A32NX_ENG_1_N3_VIB_INDEX", 4.0),
        ]);
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert!((out["DEEP_ENG_1_VIB_FAN_INDEX"] - 2.5).abs() < 1e-9);
        assert!((out["DEEP_ENG_1_VIB_CORE_INDEX"] - 5.0).abs() < 1e-9, "{}", out["DEEP_ENG_1_VIB_CORE_INDEX"]);
        assert_eq!(out["DEEP_ENG_2_VIB_FAN_INDEX"], 0.0, "engine 2 is smooth and has its own pickups");
        assert_eq!(out["DEEP_ENG_1_VIB_FAN_VALID"], 1.0);
    }

    #[test]
    fn an_armed_vibration_pickup_fault_fabricates_freezes_or_drops_its_reading() {
        let i = index();
        let bias = i.id("77_eng.vibration_1_fan", "bias");
        let stuck = i.id("77_eng.vibration_2_core", "stuck");
        let dropout = i.id("77_eng.vibration_3_fan", "intermittent_dropout_rate_per_s");
        assert!(bias != 0 && stuck != 0 && dropout != 0);

        let smooth = truth_with(&[("A32NX_ENG_2_N2_VIB_INDEX", 0.0)]);
        let healthy = run(&mut fresh(), &smooth, &Faults::default(), 4);
        assert_eq!(healthy["DEEP_ENG_1_VIB_FAN_INDEX"], 0.0);

        let armed = Faults::from_pairs([(bias, 1.0), (stuck, 1.0)]);
        let out = run(&mut fresh(), &smooth, &armed, 4);
        assert!(
            out["DEEP_ENG_1_VIB_FAN_INDEX"] > 4.0,
            "a fully armed bias must be able to fabricate a vibration indication: {}",
            out["DEEP_ENG_1_VIB_FAN_INDEX"]
        );
        assert_eq!(out["DEEP_ENG_1_VIB_CORE_INDEX"], 0.0, "the other pickup on the same engine is a separate part");

        let rough = truth_with(&[("A32NX_ENG_2_N2_VIB_INDEX", 9.0), ("A32NX_ENG_2_N3_VIB_INDEX", 0.0)]);
        let mut frozen = fresh();
        run(&mut frozen, &smooth, &Faults::default(), 4);
        let out = run(&mut frozen, &rough, &Faults::from_pairs([(stuck, 1.0)]), 20);
        assert_eq!(out["DEEP_ENG_2_VIB_CORE_INDEX"], 0.0, "a stuck pickup must not follow the engine going rough");
        assert!(
            run(&mut fresh(), &rough, &Faults::default(), 4)["DEEP_ENG_2_VIB_CORE_INDEX"] > 8.0,
            "and a healthy one must"
        );

        let mut flaky = fresh();
        let mut dropped = 0;
        let armed = Faults::from_pairs([(dropout, 1.0)]);
        for _ in 0..400 {
            flaky.tick(&smooth, &armed);
            let mut valid = 1.0;
            flaky.publish(&mut |n, v| {
                if n == "DEEP_ENG_3_VIB_FAN_VALID" {
                    valid = v;
                }
            });
            if valid == 0.0 {
                dropped += 1;
            }
        }
        assert!(dropped > 0, "a loose connector must actually drop out");
        assert!(dropped < 400, "and must be intermittent, not a permanent loss");
    }

    #[test]
    fn a_healthy_oil_transducer_pair_reads_its_own_engines_own_oil() {
        let truth = Truth {
            engine_oil_pressure_pa: [200_000.0, 400_000.0, 600_000.0, 800_000.0],
            engine_oil_temp_c: [40.0, 60.0, 80.0, 100.0],
            ..Truth::default()
        };
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        for e in 1..=4 {
            assert!((out[&format!("DEEP_ENG_{e}_OIL_PRESSURE_SENSED_PA")] - e as f64 * 200_000.0).abs() < 1e-6);
            assert!((out[&format!("DEEP_ENG_{e}_OIL_TEMP_SENSED_C")] - (20.0 + e as f64 * 20.0)).abs() < 1e-6);
        }
    }

    #[test]
    fn an_armed_oil_sensor_fault_freezes_or_pegs_only_its_own_engine() {
        let i = index();
        let stuck = i.id("79_oil.pressure_1", "stuck");
        let open = i.id("79_oil.temperature_2", "open_circuit");
        let short = i.id("79_oil.temperature_3", "short_circuit");
        assert!(stuck != 0 && open != 0 && short != 0);

        let running = Truth {
            engine_oil_pressure_pa: [520_000.0; 4],
            engine_oil_temp_c: [95.0; 4],
            ..Truth::default()
        };
        let lost = Truth { engine_oil_pressure_pa: [0.0; 4], ..running.clone() };

        let mut healthy = fresh();
        run(&mut healthy, &running, &Faults::default(), 2);
        let healthy_out = run(&mut healthy, &lost, &Faults::default(), 2);
        assert!(healthy_out["DEEP_ENG_1_OIL_PRESSURE_SENSED_PA"] < 1.0);

        let armed = Faults::from_pairs([(stuck, 1.0), (open, 1.0), (short, 1.0)]);
        let mut broken = fresh();
        run(&mut broken, &running, &Faults::default(), 2);
        let out = run(&mut broken, &lost, &armed, 2);
        assert!(
            (out["DEEP_ENG_1_OIL_PRESSURE_SENSED_PA"] - 520_000.0).abs() < 1.0,
            "a seized transducer must go on showing the oil pressure that is gone: {}",
            out["DEEP_ENG_1_OIL_PRESSURE_SENSED_PA"]
        );
        assert!(out["DEEP_ENG_2_OIL_PRESSURE_SENSED_PA"] < 1.0, "engine 2's own transducer is healthy");
        assert_eq!(out["DEEP_ENG_2_OIL_TEMP_SENSED_C"], OIL_TEMP_RANGE_C.1, "an open circuit must peg high");
        assert_eq!(out["DEEP_ENG_3_OIL_TEMP_SENSED_C"], OIL_TEMP_RANGE_C.0, "a short must peg low");
        assert!((out["DEEP_ENG_4_OIL_TEMP_SENSED_C"] - 95.0).abs() < 1e-6, "it reached an engine it was not armed on");
    }

    #[test]
    fn a_healthy_tyre_transducer_reads_the_tyre_it_is_screwed_into() {
        let mut pressures = [0.0f64; crate::physics::tyre::WHEELS];
        for (w, p) in pressures.iter_mut().enumerate() {
            *p = 1_400_000.0 + w as f64 * 10_000.0;
        }
        let truth = Truth { tyre_pressure_pa: pressures, ..Truth::default() };
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        for (w, &wheel) in BRAKE_WHEEL_INDEX.iter().enumerate() {
            let expected = 1_400_000.0 + wheel as f64 * 10_000.0;
            let got = out[&format!("DEEP_TYRE_PRESSURE_SENSED_PA:{}", w + 1)];
            assert!((got - expected).abs() < 1e-6, "sensor {} read {got}, tyre {wheel} is at {expected}", w + 1);
        }
    }

    #[test]
    fn a_parked_aircraft_stands_on_serviced_tyres_not_flat_ones() {
        let sensors = fresh();
        let mut out = BTreeMap::new();
        sensors.publish(&mut |n, v| {
            out.insert(n.to_string(), v);
        });
        for w in 1..=16 {
            assert!(
                (out[&format!("DEEP_TYRE_PRESSURE_SENSED_PA:{w}")] - crate::physics::tyre::COLD_PRESSURE_PA).abs()
                    < 1.0,
                "wheel {w} starts at {}",
                out[&format!("DEEP_TYRE_PRESSURE_SENSED_PA:{w}")]
            );
        }
    }

    #[test]
    fn an_armed_tyre_transducer_fault_hides_a_deflating_tyre() {
        let i = index();
        let stuck = i.id("32_gear.brake_temp_left_wing_1", "open_circuit");
        assert_ne!(stuck, 0);
        let seized = i.id("32_gear.tyre_pressure_left_wing_1", "stuck");
        let drift = i.id("32_gear.tyre_pressure_left_wing_2", "drift_rate_pa_per_hr");
        assert!(seized != 0 && drift != 0);

        let serviced = Truth { tyre_pressure_pa: [crate::physics::tyre::COLD_PRESSURE_PA; crate::physics::tyre::WHEELS], ..Truth::default() };
        let mut deflated_pressures = [crate::physics::tyre::COLD_PRESSURE_PA; crate::physics::tyre::WHEELS];
        deflated_pressures[BRAKE_WHEEL_INDEX[0]] = 700_000.0;
        deflated_pressures[BRAKE_WHEEL_INDEX[1]] = 700_000.0;
        let deflated = Truth { tyre_pressure_pa: deflated_pressures, ..Truth::default() };

        let mut healthy = fresh();
        run(&mut healthy, &serviced, &Faults::default(), 2);
        let healthy_out = run(&mut healthy, &deflated, &Faults::default(), 2);
        assert!((healthy_out["DEEP_TYRE_PRESSURE_SENSED_PA:1"] - 700_000.0).abs() < 1.0);

        let mut broken = fresh();
        run(&mut broken, &serviced, &Faults::default(), 2);
        let slow = Truth { dt_s: 60.0, ..deflated.clone() };
        let out = run(&mut broken, &slow, &Faults::from_pairs([(seized, 1.0), (drift, 1.0)]), 60);
        assert!(
            (out["DEEP_TYRE_PRESSURE_SENSED_PA:1"] - crate::physics::tyre::COLD_PRESSURE_PA).abs() < 1.0,
            "a seized transducer must go on claiming a serviced tyre: {}",
            out["DEEP_TYRE_PRESSURE_SENSED_PA:1"]
        );
        let drifted = out["DEEP_TYRE_PRESSURE_SENSED_PA:2"];
        let expected = 700_000.0 + TRANSDUCER_DRIFT_FULL_SCALE_FRACTION_PER_HR * TYRE_PRESSURE_FULL_SCALE_PA;
        assert!((drifted - expected).abs() < 1.0, "a drifting transducer must read high: {drifted} against {expected}");
        assert!((out["DEEP_TYRE_PRESSURE_SENSED_PA:3"] - crate::physics::tyre::COLD_PRESSURE_PA).abs() < 1.0);
    }

    #[test]
    fn the_nose_tyres_now_have_their_own_transducer_reading_indices_16_and_17() {
        assert!(
            !BLOCKED.iter().any(|(c, _, _)| *c == "32_gear.tyre_pressure_nose_"),
            "the nose tyre pressure row must be gone from BLOCKED now that Truth::tyre_pressure_pa covers it"
        );

        let mut pressures = [crate::physics::tyre::COLD_PRESSURE_PA; crate::physics::tyre::WHEELS];
        pressures[crate::physics::tyre::BRAKED_WHEELS] = 900_000.0;
        pressures[crate::physics::tyre::BRAKED_WHEELS + 1] = 950_000.0;
        let truth = Truth { tyre_pressure_pa: pressures, ..Truth::default() };
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert!((out["DEEP_TYRE_PRESSURE_SENSED_PA:NOSE_1"] - 900_000.0).abs() < 1e-6);
        assert!((out["DEEP_TYRE_PRESSURE_SENSED_PA:NOSE_2"] - 950_000.0).abs() < 1e-6);
        assert!((out["DEEP_TYRE_PRESSURE_SENSED_PA:1"] - crate::physics::tyre::COLD_PRESSURE_PA).abs() < 1.0);

        let names: Vec<String> = {
            let mut v = Vec::new();
            fresh().publish(&mut |n, _| v.push(n.to_string()));
            v
        };
        let tyre_vars = names.iter().filter(|n| n.starts_with("DEEP_TYRE_PRESSURE_SENSED_PA:")).count();
        assert_eq!(tyre_vars, 18, "sixteen main wheels plus the two nose wheels, no more");

        let i = index();
        let stuck = i.id("32_gear.tyre_pressure_nose_1", "stuck");
        assert_ne!(stuck, 0);
        let mut broken = fresh();
        run(&mut broken, &truth, &Faults::default(), 2);
        let deflating =
            Truth { tyre_pressure_pa: { let mut p = pressures; p[crate::physics::tyre::BRAKED_WHEELS] = 100_000.0; p }, ..Truth::default() };
        let out = run(&mut broken, &deflating, &Faults::from_pairs([(stuck, 1.0)]), 2);
        assert!(
            (out["DEEP_TYRE_PRESSURE_SENSED_PA:NOSE_1"] - 900_000.0).abs() < 1.0,
            "a stuck nose transducer must go on claiming the pressure that is gone: {}",
            out["DEEP_TYRE_PRESSURE_SENSED_PA:NOSE_1"]
        );
    }

    #[test]
    fn an_armed_fuel_flow_transmitter_fault_under_reads_the_fuel_the_engine_burns() {
        let i = index();
        let seized = i.id("73_fuel.flow_transmitter_1", "stuck_rotor");
        let worn = i.id("73_fuel.flow_transmitter_2", "bearing_wear");
        let blocked = i.id("73_fuel.flow_transmitter_3", "debris_blockage");
        assert!(seized != 0 && worn != 0 && blocked != 0);
        let truth = Truth { engine_fuel_flow_kg_s: [2.0; 4], ..Truth::default() };

        let healthy = run(&mut fresh(), &truth, &Faults::default(), 1);
        for e in 1..=4 {
            assert!((healthy[&format!("DEEP_FF_XMTR_{e}_INDICATED_KG_S")] - 2.0).abs() < 1e-9);
        }

        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(seized, 1.0), (worn, 0.4), (blocked, 0.25)]), 1);
        assert_eq!(out["DEEP_FF_XMTR_1_INDICATED_KG_S"], 0.0);
        assert!((out["DEEP_FF_XMTR_2_INDICATED_KG_S"] - 1.2).abs() < 1e-9, "{}", out["DEEP_FF_XMTR_2_INDICATED_KG_S"]);
        assert!((out["DEEP_FF_XMTR_3_INDICATED_KG_S"] - 1.5).abs() < 1e-9);
        assert!((out["DEEP_FF_XMTR_4_INDICATED_KG_S"] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn a_seized_rotor_raises_a_fault_discrete_only_while_the_engine_is_actually_running() {
        let seized = index().id("73_fuel.flow_transmitter_1", "stuck_rotor");
        assert_ne!(seized, 0);

        let running = Truth { engine_fuel_flow_kg_s: [2.0; 4], ..Truth::default() };
        let out = run(&mut fresh(), &running, &Faults::from_pairs([(seized, 1.0)]), 1);
        assert_eq!(out["DEEP_FF_XMTR_1_FAULT"], 1.0, "a stuck rotor under-reading a running engine is a fault");
        assert_eq!(out["DEEP_FF_XMTR_2_FAULT"], 0.0, "the healthy engine 2 transmitter must not fault");

        let shutdown = Truth { engine_fuel_flow_kg_s: [0.0; 4], ..Truth::default() };
        let out = run(&mut fresh(), &shutdown, &Faults::default(), 1);
        assert_eq!(
            out["DEEP_FF_XMTR_1_FAULT"], 0.0,
            "a shut-down engine's own healthy zero reading must not be flagged as a fault"
        );
    }

    #[test]
    fn an_armed_cpc_transducer_drift_biases_one_controllers_cabin_pressure_only() {
        let id = index().id("21_cab.cpc_1_absolute_pressure", "drift_rate_pa_per_hr");
        assert_ne!(id, 0);
        let truth = Truth { dt_s: 60.0, cabin_pressure_pa: 80_000.0, ..Truth::default() };
        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(id, 1.0)]), 60);
        let expected = 80_000.0 + TRANSDUCER_DRIFT_FULL_SCALE_FRACTION_PER_HR * CABIN_ABSOLUTE_FULL_SCALE_PA;
        assert!(
            (out["DEEP_CPC_1_CABIN_PRESSURE_SENSED_PA"] - expected).abs() < 1.0,
            "{}",
            out["DEEP_CPC_1_CABIN_PRESSURE_SENSED_PA"]
        );
        assert!(
            (out["DEEP_CPC_2_CABIN_PRESSURE_SENSED_PA"] - 80_000.0).abs() < 1e-6,
            "the redundant CPC must be unaffected"
        );
    }

    #[test]
    fn an_armed_cargo_door_proximity_fault_claims_a_closed_door_that_is_open() {
        let id = index().id("52_doors.prox_cargo_16_closed", "stuck_near");
        assert_ne!(id, 0);
        let open = truth_with(&[("CABIN_CARGO_DOOR_PERCENT:1", 100.0)]);
        let healthy = run(&mut fresh(), &open, &Faults::default(), 2);
        assert_eq!(healthy["DEEP_PROX_DOOR_CARGO_16_CLOSED_NEAR"], 0.0);
        assert_eq!(healthy["DEEP_PROX_DOOR_CARGO_16_OPEN_NEAR"], 1.0);

        let out = run(&mut fresh(), &open, &Faults::from_pairs([(id, 1.0)]), 2);
        assert_eq!(out["DEEP_PROX_DOOR_CARGO_16_CLOSED_NEAR"], 1.0);
        assert_eq!(out["DEEP_PROX_DOOR_CARGO_16_OPEN_NEAR"], 1.0, "the open sensor is a different part and still true");
    }

    #[test]
    fn a_passenger_and_the_aft_cargo_door_proximity_sensor_follow_their_own_doors_true_position() {
        for row in ["52_doors.prox_m1l_", "52_doors.prox_m2l_", "52_doors.prox_m2r_", "52_doors.prox_m4l_", "52_doors.prox_m5l_", "52_doors.prox_u1l_", "52_doors.prox_cargo_17_"] {
            assert!(!BLOCKED.iter().any(|(c, _, _)| *c == row), "{row} must be gone from BLOCKED");
        }

        let mut fractions = [0.0; DOOR_NAMES.len()];
        fractions[0] = 1.0;
        fractions[7] = 0.0;
        let truth = Truth { door_open_fraction: fractions, ..Truth::default() };
        let out = run(&mut fresh(), &truth, &Faults::default(), 2);
        assert_eq!(out["DEEP_PROX_DOOR_M1L_OPEN_NEAR"], 1.0, "M1L is open");
        assert_eq!(out["DEEP_PROX_DOOR_M1L_CLOSED_NEAR"], 0.0);
        assert_eq!(out["DEEP_PROX_DOOR_CARGO_AFT_CLOSED_NEAR"], 1.0, "the aft cargo door is shut");
        assert_eq!(out["DEEP_PROX_DOOR_CARGO_AFT_OPEN_NEAR"], 0.0);
        assert_eq!(out["DEEP_PROX_DOOR_U1L_CLOSED_NEAR"], 1.0, "U1L was never opened");

        let mut swapped = [0.0; DOOR_NAMES.len()];
        swapped[5] = 1.0;
        let swapped_truth = Truth { door_open_fraction: swapped, ..Truth::default() };
        let out = run(&mut fresh(), &swapped_truth, &Faults::default(), 2);
        assert_eq!(out["DEEP_PROX_DOOR_U1L_OPEN_NEAR"], 1.0);
        assert_eq!(out["DEEP_PROX_DOOR_M1L_OPEN_NEAR"], 0.0, "M1L is shut this time");
    }

    #[test]
    fn an_engine_oil_quantity_probe_tracks_the_tank_and_a_fault_hides_or_zeros_it() {
        assert!(!BLOCKED.iter().any(|(c, _, _)| *c == "79_oil.quantity_"));
        let truth = Truth { engine_oil_quantity_fraction: [1.0, 0.6, 0.6, 0.6], ..Truth::default() };
        let healthy = run(&mut fresh(), &truth, &Faults::default(), 1);
        assert!((healthy["DEEP_ENG_1_OIL_QTY_SENSED_FRAC"] - 1.0).abs() < 1e-9);
        assert!((healthy["DEEP_ENG_2_OIL_QTY_SENSED_FRAC"] - 0.6).abs() < 1e-9);

        let i = index();
        let contamination = i.id("79_oil.quantity_2", "contamination_frac");
        let open = i.id("79_oil.quantity_3", "open_circuit");
        assert!(contamination != 0 && open != 0);
        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(contamination, 1.0), (open, 1.0)]), 1);
        assert!(out["DEEP_ENG_2_OIL_QTY_SENSED_FRAC"] > 0.6, "water contamination must read high");
        assert_eq!(out["DEEP_ENG_3_OIL_QTY_SENSED_FRAC"], 0.0, "an open circuit must read a conservative zero");
        assert!((out["DEEP_ENG_4_OIL_QTY_SENSED_FRAC"] - 0.6).abs() < 1e-9, "it reached an engine it was not armed on");
    }

    #[test]
    fn a_tgt_harness_averages_the_true_tgt_and_a_faulted_junction_biases_it() {
        assert!(!BLOCKED.iter().any(|(c, _, _)| *c == "77_eng.tgt_harness_"));
        let truth = Truth { engine_tgt_c: [650.0, 650.0, 650.0, 650.0], ..Truth::default() };
        let healthy = run(&mut fresh(), &truth, &Faults::default(), 1);
        assert!((healthy["DEEP_ENG_1_TGT_SENSED_C"] - 650.0).abs() < 1e-6);

        let drift = index().id("77_eng.tgt_harness_2", "drift_k");
        assert_ne!(drift, 0);
        let armed = Faults::from_pairs([(drift, 1.0)]);
        let out = run(&mut fresh(), &truth, &armed, 1);
        let expected = 650.0 + TGT_DRIFT_FULL_SCALE_K / TGT_JUNCTION_COUNT as f64;
        assert!((out["DEEP_ENG_2_TGT_SENSED_C"] - expected).abs() < 1e-6, "{}", out["DEEP_ENG_2_TGT_SENSED_C"]);
        assert!((out["DEEP_ENG_1_TGT_SENSED_C"] - 650.0).abs() < 1e-6, "it reached an engine it was not armed on");
    }

    #[test]
    fn a_t25_probe_reads_its_own_engines_hp_compressor_inlet_and_pegs_on_a_fault() {
        assert!(!BLOCKED.iter().any(|(c, _, _)| *c == "77_eng.t25_"));
        let truth = Truth { engine_t25_c: [180.0, 220.0, 260.0, 300.0], ..Truth::default() };
        let healthy = run(&mut fresh(), &truth, &Faults::default(), 1);
        for e in 1..=4 {
            assert!((healthy[&format!("DEEP_ENG_{e}_T25_SENSED_C")] - (140.0 + e as f64 * 40.0)).abs() < 1e-6);
        }

        let i = index();
        let open = i.id("77_eng.t25_1", "open_circuit");
        let short = i.id("77_eng.t25_2", "short_circuit");
        assert!(open != 0 && short != 0);
        let out = run(&mut fresh(), &truth, &Faults::from_pairs([(open, 1.0), (short, 1.0)]), 1);
        assert_eq!(out["DEEP_ENG_1_T25_SENSED_C"], T25_RANGE_C.1, "an open circuit must peg high");
        assert_eq!(out["DEEP_ENG_2_T25_SENSED_C"], T25_RANGE_C.0, "a short must peg low");
        assert!((out["DEEP_ENG_3_T25_SENSED_C"] - 260.0).abs() < 1e-6, "it reached an engine it was not armed on");
    }

    #[test]
    fn the_brake_sensors_read_the_real_aircrafts_brakes_one_frame_behind() {
        use crate::deep::integration::failure_audit::{baseline, profiles, reference_faults};
        let profile = profiles().into_iter().find(|p| p.name == "touchdown").expect("the touchdown profile");
        let base = baseline(&(profile.truth)(), &reference_faults(), profile.frames);
        let frame = |f: usize, name: &str| {
            let i = base.names.iter().position(|n| n == name).unwrap_or_else(|| panic!("{name} is never published"));
            base.frames[f][i]
        };
        let n = base.frames.len() - 1;
        let at = |name: &str| frame(n, name);

        assert!(at("BRAKE_STACK_TEMP_C:1") > 20.0, "the rollout should have heated the brakes");
        assert!(
            at("BRAKE_STACK_TEMP_C:1") - frame(n - 1, "BRAKE_STACK_TEMP_C:1") > 0.1,
            "and they have to still be heating, or a frame of lag would be invisible"
        );
        for (w, &gear_index) in BRAKE_WHEEL_INDEX.iter().enumerate() {
            let real = frame(n - 1, &format!("BRAKE_STACK_TEMP_C:{}", gear_index + 1));
            let sensed = at(&format!("DEEP_BRAKE_TEMP_SENSED_C:{}", w + 1));
            assert!((sensed - real).abs() < 1e-9, "wheel {} sensed {sensed} against a real {real}", w + 1);
        }
        assert_eq!(at("GEAR_DOWNLOCKED:2"), 1.0);
        assert_eq!(at("DEEP_PROX_GEAR_LEFT_WING_DOWNLOCK_NEAR"), 1.0);
        assert_eq!(at("DEEP_PROX_GEAR_LEFT_WING_UPLOCK_NEAR"), 0.0);
        assert_eq!(at("DEEP_PROX_GEAR_LEFT_WING_WOW_NEAR"), 1.0);
    }

    #[test]
    fn a_cold_dark_aircraft_publishes_nothing_but_finite_numbers() {
        let out = run(&mut fresh(), &Truth::default(), &Faults::default(), 120);
        assert!(!out.is_empty());
        for (name, value) in &out {
            assert!(value.is_finite(), "{name} = {value}");
        }
        for n in 1..=8 {
            assert_eq!(out[&format!("DEEP_SMOKE_LAV_{n}_ALARM")], 0.0);
        }
        assert_eq!(out["DEEP_SMOKE_FWD_CARGO_A_ALARM"], 0.0);
    }

    #[test]
    fn the_published_name_set_is_stable_and_free_of_duplicates() {
        let mut s = fresh();
        let mut names = Vec::new();
        s.publish(&mut |n, _| names.push(n.to_string()));
        let unique: BTreeSet<&String> = names.iter().collect();
        assert_eq!(unique.len(), names.len(), "a variable is published twice");

        s.tick(&Truth::default(), &Faults::default());
        let mut after = Vec::new();
        s.publish(&mut |n, _| after.push(n.to_string()));
        assert_eq!(names, after);
    }

    #[test]
    fn beer_lambert_obscuration_is_monotone_bounded_and_zero_in_clean_air() {
        assert_eq!(obscuration_pct_per_ft(0.0), 0.0);
        assert_eq!(obscuration_pct_per_ft(-1.0), 0.0, "a negative concentration is not negative light");
        assert!(obscuration_pct_per_ft(1.0e-5) < obscuration_pct_per_ft(1.0e-4));
        assert!(obscuration_pct_per_ft(1.0) <= 100.0);
        assert!((obscuration_pct_per_ft(7.62e-6) - 2.0).abs() < 0.05, "{}", obscuration_pct_per_ft(7.62e-6));
    }

    #[test]
    #[ignore = "tens of seconds: every sensor failure against the whole profile set"]
    fn sensor_failures_that_move_nothing() {
        use crate::deep::api::Area as RegArea;
        use crate::deep::integration::failure_audit::sweep;
        let r = crate::deep::registry();
        let mine: Vec<_> = r.failures.iter().filter(|f| f.area == RegArea::Sensors).cloned().collect();
        let verdicts = sweep(&mine, &mut |_, _, _| {});
        let live = verdicts.iter().filter(|v| v.is_live()).count();
        let mut per_family: BTreeMap<String, (usize, usize)> = BTreeMap::new();
        for v in &verdicts {
            let family = v.component.split_once('.').map_or(v.component.clone(), |(a, b)| {
                format!("{a}.{}", b.trim_end_matches(|c: char| c.is_ascii_digit() || c == '_'))
            });
            let e = per_family.entry(family).or_default();
            e.0 += 1;
            if v.is_live() {
                e.1 += 1;
            }
        }
        println!("SENSORS {live} of {} registered failures move something published", verdicts.len());
        for (family, (total, live)) in &per_family {
            println!("SENSORS   {family:<34} live {live:>4} / {total:>4}");
        }
        for v in verdicts.iter().filter(|v| !v.is_live()) {
            println!("SENSORS   dead {} {} | {}", v.id, v.name, v.model_field);
        }
    }

    #[test]
    #[ignore = "a timing measurement, not a check"]
    fn what_a_frame_of_discrete_sensing_costs() {
        use crate::deep::integration::failure_audit::{baseline, profiles, reference_faults};
        use std::time::Instant;

        let profile = profiles().into_iter().find(|p| p.name == "cruise").expect("the cruise profile");
        let mut truth = (profile.truth)();
        let base = baseline(&truth, &reference_faults(), 2);
        let last = base.frames.last().expect("a frame");
        for (i, name) in base.names.iter().enumerate() {
            truth.published.insert(name.clone(), last[i]);
        }
        println!("TIMING published frame carries {} variables", truth.published.len());

        let faults = Faults::default();
        let mut sensors = fresh();
        for _ in 0..200 {
            sensors.tick(&truth, &faults);
        }
        const N: u32 = 20_000;
        let t0 = Instant::now();
        for _ in 0..N {
            sensors.tick(&truth, &faults);
            sensors.publish(&mut |_, _| {});
        }
        let per_frame_us = t0.elapsed().as_secs_f64() * 1e6 / N as f64;

        let mut air_data = crate::deep::sensors::live::LiveSensors::new();
        use crate::deep::live::Area as _;
        for _ in 0..200 {
            air_data.tick(&truth, &faults);
        }
        let t1 = Instant::now();
        for _ in 0..N {
            air_data.tick(&truth, &faults);
            air_data.publish(&mut |_, _| {});
        }
        let whole_area_us = t1.elapsed().as_secs_f64() * 1e6 / N as f64;

        let mut published = 0usize;
        sensors.publish(&mut |_, _| published += 1);
        println!("TIMING discrete set: {published} variables, {per_frame_us:.1} us/frame");
        println!("TIMING whole sensors area (air data + discrete): {whole_area_us:.1} us/frame");
        println!("TIMING at 60 Hz that is {:.2}% of a frame budget", whole_area_us * 60.0 / 10_000.0);
    }
}

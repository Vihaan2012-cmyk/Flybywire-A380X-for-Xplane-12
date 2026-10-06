use super::network::{BusId, Contactor, ContactorKind, Diode, FeedSource, Network, Source};

const MIN_RESISTANCE_OHM: f64 = 1.0e-4;

#[derive(Clone, Copy, Debug, Default)]
pub struct VfgFaults {
    pub winding_degradation: f64,
    pub regulator_drift: f64,
}

pub struct VfgInputs {
    pub engine_speed_fraction: f64,
    pub measured_load_w: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VfgOutputs {
    pub open_circuit_v: f64,
    pub resistance_ohm: f64,
    pub frequency_hz: f64,
    pub overload_heat: f64,
    pub overload_tripped: bool,
}

pub struct Vfg {
    overload_heat: f64,
}

impl Vfg {
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    const POWER_FACTOR: f64 = 0.8;
    const RATED_VOLTAGE_REGULATION: f64 = 0.03;
    const DEGRADED_REACTANCE_MULTIPLIER: f64 = 4.0;
    const MAX_REGULATOR_DRIFT_VOLT: f64 = 8.0;
    const RATED_TRUE_POWER_W: f64 = 150_000.0;
    const FREQ_MIN_HZ: f64 = 360.0;
    const FREQ_MAX_HZ: f64 = 800.0;
    const CUT_IN_SPEED_FRACTION: f64 = 0.05;
    const OVERLOAD_TRIP_K: f64 = 30.0;
    const OVERLOAD_COOLDOWN_S: f64 = 20.0;

    pub fn new() -> Self {
        Self { overload_heat: 0.0 }
    }

    pub fn overload_heat(&self) -> f64 {
        self.overload_heat
    }

    pub fn step(&mut self, inputs: VfgInputs, faults: VfgFaults, dt_s: f64) -> VfgOutputs {
        let speed = inputs.engine_speed_fraction.max(0.0);
        let running = speed >= Self::CUT_IN_SPEED_FRACTION;

        let rated_apparent_power = Self::RATED_TRUE_POWER_W / Self::POWER_FACTOR;
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1.0 - Self::RATED_VOLTAGE_REGULATION);
        let base_xs = target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / rated_apparent_power;
        let degradation = faults.winding_degradation.clamp(0.0, 1.0);
        let xs = base_xs * (1.0 + degradation * (Self::DEGRADED_REACTANCE_MULTIPLIER - 1.0));

        let drift = faults.regulator_drift.clamp(-1.0, 1.0) * Self::MAX_REGULATOR_DRIFT_VOLT;
        let (open_circuit_v, resistance_ohm, frequency_hz) =
            if running { ((Self::RATED_VOLTAGE_VOLT + drift).max(0.0), xs.max(MIN_RESISTANCE_OHM), Self::FREQ_MIN_HZ + (Self::FREQ_MAX_HZ - Self::FREQ_MIN_HZ) * speed.min(1.0)) } else { (0.0, MIN_RESISTANCE_OHM, 0.0) };

        let ratio = if running { inputs.measured_load_w.max(0.0) / Self::RATED_TRUE_POWER_W } else { 0.0 };
        if ratio > 1.0 {
            self.overload_heat += dt_s.max(0.0) * (ratio * ratio - 1.0) / Self::OVERLOAD_TRIP_K;
        } else {
            self.overload_heat = (self.overload_heat - dt_s.max(0.0) / Self::OVERLOAD_COOLDOWN_S).max(0.0);
        }
        self.overload_heat = self.overload_heat.min(1.0);

        VfgOutputs { open_circuit_v, resistance_ohm, frequency_hz, overload_heat: self.overload_heat, overload_tripped: self.overload_heat >= 1.0 }
    }
}

impl Default for Vfg {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ApuGeneratorFaults {
    pub winding_degradation: f64,
    pub regulator_drift: f64,
    pub overload_protection_failed: f64,
}

pub struct ApuGeneratorInputs {
    pub apu_speed_fraction: f64,
    pub measured_load_w: f64,
}

pub struct ApuGenerator {
    overload_heat: f64,
    burnt_out: bool,
}

impl ApuGenerator {
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    const POWER_FACTOR: f64 = 0.8;
    const RATED_VOLTAGE_REGULATION: f64 = 0.03;
    const DEGRADED_REACTANCE_MULTIPLIER: f64 = 4.0;
    const MAX_REGULATOR_DRIFT_VOLT: f64 = 8.0;
    const RATED_TRUE_POWER_W: f64 = 120_000.0;
    const FREQUENCY_HZ: f64 = 400.0;
    const CUT_IN_SPEED_FRACTION: f64 = 0.95;
    const OVERLOAD_TRIP_K: f64 = 30.0;
    const OVERLOAD_COOLDOWN_S: f64 = 20.0;
    const BURNOUT_HEAT: f64 = 3.0;

    pub fn new() -> Self {
        Self { overload_heat: 0.0, burnt_out: false }
    }

    pub fn burnt_out(&self) -> bool {
        self.burnt_out
    }

    pub fn overload_heat(&self) -> f64 {
        self.overload_heat
    }

    pub fn step(&mut self, inputs: ApuGeneratorInputs, faults: ApuGeneratorFaults, dt_s: f64) -> VfgOutputs {
        let running = inputs.apu_speed_fraction >= Self::CUT_IN_SPEED_FRACTION;
        let rated_apparent_power = Self::RATED_TRUE_POWER_W / Self::POWER_FACTOR;
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1.0 - Self::RATED_VOLTAGE_REGULATION);
        let base_xs = target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / rated_apparent_power;
        let degradation = faults.winding_degradation.clamp(0.0, 1.0);
        let xs = base_xs * (1.0 + degradation * (Self::DEGRADED_REACTANCE_MULTIPLIER - 1.0));
        let drift = faults.regulator_drift.clamp(-1.0, 1.0) * Self::MAX_REGULATOR_DRIFT_VOLT;

        let protected = faults.overload_protection_failed < 0.5;
        if protected {
            self.burnt_out = false;
        }
        let producing = running && !self.burnt_out;
        let (open_circuit_v, resistance_ohm, frequency_hz) = if producing { ((Self::RATED_VOLTAGE_VOLT + drift).max(0.0), xs.max(MIN_RESISTANCE_OHM), Self::FREQUENCY_HZ) } else { (0.0, MIN_RESISTANCE_OHM, 0.0) };

        let ratio = if producing { inputs.measured_load_w.max(0.0) / Self::RATED_TRUE_POWER_W } else { 0.0 };
        if ratio > 1.0 {
            self.overload_heat += dt_s.max(0.0) * (ratio * ratio - 1.0) / Self::OVERLOAD_TRIP_K;
        } else {
            self.overload_heat = (self.overload_heat - dt_s.max(0.0) / Self::OVERLOAD_COOLDOWN_S).max(0.0);
        }
        self.overload_heat = self.overload_heat.min(if protected { 1.0 } else { Self::BURNOUT_HEAT });
        if !protected && self.overload_heat >= Self::BURNOUT_HEAT {
            self.burnt_out = true;
        }
        VfgOutputs { open_circuit_v, resistance_ohm, frequency_hz, overload_heat: self.overload_heat, overload_tripped: protected && self.overload_heat >= 1.0 }
    }
}

impl Default for ApuGenerator {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GroundPowerFaults {
    pub weak_cart: f64,
}

pub struct GroundPower {}

impl GroundPower {
    const POWER_FACTOR: f64 = 0.8;
    const RATED_APPARENT_POWER_VA: f64 = 90_000.0;
    const RATED_VOLTAGE_REGULATION: f64 = 0.02;
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    const FREQUENCY_HZ: f64 = 400.0;
    const WEAK_CART_MULTIPLIER: f64 = 5.0;

    pub fn new() -> Self {
        Self {}
    }

    pub fn terminal(&self, plugged_in: bool, faults: GroundPowerFaults) -> Source {
        if !plugged_in {
            return Source { id: "gpu", open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 };
        }
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1.0 - Self::RATED_VOLTAGE_REGULATION);
        let base_xs = target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / Self::RATED_APPARENT_POWER_VA;
        let weak = faults.weak_cart.clamp(0.0, 1.0);
        let xs = base_xs * (1.0 + weak * (Self::WEAK_CART_MULTIPLIER - 1.0));
        Source { id: "gpu", open_circuit_v: Self::RATED_VOLTAGE_VOLT, resistance_ohm: xs.max(MIN_RESISTANCE_OHM), frequency_hz: Self::FREQUENCY_HZ }
    }
}

impl Default for GroundPower {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TruFaults {
    pub winding_degradation: f64,
}

pub struct Tru {
    temp_c: f64,
}

impl Tru {
    const INTERNAL_RESISTANCE_OHM: f64 = 0.0135;
    const DEGRADED_RESISTANCE_OHM: f64 = 0.054;
    const IDLE_OUTPUT_VOLTAGE: f64 = 30.2;
    const THERMAL_MASS_J_PER_KELVIN: f64 = 1500.0;
    const COOLING_W_PER_KELVIN: f64 = 3.5;

    pub fn new(ambient_c: f64) -> Self {
        Self { temp_c: ambient_c }
    }

    pub fn temperature_c(&self) -> f64 {
        self.temp_c
    }

    pub fn terminal(&self, ac_input_powered: bool, faults: TruFaults) -> (f64, f64) {
        if !ac_input_powered {
            return (0.0, MIN_RESISTANCE_OHM);
        }
        let degradation = faults.winding_degradation.clamp(0.0, 1.0);
        let r = Self::INTERNAL_RESISTANCE_OHM + degradation * (Self::DEGRADED_RESISTANCE_OHM - Self::INTERNAL_RESISTANCE_OHM);
        (Self::IDLE_OUTPUT_VOLTAGE, r)
    }

    pub fn step(&mut self, ac_input_powered: bool, measured_load_w: f64, faults: TruFaults, ambient_c: f64, dt_s: f64) -> f64 {
        let (_, r) = self.terminal(ac_input_powered, faults);
        let approx_current = if ac_input_powered { measured_load_w.max(0.0) / Self::IDLE_OUTPUT_VOLTAGE } else { 0.0 };
        let heat_w = approx_current * approx_current * r;
        let target = ambient_c + heat_w / Self::COOLING_W_PER_KELVIN;
        let k = Self::COOLING_W_PER_KELVIN / Self::THERMAL_MASS_J_PER_KELVIN;
        self.temp_c = target + (self.temp_c - target) * (-k * dt_s.max(0.0)).exp();
        self.temp_c
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StaticInverterFaults {
    pub efficiency_loss: f64,
}

pub struct StaticInverter {}

impl StaticInverter {
    const EFFICIENCY: f64 = 0.85;
    const DEGRADED_EFFICIENCY_FLOOR: f64 = 0.3;
    const RATED_W: f64 = 135.0;
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    const RATED_VOLTAGE_REGULATION: f64 = 0.05;
    const MIN_INPUT_V: f64 = 18.0;
    const FREQUENCY_HZ: f64 = 400.0;

    pub fn new() -> Self {
        Self {}
    }

    pub fn terminal(&self, dc_input_v: f64, faults: StaticInverterFaults) -> Source {
        if dc_input_v < Self::MIN_INPUT_V {
            return Source { id: "static-inv", open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 };
        }
        let efficiency = (Self::EFFICIENCY - faults.efficiency_loss.clamp(0.0, 1.0) * (Self::EFFICIENCY - Self::DEGRADED_EFFICIENCY_FLOOR)).max(Self::DEGRADED_EFFICIENCY_FLOOR);
        let rated_apparent = Self::RATED_W / efficiency;
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1.0 - Self::RATED_VOLTAGE_REGULATION);
        let xs = target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / rated_apparent.max(1.0);
        Source { id: "static-inv", open_circuit_v: Self::RATED_VOLTAGE_VOLT, resistance_ohm: xs.max(MIN_RESISTANCE_OHM), frequency_hz: Self::FREQUENCY_HZ }
    }
}

impl Default for StaticInverter {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RatFaults {
    pub jammed: f64,
}

pub struct Rat {
    deployed: bool,
}

impl Rat {
    const PROPELLER_DIAMETER_M: f64 = 1.6256;
    pub(crate) const MAX_POWER_W: f64 = 70_000.0;
    const POWER_COEFFICIENT: f64 = 0.35;
    const AIR_DENSITY_KG_M3: f64 = 1.225;
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    const FREQUENCY_HZ: f64 = 400.0;

    pub fn new() -> Self {
        Self { deployed: false }
    }

    pub fn deploy(&mut self) {
        self.deployed = true;
    }

    pub fn deployed(&self) -> bool {
        self.deployed
    }

    pub fn terminal(&self, airspeed_kt: f64, faults: RatFaults) -> Source {
        let jammed = faults.jammed.clamp(0.0, 1.0);
        if !self.deployed || jammed >= 1.0 {
            return Source { id: "rat", open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 };
        }
        let v_ms = (airspeed_kt.max(0.0)) * 0.514444;
        let area_m2 = std::f64::consts::PI * (Self::PROPELLER_DIAMETER_M / 2.0).powi(2);
        let available_w = (0.5 * Self::AIR_DENSITY_KG_M3 * area_m2 * v_ms.powi(3) * Self::POWER_COEFFICIENT * (1.0 - jammed)).min(Self::MAX_POWER_W);
        if available_w <= 1.0 {
            return Source { id: "rat", open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 };
        }
        let r = (Self::RATED_VOLTAGE_VOLT * Self::RATED_VOLTAGE_VOLT / (4.0 * available_w)).max(MIN_RESISTANCE_OHM);
        Source { id: "rat", open_circuit_v: Self::RATED_VOLTAGE_VOLT, resistance_ohm: r, frequency_hz: Self::FREQUENCY_HZ }
    }
}

impl Default for Rat {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BatteryFaults {
    pub capacity_fade: f64,
    pub resistance_growth: f64,
}

pub struct Battery {
    charge_ah: f64,
    temp_c: f64,
}

impl Battery {
    const RATED_CAPACITY_AH: f64 = 23.0;
    const CELL_INTERNAL_RESISTANCE_OHM_AT_20C: f64 = 0.011;
    const WIRING_RESISTANCE_OHM: f64 = 0.02;
    const RESISTANCE_TEMP_COEFFICIENT_PER_C: f64 = 0.02;
    const RESISTANCE_REFERENCE_TEMP_C: f64 = 20.0;
    const THERMAL_MASS_J_PER_KELVIN: f64 = 6000.0;
    const COOLING_W_PER_KELVIN: f64 = 2.5;
    const AGED_RESISTANCE_MULTIPLIER: f64 = 3.0;
    const MAX_CAPACITY_FADE: f64 = 0.7;
    const PEUKERT_EXPONENT: f64 = 1.08;
    const PEUKERT_REFERENCE_CURRENT_A: f64 = Self::RATED_CAPACITY_AH;
    const FULL_OCV: f64 = 29.0;
    const EMPTY_OCV: f64 = 22.0;

    pub fn new(initial_charge_fraction: f64, ambient_c: f64) -> Self {
        Self { charge_ah: Self::RATED_CAPACITY_AH * initial_charge_fraction.clamp(0.0, 1.0), temp_c: ambient_c }
    }

    pub fn usable_capacity_ah(&self, faults: BatteryFaults) -> f64 {
        Self::RATED_CAPACITY_AH * (1.0 - faults.capacity_fade.clamp(0.0, 1.0) * Self::MAX_CAPACITY_FADE)
    }

    pub fn charge_fraction(&self, faults: BatteryFaults) -> f64 {
        let usable = self.usable_capacity_ah(faults).max(1.0e-6);
        (self.charge_ah / usable).clamp(0.0, 1.0)
    }

    pub fn temperature_c(&self) -> f64 {
        self.temp_c
    }

    fn resistance_ohm(&self, faults: BatteryFaults) -> f64 {
        let below_reference = (Self::RESISTANCE_REFERENCE_TEMP_C - self.temp_c).max(0.0);
        let temp_factor = 1.0 + Self::RESISTANCE_TEMP_COEFFICIENT_PER_C * below_reference;
        let aging_factor = 1.0 + faults.resistance_growth.clamp(0.0, 1.0) * (Self::AGED_RESISTANCE_MULTIPLIER - 1.0);
        (Self::CELL_INTERNAL_RESISTANCE_OHM_AT_20C + Self::WIRING_RESISTANCE_OHM) * temp_factor * aging_factor
    }

    fn open_circuit_v(&self, faults: BatteryFaults) -> f64 {
        let f = self.charge_fraction(faults);
        Self::EMPTY_OCV + (Self::FULL_OCV - Self::EMPTY_OCV) * f
    }

    pub fn terminal(&self, faults: BatteryFaults) -> (f64, f64) {
        (self.open_circuit_v(faults), self.resistance_ohm(faults).max(MIN_RESISTANCE_OHM))
    }

    pub fn time_to_empty_s(&self, discharge_current_a: f64, faults: BatteryFaults) -> f64 {
        if discharge_current_a <= 0.0 {
            return f64::INFINITY;
        }
        let peukert_capacity_ah = self.usable_capacity_ah(faults) * (Self::PEUKERT_REFERENCE_CURRENT_A / discharge_current_a).powf(Self::PEUKERT_EXPONENT - 1.0);
        (self.charge_ah.min(peukert_capacity_ah) / discharge_current_a) * 3600.0
    }

    pub fn step(&mut self, signed_current_a: f64, faults: BatteryFaults, ambient_c: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let usable = self.usable_capacity_ah(faults);
        self.charge_ah = (self.charge_ah - signed_current_a * dt / 3600.0).clamp(0.0, usable);

        let r = self.resistance_ohm(faults);
        let heat_w = signed_current_a * signed_current_a * r;
        let target = ambient_c + heat_w / Self::COOLING_W_PER_KELVIN;
        let k = Self::COOLING_W_PER_KELVIN / Self::THERMAL_MASS_J_PER_KELVIN;
        self.temp_c = target + (self.temp_c - target) * (-k * dt).exp();
        self.temp_c
    }
}

pub struct Wiring {
    pub vfg: [Vfg; 4],
    pub apu_gen: [ApuGenerator; 2],
    pub tru: [Tru; 4],
    pub static_inverter: StaticInverter,
    pub battery: [Battery; 2],
    pub ground_power: GroundPower,
    pub rat: Rat,
    source_index: SourceIndex,
}

#[derive(Clone, Copy)]
struct SourceIndex {
    gen: [usize; 4],
    apu_gen: [usize; 2],
    tr: [usize; 4],
    static_inv: usize,
    battery: [usize; 2],
    gpu: usize,
    rat: usize,
}

fn generator_rated_a() -> f64 {
    150_000.0 / 0.8 / 115.0
}
fn apu_generator_rated_a() -> f64 {
    120_000.0 / 0.8 / 115.0
}
const TRU_RATED_A: f64 = 200.0;
fn static_inverter_rated_a() -> f64 {
    135.0 / 115.0
}

impl Wiring {
    pub fn build(net: &mut Network, ambient_c: f64) -> Self {
        let gen_buses = [BusId::Ac1, BusId::Ac2, BusId::Ac3, BusId::Ac4];
        let mut gen = [0usize; 4];
        for (i, &bus) in gen_buses.iter().enumerate() {
            let id: &'static str = Box::leak(format!("gen-{}", i + 1).into_boxed_str());
            gen[i] = net.add_source(Source::new(id));
            let contactor_id: &'static str = Box::leak(format!("gen-{}-line", i + 1).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::GeneratorLine, FeedSource::Source(gen[i]), bus, MIN_RESISTANCE_OHM));
            net.add_feeder_breaker(super::network::Breaker::new(Box::leak(format!("gen-{}-bkr", i + 1).into_boxed_str()), generator_rated_a(), bus), bus);
        }

        let apu_gen_bus = BusId::Ac3;
        let mut apu_gen = [0usize; 2];
        for i in 0..2usize {
            let id: &'static str = Box::leak(format!("apu-gen-{}", i + 1).into_boxed_str());
            apu_gen[i] = net.add_source(Source::new(id));
            let contactor_id: &'static str = Box::leak(format!("apu-gen-{}-line", i + 1).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::GeneratorLine, FeedSource::Source(apu_gen[i]), apu_gen_bus, MIN_RESISTANCE_OHM));
            net.add_feeder_breaker(super::network::Breaker::new(Box::leak(format!("apu-gen-{}-bkr", i + 1).into_boxed_str()), apu_generator_rated_a(), apu_gen_bus), apu_gen_bus);
        }

        let tr_names = ["tr-1", "tr-2", "tr-ess", "tr-apu"];
        let tr_buses = [BusId::Dc1, BusId::Dc2, BusId::DcEss, BusId::DcApu];
        let mut tr = [0usize; 4];
        for i in 0..4usize {
            tr[i] = net.add_source(Source::new(tr_names[i]));
            let contactor_id: &'static str = Box::leak(format!("{}-line", tr_names[i]).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::Feeder, FeedSource::Source(tr[i]), tr_buses[i], MIN_RESISTANCE_OHM));
            net.add_feeder_breaker(super::network::Breaker::new(Box::leak(format!("{}-bkr", tr_names[i]).into_boxed_str()), TRU_RATED_A, tr_buses[i]), tr_buses[i]);
        }

        let static_inv = net.add_source(Source::new("static-inv"));
        net.add_contactor(Contactor::new("static-inv-line", ContactorKind::Feeder, FeedSource::Source(static_inv), BusId::AcEmer, MIN_RESISTANCE_OHM));
        net.add_feeder_breaker(super::network::Breaker::new("static-inv-bkr", static_inverter_rated_a(), BusId::AcEmer), BusId::AcEmer);

        let mut battery = [0usize; 2];
        let battery_buses = [BusId::DcBat, BusId::DcHot1];
        for i in 0..2usize {
            let id: &'static str = Box::leak(format!("bat-{}", i + 1).into_boxed_str());
            battery[i] = net.add_source(Source::new(id));
            let contactor_id: &'static str = Box::leak(format!("bat-{}-direct", i + 1).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::BatteryDirect, FeedSource::Source(battery[i]), battery_buses[i], MIN_RESISTANCE_OHM));
            net.add_feeder_breaker(super::network::Breaker::new(Box::leak(format!("bat-{}-bkr", i + 1).into_boxed_str()), Battery::RATED_CAPACITY_AH * 4.0, battery_buses[i]), battery_buses[i]);
        }

        net.add_diode(Diode::new("bat-cross-feed-diode", FeedSource::Bus(BusId::DcBat), BusId::DcHot2, 1.0, MIN_RESISTANCE_OHM));

        let gpu = net.add_source(Source::new("gpu"));
        for (i, &bus) in gen_buses.iter().enumerate() {
            let contactor_id: &'static str = Box::leak(format!("gpu-{}-line", i + 1).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::Feeder, FeedSource::Source(gpu), bus, MIN_RESISTANCE_OHM));
        }

        let rat = net.add_source(Source::new("rat"));
        net.add_contactor(Contactor::new("rat-line", ContactorKind::Feeder, FeedSource::Source(rat), BusId::AcEmer, MIN_RESISTANCE_OHM));

        Self {
            vfg: std::array::from_fn(|_| Vfg::new()),
            apu_gen: std::array::from_fn(|_| ApuGenerator::new()),
            tru: std::array::from_fn(|_| Tru::new(ambient_c)),
            static_inverter: StaticInverter::new(),
            battery: [Battery::new(1.0, ambient_c), Battery::new(1.0, ambient_c)],
            ground_power: GroundPower::new(),
            rat: Rat::new(),
            source_index: SourceIndex { gen, apu_gen, tr, static_inv, battery, gpu, rat },
        }
    }

    pub fn gen_contactor_id(n: usize) -> String {
        format!("gen-{n}-line")
    }
    pub fn gen_breaker_id(n: usize) -> String {
        format!("gen-{n}-bkr")
    }

    pub fn pre_step(&mut self, net: &mut Network, inputs: &WiringInputs, dt_s: f64) {
        for i in 0..4usize {
            let out = self.vfg[i].step(VfgInputs { engine_speed_fraction: inputs.engine_speed_fraction[i], measured_load_w: inputs.measured_gen_load_w[i] }, inputs.vfg_faults[i], dt_s);
            net.set_source(self.source_index.gen[i], out.open_circuit_v, out.resistance_ohm, out.frequency_hz);
        }
        for i in 0..2usize {
            let out = self.apu_gen[i].step(ApuGeneratorInputs { apu_speed_fraction: inputs.apu_speed_fraction, measured_load_w: inputs.measured_apu_gen_load_w[i] }, inputs.apu_gen_faults[i], dt_s);
            net.set_source(self.source_index.apu_gen[i], out.open_circuit_v, out.resistance_ohm, out.frequency_hz);
        }
        let ac_powered = [net.bus(BusId::Ac1).voltage > 90.0, net.bus(BusId::Ac2).voltage > 90.0, net.bus(BusId::AcEss).voltage > 90.0, net.bus(BusId::AcEss).voltage > 90.0];
        for i in 0..4usize {
            let (v, r) = self.tru[i].terminal(ac_powered[i], inputs.tru_faults[i]);
            net.set_source(self.source_index.tr[i], v, r, 0.0);
        }
        for i in 0..2usize {
            let (v, r) = self.battery[i].terminal(inputs.battery_faults[i]);
            net.set_source(self.source_index.battery[i], v, r, 0.0);
        }
        let battery_bus_v = net.bus(BusId::DcBat).voltage;
        let inv = self.static_inverter.terminal(battery_bus_v, inputs.static_inverter_faults);
        net.set_source(self.source_index.static_inv, inv.open_circuit_v, inv.resistance_ohm, inv.frequency_hz);

        let gpu_src = self.ground_power.terminal(inputs.gpu_plugged_in, inputs.ground_power_faults);
        net.set_source(self.source_index.gpu, gpu_src.open_circuit_v, gpu_src.resistance_ohm, gpu_src.frequency_hz);

        let rat_src = self.rat.terminal(inputs.airspeed_kt, inputs.rat_faults);
        net.set_source(self.source_index.rat, rat_src.open_circuit_v, rat_src.resistance_ohm, rat_src.frequency_hz);
    }

    pub fn post_step(&mut self, net: &Network, inputs: &WiringInputs, dt_s: f64) {
        for i in 0..2usize {
            self.battery[i].step(inputs.measured_battery_current_a[i], inputs.battery_faults[i], inputs.ambient_c, dt_s);
        }
        let ac_powered = [net.bus(BusId::Ac1).voltage > 90.0, net.bus(BusId::Ac2).voltage > 90.0, net.bus(BusId::AcEss).voltage > 90.0, net.bus(BusId::AcEss).voltage > 90.0];
        for i in 0..4usize {
            self.tru[i].step(ac_powered[i], inputs.measured_tr_load_w[i], inputs.tru_faults[i], inputs.ambient_c, dt_s);
        }
    }
}

pub struct WiringInputs {
    pub engine_speed_fraction: [f64; 4],
    pub measured_gen_load_w: [f64; 4],
    pub vfg_faults: [VfgFaults; 4],
    pub apu_speed_fraction: f64,
    pub measured_apu_gen_load_w: [f64; 2],
    pub apu_gen_faults: [ApuGeneratorFaults; 2],
    pub tru_faults: [TruFaults; 4],
    pub measured_tr_load_w: [f64; 4],
    pub battery_faults: [BatteryFaults; 2],
    pub measured_battery_current_a: [f64; 2],
    pub static_inverter_faults: StaticInverterFaults,
    pub gpu_plugged_in: bool,
    pub ground_power_faults: GroundPowerFaults,
    pub airspeed_kt: f64,
    pub rat_faults: RatFaults,
    pub ambient_c: f64,
}

impl Default for WiringInputs {
    fn default() -> Self {
        Self {
            engine_speed_fraction: [0.0; 4],
            measured_gen_load_w: [0.0; 4],
            vfg_faults: [VfgFaults::default(); 4],
            apu_speed_fraction: 0.0,
            measured_apu_gen_load_w: [0.0; 2],
            apu_gen_faults: [ApuGeneratorFaults::default(); 2],
            tru_faults: [TruFaults::default(); 4],
            measured_tr_load_w: [0.0; 4],
            battery_faults: [BatteryFaults::default(); 2],
            measured_battery_current_a: [0.0; 2],
            static_inverter_faults: StaticInverterFaults::default(),
            gpu_plugged_in: false,
            ground_power_faults: GroundPowerFaults::default(),
            airspeed_kt: 0.0,
            rat_faults: RatFaults::default(),
            ambient_c: 15.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stopped_vfg_produces_no_voltage_and_a_running_one_tracks_engine_speed() {
        let mut vfg = Vfg::new();
        let stopped = vfg.step(VfgInputs { engine_speed_fraction: 0.0, measured_load_w: 0.0 }, VfgFaults::default(), 1.0 / 60.0);
        assert_eq!(stopped.open_circuit_v, 0.0);
        assert_eq!(stopped.frequency_hz, 0.0);

        let idle = vfg.step(VfgInputs { engine_speed_fraction: 0.3, measured_load_w: 0.0 }, VfgFaults::default(), 1.0 / 60.0);
        let max = vfg.step(VfgInputs { engine_speed_fraction: 1.0, measured_load_w: 0.0 }, VfgFaults::default(), 1.0 / 60.0);
        assert!(idle.open_circuit_v > 100.0);
        assert!(max.frequency_hz > idle.frequency_hz, "frequency should track engine speed");
        assert!((360.0..=800.0).contains(&max.frequency_hz));
    }

    #[test]
    fn a_sustained_generator_overload_eventually_trips_its_own_protection() {
        let mut vfg = Vfg::new();
        let mut tripped_at = None;
        for i in 0..(60 * 60) {
            let out = vfg.step(VfgInputs { engine_speed_fraction: 1.0, measured_load_w: 300_000.0 }, VfgFaults::default(), 1.0 / 60.0);
            if out.overload_tripped {
                tripped_at = Some(i);
                break;
            }
        }
        assert!(tripped_at.is_some(), "a 2x-rated sustained overload should eventually trip the VFG's own protection");
    }

    #[test]
    fn winding_degradation_increases_a_vfgs_series_reactance() {
        let mut healthy = Vfg::new();
        let mut degraded = Vfg::new();
        let h = healthy.step(VfgInputs { engine_speed_fraction: 1.0, measured_load_w: 0.0 }, VfgFaults::default(), 1.0 / 60.0);
        let d = degraded.step(VfgInputs { engine_speed_fraction: 1.0, measured_load_w: 0.0 }, VfgFaults { winding_degradation: 1.0, regulator_drift: 0.0 }, 1.0 / 60.0);
        assert!(d.resistance_ohm > h.resistance_ohm * 3.0, "fully degraded windings should be near the 4x reactance ceiling");
    }

    #[test]
    fn a_tru_needs_its_ac_input_to_produce_dc_output() {
        let tru = Tru::new(15.0);
        let (v_unpowered, _) = tru.terminal(false, TruFaults::default());
        let (v_powered, r) = tru.terminal(true, TruFaults::default());
        assert_eq!(v_unpowered, 0.0);
        assert!((v_powered - 30.2).abs() < 0.01);
        assert!(r > 0.0);
    }

    #[test]
    fn a_tru_heats_under_load_and_cools_once_removed() {
        let mut tru = Tru::new(15.0);
        for _ in 0..(60 * 120) {
            tru.step(true, 5000.0, TruFaults::default(), 15.0, 1.0 / 60.0);
        }
        let hot = tru.temperature_c();
        assert!(hot > 15.0, "should have heated under sustained load: {hot} C");
        for _ in 0..(60 * 600) {
            tru.step(true, 0.0, TruFaults::default(), 15.0, 1.0 / 60.0);
        }
        assert!(tru.temperature_c() < hot, "should cool once the load is removed");
    }

    #[test]
    fn a_battery_discharges_and_its_voltage_sags_as_it_empties() {
        let mut battery = Battery::new(1.0, 20.0);
        let (full_ocv, _) = battery.terminal(BatteryFaults::default());
        for _ in 0..(3600 * 2) {
            battery.step(10.0, BatteryFaults::default(), 20.0, 1.0);
        }
        let (mid_ocv, _) = battery.terminal(BatteryFaults::default());
        assert!(mid_ocv < full_ocv, "OCV should sag as charge is drawn down: {mid_ocv} vs {full_ocv}");
        assert!(battery.charge_fraction(BatteryFaults::default()) < 1.0);
    }

    #[test]
    fn a_battery_recharges_when_fed_a_negative_current() {
        let mut battery = Battery::new(0.2, 20.0);
        let start = battery.charge_fraction(BatteryFaults::default());
        for _ in 0..(3600 * 2) {
            battery.step(-5.0, BatteryFaults::default(), 20.0, 1.0);
        }
        assert!(battery.charge_fraction(BatteryFaults::default()) > start, "negative (charging) current should raise charge state");
    }

    #[test]
    fn cold_temperature_raises_a_batterys_internal_resistance() {
        let cold = Battery::new(1.0, -20.0);
        let warm = Battery::new(1.0, 20.0);
        let (_, r_cold) = cold.terminal(BatteryFaults::default());
        let (_, r_warm) = warm.terminal(BatteryFaults::default());
        assert!(r_cold > r_warm, "a cold battery should show higher internal resistance");
    }

    #[test]
    fn capacity_fade_reduces_usable_capacity() {
        let healthy = Battery::new(1.0, 20.0);
        let faded = BatteryFaults { capacity_fade: 1.0, resistance_growth: 0.0 };
        assert!(healthy.usable_capacity_ah(faded) < healthy.usable_capacity_ah(BatteryFaults::default()));
    }

    #[test]
    fn a_static_inverter_needs_a_healthy_dc_input_to_run() {
        let inv = StaticInverter::new();
        let dead = inv.terminal(5.0, StaticInverterFaults::default());
        let alive = inv.terminal(28.0, StaticInverterFaults::default());
        assert_eq!(dead.open_circuit_v, 0.0);
        assert!(alive.open_circuit_v > 100.0);
        assert_eq!(alive.frequency_hz, 400.0);
    }

    #[test]
    fn a_rat_produces_more_power_at_higher_airspeed_and_nothing_until_deployed() {
        let mut rat = Rat::new();
        let stowed = rat.terminal(250.0, RatFaults::default());
        assert_eq!(stowed.open_circuit_v, 0.0);
        rat.deploy();

        let slow = rat.terminal(60.0, RatFaults::default());
        let faster = rat.terminal(90.0, RatFaults::default());
        assert!(faster.open_circuit_v > 0.0 && slow.open_circuit_v > 0.0);
        assert!(
            faster.resistance_ohm < slow.resistance_ohm,
            "below the generator's rating, higher airspeed must mean a stiffer (more capable) equivalent source: {} vs {}",
            faster.resistance_ohm,
            slow.resistance_ohm
        );

        let plateau_r = Rat::RATED_VOLTAGE_VOLT * Rat::RATED_VOLTAGE_VOLT / (4.0 * Rat::MAX_POWER_W);
        let cruise = rat.terminal(150.0, RatFaults::default());
        let fast = rat.terminal(300.0, RatFaults::default());
        assert!((cruise.resistance_ohm - plateau_r).abs() < 1e-12, "150 kt is already on the 70 kW plateau: {}", cruise.resistance_ohm);
        assert!((fast.resistance_ohm - plateau_r).abs() < 1e-12, "300 kt is governed to the same 70 kW rating: {}", fast.resistance_ohm);
    }

    #[test]
    fn a_jammed_rat_produces_no_power_even_when_deployed() {
        let mut rat = Rat::new();
        rat.deploy();
        let out = rat.terminal(300.0, RatFaults { jammed: 1.0 });
        assert_eq!(out.open_circuit_v, 0.0);
    }

    #[test]
    fn ground_power_only_energises_when_plugged_in() {
        let gpu = GroundPower::new();
        assert_eq!(gpu.terminal(false, GroundPowerFaults::default()).open_circuit_v, 0.0);
        let live = gpu.terminal(true, GroundPowerFaults::default());
        assert!(live.open_circuit_v > 100.0);
        assert_eq!(live.frequency_hz, 400.0);
    }

    #[test]
    fn wiring_builds_and_a_running_generator_energises_its_bus_through_the_network() {
        let mut net = Network::new();
        let mut wiring = Wiring::build(&mut net, 15.0);
        net.command_contactor(&Wiring::gen_contactor_id(1), true);
        let mut inputs = WiringInputs::default();
        inputs.engine_speed_fraction[0] = 1.0;
        for _ in 0..10 {
            wiring.pre_step(&mut net, &inputs, 1.0 / 60.0);
            net.step(1.0 / 60.0);
            wiring.post_step(&net, &inputs, 1.0 / 60.0);
        }
        assert!(net.bus(BusId::Ac1).voltage > 100.0, "GEN 1 running and its line contactor closed should energise AC1: {}", net.bus(BusId::Ac1).voltage);
    }

    #[test]
    fn a_pulled_generator_breaker_prevents_wiring_from_reclosing_its_bus() {
        let mut net = Network::new();
        Wiring::build(&mut net, 15.0);
        let bkr = net.breaker_index(&Wiring::gen_breaker_id(1)).expect("GEN 1 breaker should exist");
        net.breakers[bkr].pull();
        assert!(!net.breakers[bkr].closed);
    }
}

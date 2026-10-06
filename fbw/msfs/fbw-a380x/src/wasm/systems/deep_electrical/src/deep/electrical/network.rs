use std::collections::HashMap;

const ITERATIONS: usize = 15;
const FREQUENCY_PASSES: usize = 4;
const MIN_RESISTANCE_OHM: f64 = 1.0e-4;
const MIN_ADMITTANCE: f64 = 1.0e-9;
const MIN_VOLTAGE_V: f64 = 1.0e-6;
const POWERED_VOLTAGE_FRACTION: f64 = 0.5;
const UNDERVOLTAGE_RESTART_MARGIN: f64 = 1.05;
const UNDERVOLTAGE_LOCKOUT_S: f64 = 5.0;
const HIGH_RESISTANCE_MAX_EXTRA_FRACTION: f64 = 0.5;
const INTERMITTENT_MAX_RATE_HZ: f64 = 2.0;
const BUS_FAULT_RESISTANCE_OHM: f64 = 0.002;

const THERMAL_TRIP_K: f64 = 30.0;
const MAGNETIC_TRIP_MULTIPLE: f64 = 10.0;
const COOLDOWN_SECONDS: f64 = 20.0;
const NUISANCE_TIME_CONSTANT_S: f64 = 5.0;

const WELD_SALT: u64 = 0x1111_1111_1111_1111;
const FAIL_TO_CLOSE_SALT: u64 = 0x2222_2222_2222_2222;
const DIODE_OPEN_SALT: u64 = 0x3333_3333_3333_3333;
const FAILS_TO_TRIP_SALT: u64 = 0x4444_4444_4444_4444;
const INTERMITTENT_SALT: u64 = 0x5555_5555_5555_5555;

fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

pub(super) fn fnv1a(s: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01B3);
    }
    h
}

fn uniform01(seed: u64, tick: u64) -> f64 {
    let h = splitmix64(seed ^ splitmix64(tick));
    (h >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
}

pub(super) fn bernoulli(seed: u64, tick: u64, p: f64) -> bool {
    if p <= 0.0 {
        false
    } else if p >= 1.0 {
        true
    } else {
        uniform01(seed, tick) < p
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BusId {
    Ac1,
    Ac2,
    Ac3,
    Ac4,
    AcEss,
    AcEssShed,
    AcEmer,
    AcGndFltSvc,
    Dc1,
    Dc2,
    DcEss,
    DcEssShed,
    DcBat,
    DcHot1,
    DcHot2,
    DcApu,
    DcGndFltSvc,
}

pub const ALL_BUS_IDS: [BusId; 17] = [
    BusId::Ac1,
    BusId::Ac2,
    BusId::Ac3,
    BusId::Ac4,
    BusId::AcEss,
    BusId::AcEssShed,
    BusId::AcEmer,
    BusId::AcGndFltSvc,
    BusId::Dc1,
    BusId::Dc2,
    BusId::DcEss,
    BusId::DcEssShed,
    BusId::DcBat,
    BusId::DcHot1,
    BusId::DcHot2,
    BusId::DcApu,
    BusId::DcGndFltSvc,
];

impl BusId {
    pub const fn index(self) -> usize {
        match self {
            BusId::Ac1 => 0,
            BusId::Ac2 => 1,
            BusId::Ac3 => 2,
            BusId::Ac4 => 3,
            BusId::AcEss => 4,
            BusId::AcEssShed => 5,
            BusId::AcEmer => 6,
            BusId::AcGndFltSvc => 7,
            BusId::Dc1 => 8,
            BusId::Dc2 => 9,
            BusId::DcEss => 10,
            BusId::DcEssShed => 11,
            BusId::DcBat => 12,
            BusId::DcHot1 => 13,
            BusId::DcHot2 => 14,
            BusId::DcApu => 15,
            BusId::DcGndFltSvc => 16,
        }
    }

    pub const fn is_ac(self) -> bool {
        matches!(
            self,
            BusId::Ac1 | BusId::Ac2 | BusId::Ac3 | BusId::Ac4 | BusId::AcEss | BusId::AcEssShed | BusId::AcEmer | BusId::AcGndFltSvc
        )
    }

    pub const fn nominal_voltage(self) -> f64 {
        if self.is_ac() {
            115.0
        } else {
            28.0
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            BusId::Ac1 => "AC1",
            BusId::Ac2 => "AC2",
            BusId::Ac3 => "AC3",
            BusId::Ac4 => "AC4",
            BusId::AcEss => "AC_ESS",
            BusId::AcEssShed => "AC_ESS_SHED",
            BusId::AcEmer => "AC_EMER",
            BusId::AcGndFltSvc => "AC_GND_FLT_SVC",
            BusId::Dc1 => "DC1",
            BusId::Dc2 => "DC2",
            BusId::DcEss => "DC_ESS",
            BusId::DcEssShed => "DC_ESS_SHED",
            BusId::DcBat => "DC_BAT",
            BusId::DcHot1 => "DC_HOT1",
            BusId::DcHot2 => "DC_HOT2",
            BusId::DcApu => "DC_APU",
            BusId::DcGndFltSvc => "DC_GND_FLT_SVC",
        }
    }
}

const NUM_BUSES: usize = 17;

#[derive(Clone, Copy, Debug, Default)]
pub struct BusFaults {
    pub short_to_ground: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Bus {
    pub id: BusId,
    pub voltage: f64,
    pub frequency_hz: f64,
    pub faults: BusFaults,
}

impl Bus {
    fn new(id: BusId) -> Self {
        Self { id, voltage: id.nominal_voltage(), frequency_hz: 0.0, faults: BusFaults::default() }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Source {
    pub id: &'static str,
    pub open_circuit_v: f64,
    pub resistance_ohm: f64,
    pub frequency_hz: f64,
}

impl Source {
    pub fn new(id: &'static str) -> Self {
        Self { id, open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedSource {
    Source(usize),
    Bus(BusId),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactorKind {
    GeneratorLine,
    BusTie,
    Feeder,
    BatteryDirect,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactorFaults {
    pub fails_to_close: f64,
    pub welded_closed: f64,
}

pub struct Contactor {
    pub id: &'static str,
    pub kind: ContactorKind,
    pub from: FeedSource,
    pub to: BusId,
    pub resistance_ohm: f64,
    pub commanded_closed: bool,
    pub faults: ContactorFaults,
    pub closed: bool,
}

impl Contactor {
    pub fn new(id: &'static str, kind: ContactorKind, from: FeedSource, to: BusId, resistance_ohm: f64) -> Self {
        Self { id, kind, from, to, resistance_ohm, commanded_closed: false, faults: ContactorFaults::default(), closed: false }
    }

    fn resolve(&mut self, tick: u64) {
        let seed = fnv1a(self.id);
        if bernoulli(seed ^ WELD_SALT, tick, self.faults.welded_closed) {
            self.closed = true;
            return;
        }
        if !self.commanded_closed {
            self.closed = false;
            return;
        }
        self.closed = !bernoulli(seed ^ FAIL_TO_CLOSE_SALT, tick, self.faults.fails_to_close);
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DiodeFaults {
    pub open_circuit: f64,
}

pub struct Diode {
    pub id: &'static str,
    pub from: FeedSource,
    pub to: BusId,
    pub forward_drop_v: f64,
    pub resistance_ohm: f64,
    pub faults: DiodeFaults,
}

impl Diode {
    pub fn new(id: &'static str, from: FeedSource, to: BusId, forward_drop_v: f64, resistance_ohm: f64) -> Self {
        Self { id, from, to, forward_drop_v, resistance_ohm, faults: DiodeFaults::default() }
    }

    fn open_fault(&self, tick: u64) -> bool {
        bernoulli(fnv1a(self.id) ^ DIODE_OPEN_SALT, tick, self.faults.open_circuit)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TripCause {
    Thermal,
    Magnetic,
    Nuisance,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BreakerFaults {
    pub fails_to_trip: f64,
    pub nuisance_trip: f64,
}

pub struct Breaker {
    pub id: &'static str,
    pub rated_a: f64,
    pub bus: BusId,
    pub closed: bool,
    heat: f64,
    pub trip_cause: Option<TripCause>,
    pub faults: BreakerFaults,
    pub current_a: f64,
}

impl Breaker {
    pub fn new(id: &'static str, rated_a: f64, bus: BusId) -> Self {
        Self { id, rated_a, bus, closed: true, heat: 0.0, trip_cause: None, faults: BreakerFaults::default(), current_a: 0.0 }
    }

    pub fn reset(&mut self) {
        self.closed = true;
        self.heat = 0.0;
        self.trip_cause = None;
    }

    pub fn pull(&mut self) {
        self.closed = false;
    }

    pub fn heat_fraction(&self) -> f64 {
        self.heat
    }

    fn step(&mut self, current_a: f64, dt_s: f64, tick: u64) {
        if !self.closed {
            self.current_a = 0.0;
            self.heat = (self.heat - dt_s / COOLDOWN_SECONDS).max(0.0);
            return;
        }
        self.current_a = current_a;

        let nuisance = self.faults.nuisance_trip.clamp(0.0, 1.0);
        if nuisance > 0.0 {
            self.heat += dt_s * nuisance / NUISANCE_TIME_CONSTANT_S;
        }

        let ratio = if self.rated_a > 0.0 { current_a / self.rated_a } else { 0.0 };
        if ratio >= MAGNETIC_TRIP_MULTIPLE {
            self.attempt_trip(TripCause::Magnetic, tick);
            return;
        }
        if ratio > 1.0 {
            self.heat += dt_s * (ratio * ratio - 1.0) / THERMAL_TRIP_K;
        } else if nuisance <= 0.0 {
            self.heat = (self.heat - dt_s / COOLDOWN_SECONDS).max(0.0);
        }
        if self.heat >= 1.0 {
            let cause = if ratio > 1.0 { TripCause::Thermal } else { TripCause::Nuisance };
            self.attempt_trip(cause, tick);
        }
    }

    fn attempt_trip(&mut self, cause: TripCause, tick: u64) {
        if bernoulli(fnv1a(self.id) ^ FAILS_TO_TRIP_SALT, tick, self.faults.fails_to_trip) {
            self.heat = 0.999;
            return;
        }
        self.closed = false;
        self.trip_cause = Some(cause);
        self.heat = 0.0;
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LoadFaults {
    pub open_circuit: f64,
    pub short_to_ground: f64,
    pub high_resistance: f64,
    pub intermittent: f64,
}

#[derive(Clone, Debug)]
pub struct LoadSpec {
    pub id: &'static str,
    pub name: &'static str,
    pub ata: u16,
    pub bus: BusId,
    pub rated_power_w: f64,
    pub power_factor: f64,
    pub min_operating_voltage: f64,
    pub inrush_multiple: f64,
    pub inrush_duration_s: f64,
    pub wiring_resistance_ohm: f64,
    pub rated_frequency_hz: f64,
    pub basis: &'static str,
}

#[derive(Clone, Copy, Debug)]
pub struct LoadFeed {
    pub bus: BusId,
    pub breaker: usize,
    pub priority: u8,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LoadOutputs {
    pub current_a: f64,
    pub power_w: f64,
    pub powered: bool,
    pub fault_current_a: f64,
}

pub struct Load {
    pub spec: LoadSpec,
    pub feeds: Vec<LoadFeed>,
    pub commanded_on: bool,
    pub faults: LoadFaults,
    time_energized_s: f64,
    was_on: bool,
    pub current_a: f64,
    pub powered: bool,
    pub active_feed: Option<usize>,
    undervoltage_latched: bool,
    undervoltage_lockout_s: f64,
}

impl Load {
    pub fn new(spec: LoadSpec, breaker: usize) -> Self {
        let bus = spec.bus;
        Self::new_multi_feed(spec, vec![LoadFeed { bus, breaker, priority: 0 }])
    }

    pub fn new_multi_feed(spec: LoadSpec, feeds: Vec<LoadFeed>) -> Self {
        Self { spec, feeds, commanded_on: true, faults: LoadFaults::default(), time_energized_s: 0.0, was_on: false, current_a: 0.0, powered: false, active_feed: None, undervoltage_latched: false, undervoltage_lockout_s: 0.0 }
    }

    fn health(&self) -> f64 {
        (1.0 - self.faults.open_circuit.clamp(0.0, 1.0)).max(0.0)
    }

    fn inrush_multiplier(&self, dt_s: f64) -> f64 {
        if self.spec.inrush_duration_s <= 0.0 || self.spec.inrush_multiple <= 1.0 {
            return 1.0;
        }
        let tau = (self.spec.inrush_duration_s / 3.0).max(1.0e-6);
        let t = self.time_energized_s.max(0.0);
        let start = (-t / tau).exp();
        if dt_s <= 0.0 {
            return 1.0 + (self.spec.inrush_multiple - 1.0) * start;
        }
        let end = (-(t + dt_s) / tau).exp();
        1.0 + (self.spec.inrush_multiple - 1.0) * (tau / dt_s) * (start - end)
    }

    fn frequency_multiplier(&self, freq_hz: f64) -> f64 {
        if self.spec.rated_frequency_hz <= 0.0 {
            1.0
        } else if freq_hz <= 0.0 {
            0.0
        } else {
            (freq_hz / self.spec.rated_frequency_hz).powi(3)
        }
    }

    fn intermittent_dropout(&self, dt_s: f64, tick: u64) -> bool {
        let severity = self.faults.intermittent.clamp(0.0, 1.0);
        if severity <= 0.0 {
            return false;
        }
        let rate = severity * INTERMITTENT_MAX_RATE_HZ;
        let p = 1.0 - (-rate * dt_s.max(0.0)).exp();
        bernoulli(fnv1a(self.spec.id) ^ INTERMITTENT_SALT, tick, p)
    }

    fn short_conductance(&self) -> f64 {
        let short = self.faults.short_to_ground.clamp(0.0, 1.0);
        if short <= 0.0 {
            0.0
        } else {
            short / self.spec.wiring_resistance_ohm.max(MIN_RESISTANCE_OHM)
        }
    }

    fn undervoltage_threshold(&self) -> f64 {
        if self.undervoltage_latched {
            self.spec.min_operating_voltage * UNDERVOLTAGE_RESTART_MARGIN
        } else {
            self.spec.min_operating_voltage
        }
    }

    fn undervoltage_locked_out(&self) -> bool {
        self.undervoltage_lockout_s > 0.0
    }

    fn select_feed(&self, voltages: &[f64], breakers: &[Breaker]) -> Option<(usize, f64)> {
        if self.undervoltage_locked_out() {
            return None;
        }
        let threshold = self.undervoltage_threshold();
        for (i, feed) in self.feeds.iter().enumerate() {
            if feed.breaker >= breakers.len() || !breakers[feed.breaker].closed {
                continue;
            }
            let v = voltages[feed.bus.index()];
            if v >= threshold {
                return Some((i, v));
            }
        }
        None
    }

    fn contribution(&self, v: f64, freq_hz: f64, dt_s: f64, tick: u64) -> (f64, f64) {
        if !self.commanded_on {
            return (0.0, 0.0);
        }
        if self.intermittent_dropout(dt_s, tick) {
            return (0.0, 0.0);
        }
        let health = self.health();
        if health <= 0.0 {
            return (0.0, self.short_conductance());
        }
        let base_p = self.spec.rated_power_w * health * self.inrush_multiplier(dt_s) * self.frequency_multiplier(freq_hz);
        let hr = self.faults.high_resistance.clamp(0.0, 1.0);
        let p = base_p * (1.0 + hr * HIGH_RESISTANCE_MAX_EXTRA_FRACTION);
        (p, self.short_conductance())
    }

    pub fn step(&mut self, voltages: &[f64], frequencies: &[f64], breakers: &[Breaker], dt_s: f64, tick: u64) -> LoadOutputs {
        let selection = self.select_feed(voltages, breakers);
        self.undervoltage_lockout_s = (self.undervoltage_lockout_s - dt_s.max(0.0)).max(0.0);
        let Some((feed_idx, v)) = selection else {
            let best_energised_feed_voltage = self
                .feeds
                .iter()
                .filter(|f| f.breaker < breakers.len() && breakers[f.breaker].closed)
                .map(|f| voltages[f.bus.index()])
                .fold(0.0_f64, f64::max);
            if best_energised_feed_voltage > 0.0 && !self.undervoltage_latched {
                self.undervoltage_latched = true;
                self.undervoltage_lockout_s = UNDERVOLTAGE_LOCKOUT_S;
            }
            self.was_on = false;
            self.time_energized_s = 0.0;
            self.powered = false;
            self.current_a = 0.0;
            self.active_feed = None;
            return LoadOutputs::default();
        };
        self.active_feed = Some(feed_idx);
        self.undervoltage_latched = false;

        let freq_hz = frequencies[self.feeds[feed_idx].bus.index()];
        let (p, g) = self.contribution(v, freq_hz, dt_s, tick);
        let energised = v > MIN_VOLTAGE_V;
        let is_on = energised && (p > 0.0 || g > 0.0);
        if is_on && !self.was_on {
            self.time_energized_s = 0.0;
        }
        self.time_energized_s = if is_on { self.time_energized_s + dt_s } else { 0.0 };
        self.was_on = is_on;

        let fault_current_a = g * v;
        let base_current_a = if energised && self.spec.power_factor > 1.0e-6 { p / (v * self.spec.power_factor) } else { 0.0 };
        let current_a = base_current_a + fault_current_a;
        self.current_a = current_a;
        self.powered = energised && p > 0.0;

        LoadOutputs { current_a, power_w: p, powered: self.powered, fault_current_a }
    }
}

#[derive(Clone, Debug)]
pub struct NetworkReport {
    pub bus_voltage: [f64; NUM_BUSES],
    pub bus_powered: [bool; NUM_BUSES],
    pub total_power_w: f64,
    pub tripped_breakers: Vec<&'static str>,
}

impl Default for NetworkReport {
    fn default() -> Self {
        Self { bus_voltage: [0.0; NUM_BUSES], bus_powered: [false; NUM_BUSES], total_power_w: 0.0, tripped_breakers: Vec::new() }
    }
}

fn solve_bus_voltage(vth_over_r: f64, y_th: f64, agg_p: f64, agg_g: f64) -> f64 {
    if y_th <= MIN_ADMITTANCE {
        return 0.0;
    }
    let rth = 1.0 / y_th;
    let vth = vth_over_r * rth;
    let a = 1.0 + rth * agg_g;
    let disc = vth * vth - 4.0 * a * rth * agg_p;
    if disc < 0.0 {
        (vth / (2.0 * a)).max(0.0)
    } else {
        ((vth + disc.sqrt()) / (2.0 * a)).max(0.0)
    }
}

pub struct Network {
    pub buses: Vec<Bus>,
    pub sources: Vec<Source>,
    pub contactors: Vec<Contactor>,
    pub diodes: Vec<Diode>,
    pub breakers: Vec<Breaker>,
    pub loads: Vec<Load>,
    feeder_breakers: Vec<(usize, BusId)>,
    tick: u64,
}

impl Network {
    pub fn new() -> Self {
        Self {
            buses: ALL_BUS_IDS.iter().map(|&id| Bus::new(id)).collect(),
            sources: Vec::new(),
            contactors: Vec::new(),
            diodes: Vec::new(),
            breakers: Vec::new(),
            loads: Vec::new(),
            feeder_breakers: Vec::new(),
            tick: 0,
        }
    }

    pub fn bus(&self, id: BusId) -> &Bus {
        &self.buses[id.index()]
    }

    pub fn set_bus_fault(&mut self, id: BusId, short_to_ground: f64) {
        self.buses[id.index()].faults.short_to_ground = short_to_ground.clamp(0.0, 1.0);
    }

    pub fn add_source(&mut self, source: Source) -> usize {
        self.sources.push(source);
        self.sources.len() - 1
    }

    pub fn add_contactor(&mut self, contactor: Contactor) -> usize {
        self.contactors.push(contactor);
        self.contactors.len() - 1
    }

    pub fn add_diode(&mut self, diode: Diode) -> usize {
        self.diodes.push(diode);
        self.diodes.len() - 1
    }

    pub fn add_breaker(&mut self, breaker: Breaker) -> usize {
        self.breakers.push(breaker);
        self.breakers.len() - 1
    }

    pub fn add_feeder_breaker(&mut self, breaker: Breaker, bus: BusId) -> usize {
        let idx = self.add_breaker(breaker);
        self.feeder_breakers.push((idx, bus));
        idx
    }

    pub fn add_load(&mut self, spec: LoadSpec, breaker: usize) -> usize {
        self.loads.push(Load::new(spec, breaker));
        self.loads.len() - 1
    }

    pub fn add_load_multi_feed(&mut self, spec: LoadSpec, feeds: Vec<LoadFeed>) -> usize {
        self.loads.push(Load::new_multi_feed(spec, feeds));
        self.loads.len() - 1
    }

    pub fn contactor_index(&self, id: &str) -> Option<usize> {
        self.contactors.iter().position(|c| c.id == id)
    }

    pub fn breaker_index(&self, id: &str) -> Option<usize> {
        self.breakers.iter().position(|b| b.id == id)
    }

    pub fn load_index(&self, id: &str) -> Option<usize> {
        self.loads.iter().position(|l| l.spec.id == id)
    }

    pub fn diode_index(&self, id: &str) -> Option<usize> {
        self.diodes.iter().position(|d| d.id == id)
    }

    pub fn command_contactor(&mut self, id: &str, closed: bool) {
        if let Some(c) = self.contactors.iter_mut().find(|c| c.id == id) {
            c.commanded_closed = closed;
        }
    }

    pub fn set_source(&mut self, index: usize, open_circuit_v: f64, resistance_ohm: f64, frequency_hz: f64) {
        if let Some(s) = self.sources.get_mut(index) {
            s.open_circuit_v = open_circuit_v.max(0.0);
            s.resistance_ohm = resistance_ohm.max(MIN_RESISTANCE_OHM);
            s.frequency_hz = frequency_hz;
        }
    }

    pub fn step(&mut self, dt_s: f64) -> NetworkReport {
        let dt = dt_s.max(0.0);
        self.tick = self.tick.wrapping_add(1);
        let tick = self.tick;

        for c in &mut self.contactors {
            c.resolve(tick);
        }

        let energised = self.energised_buses();
        let mut voltage = [0.0f64; NUM_BUSES];
        for (i, b) in self.buses.iter().enumerate() {
            voltage[i] = if !energised[i] {
                0.0
            } else if b.voltage > 0.0 {
                b.voltage
            } else {
                b.id.nominal_voltage()
            };
        }

        let frequency = self.resolve_frequency();

        for _ in 0..ITERATIONS {
            voltage = self.relax(&voltage, &frequency, dt, tick);
            for i in 0..NUM_BUSES {
                if !energised[i] {
                    voltage[i] = 0.0;
                }
            }
        }

        for i in 0..NUM_BUSES {
            self.buses[i].voltage = voltage[i];
        }
        self.commit_frequency(&voltage, &frequency);

        let mut breaker_current = vec![0.0f64; self.breakers.len()];
        for i in 0..self.loads.len() {
            let out = self.loads[i].step(&voltage, &frequency, &self.breakers, dt, tick);
            if let Some(feed_idx) = self.loads[i].active_feed {
                let bkr = self.loads[i].feeds[feed_idx].breaker;
                breaker_current[bkr] += out.current_a;
            }
        }
        for &(idx, bus) in &self.feeder_breakers {
            let bus_idx = bus.index();
            let mut total = 0.0;
            for load in &self.loads {
                let on_this_bus = matches!(load.active_feed, Some(fi) if load.feeds[fi].bus == bus);
                if on_this_bus {
                    total += load.current_a;
                }
            }
            let b = &self.buses[bus_idx];
            let short = b.faults.short_to_ground.clamp(0.0, 1.0);
            if short > 0.0 {
                total += (short / BUS_FAULT_RESISTANCE_OHM) * b.voltage;
            }
            breaker_current[idx] = total;
        }

        let mut tripped = Vec::new();
        for (i, b) in self.breakers.iter_mut().enumerate() {
            let was_closed = b.closed;
            b.step(breaker_current[i], dt, tick);
            if was_closed && !b.closed {
                tripped.push(b.id);
            }
        }

        let mut report = NetworkReport::default();
        let mut total_power = 0.0;
        for i in 0..NUM_BUSES {
            report.bus_voltage[i] = self.buses[i].voltage;
            report.bus_powered[i] = self.buses[i].voltage >= self.buses[i].id.nominal_voltage() * POWERED_VOLTAGE_FRACTION;
        }
        for load in &self.loads {
            if let Some(fi) = load.active_feed {
                let bus_idx = load.feeds[fi].bus.index();
                total_power += load.spec.power_factor * load.current_a * self.buses[bus_idx].voltage;
            }
        }
        report.total_power_w = total_power;
        report.tripped_breakers = tripped;
        report
    }

    fn relax(&self, voltage: &[f64; NUM_BUSES], frequency: &[f64; NUM_BUSES], dt: f64, tick: u64) -> [f64; NUM_BUSES] {
        let mut agg_p = [0.0f64; NUM_BUSES];
        let mut agg_g = [0.0f64; NUM_BUSES];
        for load in &self.loads {
            let Some((feed_idx, _v)) = load.select_feed(voltage, &self.breakers) else {
                continue;
            };
            let idx = load.feeds[feed_idx].bus.index();
            let (p, g) = load.contribution(voltage[idx], frequency[idx], dt, tick);
            agg_p[idx] += p;
            agg_g[idx] += g;
        }
        for bus in &self.buses {
            let short = bus.faults.short_to_ground.clamp(0.0, 1.0);
            if short > 0.0 {
                agg_g[bus.id.index()] += short / BUS_FAULT_RESISTANCE_OHM;
            }
        }

        let mut vth = [0.0f64; NUM_BUSES];
        let mut yth = [0.0f64; NUM_BUSES];
        for c in &self.contactors {
            if !c.closed {
                continue;
            }
            match c.from {
                FeedSource::Source(i) => {
                    if let Some(src) = self.sources.get(i) {
                        let r = (c.resistance_ohm + src.resistance_ohm).max(MIN_RESISTANCE_OHM);
                        let b = c.to.index();
                        vth[b] += src.open_circuit_v / r;
                        yth[b] += 1.0 / r;
                    }
                }
                FeedSource::Bus(a) => {
                    let r = c.resistance_ohm.max(MIN_RESISTANCE_OHM);
                    let ai = a.index();
                    let bi = c.to.index();
                    vth[bi] += voltage[ai] / r;
                    yth[bi] += 1.0 / r;
                    vth[ai] += voltage[bi] / r;
                    yth[ai] += 1.0 / r;
                }
            }
        }
        let mut open_v = [0.0f64; NUM_BUSES];
        for i in 0..NUM_BUSES {
            open_v[i] = solve_bus_voltage(vth[i], yth[i], agg_p[i], agg_g[i]);
        }
        for d in &self.diodes {
            if d.open_fault(tick) {
                continue;
            }
            let (from_v, source_r) = match d.from {
                FeedSource::Source(i) => self.sources.get(i).map_or((0.0, 0.0), |s| (s.open_circuit_v, s.resistance_ohm)),
                FeedSource::Bus(b) => (voltage[b.index()], 0.0),
            };
            let biased = from_v - d.forward_drop_v;
            if biased > 0.0 && biased > open_v[d.to.index()] {
                let r = (d.resistance_ohm + source_r).max(MIN_RESISTANCE_OHM);
                let bi = d.to.index();
                vth[bi] += biased / r;
                yth[bi] += 1.0 / r;
            }
        }

        let mut next = [0.0f64; NUM_BUSES];
        for i in 0..NUM_BUSES {
            next[i] = solve_bus_voltage(vth[i], yth[i], agg_p[i], agg_g[i]);
        }
        next
    }

    fn energised_buses(&self) -> [bool; NUM_BUSES] {
        let mut live = [false; NUM_BUSES];
        for _ in 0..NUM_BUSES {
            let mut changed = false;
            let mark = |i: usize, live: &mut [bool; NUM_BUSES], changed: &mut bool| {
                if !live[i] {
                    live[i] = true;
                    *changed = true;
                }
            };
            for c in &self.contactors {
                if !c.closed {
                    continue;
                }
                match c.from {
                    FeedSource::Source(i) => {
                        if self.sources.get(i).is_some_and(|s| s.open_circuit_v > 0.0) {
                            mark(c.to.index(), &mut live, &mut changed);
                        }
                    }
                    FeedSource::Bus(a) => {
                        let (ai, bi) = (a.index(), c.to.index());
                        if live[ai] {
                            mark(bi, &mut live, &mut changed);
                        }
                        if live[bi] {
                            mark(ai, &mut live, &mut changed);
                        }
                    }
                }
            }
            for d in &self.diodes {
                if d.open_fault(self.tick) {
                    continue;
                }
                let upstream_live = match d.from {
                    FeedSource::Source(i) => self.sources.get(i).is_some_and(|s| s.open_circuit_v > d.forward_drop_v),
                    FeedSource::Bus(b) => live[b.index()],
                };
                if upstream_live {
                    mark(d.to.index(), &mut live, &mut changed);
                }
            }
            if !changed {
                break;
            }
        }
        live
    }

    fn resolve_frequency(&self) -> [f64; NUM_BUSES] {
        let mut freq = [0.0f64; NUM_BUSES];
        for _ in 0..FREQUENCY_PASSES {
            for c in &self.contactors {
                if !c.closed {
                    continue;
                }
                match c.from {
                    FeedSource::Source(i) => {
                        if let Some(src) = self.sources.get(i) {
                            if src.frequency_hz > 0.0 {
                                freq[c.to.index()] = src.frequency_hz;
                            }
                        }
                    }
                    FeedSource::Bus(a) => {
                        let ai = a.index();
                        let bi = c.to.index();
                        if freq[ai] > 0.0 && freq[bi] <= 0.0 {
                            freq[bi] = freq[ai];
                        } else if freq[bi] > 0.0 && freq[ai] <= 0.0 {
                            freq[ai] = freq[bi];
                        }
                    }
                }
            }
        }
        freq
    }

    fn commit_frequency(&mut self, voltage: &[f64; NUM_BUSES], freq: &[f64; NUM_BUSES]) {
        for i in 0..NUM_BUSES {
            let bus_id = self.buses[i].id;
            self.buses[i].frequency_hz = if bus_id.is_ac() && voltage[i] >= bus_id.nominal_voltage() * POWERED_VOLTAGE_FRACTION { freq[i] } else { 0.0 };
        }
    }
}

impl Default for Network {
    fn default() -> Self {
        Self::new()
    }
}

pub fn loads_on_bus(network: &Network, bus: BusId) -> Vec<usize> {
    network.loads.iter().enumerate().filter(|(_, l)| l.spec.bus == bus).map(|(i, _)| i).collect()
}

pub fn index_by_id<'a, T>(items: &'a [T], id_of: impl Fn(&'a T) -> &'static str) -> HashMap<&'static str, usize> {
    items.iter().enumerate().map(|(i, item)| (id_of(item), i)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn network_with_one_source_and_load(open_circuit_v: f64, source_r: f64, load_w: f64) -> (Network, usize, usize, usize) {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v, resistance_ohm: source_r, frequency_hz: 0.0 });
        let gen_contactor = net.add_contactor(Contactor::new("gen-line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gen_contactor].commanded_closed = true;
        let breaker = net.add_breaker(Breaker::new("bkr", 100.0, BusId::Dc1));
        let spec = LoadSpec {
            id: "load",
            name: "Test Load",
            ata: 24,
            bus: BusId::Dc1,
            rated_power_w: load_w,
            power_factor: 1.0,
            min_operating_voltage: 0.0,
            inrush_multiple: 1.0,
            inrush_duration_s: 0.0,
            wiring_resistance_ohm: 0.05,
            rated_frequency_hz: 0.0,
            basis: "test",
        };
        let load = net.add_load(spec, breaker);
        (net, src, breaker, load)
    }

    #[test]
    fn a_single_source_and_load_matches_the_closed_form_quadratic() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        let v = net.bus(BusId::Dc1).voltage;
        const TOTAL_R: f64 = 0.02 + 0.01;
        let expected = (28.0 + (28.0f64.powi(2) - 4.0 * TOTAL_R * 300.0).sqrt()) / 2.0;
        assert!((v - expected).abs() < 0.05, "solved {v}, expected {expected}");
        let expected_current = 300.0 / expected;
        assert!((net.loads[load].current_a - expected_current).abs() < 0.05);
    }

    #[test]
    fn nothing_connected_never_produces_nan_and_reads_zero() {
        let mut net = Network::new();
        let report = net.step(1.0 / 60.0);
        for v in report.bus_voltage {
            assert!(v.is_finite());
            assert_eq!(v, 0.0);
        }
        assert_eq!(report.total_power_w, 0.0);
        let report0 = net.step(0.0);
        assert!(report0.bus_voltage.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn an_open_breaker_cuts_load_current_to_zero() {
        let (mut net, _src, bkr, load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        net.breakers[bkr].pull();
        net.step(1.0 / 60.0);
        assert_eq!(net.loads[load].current_a, 0.0);
        assert!(!net.loads[load].powered);
    }

    #[test]
    fn a_dead_short_trips_the_magnetic_curve_within_one_tick() {
        let (mut net, _src, bkr, load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        net.loads[load].faults.short_to_ground = 1.0;
        net.loads[load].spec.wiring_resistance_ohm = 0.001;
        net.step(1.0 / 60.0);
        assert!(!net.breakers[bkr].closed, "expected an instant magnetic trip");
        assert_eq!(net.breakers[bkr].trip_cause, Some(TripCause::Magnetic));
    }

    #[test]
    fn a_moderate_overload_trips_thermally_near_its_predicted_time() {
        let (mut net, _src, bkr, _load) = network_with_one_source_and_load(28.0, 0.001, 5160.0);
        const DT: f64 = 1.0 / 60.0;
        let mut elapsed = 0.0;
        let mut tripped_at = None;
        while elapsed < 30.0 {
            net.step(DT);
            elapsed += DT;
            if !net.breakers[bkr].closed {
                tripped_at = Some(elapsed);
                break;
            }
        }
        let t = tripped_at.expect("expected a thermal trip within 30s");
        assert!((t - 10.0).abs() < 1.0, "expected ~10s, got {t}s");
        assert_eq!(net.breakers[bkr].trip_cause, Some(TripCause::Thermal));
    }

    #[test]
    fn a_breaker_that_always_fails_to_trip_stays_closed_under_sustained_overload() {
        let (mut net, _src, bkr, _load) = network_with_one_source_and_load(28.0, 0.001, 5600.0);
        net.breakers[bkr].faults.fails_to_trip = 1.0;
        for _ in 0..(60 * 30) {
            net.step(1.0 / 60.0);
        }
        assert!(net.breakers[bkr].closed, "a jammed breaker should never actually open");
        assert!(net.breakers[bkr].heat_fraction() > 0.9, "should sit right at the trip threshold");
    }

    #[test]
    fn a_welded_contactor_stays_closed_even_when_commanded_open() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 100.0);
        let gc = net.contactor_index("gen-line").unwrap();
        net.contactors[gc].faults.welded_closed = 1.0;
        net.contactors[gc].commanded_closed = false;
        net.step(1.0 / 60.0);
        assert!(net.contactors[gc].closed);
        assert!(net.loads[load].powered, "the bus should still be live through the welded contactor");
    }

    #[test]
    fn a_contactor_that_always_fails_to_close_never_energises_its_bus() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 100.0);
        let gc = net.contactor_index("gen-line").unwrap();
        net.contactors[gc].faults.fails_to_close = 1.0;
        net.step(1.0 / 60.0);
        assert!(!net.contactors[gc].closed);
        assert_eq!(net.bus(BusId::Dc1).voltage, 0.0);
        assert!(!net.loads[load].powered);
    }

    #[test]
    fn a_bus_tie_carries_power_to_a_bus_with_no_source_of_its_own() {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.02, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gen-line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let tie = net.add_contactor(Contactor::new("tie", ContactorKind::BusTie, FeedSource::Bus(BusId::Dc1), BusId::Dc2, 0.02));
        net.contactors[tie].commanded_closed = true;
        let bkr = net.add_breaker(Breaker::new("bkr2", 50.0, BusId::Dc2));
        let spec = LoadSpec {
            id: "load2",
            name: "Tied Load",
            ata: 24,
            bus: BusId::Dc2,
            rated_power_w: 100.0,
            power_factor: 1.0,
            min_operating_voltage: 0.0,
            inrush_multiple: 1.0,
            inrush_duration_s: 0.0,
            wiring_resistance_ohm: 0.05,
            rated_frequency_hz: 0.0,
            basis: "test",
        };
        let load = net.add_load(spec, bkr);
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        assert!(net.bus(BusId::Dc2).voltage > 20.0, "tie should carry real voltage across, got {}", net.bus(BusId::Dc2).voltage);
        assert!(net.loads[load].powered);
        net.contactors[tie].commanded_closed = false;
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        assert_eq!(net.bus(BusId::Dc2).voltage, 0.0);
    }

    #[test]
    fn a_diode_does_not_backfeed_the_lower_side_into_the_higher_one() {
        let mut net = Network::new();
        let strong = net.add_source(Source { id: "strong", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let weak = net.add_source(Source { id: "weak", open_circuit_v: 10.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc1 = net.add_contactor(Contactor::new("gc1", ContactorKind::GeneratorLine, FeedSource::Source(strong), BusId::DcBat, 0.01));
        net.contactors[gc1].commanded_closed = true;
        let gc2 = net.add_contactor(Contactor::new("gc2", ContactorKind::GeneratorLine, FeedSource::Source(weak), BusId::DcHot1, 0.01));
        net.contactors[gc2].commanded_closed = true;
        net.add_diode(Diode::new("d", FeedSource::Bus(BusId::DcHot1), BusId::DcBat, 0.7, 0.01));
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        assert!(net.bus(BusId::DcBat).voltage > 25.0, "reverse-biased diode should not have loaded down DcBat: {}", net.bus(BusId::DcBat).voltage);
    }

    #[test]
    fn a_diode_forward_feeds_with_its_own_drop() {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gc", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::DcBat, 0.01));
        net.contactors[gc].commanded_closed = true;
        net.add_diode(Diode::new("d", FeedSource::Bus(BusId::DcBat), BusId::DcHot1, 1.0, 0.01));
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        let hot1 = net.bus(BusId::DcHot1).voltage;
        let bat = net.bus(BusId::DcBat).voltage;
        assert!(hot1 > 0.0 && hot1 < bat, "hot bus {hot1} should be fed but below the battery bus {bat} by roughly the diode drop");
    }

    #[test]
    fn a_bus_fault_collapses_its_own_voltage_and_can_trip_its_feeder() {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.02, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gen-line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let feeder = net.add_feeder_breaker(Breaker::new("feeder", 50.0, BusId::Dc1), BusId::Dc1);
        net.set_bus_fault(BusId::Dc1, 1.0);
        for _ in 0..(60 * 15) {
            net.step(1.0 / 60.0);
        }
        assert!(net.bus(BusId::Dc1).voltage < 15.0, "a dead bus short should sag the bus hard: {}", net.bus(BusId::Dc1).voltage);
        assert!(!net.breakers[feeder].closed, "the feeder breaker should trip on the bus fault current");
    }

    #[test]
    fn a_feeder_breaker_sums_every_load_on_its_bus_not_just_one() {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gen-line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let feeder = net.add_feeder_breaker(Breaker::new("feeder", 100.0, BusId::Dc1), BusId::Dc1);
        for i in 0..3 {
            let id: &'static str = Box::leak(format!("l{i}").into_boxed_str());
            let bkr = net.add_breaker(Breaker::new(id, 50.0, BusId::Dc1));
            let spec = LoadSpec { id, name: id, ata: 24, bus: BusId::Dc1, rated_power_w: 100.0, power_factor: 1.0, min_operating_voltage: 0.0, inrush_multiple: 1.0, inrush_duration_s: 0.0, wiring_resistance_ohm: 0.05, rated_frequency_hz: 0.0, basis: "test" };
            net.add_load(spec, bkr);
        }
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        let sum_of_loads: f64 = net.loads.iter().map(|l| l.current_a).sum();
        assert!((net.breakers[feeder].current_a - sum_of_loads).abs() < 1e-6, "feeder current {} should equal the sum of all three loads {}", net.breakers[feeder].current_a, sum_of_loads);
    }

    #[test]
    fn undervoltage_drops_a_load_before_full_bus_death() {
        let (mut net, src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 50.0);
        net.loads[load].spec.min_operating_voltage = 26.0;
        net.step(1.0 / 60.0);
        assert!(net.loads[load].powered);
        net.set_source(src, 20.0, 0.02, 0.0);
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        assert!(net.bus(BusId::Dc1).voltage > 0.0, "bus should still have some voltage");
        assert!(!net.loads[load].powered, "load should have dropped out below its own min operating voltage");
    }

    #[test]
    fn inrush_current_decays_toward_the_steady_state_value() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.01, 200.0);
        net.loads[load].spec.inrush_multiple = 4.0;
        net.loads[load].spec.inrush_duration_s = 0.3;
        net.step(1.0 / 60.0);
        let first_tick_current = net.loads[load].current_a;
        for _ in 0..120 {
            net.step(1.0 / 60.0);
        }
        let steady_current = net.loads[load].current_a;
        assert!(first_tick_current > steady_current * 1.5, "expected a real inrush spike: first {first_tick_current} A vs steady {steady_current} A");
    }

    #[test]
    fn an_intermittent_fault_at_full_severity_reliably_drops_the_load() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 50.0);
        net.loads[load].faults.intermittent = 1.0;
        net.step(1000.0);
        assert!(!net.loads[load].powered);
        assert_eq!(net.loads[load].current_a, 0.0);
    }

    #[test]
    fn an_open_circuit_fault_at_full_severity_draws_no_current() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 200.0);
        net.loads[load].faults.open_circuit = 1.0;
        net.step(1.0 / 60.0);
        assert_eq!(net.loads[load].current_a, 0.0);
        assert!(!net.loads[load].powered);
    }

    #[test]
    fn a_high_resistance_fault_draws_more_current_for_the_same_load() {
        let (mut healthy_net, _s1, _b1, healthy_load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        healthy_net.step(1.0 / 60.0);
        let healthy_current = healthy_net.loads[healthy_load].current_a;

        let (mut degraded_net, _s2, _b2, degraded_load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        degraded_net.loads[degraded_load].faults.high_resistance = 1.0;
        degraded_net.step(1.0 / 60.0);
        let degraded_current = degraded_net.loads[degraded_load].current_a;

        assert!(degraded_current > healthy_current, "high-resistance fault should draw more current: {degraded_current} vs {healthy_current}");
    }

    #[test]
    fn a_short_to_ground_current_is_limited_by_wiring_resistance() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 10.0);
        net.loads[load].faults.short_to_ground = 1.0;
        net.loads[load].spec.wiring_resistance_ohm = 2.0;
        net.step(1.0 / 60.0);
        let v = net.bus(BusId::Dc1).voltage;
        let expected_fault_current = v / 2.0;
        assert!((net.loads[load].current_a - expected_fault_current).abs() < 0.5, "current {} should be close to V/R = {}", net.loads[load].current_a, expected_fault_current);
    }

    #[test]
    fn network_converges_within_its_own_iteration_budget() {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gc", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let tie1 = net.add_contactor(Contactor::new("tie1", ContactorKind::BusTie, FeedSource::Bus(BusId::Dc1), BusId::Dc2, 0.02));
        net.contactors[tie1].commanded_closed = true;
        let tie2 = net.add_contactor(Contactor::new("tie2", ContactorKind::BusTie, FeedSource::Bus(BusId::Dc2), BusId::DcEss, 0.02));
        net.contactors[tie2].commanded_closed = true;
        for (id, bus) in [("b1", BusId::Dc1), ("b2", BusId::Dc2), ("b3", BusId::DcEss)] {
            let bkr = net.add_breaker(Breaker::new(id, 50.0, bus));
            let spec = LoadSpec { id, name: id, ata: 24, bus, rated_power_w: 80.0, power_factor: 1.0, min_operating_voltage: 0.0, inrush_multiple: 1.0, inrush_duration_s: 0.0, wiring_resistance_ohm: 0.05, rated_frequency_hz: 0.0, basis: "test" };
            net.add_load(spec, bkr);
        }
        for _ in 0..10 {
            net.step(1.0 / 60.0);
        }
        let v = net.bus(BusId::DcEss).voltage;
        assert!(v.is_finite() && v > 0.0);
        let mut voltage = [28.0f64; NUM_BUSES];
        let frequency = [0.0f64; NUM_BUSES];
        for _ in 0..300 {
            voltage = net.relax(&voltage, &frequency, 1.0 / 60.0, 999);
        }
        assert!((voltage[BusId::DcEss.index()] - v).abs() < 0.1, "extra sweeps should not move the answer much: {} vs {}", voltage[BusId::DcEss.index()], v);
    }

    #[test]
    fn a_tie_ring_with_no_source_on_it_holds_no_voltage() {
        let mut net = Network::new();
        for (id, from, to) in [("t12", BusId::Ac1, BusId::Ac2), ("t23", BusId::Ac2, BusId::Ac3), ("t34", BusId::Ac3, BusId::Ac4)] {
            let c = net.add_contactor(Contactor::new(id, ContactorKind::BusTie, FeedSource::Bus(from), to, 0.02));
            net.contactors[c].commanded_closed = true;
        }
        for (id, bus) in [("l1", BusId::Ac1), ("l2", BusId::Ac2), ("l3", BusId::Ac3), ("l4", BusId::Ac4)] {
            let bkr = net.add_breaker(Breaker::new(id, 50.0, bus));
            let spec = LoadSpec {
                id,
                name: id,
                ata: 24,
                bus,
                rated_power_w: 300.0,
                power_factor: 1.0,
                min_operating_voltage: 0.0,
                inrush_multiple: 1.0,
                inrush_duration_s: 0.0,
                wiring_resistance_ohm: 0.05,
                rated_frequency_hz: 0.0,
                basis: "test",
            };
            net.add_load(spec, bkr);
        }
        for frame in 0..200 {
            let report = net.step(1.0 / 60.0);
            for bus in [BusId::Ac1, BusId::Ac2, BusId::Ac3, BusId::Ac4] {
                let v = net.bus(bus).voltage;
                assert_eq!(v, 0.0, "{} is fed by nothing but sits at {v} V on frame {frame}", bus.label());
                assert!(!report.bus_powered[bus.index()], "{} reads powered with no source anywhere on its ring", bus.label());
            }
        }
        let src = net.add_source(Source { id: "src", open_circuit_v: 115.0, resistance_ohm: 0.01, frequency_hz: 400.0 });
        let line = net.add_contactor(Contactor::new("line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Ac1, 0.01));
        net.contactors[line].commanded_closed = true;
        for _ in 0..30 {
            net.step(1.0 / 60.0);
        }
        for bus in [BusId::Ac1, BusId::Ac2, BusId::Ac3, BusId::Ac4] {
            assert!(net.bus(bus).voltage > 100.0, "{} should be carried through the tie ring by the one real source: {} V", bus.label(), net.bus(bus).voltage);
        }
    }
}

//! Fluid thermal model: a lumped fluid temperature per hydraulic circuit,
//! heated by pump mechanical/volumetric loss and by every throttling
//! pressure drop (a relief valve dumping to return, a priority valve
//! holding back flow, a leak -- lost hydraulic power that is not extracted
//! as useful work across a restriction is dissipated as heat right there,
//! `Q_w = dP_pa * flow_m3_s`), cooled by a fuel-cooled heat exchanger and a
//! passive loss to the surrounding bay, with the result feeding back into
//! the network's own line resistances through `fluid::dynamic_viscosity_pa_s`
//! (cold fluid runs high pressure drops, exactly like this crate's existing
//! engine oil model, `physics::engine::oil.rs`'s own module doc).
//!
//! Cooling: this crate's own `physics::fluids.rs` already documents the
//! A380's hydraulic/fuel heat exchangers ("two fuel/hydraulic heat
//! exchangers per circuit, one per pylon", cited there to Power & Motion
//! Technology, "Hydraulics onboard the A380") and settles on
//! `HHX_EFFECTIVENESS = 0.6`, a mid-range plate/shell-and-tube figure (no
//! public A380 HHX effectiveness exists); the same figure and reasoning is
//! reused here, restated to keep this directory self-contained. Skydrol
//! LD-4's own technical bulletin gives a 225 F (107 C) maximum *continuous*
//! operating temperature and a -40 C..205 C overall usable range (Eastman
//! Pub. No. 7249153C); `OVERHEAT_K` below is the continuous limit.

pub const OVERHEAT_K: f64 = 107.0 + 273.15;

/// GENERIC: no public A380 hydraulic bay thermal figures exist. Sized so a
/// circuit's fluid takes minutes, not seconds, to change temperature (a real
/// hydraulic system's thermal response is slow), and so the fuel-cooled heat
/// exchanger can hold cruise temperatures in a plausible 50-90 C band at
/// representative flow -- the same "settles in a sane band" sanity check
/// `physics::engine::oil.rs`'s own tests use for its tank temperature.
#[derive(Clone, Copy, Debug)]
pub struct ThermalSizing {
    /// Total trapped fluid mass this circuit holds (reservoir + lines +
    /// manifolds), kg -- the thermal mass that stores/releases heat.
    pub fluid_mass_kg: f64,
    pub ambient_loss_w_per_k: f64,
}
impl ThermalSizing {
    /// ~12-12.7 US gal reservoir plus the circuit's own trapped line/
    /// manifold volume, order of magnitude 60 L total, at this fluid's
    /// ~1000 kg/m^3 (`fluid::density_kg_m3`).
    pub fn a380_circuit() -> Self {
        Self { fluid_mass_kg: 60.0, ambient_loss_w_per_k: 40.0 }
    }

    /// ECAM completeness pass (E-FIRE §I, `290800037`/`290800038` HYD
    /// {G,Y} SYS TEMP HI): sizing for a **second**, small `ThermalState`
    /// representing the manifold/pump-discharge fluid mass alone -- as
    /// distinct from [`Self::a380_circuit`]'s ~60 kg figure above, which
    /// lumps in the whole reservoir and every downstream line. **GENERIC**:
    /// no public A380 figure exists for this narrower quantity either; the
    /// mass is sized as a small fraction of the whole-circuit figure (the
    /// manifold/pump-discharge volume genuinely is a small fraction of a
    /// circuit's total trapped fluid), and the conductance is sized to the
    /// physically meaningful order of magnitude at which sustained high
    /// pump duty separates the manifold's temperature measurably from the
    /// reservoir's own, while the two converge again at low flow --
    /// exactly what closes the `RESERVOIR_OVHT`/`SYS_TEMP_HI` duplicate
    /// this design sheet's own module doc records (before this, both ids
    /// would have been the identical comparison on the identical lumped
    /// state).
    ///
    /// The "ambient" this small mass exchanges with, at the call site
    /// (`topology::Circuit::step`), is the reservoir's own current
    /// temperature -- not true outside air -- so this conductance is a
    /// manifold-to-reservoir figure, not a manifold-to-bay one.
    pub fn a380_manifold() -> Self {
        Self { fluid_mass_kg: 5.0, ambient_loss_w_per_k: 15.0 }
    }
}

// ---------------------------------------------------------------------------
// ECAM completeness pass (E-FIRE §G/§H): pure monitored-discrete faults on
// the fuel/hydraulic heat exchanger's own valve, its air-side leak-
// detection switch, and the dual overheat-detection channels. None of
// these change the heat balance above: each is "this monitored circuit
// itself is known-faulted," a direct pass-through with no threshold
// invented -- the same class of mapping `sensors::smoke_detector`'s own
// `circuit_fault` field uses (module doc there), applied here to the three
// classes of monitored hardware ATA 29-30's unwired procedures name.
// ---------------------------------------------------------------------------

/// The fuel/hydraulic heat exchanger's own valve health, independent of
/// the `Hsmu`'s commanded open/closed decision above: `Hsmu` decides
/// *when* the valve should open (a control law, already sourced from the
/// FCOM); this is the valve's own monitored position/circuit, which can be
/// known-faulted regardless of what the HSMU commands. `290800013`/
/// `290800014` HYD {G,Y} FUEL HEAT EXCHANGER VLV FAULT.
#[derive(Clone, Copy, Debug, Default)]
pub struct HxValveFaults {
    /// 0 healthy .. 1 (any nonzero) the valve's own monitored position/
    /// circuit reports faulted.
    pub stuck: f64,
}
impl HxValveFaults {
    pub fn fault(&self) -> bool {
        self.stuck > 0.0
    }
}

/// The heat exchanger's air-side leak-detection switch. `leak` is the leak
/// itself -- a registered fault's own armed state, pass-through, the same
/// class of leak failure `thermal_zones`' own duct-leak faults already use
/// elsewhere in this crate (`290800015`/`290800016` HYD {G,Y} HEAT
/// EXCHANGER AIR LEAK). `circuit_fault` is the switch's *own* monitoring
/// circuit, independent of whether a leak is actually present
/// (`290800017`/`290800018` ... AIR LEAK DET FAULT).
#[derive(Clone, Copy, Debug, Default)]
pub struct AirLeakSwitchFaults {
    pub leak: f64,
    pub circuit_fault: f64,
}
impl AirLeakSwitchFaults {
    pub fn leak_detected(&self) -> bool {
        self.leak > 0.0
    }
    pub fn fault(&self) -> bool {
        self.circuit_fault > 0.0
    }
}

/// One overheat-detection channel (A or B): the same dual-channel/loop
/// redundancy pattern this exact crate already uses for exactly this class
/// of monitored quantity (`fire_and_smoke_protection.rs`'s A/B fire-
/// detection loops, and this crate's own `ENG n IGN A/B FAULT` chain-fault
/// pattern). When healthy, a channel reports the same `ThermalState::step`
/// overheat verdict this file already computes (unchanged, still the
/// Skydrol LD-4-sourced `OVERHEAT_K`); each channel independently carries
/// its own `circuit_fault`, a monitored discrete orthogonal to that
/// verdict (`290800023`..`290800026` ... SYS CHAN A/B OVHT DET FAULT).
#[derive(Clone, Copy, Debug, Default)]
pub struct OverheatChannelFaults {
    pub circuit_fault: f64,
}
impl OverheatChannelFaults {
    pub fn fault(&self) -> bool {
        self.circuit_fault > 0.0
    }
}

#[cfg(test)]
mod discrete_fault_tests {
    use super::*;

    #[test]
    fn hx_valve_fault_is_a_direct_pass_through() {
        assert!(!HxValveFaults::default().fault());
        assert!(HxValveFaults { stuck: 1.0 }.fault());
        assert!(HxValveFaults { stuck: 0.3 }.fault(), "any nonzero magnitude raises the discrete, matching the smoke-detector circuit_fault convention");
    }

    #[test]
    fn air_leak_switch_separates_the_leak_from_its_own_circuit_fault() {
        let leaking = AirLeakSwitchFaults { leak: 1.0, circuit_fault: 0.0 };
        assert!(leaking.leak_detected());
        assert!(!leaking.fault(), "a real leak must not, by itself, raise the switch's own DET FAULT");
        let switch_faulted = AirLeakSwitchFaults { leak: 0.0, circuit_fault: 1.0 };
        assert!(!switch_faulted.leak_detected());
        assert!(switch_faulted.fault());
    }

    #[test]
    fn overheat_channel_fault_is_a_direct_pass_through() {
        assert!(!OverheatChannelFaults::default().fault());
        assert!(OverheatChannelFaults { circuit_fault: 1.0 }.fault());
    }
}

/// GENERIC: no published specific heat for this fluid family; phosphate
/// esters run somewhat below mineral oil's ~2000 J/(kg K)
/// (`physics::engine::oil.rs`'s `OIL_CP`), a representative order of
/// magnitude for a similar-density synthetic fluid.
const FLUID_CP_J_KG_K: f64 = 1900.0;
const HHX_EFFECTIVENESS: f64 = 0.6;
/// Matches this crate's own Jet A specific heat, `physics::engine::oil.rs`'s
/// `FUEL_CP`.
const FUEL_CP_J_KG_K: f64 = 2010.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct ThermalOutputs {
    pub temp_k: f64,
    pub temp_c: f64,
    pub overheat: bool,
    pub fuel_heat_w: f64,
    pub fuel_out_k: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ThermalState {
    temp_k: f64,
}
impl ThermalState {
    pub fn new(temp_k: f64) -> Self {
        Self { temp_k }
    }
    pub fn temp_k(&self) -> f64 {
        self.temp_k
    }
    pub fn temp_c(&self) -> f64 {
        self.temp_k - 273.15
    }

    /// `pump_heat_w`/`throttling_heat_w`: this tick's dissipated power (both
    /// always >= 0 physically). `fluid_flow_kg_s`: the circuit's own flow
    /// passing through its heat exchanger (pump delivery, roughly); only the
    /// flowing fraction of the fluid mass is actively cooled each tick, the
    /// same convention `physics::engine::oil.rs`'s FCOC uses. `fuel_kg_s`/
    /// `fuel_temp_k`: the engine feed fuel available to absorb that heat.
    /// `ambient_k`: the hydraulic bay/nacelle temperature the passive loss
    /// term sinks to. An exact exponential step (unconditionally stable
    /// regardless of `dt_s`), matching this crate's established convention
    /// for first-order thermal lags.
    pub fn step(
        &mut self,
        sizing: &ThermalSizing,
        pump_heat_w: f64,
        throttling_heat_w: f64,
        fluid_flow_kg_s: f64,
        fuel_kg_s: f64,
        fuel_temp_k: f64,
        ambient_k: f64,
        dt_s: f64,
    ) -> ThermalOutputs {
        let dt = dt_s.max(0.0);
        let heat_in_w = pump_heat_w.max(0.0) + throttling_heat_w.max(0.0);

        let fluid_capacity_w_per_k = fluid_flow_kg_s.max(0.0) * FLUID_CP_J_KG_K;
        let fuel_capacity_w_per_k = fuel_kg_s.max(0.0) * FUEL_CP_J_KG_K;
        let hx_conductance_w_per_k = HHX_EFFECTIVENESS * fluid_capacity_w_per_k.min(fuel_capacity_w_per_k);
        let fuel_heat_w = hx_conductance_w_per_k * (self.temp_k - fuel_temp_k);
        let fuel_out_k = if fuel_capacity_w_per_k > 0.0 { fuel_temp_k + fuel_heat_w / fuel_capacity_w_per_k } else { fuel_temp_k };

        let ambient_loss_w_per_k = sizing.ambient_loss_w_per_k.max(0.0);
        let conductance = (hx_conductance_w_per_k + ambient_loss_w_per_k).max(1e-9);
        let target_k = (heat_in_w + hx_conductance_w_per_k * fuel_temp_k + ambient_loss_w_per_k * ambient_k) / conductance;
        let thermal_mass_j_per_k = sizing.fluid_mass_kg.max(1e-6) * FLUID_CP_J_KG_K;
        let rate = conductance / thermal_mass_j_per_k;
        self.temp_k = target_k + (self.temp_k - target_k) * (-rate * dt).exp();

        ThermalOutputs { temp_k: self.temp_k, temp_c: self.temp_c(), overheat: self.temp_k > OVERHEAT_K, fuel_heat_w, fuel_out_k }
    }
}

/// Heat dissipated by a throttling restriction carrying `flow_m3_s` (always
/// >= 0, direction irrelevant to the heat produced) across `dp_pa` -- the
/// lost hydraulic power a relief valve, priority valve, or leak converts
/// entirely to heat rather than useful work. Callers (`topology.rs`) sum
/// this across every throttling element in a circuit each tick.
pub fn throttling_heat_w(flow_m3_s: f64, dp_pa: f64) -> f64 {
    flow_m3_s.abs() * dp_pa.abs()
}

// ---------------------------------------------------------------------------
// The HSMU's cooling control.
//
// The model above says how hot the fluid gets and how the fuel-cooled
// exchanger takes heat out of it. What it did not have is the unit that
// decides *when* any of that cooling runs -- and on the A380 that decision
// is neither continuous nor simple.
//
// Every threshold below is the FCOM's (DSC-29-10 p.2066, "the HSMU"):
//
//   * inner air/hydraulic heat exchanger: fans on above 55 C, off below 35 C;
//   * outer air/hydraulic heat exchanger: fans on above 20 C, off below 0 C
//     -- a much colder pair, so the outer exchanger is working almost
//     whenever the aircraft is;
//   * the fans are inhibited above M 0.45, *unless* wing anti-ice has been
//     turned on during the flight. That exception is a latch on the flight,
//     not a live state, which is why the input is named for what it is;
//   * the fuel/hydraulic exchanger's valves open above 85 C and close below
//     40 C. The direction matters: 85 C *engages* extra cooling; it is not
//     a shutoff, and reading it as one gets the failure case backwards;
//   * and the FQMS refuses that exchanger outright on low feed tank 1(4)
//     quantity, feed tank fuel above 53 C, crossfeed valves open, gravity
//     feed, low feed pump pressure, or electrical emergency configuration.
//
// The FCOM also states the causal chain this makes possible: the
// fuel/hydraulic exchanger "is active when an air/hydraulic heat exchanger
// fails", that failure being "detected by an abnormal increase of the
// hydraulic reservoir temperature". Nothing needs to assert that here --
// losing the air cooling raises the temperature, and the temperature opens
// the valves.
// ---------------------------------------------------------------------------

/// Inner air/hydraulic exchanger: fans on above this fluid temperature, C.
pub const INNER_FAN_ON_C: f64 = 55.0;
/// Inner exchanger: fans off below this, C.
pub const INNER_FAN_OFF_C: f64 = 35.0;
/// Outer air/hydraulic exchanger: fans on above this, C.
pub const OUTER_FAN_ON_C: f64 = 20.0;
/// Outer exchanger: fans off below this, C.
pub const OUTER_FAN_OFF_C: f64 = 0.0;
/// Fans are inhibited above this Mach number.
pub const FAN_INHIBIT_MACH: f64 = 0.45;
/// The fuel/hydraulic exchanger's valves open above this, C.
pub const FUEL_EXCHANGER_OPEN_C: f64 = 85.0;
/// ...and close below this, C.
pub const FUEL_EXCHANGER_CLOSE_C: f64 = 40.0;
/// The FQMS inhibits the fuel/hydraulic exchanger above this feed tank fuel
/// temperature, C.
pub const FEED_TANK_FUEL_INHIBIT_C: f64 = 53.0;

/// GENERIC: how much conductance one air/hydraulic exchanger adds with its
/// fans running, W/K, and with them stopped but still passing air. No
/// published A380 figure exists, so these are sized from the one behaviour
/// the FCOM does pin down: the fuel/hydraulic exchanger is "active when an
/// air/hydraulic heat exchanger fails", which means the air exchangers alone
/// must hold a healthy circuit *below* the 85 C that opens the fuel
/// exchanger's valves. Two of them at this conductance carry a circuit's
/// heat rejection with the fluid comfortably under that, and losing both
/// takes it well over -- which is the whole point of the fuel exchanger
/// existing.
const AIR_EXCHANGER_FAN_W_PER_K: f64 = 800.0;
const AIR_EXCHANGER_STILL_W_PER_K: f64 = 150.0;

/// The FQMS's reasons for refusing the fuel/hydraulic exchanger. Fuel
/// temperature is not here: [`Hsmu::step`] checks it itself, so the sourced
/// 53 C sits with the other thresholds rather than in a caller.
#[derive(Clone, Copy, Debug, Default)]
pub struct FqmsInhibit {
    pub low_feed_tank_quantity: bool,
    pub crossfeed_open: bool,
    pub gravity_feed: bool,
    pub low_feed_pump_pressure: bool,
    pub electrical_emergency: bool,
}

impl FqmsInhibit {
    pub fn any(&self) -> bool {
        self.low_feed_tank_quantity || self.crossfeed_open || self.gravity_feed || self.low_feed_pump_pressure || self.electrical_emergency
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct HsmuInputs {
    pub mach: f64,
    /// Latched for the flight, not a live state -- see the section comment.
    pub wing_anti_ice_used_this_flight: bool,
    /// Fuel temperature in the feed tank this circuit's exchanger draws
    /// from: tank 1 for green, tank 4 for yellow.
    pub feed_tank_fuel_c: f64,
    pub fqms_inhibit: FqmsInhibit,
    /// `[inner, outer]`: whether each air/hydraulic exchanger still works.
    pub air_exchanger_ok: [bool; 2],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HsmuOutputs {
    /// `[inner, outer]`.
    pub fans_running: [bool; 2],
    /// Temperature wanted the fans, but the Mach inhibit held them off.
    pub fans_inhibited: bool,
    pub fuel_exchanger_valves_open: bool,
    /// Temperature wanted the exchanger, but the FQMS refused it.
    pub fuel_exchanger_inhibited: bool,
}

/// One circuit's Hydraulic System Monitoring Unit, as far as cooling goes.
#[derive(Clone, Copy, Debug, Default)]
pub struct Hsmu {
    fans_on: [bool; 2],
    fuel_valves_open: bool,
}

impl Hsmu {
    pub fn new() -> Self {
        Self::default()
    }

    /// On above `on_c`, off below `off_c`, unchanged in between.
    fn hysteresis(state: bool, value_c: f64, on_c: f64, off_c: f64) -> bool {
        if value_c > on_c {
            true
        } else if value_c < off_c {
            false
        } else {
            state
        }
    }

    /// Decide this tick's cooling from the fluid temperature. Pure control:
    /// the heat balance itself stays in [`ThermalState::step`], which the
    /// caller feeds using [`Self::cooled_sizing`] and by gating its
    /// `fuel_kg_s` on [`HsmuOutputs::fuel_exchanger_valves_open`].
    pub fn step(&mut self, fluid_c: f64, inputs: &HsmuInputs) -> HsmuOutputs {
        let wanted = [
            Self::hysteresis(self.fans_on[0], fluid_c, INNER_FAN_ON_C, INNER_FAN_OFF_C),
            Self::hysteresis(self.fans_on[1], fluid_c, OUTER_FAN_ON_C, OUTER_FAN_OFF_C),
        ];
        let inhibited = inputs.mach > FAN_INHIBIT_MACH && !inputs.wing_anti_ice_used_this_flight;
        self.fans_on = if inhibited { [false; 2] } else { wanted };

        let fuel_too_hot = inputs.feed_tank_fuel_c > FEED_TANK_FUEL_INHIBIT_C;
        let refused = inputs.fqms_inhibit.any() || fuel_too_hot;
        let wanted_by_temp = Self::hysteresis(self.fuel_valves_open, fluid_c, FUEL_EXCHANGER_OPEN_C, FUEL_EXCHANGER_CLOSE_C);
        self.fuel_valves_open = wanted_by_temp && !refused;

        HsmuOutputs {
            fans_running: self.fans_on,
            fans_inhibited: inhibited && (wanted[0] || wanted[1]),
            fuel_exchanger_valves_open: self.fuel_valves_open,
            fuel_exchanger_inhibited: wanted_by_temp && refused,
        }
    }

    /// `sizing` with the air/hydraulic exchangers' conductance folded into
    /// its ambient loss, which is how the fans reach the heat balance. An
    /// exchanger that has failed contributes nothing at all.
    pub fn cooled_sizing(&self, sizing: &ThermalSizing, air_exchanger_ok: [bool; 2]) -> ThermalSizing {
        let mut extra = 0.0;
        for i in 0..2 {
            if air_exchanger_ok[i] {
                extra += if self.fans_on[i] { AIR_EXCHANGER_FAN_W_PER_K } else { AIR_EXCHANGER_STILL_W_PER_K };
            }
        }
        ThermalSizing { fluid_mass_kg: sizing.fluid_mass_kg, ambient_loss_w_per_k: sizing.ambient_loss_w_per_k + extra }
    }

    pub fn fans_running(&self) -> [bool; 2] {
        self.fans_on
    }

    pub fn fuel_exchanger_valves_open(&self) -> bool {
        self.fuel_valves_open
    }
}

#[cfg(test)]
mod hsmu_tests {
    use super::*;

    fn inputs() -> HsmuInputs {
        HsmuInputs { mach: 0.0, wing_anti_ice_used_this_flight: false, feed_tank_fuel_c: 10.0, fqms_inhibit: FqmsInhibit::default(), air_exchanger_ok: [true; 2] }
    }

    /// The two exchangers have different, sourced thresholds, and each holds
    /// its state inside its own band.
    #[test]
    fn each_exchangers_fans_follow_their_own_sourced_hysteresis() {
        let mut h = Hsmu::new();
        assert!(!h.step(50.0, &inputs()).fans_running[0], "50 C sits inside the inner band, so off stays off");
        assert!(h.step(56.0, &inputs()).fans_running[0], "above 55 C the inner fans run");
        assert!(h.step(40.0, &inputs()).fans_running[0], "40 C is inside the band, so on stays on");
        assert!(!h.step(34.0, &inputs()).fans_running[0], "below 35 C they stop");

        // The outer pair is far colder, so it runs almost always.
        let mut h = Hsmu::new();
        let out = h.step(25.0, &inputs());
        assert!(out.fans_running[1], "above 20 C the outer fans run");
        assert!(!out.fans_running[0], "...while the inner ones are nowhere near theirs");
    }

    /// Inhibited above M 0.45 -- unless wing anti-ice has been used.
    #[test]
    fn the_mach_inhibit_applies_unless_wing_anti_ice_has_been_used() {
        let mut h = Hsmu::new();
        let mut i = inputs();
        i.mach = 0.8;
        let out = h.step(70.0, &i);
        assert!(!out.fans_running[0] && !out.fans_running[1], "fast and clean, the fans are held off");
        assert!(out.fans_inhibited, "and it is reported, because temperature did want them");

        i.wing_anti_ice_used_this_flight = true;
        let out = h.step(70.0, &i);
        assert!(out.fans_running[0], "wing anti-ice used this flight lifts the inhibit");
        assert!(!out.fans_inhibited);

        i.wing_anti_ice_used_this_flight = false;
        i.mach = 0.3;
        assert!(h.step(70.0, &i).fans_running[0], "slow again, no inhibit");
    }

    /// 85 C *opens* the fuel exchanger. Reading it as a shutoff gets the
    /// failure case exactly backwards.
    #[test]
    fn the_fuel_exchanger_opens_hot_and_closes_cold() {
        let mut h = Hsmu::new();
        assert!(!h.step(80.0, &inputs()).fuel_exchanger_valves_open, "80 C is below the opening threshold");
        assert!(h.step(86.0, &inputs()).fuel_exchanger_valves_open, "above 85 C the valves open");
        assert!(h.step(50.0, &inputs()).fuel_exchanger_valves_open, "50 C is inside the band, they stay open");
        assert!(!h.step(39.0, &inputs()).fuel_exchanger_valves_open, "below 40 C they close");
    }

    /// Every FQMS condition refuses it on its own, the 53 C fuel temperature
    /// included, and each reports the refusal.
    #[test]
    fn every_fqms_condition_refuses_the_fuel_exchanger() {
        let cases: [(&str, fn(&mut HsmuInputs)); 6] = [
            ("low feed tank quantity", |i| i.fqms_inhibit.low_feed_tank_quantity = true),
            ("crossfeed open", |i| i.fqms_inhibit.crossfeed_open = true),
            ("gravity feed", |i| i.fqms_inhibit.gravity_feed = true),
            ("low feed pump pressure", |i| i.fqms_inhibit.low_feed_pump_pressure = true),
            ("electrical emergency", |i| i.fqms_inhibit.electrical_emergency = true),
            ("feed tank fuel above 53 C", |i| i.feed_tank_fuel_c = 54.0),
        ];
        for (name, arm) in cases {
            let mut h = Hsmu::new();
            let mut i = inputs();
            arm(&mut i);
            let out = h.step(90.0, &i);
            assert!(!out.fuel_exchanger_valves_open, "{name} must refuse the exchanger");
            assert!(out.fuel_exchanger_inhibited, "{name} should say it refused");
        }
        // Exactly 53 C is not "above" 53 C.
        let mut h = Hsmu::new();
        let mut i = inputs();
        i.feed_tank_fuel_c = FEED_TANK_FUEL_INHIBIT_C;
        assert!(h.step(90.0, &i).fuel_exchanger_valves_open);
    }

    /// Running fans must actually cool: the conductance they add has to
    /// dominate the passive loss, and a failed exchanger must add nothing.
    #[test]
    fn the_fans_conductance_reaches_the_heat_balance() {
        let base = ThermalSizing::a380_circuit();
        let mut h = Hsmu::new();
        h.step(10.0, &inputs());
        let cold = h.cooled_sizing(&base, [true; 2]);
        h.step(90.0, &inputs());
        let hot = h.cooled_sizing(&base, [true; 2]);
        assert!(hot.ambient_loss_w_per_k > cold.ambient_loss_w_per_k, "running fans must add conductance");
        assert!(hot.ambient_loss_w_per_k > 2.0 * base.ambient_loss_w_per_k, "and dominate the passive loss");
        assert_eq!(hot.fluid_mass_kg, base.fluid_mass_kg, "cooling must not change the thermal mass");

        let failed = h.cooled_sizing(&base, [false; 2]);
        assert_eq!(failed.ambient_loss_w_per_k, base.ambient_loss_w_per_k, "failed exchangers add nothing");
    }

    /// The FCOM's own causal chain, falling out of the heat balance rather
    /// than being asserted: losing the air cooling drives the fluid up to
    /// the temperature that brings the fuel exchanger in.
    #[test]
    fn losing_the_air_exchangers_heats_the_fluid_until_the_fuel_exchanger_takes_over() {
        let sizing = ThermalSizing::a380_circuit();
        let run = |air_ok: [bool; 2]| {
            let mut h = Hsmu::new();
            let mut state = ThermalState::new(298.15);
            let mut i = inputs();
            i.air_exchanger_ok = air_ok;
            i.feed_tank_fuel_c = 20.0;
            let mut out = HsmuOutputs::default();
            for _ in 0..7_200 {
                out = h.step(state.temp_c(), &i);
                let cooled = h.cooled_sizing(&sizing, air_ok);
                // The valves gate the fuel flow: that is how the HSMU's
                // decision reaches the heat balance.
                let fuel_kg_s = if out.fuel_exchanger_valves_open { 0.5 } else { 0.0 };
                state.step(&cooled, 60_000.0, 0.0, 2.0, fuel_kg_s, 293.15, 313.15, 0.5);
            }
            (state.temp_c(), out)
        };

        let (healthy_c, healthy) = run([true; 2]);
        assert!(healthy_c < FUEL_EXCHANGER_OPEN_C, "healthy air cooling should stay below 85 C, got {healthy_c}");
        assert!(!healthy.fuel_exchanger_valves_open, "so the fuel exchanger is never called on");
        assert!(healthy.fans_running[0] || healthy.fans_running[1], "something should be cooling");

        let (failed_c, failed) = run([false; 2]);
        assert!(failed_c > healthy_c, "losing the air exchangers must run hotter");
        assert!(failed.fuel_exchanger_valves_open, "and must bring the fuel exchanger in");
        // The fuel sink arrests the rise rather than letting it run away,
        // but not below the band that keeps the valves open.
        assert!(failed_c.is_finite() && failed_c > FUEL_EXCHANGER_CLOSE_C);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heating_with_no_cooling_path_raises_temperature() {
        let mut t = ThermalState::new(288.15);
        let sizing = ThermalSizing::a380_circuit();
        let mut out = ThermalOutputs::default();
        for _ in 0..600 {
            out = t.step(&sizing, 2000.0, 0.0, 0.0, 0.0, 288.15, 288.15, 1.0);
        }
        assert!(out.temp_k > 288.15);
    }

    #[test]
    fn the_fuel_heat_exchanger_removes_heat_and_warms_the_fuel() {
        let mut t = ThermalState::new(360.0);
        let sizing = ThermalSizing::a380_circuit();
        let out = t.step(&sizing, 0.0, 0.0, 1.0, 1.0, 288.15, 288.15, 0.01);
        assert!(out.fuel_heat_w > 0.0);
        assert!(out.fuel_out_k > 288.15);
    }

    #[test]
    fn settles_to_a_stable_temperature_under_constant_heat_load() {
        let mut t = ThermalState::new(288.15);
        let sizing = ThermalSizing::a380_circuit();
        let mut last = 0.0;
        for _ in 0..20000 {
            let out = t.step(&sizing, 3000.0, 500.0, 0.5, 1.5, 288.15, 288.15, 1.0);
            last = out.temp_k;
        }
        assert!(last.is_finite());
        assert!(last < 500.0, "should settle, not run away: {last} K");
        assert!(last > 288.15);
    }

    #[test]
    fn overheat_flag_trips_above_the_skydrol_continuous_limit() {
        let mut t = ThermalState::new(OVERHEAT_K + 5.0);
        let out = t.step(&ThermalSizing::a380_circuit(), 0.0, 0.0, 0.0, 0.0, 288.15, 288.15, 0.0);
        assert!(out.overheat);
        let mut cool = ThermalState::new(320.0);
        let out2 = cool.step(&ThermalSizing::a380_circuit(), 0.0, 0.0, 0.0, 0.0, 288.15, 288.15, 0.0);
        assert!(!out2.overheat);
    }

    #[test]
    fn throttling_heat_scales_with_flow_and_pressure_drop() {
        assert_eq!(throttling_heat_w(0.0, 1000.0), 0.0);
        let h1 = throttling_heat_w(1e-4, 1.0e6);
        let h2 = throttling_heat_w(2e-4, 1.0e6);
        assert!((h2 / h1 - 2.0).abs() < 1e-9);
    }

    #[test]
    fn no_nan_at_dt_zero_or_rest() {
        let mut t = ThermalState::new(288.15);
        let out = t.step(&ThermalSizing::a380_circuit(), 0.0, 0.0, 0.0, 0.0, 288.15, 288.15, 0.0);
        assert!(out.temp_k.is_finite());
        assert!((out.temp_k - 288.15).abs() < 1e-9, "dt=0 must not move the state: {}", out.temp_k);
    }
}

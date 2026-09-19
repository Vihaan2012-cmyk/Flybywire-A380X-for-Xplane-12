//! Per-wheel carbon brakes: a carbon-carbon heat-sink thermal model, energy
//! based wear, a per-wheel antiskid channel, brake fire, and the parking
//! brake's hydraulic accumulator.
//!
//! Only the 16 *braked* wheel positions are modelled here (the same 16 the
//! plugin's existing `physics::tyre::Tyres` already carries: each wing leg's
//! 4 wheels, each body leg's forward/middle-axle 4 wheels -- the body
//! bogie's rear axle, 2 wheels per leg, is steered rather than braked, the
//! brief's "16 braked" of the 22 total). This module cannot edit
//! `physics/tyre.rs` (self-contained rule), so the coordination point is
//! `BrakeWheel::stack_temp_c`: whoever wires this in can feed this module's
//! own, richer per-wheel carbon temperature into
//! `tyre::TyreWheel::step`'s `brake_temp_c` parameter in place of (or
//! blended with) FlyByWire's coarser `BRAKE_TEMPERATURE_n`, without any
//! change to `tyre.rs` itself.
//!
//! # Wheel rotational dynamics
//! This module owns the wheel's own spin state (`wheel_speed_ms`, the
//! rolling/surface speed): it is no longer an external input but an
//! integrated state of a genuine friction problem. Reducing the wheel's
//! rotational inertia `I` to an equivalent linear mass at the contact patch
//! (`M_eff = I / r^2`, the standard rigid-wheel reduction), the tyre-ground
//! interface can only supply a friction force up to Coulomb's limit `mu*N`
//! (`N` the wheel's own normal load); a friction complementarity solve --
//! compute the force that *would* hold the wheel at zero slip against the
//! ground this tick, then clamp it to that limit -- gives, when unclamped,
//! exact zero-slip rolling regardless of step size (real static friction
//! is not a stiff spring, it simply supplies whatever is needed up to its
//! capacity), and, when clamped, a genuine, physically caused skid: the
//! wheel decelerates or fails to spin up because the available friction
//! could not keep pace with the commanded brake torque, not because
//! anything told it to skid. `mu` (`MU_TIRE_GROUND`), the wheel's inertia
//! and radius are GENERIC (no public A380 wheel/tyre data), sized to
//! representative large-transport figures. Off the ground (`on_ground =
//! false`, `normal_load_n` irrelevant since the friction limit is zero
//! either way), only a small residual bearing/aerodynamic drag acts, so a
//! wheel keeps spinning for a while after liftoff, as real ones do.
//!
//! # Thermal model
//! Brake friction heat is generated at the disc/stator interface, so it
//! scales with the wheel's *own* rotational (surface) speed, not the
//! ground's: `brake_command * F_max * wheel_speed_ms`. A fully locked,
//! skidding wheel's disc is not rotating relative to its own stator, so it
//! generates essentially no *brake* heat here even while its tyre is
//! scrubbing hard against the runway (that heat goes into the tyre, the
//! separate physical element `physics::tyre.rs` already tracks) -- an
//! emergent, not scripted, distinction that now falls straight out of
//! owning the wheel's real rotational state, the same first-principles
//! kinetic-energy-to-heat idea `physics::damage.rs`'s own (coarser,
//! whole-aircraft) `update_brake_energy` already uses. `F_max` (this
//! module's total per-wheel maximum brake force) is derived, not assumed:
//! transport aircraft are commonly designed for on the order of 0.3g
//! maximum braking deceleration on a dry runway (a widely cited aviation
//! performance figure, not a specific Airbus number), so `F_max_total =
//! MLW * 0.3g`, split evenly over the 16 braked wheels. The carbon heat
//! sink's specific heat (~1650 J/(kg K)) is a representative published
//! figure for aircraft carbon-carbon brake material (materials handbooks /
//! brake literature general range 1400-2000 J/(kg K), not a specific
//! manufacturer's data sheet); the per-wheel stack mass (GENERIC, ~100 kg)
//! is order-of-magnitude for a large-transport wheel brake assembly.
//!
//! Cooling is natural/forced convection (stronger while rolling, the same
//! flow-scaling idea `physics/engine/hot_section.rs` uses for its own
//! metal-mass cooling) plus, at high stack temperature, genuine radiative
//! loss (Stefan-Boltzmann): hot carbon brakes visibly glow, a well
//! documented phenomenon, and radiative loss becomes a first-order term
//! there, not a refinement to skip.
//!
//! # Antiskid
//! A per-wheel channel comparing this wheel's own (now internally modelled)
//! rolling speed, sampled at the *start* of the tick (a one-cycle sensing
//! latency, matching a real antiskid computer's own sample-and-hold loop),
//! against the aircraft's true groundspeed: above a slip-ratio threshold it
//! releases (cuts) commanded brake pressure until slip recovers, the
//! standard (publicly documented, not A380-proprietary) antiskid modulation
//! concept. Because the wheel's speed is now a real state driven by the
//! same friction/brake-torque balance above, a channel with its antiskid
//! disabled (`antiskid_inop`) can now genuinely lock that wheel (drive
//! `wheel_speed_ms` to zero) when commanded braking exceeds the available
//! ground friction, rather than skidding being an external input.
//!
//! # Parking brake accumulator
//! A nitrogen-precharged piston accumulator stores hydraulic pressure to
//! hold the brakes applied without a running pump, following the same
//! Boyle's-law (isothermal, slow discharge) form `physics::tyre.rs` already
//! uses for a tyre's sealed-volume pressure: `P = P0 * V0 / (V0 +
//! discharged_volume)`.

/// Aircraft MLW, matching `super`'s citation (Airbus AC doc).
use super::MLW_KG;
use super::G_MS2;

/// Representative dry-runway maximum braking deceleration for a transport
/// aircraft (GENERIC, widely cited order-of-magnitude aviation performance
/// figure, not Airbus-specific), used only to derive a per-wheel maximum
/// brake force from first principles.
const MAX_BRAKING_DECEL_G: f64 = 0.3;
const BRAKED_WHEEL_COUNT: f64 = 16.0;
/// Per-wheel maximum brake force, N (derived: `MLW * MAX_BRAKING_DECEL_G *
/// G_MS2 / BRAKED_WHEEL_COUNT`).
fn max_brake_force_n() -> f64 {
    MLW_KG * MAX_BRAKING_DECEL_G * G_MS2 / BRAKED_WHEEL_COUNT
}

/// Carbon-carbon composite specific heat, J/(kg K) (representative published
/// figure for aircraft brake material, general 1400-2000 J/(kg K) range).
const CARBON_SPECIFIC_HEAT_J_KG_K: f64 = 1650.0;
/// Per-wheel carbon heat-sink mass, kg (GENERIC, order-of-magnitude large
/// transport wheel brake stack).
const STACK_MASS_KG: f64 = 100.0;
const STACK_CAPACITY_J_K: f64 = CARBON_SPECIFIC_HEAT_J_KG_K * STACK_MASS_KG;

/// Natural + forced convection coefficient at zero groundspeed and at the
/// design rolling speed respectively, W/(m^2 K) (GENERIC, order-of-magnitude
/// for an exposed wheel/brake assembly in an airstream).
const CONVECTION_STATIC_W_M2K: f64 = 12.0;
const CONVECTION_ROLLING_W_M2K: f64 = 120.0;
/// Groundspeed at which forced convection is considered fully developed,
/// m/s (GENERIC, a representative landing roll speed).
const CONVECTION_FULL_SPEED_MS: f64 = 40.0;
/// Exposed radiating/convecting area of one wheel/brake assembly, m^2
/// (GENERIC).
const SURFACE_AREA_M2: f64 = 1.2;
const EMISSIVITY: f64 = 0.85; // oxidised carbon, high emissivity.
const STEFAN_BOLTZMANN: f64 = 5.670_374e-8;

/// Wear-life energy budget per wheel, J (GENERIC: sized so many landings'
/// worth of ordinary braking wear a stack out over a plausible brake-change
/// interval, order of magnitude only).
const WEAR_LIFE_ENERGY_J: f64 = 5.0e10;

/// Sustained stack temperature above which a brake fire is considered to
/// have started, C (GENERIC: well above the highest ordinary operating
/// temperatures, informed by public discussion of hot-brake/brake-fire
/// cautions after a rejected takeoff or a dragging brake).
const FIRE_TEMP_C: f64 = 800.0;
const FIRE_ARM_SECONDS: f64 = 60.0;

/// Antiskid slip-ratio threshold above which pressure is released (GENERIC,
/// textbook antiskid design order of magnitude).
const SKID_SLIP_THRESHOLD: f64 = 0.15;
/// How fast antiskid releases/reapplies pressure, fraction/s (GENERIC).
const ANTISKID_RATE_PER_S: f64 = 4.0;

/// Rolling radius, m (GENERIC order-of-magnitude for a large transport main
/// wheel; no public A380 figure).
const WHEEL_RADIUS_M: f64 = 0.6;
/// Wheel + tyre + brake rotor rotational inertia about its axle, kg*m^2
/// (GENERIC, order-of-magnitude for a large transport wheel assembly).
const WHEEL_INERTIA_KG_M2: f64 = 25.0;
/// The wheel's inertia reduced to an equivalent linear mass at the contact
/// patch (`I / r^2`), used throughout the friction solve below.
const M_EFF_KG: f64 = WHEEL_INERTIA_KG_M2 / (WHEEL_RADIUS_M * WHEEL_RADIUS_M);
/// Tyre-ground (rubber on dry runway) friction coefficient (GENERIC, widely
/// cited order-of-magnitude aviation figure; not a specific tyre compound).
const MU_TIRE_GROUND: f64 = 0.8;
/// Residual bearing/aerodynamic drag deceleration on a free-spinning,
/// unloaded wheel, m/s^2 (GENERIC: a wheel keeps spinning for some tens of
/// seconds after liftoff before this brings it to rest).
const BEARING_DRAG_DECEL_MS2: f64 = 0.3;

/// Faults one brake wheel carries, each a fraction 0 (healthy) .. 1 (fully
/// failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct BrakeFaults {
    /// This wheel's antiskid channel is degraded/inoperative: 1.0 = no skid
    /// protection at all.
    pub antiskid_inop: f64,
    /// Mechanical drag (a partially-applied brake that never fully
    /// releases): an added, uncommanded brake-force fraction.
    pub dragging: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct BrakeWheelInputs {
    /// Commanded brake application, 0..1 (pedal/autobrake demand).
    pub commanded: f64,
    /// Whether this wheel is on the ground this tick (owning leg's strut
    /// state); off the ground the tyre-ground friction limit is zero
    /// regardless of `normal_load_n`.
    pub on_ground: bool,
    /// This wheel's own share of the leg's normal load, N (the leg's strut
    /// reaction divided by its total wheel count, braked and unbraked
    /// alike -- only meaningful while `on_ground`).
    pub normal_load_n: f64,
    /// The aircraft's true groundspeed, m/s.
    pub groundspeed_ms: f64,
    pub ambient_c: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BrakeWheelOutputs {
    pub stack_temp_c: f64,
    /// Fraction of maximum brake force actually delivered this tick, after
    /// antiskid release and dragging are applied.
    pub applied_fraction: f64,
    pub wear_fraction: f64,
    pub fire: bool,
    /// This wheel was skidding significantly (antiskid released pressure).
    pub skidding: bool,
    /// This wheel's own rolling/surface speed right now, m/s (own state,
    /// not an echo of an input -- see the module doc's wheel dynamics).
    pub wheel_speed_ms: f64,
}

pub struct BrakeWheel {
    stack_temp_c: f64,
    wear_fraction: f64,
    antiskid_release: f64,
    fire: bool,
    seconds_above_fire_temp: f64,
    wheel_speed_ms: f64,
}

impl BrakeWheel {
    pub fn new(ambient_c: f64) -> Self {
        Self { stack_temp_c: ambient_c, wear_fraction: 0.0, antiskid_release: 0.0, fire: false, seconds_above_fire_temp: 0.0, wheel_speed_ms: 0.0 }
    }

    pub fn stack_temp_c(&self) -> f64 {
        self.stack_temp_c
    }

    pub fn wheel_speed_ms(&self) -> f64 {
        self.wheel_speed_ms
    }

    pub fn step(&mut self, inputs: &BrakeWheelInputs, faults: &BrakeFaults) -> BrakeWheelOutputs {
        let dt = inputs.dt_s.max(0.0);
        let gs = inputs.groundspeed_ms.max(0.0);

        // Antiskid: slip ratio from *this wheel's own* speed, sampled
        // before this tick's dynamics update (module doc: a one-cycle
        // sensing latency).
        let slip = if gs > 0.5 { ((gs - self.wheel_speed_ms.max(0.0)) / gs).clamp(0.0, 1.0) } else { 0.0 };
        let antiskid_effective = (1.0 - faults.antiskid_inop.clamp(0.0, 1.0)).max(0.0);
        let skidding = slip > SKID_SLIP_THRESHOLD;
        let release_target = if skidding { (slip - SKID_SLIP_THRESHOLD) * antiskid_effective } else { 0.0 };
        if release_target > self.antiskid_release {
            self.antiskid_release += (ANTISKID_RATE_PER_S * dt).min(release_target - self.antiskid_release);
        } else {
            self.antiskid_release -= (ANTISKID_RATE_PER_S * dt).min(self.antiskid_release - release_target);
        }
        self.antiskid_release = self.antiskid_release.clamp(0.0, 1.0);

        let commanded = inputs.commanded.clamp(0.0, 1.0);
        let applied_fraction = (commanded * (1.0 - self.antiskid_release) + faults.dragging.clamp(0.0, 1.0)).clamp(0.0, 1.0);
        // The brake always resists the wheel's own rotation (a positive
        // magnitude opposing `wheel_speed_ms`, which never goes negative).
        let brake_resistance_n = applied_fraction * max_brake_force_n();

        // Wheel rotational dynamics: a friction complementarity solve
        // (module doc). `dt <= 0` (paused) leaves the wheel's state
        // untouched, satisfying "no change at rest/dt=0".
        if dt > 0.0 {
            let f_avail = if inputs.on_ground { MU_TIRE_GROUND * inputs.normal_load_n.max(0.0) } else { 0.0 };
            let desired_accel_force = M_EFF_KG * (gs - self.wheel_speed_ms) / dt;
            let f_req = desired_accel_force + brake_resistance_n;
            let f_friction = f_req.clamp(-f_avail, f_avail);
            let mut net_force = f_friction - brake_resistance_n;
            if !inputs.on_ground {
                net_force -= BEARING_DRAG_DECEL_MS2 * M_EFF_KG * self.wheel_speed_ms.signum();
            }
            self.wheel_speed_ms += net_force / M_EFF_KG * dt;
            self.wheel_speed_ms = self.wheel_speed_ms.max(0.0);
        }

        // Heat generation: brake friction power = resisting force x the
        // wheel's *own* surface speed (module doc: the disc/stator
        // interface, not the ground-slip speed).
        let heat_power_w = brake_resistance_n * self.wheel_speed_ms.max(0.0);

        let convection_w_m2k = CONVECTION_STATIC_W_M2K + (CONVECTION_ROLLING_W_M2K - CONVECTION_STATIC_W_M2K) * (gs / CONVECTION_FULL_SPEED_MS).min(1.0);
        let temp_k = self.stack_temp_c + 273.15;
        let ambient_k = inputs.ambient_c + 273.15;
        let radiative_w = EMISSIVITY * STEFAN_BOLTZMANN * SURFACE_AREA_M2 * (temp_k.powi(4) - ambient_k.powi(4));
        let convective_w = convection_w_m2k * SURFACE_AREA_M2 * (self.stack_temp_c - inputs.ambient_c);
        let net_w = heat_power_w - convective_w - radiative_w;
        self.stack_temp_c += net_w * dt / STACK_CAPACITY_J_K;

        // Wear: cumulative friction energy, budgeted.
        self.wear_fraction = (self.wear_fraction + heat_power_w * dt / WEAR_LIFE_ENERGY_J).min(1.0);

        if self.stack_temp_c > FIRE_TEMP_C {
            self.seconds_above_fire_temp += dt;
            if self.seconds_above_fire_temp > FIRE_ARM_SECONDS {
                self.fire = true;
            }
        } else {
            self.seconds_above_fire_temp = 0.0;
        }

        BrakeWheelOutputs { stack_temp_c: self.stack_temp_c, applied_fraction, wear_fraction: self.wear_fraction, fire: self.fire, skidding, wheel_speed_ms: self.wheel_speed_ms }
    }
}

/// The parking brake's hydraulic accumulator: a nitrogen-precharged piston
/// that holds brake pressure without a running pump. Boyle's law
/// (isothermal, slow discharge), the same form `physics::tyre.rs` already
/// uses for a sealed gas volume.
#[derive(Clone, Copy, Debug, Default)]
pub struct ParkingBrakeFaults {
    /// External/internal leak past the piston seal or the brake circuit.
    pub leak: f64,
}

/// Precharge pressure, Pa (GENERIC, sized above typical minimum holding
/// pressure with margin).
const PRECHARGE_PA: f64 = 10.0e6;
/// Gas volume at full precharge (accumulator empty of fluid), m^3
/// (GENERIC, order-of-magnitude for a large-transport parking brake
/// accumulator).
const GAS_VOLUME_M3: f64 = 0.0015;
/// Fluid volume consumed applying the parking brake once, m^3 (GENERIC).
const CHARGE_VOLUME_M3: f64 = 0.0006;
/// Leak rate at fault magnitude 1.0, m^3/s (GENERIC: bleeds the whole
/// accumulator down over a few hours).
const LEAK_RATE_M3_S_AT_FULL_MAGNITUDE: f64 = CHARGE_VOLUME_M3 / (3.0 * 3600.0);
/// Below this pressure the parking brake can no longer hold the aircraft
/// (GENERIC minimum holding pressure).
const MIN_HOLDING_PA: f64 = 4.0e6;

pub struct ParkingBrakeAccumulator {
    discharged_m3: f64,
    applied: bool,
}

impl ParkingBrakeAccumulator {
    pub fn new() -> Self {
        Self { discharged_m3: 0.0, applied: false }
    }

    pub fn pressure_pa(&self) -> f64 {
        PRECHARGE_PA * GAS_VOLUME_M3 / (GAS_VOLUME_M3 + self.discharged_m3)
    }

    /// `holding` is true iff the accumulator is applied and still above its
    /// minimum holding pressure.
    pub fn step(&mut self, parking_brake_set: bool, faults: &ParkingBrakeFaults, dt_s: f64) -> (f64, bool) {
        let dt = dt_s.max(0.0);
        if parking_brake_set && !self.applied {
            self.discharged_m3 += CHARGE_VOLUME_M3;
        }
        self.applied = parking_brake_set;
        if !parking_brake_set {
            // Fluid returns to the reservoir when released; only leaks
            // while actually holding pressure against the brakes.
            self.discharged_m3 = (self.discharged_m3 - CHARGE_VOLUME_M3).max(0.0);
        } else {
            self.discharged_m3 += LEAK_RATE_M3_S_AT_FULL_MAGNITUDE * faults.leak.clamp(0.0, 1.0) * dt;
        }
        self.discharged_m3 = self.discharged_m3.max(0.0);
        let pressure = self.pressure_pa();
        let holding = parking_brake_set && pressure >= MIN_HOLDING_PA;
        (pressure, holding)
    }
}

impl Default for ParkingBrakeAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy() -> BrakeFaults {
        BrakeFaults::default()
    }

    #[test]
    fn braking_while_rolling_heats_the_stack_above_ambient() {
        let mut w = BrakeWheel::new(15.0);
        // Normal load generous enough that available friction (mu*N)
        // comfortably exceeds the per-wheel max brake force, so the wheel
        // rolls without slip (the "braking while ROLLING" case).
        let inputs = BrakeWheelInputs { commanded: 1.0, on_ground: true, normal_load_n: 200_000.0, groundspeed_ms: 50.0, ambient_c: 15.0, dt_s: 1.0 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..120 {
            out = w.step(&inputs, &healthy());
        }
        assert!((out.wheel_speed_ms - 50.0).abs() < 1.0, "with ample friction the wheel must track groundspeed, not skid: {}", out.wheel_speed_ms);
        assert!(out.stack_temp_c > 100.0, "sustained hard braking must heat the stack well above ambient: {}", out.stack_temp_c);
        assert!(out.wear_fraction > 0.0);
    }

    #[test]
    fn no_command_means_no_heating_and_no_wear() {
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 0.0, on_ground: true, normal_load_n: 200_000.0, groundspeed_ms: 50.0, ambient_c: 15.0, dt_s: 1.0 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..60 {
            out = w.step(&inputs, &healthy());
        }
        assert!((out.stack_temp_c - 15.0).abs() < 1.0);
        assert_eq!(out.wear_fraction, 0.0);
    }

    #[test]
    fn insufficient_friction_produces_a_genuine_physically_caused_lockup() {
        // A light normal load (little friction available) under full
        // braking with no antiskid protection: the brake torque overpowers
        // the tyre-ground friction limit outright, so the wheel is driven
        // to (and held at) zero -- a real lockup emerging from the force
        // balance, not an input telling it to skid.
        let mut w = BrakeWheel::new(15.0);
        let faults = BrakeFaults { antiskid_inop: 1.0, dragging: 0.0 };
        let inputs = BrakeWheelInputs { commanded: 1.0, on_ground: true, normal_load_n: 50_000.0, groundspeed_ms: 60.0, ambient_c: 15.0, dt_s: 0.1 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..50 {
            out = w.step(&inputs, &faults);
        }
        assert!(out.wheel_speed_ms < 1.0, "brake torque past the friction limit must lock the wheel: {}", out.wheel_speed_ms);
        assert!(out.skidding);
        assert!((out.applied_fraction - 1.0).abs() < 1e-9, "an inoperative antiskid channel leaves full pressure applied even while skidding");
    }

    #[test]
    fn a_healthy_antiskid_channel_keeps_releasing_and_recovering_instead_of_a_permanent_lockup() {
        // Same light-friction, full-brake scenario as the lockup test
        // above, but with antiskid healthy: it must intervene at some
        // point (this scenario cycles -- release, the wheel catches back
        // up, brake reapplies, it skids again -- so check "ever happened
        // over the run", not the state of one arbitrary final tick).
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 1.0, on_ground: true, normal_load_n: 50_000.0, groundspeed_ms: 60.0, ambient_c: 15.0, dt_s: 0.1 };
        let mut ever_skidded = false;
        let mut min_applied_fraction = 1.0_f64;
        let mut max_wheel_speed = 0.0_f64;
        for _ in 0..500 {
            let out = w.step(&inputs, &healthy());
            ever_skidded |= out.skidding;
            min_applied_fraction = min_applied_fraction.min(out.applied_fraction);
            max_wheel_speed = max_wheel_speed.max(out.wheel_speed_ms);
        }
        assert!(ever_skidded, "the wheel must skid at least once under this brake/friction mismatch");
        assert!(min_applied_fraction < 1.0, "a healthy antiskid channel must release pressure at some point");
        assert!(max_wheel_speed > 30.0, "unlike the disabled-antiskid case, the wheel must recover partway back toward groundspeed at some point");
    }

    #[test]
    fn a_wheel_keeps_spinning_for_a_while_after_liftoff_instead_of_stopping_instantly() {
        let mut w = BrakeWheel::new(15.0);
        // Spin it up to speed on the ground first.
        let ground_inputs = BrakeWheelInputs { commanded: 0.0, on_ground: true, normal_load_n: 200_000.0, groundspeed_ms: 70.0, ambient_c: 15.0, dt_s: 0.5 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..10 {
            out = w.step(&ground_inputs, &healthy());
        }
        assert!((out.wheel_speed_ms - 70.0).abs() < 1.0);

        // Liftoff: no ground contact any more.
        let air_inputs = BrakeWheelInputs { commanded: 0.0, on_ground: false, normal_load_n: 0.0, groundspeed_ms: 90.0, ambient_c: 15.0, dt_s: 0.5 };
        let just_after = w.step(&air_inputs, &healthy());
        assert!(just_after.wheel_speed_ms > 60.0, "a wheel must not stop instantly the moment it leaves the ground: {}", just_after.wheel_speed_ms);

        let mut later = just_after;
        for _ in 0..400 {
            later = w.step(&air_inputs, &healthy());
        }
        assert!(later.wheel_speed_ms < just_after.wheel_speed_ms, "residual bearing drag must eventually slow a free-spinning wheel");
    }

    #[test]
    fn dragging_brake_heats_the_stack_with_zero_command() {
        let mut w = BrakeWheel::new(15.0);
        let faults = BrakeFaults { antiskid_inop: 0.0, dragging: 0.5 };
        let inputs = BrakeWheelInputs { commanded: 0.0, on_ground: true, normal_load_n: 100_000.0, groundspeed_ms: 30.0, ambient_c: 15.0, dt_s: 1.0 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..120 {
            out = w.step(&inputs, &faults);
        }
        assert!(out.stack_temp_c > 20.0, "a dragging brake heats up even with no pedal command: {}", out.stack_temp_c);
    }

    #[test]
    fn sustained_extreme_heat_eventually_starts_a_brake_fire() {
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 1.0, on_ground: true, normal_load_n: 150_000.0, groundspeed_ms: 80.0, ambient_c: 15.0, dt_s: 5.0 };
        let mut out = BrakeWheelOutputs::default();
        for _ in 0..400 {
            out = w.step(&inputs, &healthy());
            if out.fire {
                break;
            }
        }
        assert!(out.fire, "sustained extreme heat must eventually start a brake fire; reached {} C", out.stack_temp_c);
    }

    #[test]
    fn numerically_safe_at_rest() {
        let mut w = BrakeWheel::new(15.0);
        let inputs = BrakeWheelInputs { commanded: 0.0, on_ground: false, normal_load_n: 0.0, groundspeed_ms: 0.0, ambient_c: 15.0, dt_s: 0.0 };
        let out = w.step(&inputs, &healthy());
        assert!(out.stack_temp_c.is_finite());
        assert!(!out.applied_fraction.is_nan());
        assert!(out.wheel_speed_ms.is_finite());
    }

    #[test]
    fn the_parking_brake_holds_when_healthy_and_bleeds_down_when_leaking() {
        let mut acc = ParkingBrakeAccumulator::new();
        let (p0, holding0) = acc.step(true, &ParkingBrakeFaults::default(), 1.0);
        assert!(holding0, "a freshly applied, healthy accumulator must hold");
        assert!(p0 < PRECHARGE_PA, "applying it must have discharged some fluid, dropping pressure from the bare precharge");

        let mut leaking = ParkingBrakeAccumulator::new();
        leaking.step(true, &ParkingBrakeFaults::default(), 1.0);
        let faults = ParkingBrakeFaults { leak: 1.0 };
        let mut holding = true;
        let mut pressure = PRECHARGE_PA;
        // The leak rate is sized to bleed the accumulator down over a few
        // hours (module doc); step in 1-minute increments for long enough
        // (700 x 60 s = 11.7 h) to comfortably cross the holding minimum.
        for _ in 0..700 {
            let (p, h) = leaking.step(true, &faults, 60.0);
            pressure = p;
            holding = h;
            if !holding {
                break;
            }
        }
        assert!(!holding, "a full-magnitude leak must eventually bleed the accumulator below its holding minimum; ended at {pressure} Pa");
    }

    #[test]
    fn releasing_the_parking_brake_stops_it_reporting_as_holding() {
        let mut acc = ParkingBrakeAccumulator::new();
        acc.step(true, &ParkingBrakeFaults::default(), 1.0);
        let (_, holding) = acc.step(false, &ParkingBrakeFaults::default(), 1.0);
        assert!(!holding);
    }
}

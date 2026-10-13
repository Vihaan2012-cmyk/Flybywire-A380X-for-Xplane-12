//! Generic servo-hydraulic linear actuator ("power control unit", PCU) that
//! drives one control surface's hinge through a crank arm, the way a real
//! aileron/elevator/rudder/spoiler ram does and the way FlyByWire's own
//! `LinearActuator` does (fbw-common/src/wasm/systems/systems/src/hydraulic/
//! linear_actuator.rs), reduced here to the physics this crate needs.
//!
//! A servo valve meters hydraulic (or, for an EHA/EBHA, electrically pumped)
//! flow to a piston. Two real physical limits fall straight out of the
//! geometry: piston force is bounded by supply pressure times piston area
//! (F = P*A), and piston rate is bounded by the valve's rated flow divided
//! by that same area (v = Q/A); the valve itself is an orifice, so at a
//! fraction f of rated supply pressure the flow (and hence rate) available
//! scales with sqrt(f), not f (Q ~ Cd*A*sqrt(2*deltaP/rho), the standard
//! orifice equation used the same way in `physics::engine::oil`). The crank
//! arm turning piston stroke into hinge rotation converts both into a torque
//! and an angular rate limit.
//!
//! Three functional modes, matching FlyByWire's own `LinearActuatorMode`
//! (linear_actuator.rs:55, doc comment at 363-373):
//! - `Active`: a position servo (outer position loop -> rate demand -> inner
//!   rate loop -> torque, saturated at the actuator's own force limit).
//! - `Damping`: the valves resist the piece's own motion (used when a PCU
//!   is depowered/idle but still hydraulically connected, so 2-out-of-3
//!   actuators on a surface don't fight the one that is `Active`).
//! - `Standby`: both ports closed; trapped, near-incompressible fluid acts
//!   as a very stiff spring/damper holding whatever position the mode was
//!   entered at (FlyByWire's `ClosedValves`).
//!
//! Piston bore/rod diameters and rated flows below are cited from FlyByWire's
//! A380 actuator constructions where its source comments give real values;
//! crank arms are cited where the same file's body geometry gives them,
//! GENERIC (a representative large-transport horn length) otherwise, since
//! the actual 3-D attachment point is not published for every surface.

use std::f64::consts::PI;

/// psi -> Pa.
pub const PSI_PA: f64 = 6894.757;
/// US liquid gallon -> m^3 (NIST).
pub const GALLON_M3: f64 = 0.003785411784;

/// A380 green/yellow hydraulic system regulated pressure
/// (`A380HydraulicCircuitFactory::HYDRAULIC_TARGET_PRESSURE_PSI`,
/// a380_systems/src/hydraulic/mod.rs:214).
pub const HYDRAULIC_SUPPLY_PSI: f64 = 5250.0;
pub const HYDRAULIC_SUPPLY_PA: f64 = HYDRAULIC_SUPPLY_PSI * PSI_PA;

/// One PCU's fixed geometry: piston areas, rated valve flow and the crank
/// arm from hinge to actuator attachment.
#[derive(Clone, Copy, Debug)]
pub struct ActuatorGeometry {
    pub bore_area_m2: f64,
    /// 0 for a symmetric (double rod, e.g. aileron/elevator/rudder) ram.
    pub rod_area_m2: f64,
    pub max_flow_m3_s: f64,
    pub arm_m: f64,
}

impl ActuatorGeometry {
    pub fn new(bore_diameter_m: f64, rod_diameter_m: f64, max_flow_gal_s: f64, arm_m: f64) -> Self {
        let area = |d: f64| PI * (d * 0.5).powi(2);
        Self {
            bore_area_m2: area(bore_diameter_m),
            rod_area_m2: area(rod_diameter_m),
            max_flow_m3_s: max_flow_gal_s * GALLON_M3,
            arm_m: arm_m.max(1e-3),
        }
    }

    /// Aileron PCU: 0.07 m bore, symmetric ram (13500 daN @ 350 bar nominal
    /// gives this bore, 0.0825 US gal/s rated flow at the 81 mm/s rated
    /// travel speed; a380_systems/src/hydraulic/mod.rs:402-420). Crank arm
    /// 0.119 m is the aileron body's own `control_arm` y-offset (mod.rs:464).
    pub fn aileron() -> Self {
        Self::new(0.07, 0.0, 0.0825, 0.119)
    }

    /// Elevator PCU: 0.08 m bore, symmetric ram, 0.15 US gal/s
    /// (mod.rs:744-758). Crank arm GENERIC (not published for this surface):
    /// 0.15 m, a representative horn length for a surface this size.
    pub fn elevator() -> Self {
        Self::new(0.08, 0.0, 0.15, 0.15)
    }

    /// Rudder PCU: piston area 77.18 cm^2 (mod.rs:928), giving a 0.0991 m
    /// bore; 0.25 US gal/s rated flow (mod.rs:923; FBW's own comment there
    /// notes the 236.5 mm/s rated speed implies 0.4822 gal/s for its real
    /// piston area, so this is the more conservative of the two figures in
    /// that source). Crank arm GENERIC: 0.18 m.
    pub fn rudder() -> Self {
        let bore = 2.0 * (77.18e-4 / PI).sqrt();
        Self::new(bore, 0.0, 0.25, 0.18)
    }

    /// Spoiler PCU: 0.09 m bore, 0.05 m rod (asymmetric ram), 0.23 US gal/s
    /// (mod.rs:584-594). Crank arm derived from the spoiler body's own
    /// `control_arm`/`anchor` offsets, 0.10 and 0.26 of its 0.685 m size
    /// (mod.rs:624-629): `0.685 * hypot(0.10, 0.26)` = 0.191 m.
    pub fn spoiler() -> Self {
        let arm = 0.685 * 0.10_f64.hypot(0.26);
        Self::new(0.09, 0.05, 0.23, arm)
    }

    pub fn max_force_n(&self, pressure_pa: f64) -> f64 {
        pressure_pa.max(0.0) * self.bore_area_m2
    }

    pub fn max_torque_nm(&self, pressure_pa: f64) -> f64 {
        self.max_force_n(pressure_pa) * self.arm_m
    }

    /// Piston (hence hinge) rate available at `pressure_fraction` (0..1) of
    /// [`HYDRAULIC_SUPPLY_PA`], via the orifice sqrt(pressure) law.
    pub fn rate_limit_rad_s(&self, pressure_fraction: f64) -> f64 {
        let f = pressure_fraction.max(0.0).sqrt();
        (self.max_flow_m3_s * f / self.bore_area_m2) / self.arm_m
    }
}

/// Which power source an actuator draws from, matching FlyByWire's
/// `ElectroHydrostaticActuatorType` (linear_actuator.rs:268-272).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorPower {
    /// A conventional servo-hydraulic PCU fed only from a central
    /// (green/yellow) hydraulic circuit.
    Hydraulic,
    /// Electro-Hydrostatic Actuator: its own motor-pump only, never the
    /// central hydraulics.
    ElectroHydrostatic,
    /// Electrical Backup Hydraulic Actuator: normally fed from a central
    /// circuit, falls back to its own motor-pump when that supply is lost.
    ElectricalBackupHydraulic,
}

/// Functional mode, matching FlyByWire's `LinearActuatorMode` (see module
/// doc comment).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ActuatorMode {
    #[default]
    Standby,
    Active,
    Damping,
}

/// Faults one PCU can carry, each a fraction 0 (healthy) .. 1 (fully
/// failed) unless noted.
#[derive(Clone, Copy, Debug, Default)]
pub struct ActuatorFaults {
    /// Loss of hydraulic supply (or, for an EHA/EBHA on electric power, loss
    /// of its motor-pump output).
    pub supply_loss: f64,
    /// Mechanical seizure: at 1.0 the mechanism is pinned at the angle it
    /// jammed at, resistible only by a large enough external torque.
    pub jam: f64,
    /// Servo valve hardover: at 1.0 the valve drives full-rate in
    /// `runaway_sign`'s direction regardless of the commanded position.
    pub runaway: f64,
    /// Direction of the hardover; only its sign matters.
    pub runaway_sign: f64,
    /// The position transducer feeding the actuator's own position loop is
    /// stuck: it keeps reporting whatever angle it saw the instant this
    /// went true.
    pub transducer_frozen: bool,
    /// A transducer that reads a fixed offset high/low rather than freezing.
    pub transducer_bias_rad: f64,
    /// Servo valve internal (null-position spool) leakage: continuous flow
    /// bypasses from pressure to return without doing work on the piston.
    /// 0 healthy .. 1 fully worn. Bypassed flow comes straight out of the
    /// flow otherwise available to move the piston (it derates rate more
    /// than force, since force only needs *pressure*, which the leak path
    /// barely loads) and, more importantly, it means a "closed" valve never
    /// truly traps the piston: standby stiffness (the trapped-fluid spring)
    /// falls, so the surface creeps/droops under a sustained external load
    /// between position-loop corrections.
    pub valve_leakage: f64,
    /// Piston seal wear: continuous flow bypasses across the piston head
    /// itself (bore side to rod side), independent of the servo valve.
    /// 0 healthy .. 1 fully worn. This derates force more than rate (some
    /// of the delivered flow recirculates internally instead of displacing
    /// the piston against load) and, like valve leakage, softens the
    /// actuator's holding stiffness.
    pub piston_seal_wear: f64,
}

/// The torque law one PCU presents to the body it drives, in the one form
/// all three modes share: **a clamped affine function of that body's rate**,
///
/// `S(r) = clamp(open_torque_nm - damping_nm_s_per_rad * r, +-max_torque_nm)`
///
/// (`Active`: the inner rate loop, `open = k * rate_cmd`; `Damping`: the
/// damping orifices, `open = 0`; `Standby`: the trapped fluid's spring and
/// damper, `open` = the spring term). Callers need the law, not just its
/// value at the current rate, because `damping_nm_s_per_rad` is enormous --
/// `c/I` runs from 10^3 to 10^4 s^-1, while an explicit (or semi-implicit)
/// Euler step on a damper is only stable while `c*dt/I < 2`, which no
/// affordable sub-step satisfies. Handing the law to [`servo_rate_step`]
/// solves the step exactly instead, clamp included, at any `dt`.
#[derive(Clone, Copy, Debug, Default)]
pub struct ServoLoad {
    pub open_torque_nm: f64,
    pub damping_nm_s_per_rad: f64,
    pub max_torque_nm: f64,
}

impl ServoLoad {
    /// No servo at all (a free body, or one whose linkage has sheared).
    pub const NONE: Self = Self { open_torque_nm: 0.0, damping_nm_s_per_rad: 0.0, max_torque_nm: 0.0 };

    /// Several actuators on one body, lumped into a single clamped affine
    /// law. Inside the linear band the sum is exact and says the physically
    /// right thing about actuators that disagree: they fight each other and
    /// settle at the gradient-weighted mean of their rate demands,
    /// `sum(c_i r_i) / sum(c_i)`, which is what parallel servos on a common
    /// shaft actually do. Outside it the lumped clamp is an approximation,
    /// exact whenever they saturate together -- the case that matters, since
    /// actuators on a common surface share a rate command. An actuator with
    /// no torque ceiling left contributes nothing at all (its law is
    /// `clamp(.., +-0) == 0`), so it gets no vote; `PowerControlUnit::step`
    /// reports [`ServoLoad::NONE`] for it.
    pub fn add(&mut self, other: &Self) {
        self.open_torque_nm += other.open_torque_nm;
        self.damping_nm_s_per_rad += other.damping_nm_s_per_rad;
        self.max_torque_nm += other.max_torque_nm;
    }

    /// The same law seen through a gear ratio `g` (motor shaft rate =
    /// `g` * output rate, output torque = `g` * motor torque), so the
    /// intercept and the ceiling scale with `g` and the rate gradient with
    /// `g^2`. Used by `ths.rs`'s speed-summing differential.
    pub fn geared(&self, g: f64) -> Self {
        Self {
            open_torque_nm: g * self.open_torque_nm,
            damping_nm_s_per_rad: g * g * self.damping_nm_s_per_rad,
            max_torque_nm: g.abs() * self.max_torque_nm,
        }
    }

    /// This law's value at a given body rate.
    pub fn torque_at(&self, rate_rad_s: f64) -> f64 {
        (self.open_torque_nm - self.damping_nm_s_per_rad * rate_rad_s)
            .clamp(-self.max_torque_nm.max(0.0), self.max_torque_nm.max(0.0))
    }
}

/// What one `step` gives back: the torque the PCU is applying about the
/// hinge this tick, its current torque ceiling (for surface.rs's blow-back
/// bookkeeping), whether it is at that ceiling, and the pieces its caller's
/// integrator needs (see [`ServoLoad`] and [`servo_rate_step`]).
#[derive(Clone, Copy, Debug, Default)]
pub struct ActuatorOutput {
    pub torque_nm: f64,
    pub max_torque_nm: f64,
    pub saturated: bool,
    /// The servo's own clamped affine torque law.
    pub servo: ServoLoad,
    /// The seized mechanism's own spring/damper torque at the rate passed
    /// in. It is additive to the servo's and never clamped, so it is
    /// reported separately rather than folded into `servo`.
    pub jam_torque_nm: f64,
    /// `-d(jam_torque_nm)/d(rate)`, for the caller's implicit step.
    pub jam_damping_nm_s_per_rad: f64,
}

/// One body-rate update, solving
///
/// `I*(r' - r)/dt = S(r') + T_other(r') `,
/// `S(r) = clamp(A - c_s*r, +-T_max)`, `T_other(r) = B - c_o*r`
///
/// **exactly** for `r'` -- backward Euler in every velocity-proportional
/// term, clamp and all, and therefore stable at any `dt`. That matters for
/// more than stability: a rate-limited servo's linear band is only
/// `2*T_max/c_s` wide (for the THS, 0.0075 rad/s), narrower than the rate
/// change a single sub-step at the torque ceiling produces, so an explicit
/// step jumps clean across the band and chatters between the two clamps
/// instead of settling on the commanded rate. Solving the clamp gives the
/// physically right answer directly: accelerate at the torque ceiling,
/// stopping at the commanded rate.
///
/// `other_torque_nm` is `T_other` already evaluated at the current rate (the
/// form a caller naturally has) and `other_damping_nm_s_per_rad` is `c_o`.
/// A *negative* `c_o` -- a genuinely destabilising aerodynamic rate term,
/// see `surface::SurfaceDamping` -- is deliberately left explicit, because
/// divergence there is physics, not a numerical artefact.
///
/// The solution is a two-branch case split, valid because the right-hand
/// side is monotonically non-increasing in `r'`: try the linear branch, and
/// if the servo torque it implies lies outside the clamp, redo it with the
/// servo pinned at whichever bound it exceeded.
pub fn servo_rate_step(
    rate_rad_s: f64,
    servo: &ServoLoad,
    other_torque_nm: f64,
    other_damping_nm_s_per_rad: f64,
    inertia_kg_m2: f64,
    dt_s: f64,
) -> f64 {
    let dt = dt_s.max(0.0);
    if dt <= 0.0 {
        return rate_rad_s;
    }
    let i_over_dt = inertia_kg_m2.max(1e-9) / dt;
    let c_other = other_damping_nm_s_per_rad.max(0.0);
    let c_servo = servo.damping_nm_s_per_rad.max(0.0);
    let t_max = servo.max_torque_nm.max(0.0);
    // Re-reference the implicitly-treated part of `T_other` to zero rate;
    // anything left (including a negative `c_o`) stays as evaluated.
    let b = other_torque_nm + c_other * rate_rad_s;

    let linear = (i_over_dt * rate_rad_s + b + servo.open_torque_nm) / (i_over_dt + c_other + c_servo);
    let servo_torque = servo.open_torque_nm - c_servo * linear;
    if servo_torque.abs() <= t_max {
        return linear;
    }
    let pinned = if servo_torque > 0.0 { t_max } else { -t_max };
    (i_over_dt * rate_rad_s + b + pinned) / (i_over_dt + c_other)
}

/// One servo-hydraulic (or EHA/EBHA) power control unit.
pub struct PowerControlUnit {
    pub geometry: ActuatorGeometry,
    kp_rate_per_rad: f64,
    k_torque_per_rate: f64,
    k_damping: f64,
    k_standby_spring: f64,
    k_standby_damp: f64,
    k_jam_spring: f64,
    k_jam_damp: f64,
    prev_mode: ActuatorMode,
    standby_angle_rad: f64,
    jam_angle_rad: Option<f64>,
    frozen_transducer_rad: Option<f64>,
}

impl PowerControlUnit {
    pub fn new(geometry: ActuatorGeometry) -> Self {
        let max_t = geometry.max_torque_nm(HYDRAULIC_SUPPLY_PA).max(1.0);
        let rated_rate = geometry.rate_limit_rad_s(1.0).max(1e-3);
        Self {
            geometry,
            // GENERIC servo-loop gains: an outer position loop commanding a
            // rate (saturating at the rated rate well inside a few degrees
            // of error) feeding an inner rate loop stiff enough to reach the
            // torque ceiling well within the rated rate's error band. This
            // is the standard cascaded position/rate/force PCU architecture
            // (e.g. Roskam, "Airplane Flight Dynamics and Automatic Flight
            // Controls" Part II, ch. 4 on hydraulic servo actuators), not a
            // specific FlyByWire or Airbus number.
            kp_rate_per_rad: 8.0,
            k_torque_per_rate: max_t / rated_rate * 4.0,
            k_damping: max_t / rated_rate,
            k_standby_spring: max_t * 200.0,
            k_standby_damp: max_t * 20.0,
            k_jam_spring: max_t * 50.0,
            k_jam_damp: max_t * 5.0,
            prev_mode: ActuatorMode::Standby,
            standby_angle_rad: 0.0,
            jam_angle_rad: None,
            frozen_transducer_rad: None,
        }
    }

    /// `pressure_fraction`: 0..1 of [`HYDRAULIC_SUPPLY_PA`] (or, for an
    /// electric supply, of an equivalent EHA/EBHA pump pressure) actually
    /// reaching this actuator, from the hydraulic/electrical system model.
    pub fn step(
        &mut self,
        mode: ActuatorMode,
        commanded_angle_rad: f64,
        angle_rad: f64,
        rate_rad_s: f64,
        pressure_fraction: f64,
        faults: &ActuatorFaults,
    ) -> ActuatorOutput {
        let supply = pressure_fraction.clamp(0.0, 1.0) * (1.0 - faults.supply_loss.clamp(0.0, 1.0));
        let jam = faults.jam.clamp(0.0, 1.0);
        let leak = faults.valve_leakage.clamp(0.0, 1.0);
        let wear = faults.piston_seal_wear.clamp(0.0, 1.0);
        // GENERIC derates: valve leakage bypasses flow (so it costs rate
        // more than force), seal wear bypasses pressure across the piston
        // (so it costs force more than rate). Neither reaches zero on its
        // own at full magnitude, since some residual capability survives
        // even a badly worn actuator; a jam still overrides everything.
        let rate_derate = (1.0 - 0.6 * leak - 0.2 * wear).clamp(0.05, 1.0);
        let force_derate = (1.0 - 0.5 * wear - 0.2 * leak).clamp(0.05, 1.0);
        let max_torque = self.geometry.max_torque_nm(HYDRAULIC_SUPPLY_PA * supply) * force_derate * (1.0 - jam);
        let rate_limit = self.geometry.rate_limit_rad_s(supply) * rate_derate;
        // A leaking valve or a worn seal both mean the "trapped fluid" a
        // closed/holding actuator relies on isn't fully trapped: the whole
        // servo loop (not just its force ceiling) gets softer, so a
        // sustained external load produces steady-state droop instead of
        // being held at zero error. Applied to every mode's gain, not only
        // Standby's spring, since Active mode's own inner loop is what a
        // real degraded PCU also loses stiffness in.
        let stiffness_factor = (rate_derate * force_derate).clamp(0.05, 1.0);

        if mode == ActuatorMode::Standby && self.prev_mode != ActuatorMode::Standby {
            self.standby_angle_rad = angle_rad;
        }
        self.prev_mode = mode;

        if jam > 0.0 {
            if self.jam_angle_rad.is_none() {
                self.jam_angle_rad = Some(angle_rad);
            }
        } else {
            self.jam_angle_rad = None;
        }

        let feedback = if faults.transducer_frozen {
            *self.frozen_transducer_rad.get_or_insert(angle_rad)
        } else {
            self.frozen_transducer_rad = None;
            angle_rad + faults.transducer_bias_rad
        };

        // Every mode's torque law is `clamp(open - damping*rate, +-max)`;
        // only the intercept and the gradient differ (see `ServoLoad`).
        let servo = match mode {
            ActuatorMode::Active => {
                let normal_rate_cmd =
                    (self.kp_rate_per_rad * (commanded_angle_rad - feedback)).clamp(-rate_limit, rate_limit);
                let r = faults.runaway.clamp(0.0, 1.0);
                let rate_cmd = if r > 0.0 {
                    let sign = if faults.runaway_sign >= 0.0 { 1.0 } else { -1.0 };
                    (1.0 - r) * normal_rate_cmd + r * sign * rate_limit
                } else {
                    normal_rate_cmd
                };
                let k = stiffness_factor * self.k_torque_per_rate;
                ServoLoad { open_torque_nm: k * rate_cmd, damping_nm_s_per_rad: k, max_torque_nm: max_torque }
            }
            ActuatorMode::Damping => ServoLoad {
                open_torque_nm: 0.0,
                damping_nm_s_per_rad: stiffness_factor * self.k_damping,
                max_torque_nm: max_torque,
            },
            ActuatorMode::Standby => ServoLoad {
                open_torque_nm: -stiffness_factor * self.k_standby_spring * (angle_rad - self.standby_angle_rad),
                damping_nm_s_per_rad: stiffness_factor * self.k_standby_damp,
                max_torque_nm: max_torque,
            },
        };
        // With no torque ceiling left -- no supply, or a total jam -- the law
        // `clamp(.., +-0)` is identically zero: such an actuator exerts
        // nothing and must not be given a share of a lumped `ServoLoad`
        // either (see `ServoLoad::add`).
        let servo = if max_torque <= 0.0 { ServoLoad::NONE } else { servo };
        let unclamped = servo.open_torque_nm - servo.damping_nm_s_per_rad * rate_rad_s;
        let saturated = unclamped.abs() > max_torque;
        let mut torque = servo.torque_at(rate_rad_s);

        // The seized mechanism's own resistance is additive to whatever the
        // (now authority-reduced) servo can still do, so a partial jam
        // fights the servo rather than simply capping it. It is never
        // clamped, so the caller's integrator is handed it separately.
        let mut jam_torque = 0.0;
        let mut jam_damping = 0.0;
        if let Some(seize_angle) = self.jam_angle_rad {
            jam_damping = self.k_jam_damp * jam;
            jam_torque = -self.k_jam_spring * jam * (angle_rad - seize_angle) - jam_damping * rate_rad_s;
            torque += jam_torque;
        }

        ActuatorOutput {
            torque_nm: torque,
            max_torque_nm: max_torque,
            saturated,
            servo,
            jam_torque_nm: jam_torque,
            jam_damping_nm_s_per_rad: jam_damping,
        }
    }
}

/// Faults an EHA/EBHA's own electric motor-pump can carry.
#[derive(Clone, Copy, Debug, Default)]
pub struct ElectricPumpFaults {
    /// 0 healthy .. 1 dead motor/pump.
    pub motor_failure: f64,
}

/// The electric motor and small local hydraulic pump behind an EHA or EBHA,
/// spinning up/down with a first-order lag (GENERIC time constant,
/// representative of a several-kW variable-speed pump motor) rather than
/// instantaneously, so the pressure it can deliver ramps rather than steps
/// when the bus comes up or a failure occurs -- the same exact-exponential
/// convention used by `physics::engine::oil`'s chamber temperatures.
pub struct ElectricMotorPump {
    speed_frac: f64,
    time_constant_s: f64,
}

impl ElectricMotorPump {
    pub fn new(time_constant_s: f64) -> Self {
        Self { speed_frac: 0.0, time_constant_s: time_constant_s.max(1e-3) }
    }

    /// `electrical_power_fraction`: 0 (bus dead) .. 1 (bus healthy). Returns
    /// the pressure fraction (of [`HYDRAULIC_SUPPLY_PA`]) now available.
    pub fn step(&mut self, electrical_power_fraction: f64, faults: &ElectricPumpFaults, dt_s: f64) -> f64 {
        let target = electrical_power_fraction.clamp(0.0, 1.0) * (1.0 - faults.motor_failure.clamp(0.0, 1.0));
        let k = 1.0 / self.time_constant_s;
        self.speed_frac = target + (self.speed_frac - target) * (-k * dt_s.max(0.0)).exp();
        self.speed_frac.clamp(0.0, 1.0)
    }

    pub fn pressure_fraction(&self) -> f64 {
        self.speed_frac
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;

    #[test]
    fn aileron_geometry_reproduces_flybywires_cited_force_and_rate() {
        let g = ActuatorGeometry::aileron();
        // 13500 daN @ 350 bar nominal (mod.rs:407); a380_systems runs the
        // circuit at 5250 psi (362 bar) so this reads a little higher.
        let force_dan = g.max_force_n(HYDRAULIC_SUPPLY_PA) / 10.0;
        assert!((force_dan - 13934.0).abs() < 50.0, "{force_dan} daN");
        // 81 mm/s rated piston speed (mod.rs:410); this actuator's
        // rate_limit_rad_s is per-radian-of-hinge, so undo the crank arm to
        // get piston speed back for the comparison.
        let piston_mm_s = g.rate_limit_rad_s(1.0) * g.arm_m * 1000.0;
        assert!((piston_mm_s - 81.0).abs() < 1.0, "{piston_mm_s} mm/s");
    }

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::elevator());
        let out = pcu.step(ActuatorMode::Active, 0.0, 0.0, 0.0, 0.0, &ActuatorFaults::default());
        assert!(out.torque_nm.is_finite() && out.max_torque_nm.is_finite());
        let mut pump = ElectricMotorPump::new(0.5);
        let p = pump.step(1.0, &ElectricPumpFaults::default(), 0.0);
        assert!(p.is_finite());
    }

    #[test]
    fn active_mode_drives_toward_the_commanded_angle() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::aileron());
        // Below commanded angle: torque should push positive (toward it).
        let out = pcu.step(ActuatorMode::Active, 0.2, 0.0, 0.0, 1.0, &ActuatorFaults::default());
        assert!(out.torque_nm > 0.0);
        assert!(out.torque_nm <= out.max_torque_nm + 1e-6);
        // Above commanded angle: torque should push negative.
        let out = pcu.step(ActuatorMode::Active, -0.2, 0.0, 0.0, 1.0, &ActuatorFaults::default());
        assert!(out.torque_nm < 0.0);
    }

    #[test]
    fn valve_leakage_and_seal_wear_cause_standby_droop_under_a_sustained_load() {
        // A leaking valve or worn seal means the "trapped fluid" standby
        // relies on isn't fully trapped: the actuator should visibly droop
        // further, in standby, under the same constant external load than
        // a healthy one -- not just lose peak force.
        const FINE_DT: f64 = 0.0005;
        let mut healthy = PowerControlUnit::new(ActuatorGeometry::elevator());
        let mut degraded = PowerControlUnit::new(ActuatorGeometry::elevator());
        let degraded_faults = ActuatorFaults { valve_leakage: 1.0, piston_seal_wear: 1.0, ..Default::default() };
        let inertia = 50.0;
        let external_torque = 2000.0; // well under either's force ceiling: stays in the linear regime
        let (mut ha, mut hr, mut da, mut dr) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
        for _ in 0..200_000 {
            let ho = healthy.step(ActuatorMode::Standby, 0.0, ha, hr, 1.0, &ActuatorFaults::default());
            hr = servo_rate_step(hr, &ho.servo, external_torque, 0.0, inertia, FINE_DT);
            ha += hr * FINE_DT;

            let deg_out = degraded.step(ActuatorMode::Standby, 0.0, da, dr, 1.0, &degraded_faults);
            dr = servo_rate_step(dr, &deg_out.servo, external_torque, 0.0, inertia, FINE_DT);
            da += dr * FINE_DT;
        }
        assert!(ha.abs() > 0.0 && da.abs() > 0.0);
        // Hand calculation. Standby is a spring of `k_standby_spring =
        // max_torque * 200`, scaled by `stiffness_factor`; at rest the droop
        // is just `external_torque / (stiffness_factor * k)`.
        //   elevator max torque = 5250 psi * pi*(0.08/2)^2 * 0.15 m
        //                       = 36.20 MPa * 5.0265e-3 m^2 * 0.15 = 27292 N*m
        //   healthy  k = 27292*200 = 5.458e6 N*m/rad, sf = 1
        //            -> droop = 2000 / 5.458e6 = 3.66e-4 rad
        //   degraded sf = rate_derate*force_derate = (1-0.6-0.2)*(1-0.5-0.2)
        //                 = 0.2*0.3 = 0.06
        //            -> droop = 2000 / (0.06*5.458e6) = 6.11e-3 rad
        // i.e. a factor 1/0.06 = 16.7, comfortably past the 5x this asserts.
        assert!((ha.abs() - 3.66e-4).abs() < 1e-5, "healthy droop {ha} should match the spring hand calculation");
        assert!((da.abs() - 6.11e-3).abs() < 1e-4, "degraded droop {da} should match the softened-spring hand calculation");
        assert!(da.abs() > ha.abs() * 5.0, "degraded droop {da} should far exceed healthy droop {ha}");
    }

    #[test]
    fn integrating_active_mode_actually_reaches_the_command() {
        // This PCU's inner rate loop is deliberately stiff (see `new`'s doc
        // comment: reaches its torque ceiling well inside the rated rate's
        // error band), which any caller must integrate at a fine enough
        // step to resolve -- exactly why `surface::ControlSurface` and
        // `high_lift::HighLiftSystem` sub-step internally. This test does
        // the same sub-stepping by hand rather than exercising the PCU at
        // an unrealistically large step no real caller would use.
        const FINE_DT: f64 = 0.0005;
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::elevator());
        let inertia = 50.0; // kg*m^2, arbitrary for this closed-loop check
        let mut angle = 0.0_f64;
        let mut rate = 0.0_f64;
        for _ in 0..200_000 {
            let out = pcu.step(ActuatorMode::Active, 0.1, angle, rate, 1.0, &ActuatorFaults::default());
            rate = servo_rate_step(rate, &out.servo, 0.0, 0.0, inertia, FINE_DT);
            angle += rate * FINE_DT;
        }
        assert!((angle - 0.1).abs() < 0.01, "settled at {angle}");
    }

    #[test]
    fn damping_mode_only_resists_motion_never_drives() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::spoiler());
        let out = pcu.step(ActuatorMode::Damping, 0.5, 0.0, 2.0, 1.0, &ActuatorFaults::default());
        assert!(out.torque_nm < 0.0); // opposes the positive rate
        let out2 = pcu.step(ActuatorMode::Damping, 0.5, 0.0, 0.0, 1.0, &ActuatorFaults::default());
        assert_eq!(out2.torque_nm, 0.0); // at rest, no drive at all
    }

    #[test]
    fn standby_mode_holds_the_angle_it_was_entered_at() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::rudder());
        // Enters standby at angle 0.3: a later request for 0 should be
        // ignored, and it should resist displacement away from 0.3.
        pcu.step(ActuatorMode::Standby, 0.0, 0.3, 0.0, 1.0, &ActuatorFaults::default());
        let out = pcu.step(ActuatorMode::Standby, 0.0, 0.4, 0.0, 1.0, &ActuatorFaults::default());
        assert!(out.torque_nm < 0.0); // pulls back toward 0.3, not 0
    }

    #[test]
    fn supply_loss_shrinks_both_force_and_rate_limit() {
        let mut healthy = PowerControlUnit::new(ActuatorGeometry::aileron());
        let mut starved = PowerControlUnit::new(ActuatorGeometry::aileron());
        let h = healthy.step(ActuatorMode::Active, 1.0, 0.0, 0.0, 1.0, &ActuatorFaults::default());
        let s = starved.step(
            ActuatorMode::Active,
            1.0,
            0.0,
            0.0,
            1.0,
            &ActuatorFaults { supply_loss: 0.9, ..Default::default() },
        );
        assert!(s.max_torque_nm < 0.2 * h.max_torque_nm);
    }

    #[test]
    fn a_full_jam_freezes_the_mechanism_against_a_moderate_load() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::elevator());
        let faults = ActuatorFaults { jam: 1.0, ..Default::default() };
        // Jams at angle 0.1; an aerodynamic-scale external torque tries to
        // push it further. The jam's own resistance should dominate a
        // torque well under its jam-spring strength for a small excursion.
        let out = pcu.step(ActuatorMode::Active, 0.5, 0.1, 0.0, 1.0, &faults);
        assert_eq!(out.max_torque_nm, 0.0); // no servo authority left
        let excursion = pcu.step(ActuatorMode::Active, 0.5, 0.11, 0.0, 1.0, &faults);
        assert!(excursion.torque_nm < 0.0); // resists the excursion
    }

    #[test]
    fn runaway_drives_full_rate_in_its_own_direction_regardless_of_command() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::rudder());
        let faults = ActuatorFaults { runaway: 1.0, runaway_sign: -1.0, ..Default::default() };
        // Commanded straight to +1 rad, but the hardover should still drive
        // negative.
        let out = pcu.step(ActuatorMode::Active, 1.0, 0.0, 0.0, 1.0, &faults);
        assert!(out.torque_nm < 0.0);
    }

    #[test]
    fn a_frozen_transducer_chases_a_stale_reading_even_as_the_true_angle_moves() {
        let mut pcu = PowerControlUnit::new(ActuatorGeometry::aileron());
        let faults = ActuatorFaults { transducer_frozen: true, ..Default::default() };
        // Feedback freezes at the angle seen the first tick (0.0), so a
        // command of 0.0 with the *true* angle having since drifted to 0.2
        // should read as "already there" and stop driving back toward 0.2.
        pcu.step(ActuatorMode::Active, 0.0, 0.0, 0.0, 1.0, &faults);
        let out = pcu.step(ActuatorMode::Active, 0.0, 0.2, 0.0, 1.0, &faults);
        assert!((out.torque_nm).abs() < 1e-9, "should see no error: {}", out.torque_nm);
    }

    #[test]
    fn electric_pump_ramps_up_and_respects_motor_failure() {
        let mut pump = ElectricMotorPump::new(0.2);
        let mut p = 0.0;
        for _ in 0..1000 {
            p = pump.step(1.0, &ElectricPumpFaults::default(), DT);
        }
        assert!(p > 0.99);
        let mut dead = ElectricMotorPump::new(0.2);
        for _ in 0..1000 {
            p = dead.step(1.0, &ElectricPumpFaults { motor_failure: 1.0 }, DT);
        }
        assert!(p < 1e-6);
    }
}

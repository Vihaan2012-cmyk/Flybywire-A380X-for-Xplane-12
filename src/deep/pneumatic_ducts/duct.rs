//! Duct-network physics primitives: a bleed-air duct section modelled as a
//! lumped compressible-gas volume (ideal gas law), connected to its
//! neighbours -- or to a leak's ambient sink -- by compressible orifice
//! flow: the same functional form every A380 bleed valve in FlyByWire's own
//! ported systems uses (`compressible_orifice_mass_flow_rate`, fbw-common
//! `systems/src/pneumatic/valve.rs:40-79`; source there: Anderson, *Modern
//! Compressible Flow*, 3rd ed., section 3.6, isentropic flow through a
//! converging nozzle -- the same physics behind the IEC 60534-2-3 gas valve
//! sizing standard) and this crate's own `physics::bays` leak model already
//! reuses (`orifice_mass_flow_kg_s`, `bays.rs:543-545` and its callers).
//! Duplicated here in plain `f64` (no `uom`) because this workstream's
//! directory must stay self-contained and dependency-free
//! (`docs/deep/BRIEF.md` hard rule 2) -- exactly the precedent
//! `valve.rs` itself sets for its own duplicated `GAMMA`/`R`
//! ("duplicated here, same as air_cycle_machine.rs already does").
//!
//! A duct volume's mass/pressure/temperature response to gas added or
//! removed uses the same ideal-gas charge/discharge relations as
//! `PneumaticContainer::calc_new_pressure_and_temperature_for_mass_flow`
//! (fbw-common `systems/src/pneumatic/mod.rs:151-191`): adding gas mixes it
//! into the existing charge and isentropically compresses what was already
//! there into the smaller remaining free volume; removing gas isentropically
//! expands what is left. This is the standard ideal-gas "filling and
//! emptying of vessels" result found in any compressible-flow text (e.g.
//! Anderson again, ch. 3), not something invented for this crate -- FBW's
//! own trait is simply the same relation already written down once.
//!
//! A duct section also loses heat through its own lagging (insulation
//! blanket) to the airframe zone its run passes through: unlike the
//! `Precooler`'s flowing-gas heat exchange (see `precooler.rs`, which uses
//! the standard NTU-effectiveness method because two flowing streams are
//! involved), a duct's lagging loses heat from an otherwise slowly-changing
//! *static* charge of gas at constant volume, so the correct capacity is
//! `m * Cv` (constant-*volume* process), not `m * Cp` -- unlike a flowing
//! stream, the gas here is not also doing flow work against a pressure
//! difference as it cools.

/// Ratio of specific heats for dry air (diatomic ideal gas), standard.
pub const GAMMA: f64 = 1.4;
/// Specific gas constant for dry air, J/(kg*K). Standard value; matches
/// `PneumaticContainer::GAS_CONSTANT_DRY_AIR` (fbw-common
/// `pneumatic/mod.rs:55`) and `physics::bays::R_AIR`.
pub const R_AIR_J_KG_K: f64 = 287.057005;
/// Dry air specific heat at constant pressure, J/(kg*K). Standard value;
/// matches `physics::bays::CP_AIR_J_PER_KG_K` and FBW's own `Precooler::
/// HEAT_CAPACITY_CONSTANT_PRESSURE` (fbw-common `pneumatic/mod.rs:575`).
pub const CP_AIR_J_KG_K: f64 = 1005.0;
/// Dry air specific heat at constant volume, J/(kg*K): `Cp - R`, the
/// standard ideal-gas relation. Used for a *static* duct volume's own
/// heat-loss capacity (see module docs), not for flowing-stream exchanges.
pub const CV_AIR_J_KG_K: f64 = CP_AIR_J_KG_K - R_AIR_J_KG_K;
/// Critical (choked) pressure ratio for gamma = 1.4 dry air:
/// `(2/(gamma+1))^(gamma/(gamma-1))`.
const CRITICAL_PRESSURE_RATIO: f64 = 0.528_281_787_717_685_7;

/// Standard compressible (isentropic) flow of an ideal gas through a
/// converging orifice: choked above the critical pressure ratio, subsonic
/// (but still compressible) below it. Always non-negative; returns 0 for a
/// closed/zero-area orifice or when the "downstream" side is not actually
/// lower pressure. See module docs for the citation.
pub fn orifice_mass_flow_kg_s(
    discharge_coefficient: f64,
    area_m2: f64,
    upstream_pa: f64,
    upstream_k: f64,
    downstream_pa: f64,
) -> f64 {
    let p1 = upstream_pa.max(0.0);
    let p2 = downstream_pa.max(0.0);
    let t1 = upstream_k.max(1.0);
    let a = area_m2.max(0.0);
    if p1 <= 0.0 || a <= 0.0 || p2 >= p1 {
        return 0.0;
    }
    let pressure_ratio = (p2 / p1).clamp(0.0, 1.0);
    let flow_function = if pressure_ratio <= CRITICAL_PRESSURE_RATIO {
        (GAMMA * (2.0 / (GAMMA + 1.0)).powf((GAMMA + 1.0) / (GAMMA - 1.0))).sqrt()
    } else {
        (2.0 * GAMMA / (GAMMA - 1.0)
            * (pressure_ratio.powf(2.0 / GAMMA) - pressure_ratio.powf((GAMMA + 1.0) / GAMMA)))
        .max(0.0)
        .sqrt()
    };
    discharge_coefficient.max(0.0) * a * p1 / (R_AIR_J_KG_K * t1).sqrt() * flow_function
}

/// A lumped compressible-gas volume: the state a duct section, manifold or
/// plenum carries between ticks.
#[derive(Clone, Copy, Debug)]
pub struct DuctVolume {
    volume_m3: f64,
    pressure_pa: f64,
    temp_k: f64,
    mass_kg: f64,
}

impl DuctVolume {
    pub fn new(volume_m3: f64, pressure_pa: f64, temp_k: f64) -> Self {
        let volume_m3 = volume_m3.max(1e-6);
        let temp_k = temp_k.max(1.0);
        let pressure_pa = pressure_pa.max(0.0);
        let mass_kg = pressure_pa * volume_m3 / (R_AIR_J_KG_K * temp_k);
        Self { volume_m3, pressure_pa, temp_k, mass_kg }
    }

    pub fn volume_m3(&self) -> f64 {
        self.volume_m3
    }
    pub fn pressure_pa(&self) -> f64 {
        self.pressure_pa
    }
    pub fn temp_k(&self) -> f64 {
        self.temp_k
    }
    pub fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    /// Add (`dm_kg > 0`) or remove (`dm_kg < 0`) gas. Incoming gas arrives
    /// at `at_temp_k`/`at_pa` (its own upstream condition); outgoing gas
    /// leaves at this volume's own current condition, so only the sign and
    /// magnitude of `dm_kg` matter on withdrawal. Same relations as
    /// `PneumaticContainer::calc_new_pressure_and_temperature_for_mass_flow`
    /// (module docs); safe for an empty/near-empty volume (no NaN/div-by-0).
    pub fn add_mass(&mut self, dm_kg: f64, at_temp_k: f64, at_pa: f64) {
        let mass = self.mass_kg.max(0.0);
        let new_mass = (mass + dm_kg).max(0.0);
        if dm_kg > 0.0 {
            let at_temp_k = at_temp_k.max(1.0);
            let at_pa = at_pa.max(1.0);
            if new_mass <= 0.0 {
                return;
            }
            let incoming_volume = dm_kg * R_AIR_J_KG_K * at_temp_k / at_pa;
            let m_c_t = mass * self.temp_k + dm_kg * at_temp_k;
            let volume_quotient = 1.0 + incoming_volume / self.volume_m3;
            let new_temp = (m_c_t / new_mass) * volume_quotient.powf(GAMMA - 1.0);
            let new_pressure = m_c_t * R_AIR_J_KG_K / (self.volume_m3 + incoming_volume)
                * volume_quotient.powf(GAMMA);
            self.temp_k = new_temp.max(1.0);
            self.pressure_pa = new_pressure.max(0.0);
            self.mass_kg = new_mass;
        } else if mass <= 0.0 {
            // Nothing left to expand from an already-empty volume.
            self.mass_kg = new_mass;
        } else {
            let ratio = (new_mass / mass).max(0.0);
            self.pressure_pa = (self.pressure_pa * ratio.powf(GAMMA)).max(0.0);
            self.temp_k = (self.temp_k * ratio.powf(GAMMA - 1.0)).max(1.0);
            self.mass_kg = new_mass;
        }
    }

    /// Lose (or, if `sink_k` is hotter, gain) heat through a conductance
    /// `ua_w_k` to a sink at `sink_k` over `dt_s`, at constant volume and
    /// mass (module docs: `Cv`, not `Cp`). An exact exponential relaxation
    /// (no explicit-Euler overshoot at large `dt_s`/`ua_w_k`). Returns the
    /// heat transferred *to the sink*, W (positive = this volume lost heat).
    pub fn conduct_to(&mut self, ua_w_k: f64, sink_k: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let capacity = (self.mass_kg.max(1e-9) * CV_AIR_J_KG_K).max(1e-6);
        let k = ua_w_k.max(0.0) / capacity;
        let before = self.temp_k;
        let after = sink_k + (before - sink_k) * (-k * dt).exp();
        let heat_w = if dt > 0.0 { (before - after) * capacity / dt } else { ua_w_k.max(0.0) * (before - sink_k) };
        // Isochoric: at fixed mass and volume, P = m*R*T/V, so pressure
        // scales exactly with temperature.
        if before > 1e-6 {
            self.pressure_pa = (self.pressure_pa * after / before).max(0.0);
        }
        self.temp_k = after.max(1.0);
        heat_w
    }
}

/// A linearised bound on how much mass can move from the higher- to the
/// lower-pressure of two volumes before a single step would overshoot
/// equilibrium (their own gas treated as locally isothermal for this bound
/// only -- a stability guard, not a physics law). Plays the same role
/// `PneumaticContainerConnector::update_move_fluid_with_orifice`'s
/// `get_mass_flow_for_equilibrium` clamp does in fbw-common (module docs'
/// citation), reproduced independently here in closed form rather than by
/// Newton iteration since these are both plain ideal-gas volumes.
fn equilibrium_clamp_kg(a: &DuctVolume, b: &DuctVolume) -> f64 {
    let ca = a.mass_kg.max(0.0) / a.pressure_pa.max(1.0); // kg of a's gas per Pa
    let cb = b.mass_kg.max(0.0) / b.pressure_pa.max(1.0);
    if ca + cb <= 0.0 {
        return 0.0;
    }
    ((a.pressure_pa - b.pressure_pa).max(0.0) * ca * cb / (ca + cb)).max(0.0)
}

/// Move gas from `a` to `b` (or the reverse, whichever is upstream) through
/// an orifice of `area_m2` over `dt_s`, clamped so a single step cannot
/// overshoot pressure equalisation. Returns the mass moved from `a` to `b`,
/// kg (negative if the net flow was from `b` to `a`).
pub fn transfer_kg(dt_s: f64, discharge_coefficient: f64, area_m2: f64, a: &mut DuctVolume, b: &mut DuctVolume) -> f64 {
    let dt = dt_s.max(0.0);
    if dt <= 0.0 || area_m2 <= 0.0 {
        return 0.0;
    }
    let (upstream, downstream, sign) = if a.pressure_pa >= b.pressure_pa { (&*a, &*b, 1.0) } else { (&*b, &*a, -1.0) };
    let rate = orifice_mass_flow_kg_s(discharge_coefficient, area_m2, upstream.pressure_pa, upstream.temp_k, downstream.pressure_pa);
    let mut dm = (rate * dt).max(0.0).min(equilibrium_clamp_kg(upstream, downstream));
    let (from_temp, from_pa) = (upstream.temp_k, upstream.pressure_pa);
    if sign > 0.0 {
        a.add_mass(-dm, from_temp, from_pa);
        b.add_mass(dm, from_temp, from_pa);
    } else {
        b.add_mass(-dm, from_temp, from_pa);
        a.add_mass(dm, from_temp, from_pa);
        dm = -dm;
    }
    dm
}

/// Move gas only from `upstream` to `downstream` through a metering path
/// gated separately from its own check valve (e.g. an engine starter
/// duct's control valve and its real non-return valve, which must stop the
/// engine's own rising pressure once it lights off from flowing back into
/// the manifold even while the control valve itself sits shut).
/// `forward_area_m2` is the *control valve's* own commanded area (0 when
/// shut -- no forward flow at all); `check_valve_seat_area_m2` is the check
/// valve's own fixed physical seat area, independent of the control
/// valve's position, since the check valve's job is specifically to guard
/// against reverse flow regardless of whether the control valve upstream
/// happens to be open or shut at this instant. `backflow_leak_fraction`
/// (0.0 = a perfect, healthy check valve .. 1.0 = the valve fails fully
/// open, equivalent to [`transfer_kg`]'s ordinary two-way flow through the
/// seat area) lets a failed check valve leak in reverse proportionally,
/// matching this crate's continuous-severity fault convention rather than
/// a binary open/shut.
pub fn one_way_transfer_kg(
    dt_s: f64,
    discharge_coefficient: f64,
    forward_area_m2: f64,
    check_valve_seat_area_m2: f64,
    upstream: &mut DuctVolume,
    downstream: &mut DuctVolume,
    backflow_leak_fraction: f64,
) -> f64 {
    let dt = dt_s.max(0.0);
    if dt <= 0.0 {
        return 0.0;
    }
    if upstream.pressure_pa >= downstream.pressure_pa {
        if forward_area_m2 <= 0.0 {
            return 0.0;
        }
        let rate = orifice_mass_flow_kg_s(discharge_coefficient, forward_area_m2, upstream.pressure_pa, upstream.temp_k, downstream.pressure_pa);
        let dm = (rate * dt).max(0.0).min(equilibrium_clamp_kg(upstream, downstream));
        let (t, p) = (upstream.temp_k, upstream.pressure_pa);
        upstream.add_mass(-dm, t, p);
        downstream.add_mass(dm, t, p);
        dm
    } else {
        let leak = backflow_leak_fraction.clamp(0.0, 1.0);
        if leak <= 0.0 || check_valve_seat_area_m2 <= 0.0 {
            return 0.0;
        }
        let rate = orifice_mass_flow_kg_s(discharge_coefficient * leak, check_valve_seat_area_m2, downstream.pressure_pa, downstream.temp_k, upstream.pressure_pa);
        let dm = (rate * dt).max(0.0).min(equilibrium_clamp_kg(downstream, upstream));
        let (t, p) = (downstream.temp_k, downstream.pressure_pa);
        downstream.add_mass(-dm, t, p);
        upstream.add_mass(dm, t, p);
        -dm
    }
}

/// A passive, spring-loaded check valve's open fraction as a function of
/// the pressure differential pushing it open: `2/pi * atan(dP / spring)`,
/// clamped to `0..1`. This is the same functional form FBW's own
/// `PurelyPneumaticValve::set_open_amount_from_pressure_difference`
/// (fbw-common `pneumatic/valve.rs:1659,1692`) uses for exactly this class
/// of valve (their `SPRING_CHARACTERISTIC = 1.` against a pressure
/// difference in psi) -- reproduced here in plain Pa per this module's
/// self-contained rule, not copied code. Used for the engine bleed IP tap
/// (`network.rs`'s `UpstreamStage`): a real IP8 bleed tap is a plain
/// pressure-operated non-return valve, not electrically commanded.
pub fn passive_valve_open_fraction(pressure_diff_pa: f64, spring_pa: f64) -> f64 {
    (2.0 / std::f64::consts::PI * (pressure_diff_pa / spring_pa.max(1.0)).atan()).clamp(0.0, 1.0)
}

/// Faults on a single duct section (0.0 = healthy .. 1.0 = fully failed).
/// `leak`/`rupture` are read by `leak.rs`; `insulation_damage` here.
#[derive(Clone, Copy, Debug, Default)]
pub struct DuctSectionFaults {
    /// A crack/seal failure: see `leak.rs`.
    pub leak: f64,
    /// The duct parts entirely: see `leak.rs`.
    pub rupture: f64,
    /// The lagging (insulation blanket) is torn or missing: this section's
    /// heat loss to its zone rises toward the bare-pipe rate.
    pub insulation_damage: f64,
}

/// One duct section: a gas volume, its nominal (lagged) heat-loss
/// conductance to the airframe zone its run passes through, and the
/// geometry `leak.rs` sizes its leak/rupture orifices from.
#[derive(Clone, Debug)]
pub struct DuctSection {
    pub gas: DuctVolume,
    /// Zone name this run passes through, matching a real A380 zone
    /// (e.g. `"WING_ROOT"`, `"PYLON_1"`, `"WING_LEADING_EDGE_L"`). A plain
    /// string, not a shared enum: `deep::thermal_zones::network`'s
    /// `ThermalNetwork` already owns the authoritative arbitrary-zone graph
    /// (its `ZoneId` is a `Vec` index assigned at that network's own
    /// construction, and per this push's hard rule 2 this module must not
    /// depend on that crate-internal type) -- a future integration pass
    /// maps this name to that network's `ZoneId` and calls
    /// `inject_heat_w` with `leak.rs`'s output, same as any other system
    /// that dumps heat into a zone there.
    pub zone: &'static str,
    /// Characteristic internal diameter, m: sizes `leak.rs`'s full-bore
    /// rupture area and, through it, a leak's smaller crack area.
    diameter_m: f64,
    /// Lagged (healthy) conductance to the zone, W/K.
    insulation_ua_w_k: f64,
}

impl DuctSection {
    /// A bare (unlagged) duct's conductance runs far higher than a
    /// healthy lagging blanket's -- thermal insulation blankets on hot
    /// bleed ducts are commonly rated to cut bare-pipe heat loss by an
    /// order of magnitude (general aerospace/industrial pipe-lagging
    /// practice; no A380-specific U-value is public). **GENERIC**: a fully
    /// damaged/missing blanket's conductance is `10x` the lagged value.
    const BARE_PIPE_MULTIPLIER: f64 = 10.0;

    pub fn new(zone: &'static str, volume_m3: f64, diameter_m: f64, insulation_ua_w_k: f64, initial_pa: f64, initial_k: f64) -> Self {
        Self {
            gas: DuctVolume::new(volume_m3, initial_pa, initial_k),
            zone,
            diameter_m: diameter_m.max(1e-3),
            insulation_ua_w_k: insulation_ua_w_k.max(0.0),
        }
    }

    pub fn full_bore_area_m2(&self) -> f64 {
        std::f64::consts::PI / 4.0 * self.diameter_m * self.diameter_m
    }

    /// This tick's effective heat-loss conductance to the zone, damage-raised.
    pub fn effective_ua_w_k(&self, faults: &DuctSectionFaults) -> f64 {
        let damage = faults.insulation_damage.clamp(0.0, 1.0);
        self.insulation_ua_w_k * (1.0 + (Self::BARE_PIPE_MULTIPLIER - 1.0) * damage)
    }

    /// Advance the lagging heat loss only (leak/rupture flow is `leak.rs`'s
    /// job, called separately since it also needs the caller's ambient
    /// condition). Returns the watts lost to `zone_k`.
    pub fn step_insulation(&mut self, zone_k: f64, dt_s: f64, faults: &DuctSectionFaults) -> f64 {
        self.gas.conduct_to(self.effective_ua_w_k(faults), zone_k, dt_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn choked_flow_does_not_increase_past_the_critical_ratio() {
        let at_ratio = |pr: f64| orifice_mass_flow_kg_s(0.65, 0.001, 300_000.0, 350.0, 300_000.0 * pr);
        let choked = at_ratio(CRITICAL_PRESSURE_RATIO * 0.5);
        let at_critical = at_ratio(CRITICAL_PRESSURE_RATIO);
        assert!((choked - at_critical).abs() / at_critical < 1e-9, "choked flow must be independent of how far below critical the ratio is");
        let just_below_critical = at_ratio(CRITICAL_PRESSURE_RATIO * 0.99);
        assert!(just_below_critical <= at_critical * 1.0001);
    }

    #[test]
    fn zero_pressure_or_area_gives_zero_flow_not_nan() {
        assert_eq!(orifice_mass_flow_kg_s(0.65, 0.0, 300_000.0, 300.0, 100_000.0), 0.0);
        assert_eq!(orifice_mass_flow_kg_s(0.65, 0.001, 0.0, 300.0, 100_000.0), 0.0);
        assert_eq!(orifice_mass_flow_kg_s(0.65, 0.001, 100_000.0, 300.0, 200_000.0), 0.0, "reversed gradient gives no flow, not a negative one");
    }

    #[test]
    fn adding_and_removing_the_same_mass_returns_to_the_start() {
        let mut v = DuctVolume::new(1.0, 300_000.0, 400.0);
        let (p0, t0, m0) = (v.pressure_pa(), v.temp_k(), v.mass_kg());
        v.add_mass(0.05, 500.0, 400_000.0);
        assert!(v.pressure_pa() > p0, "adding mass at higher pressure raises this volume's pressure");
        v.add_mass(-0.05, v.temp_k(), v.pressure_pa());
        assert!((v.mass_kg() - m0).abs() < 1e-9);
        assert!((v.pressure_pa() - p0).abs() / p0 < 1e-6, "round trip returns pressure (isentropic charge/discharge is reversible)");
        let _ = t0;
    }

    #[test]
    fn an_empty_volume_accepts_mass_without_producing_nan() {
        let mut v = DuctVolume::new(0.01, 0.0, 288.0);
        assert_eq!(v.mass_kg(), 0.0);
        v.add_mass(-0.001, 288.0, 100_000.0); // withdrawal from empty: must not panic/NaN
        assert!(v.pressure_pa().is_finite() && v.temp_k().is_finite());
        v.add_mass(0.001, 500.0, 300_000.0);
        assert!(v.pressure_pa() > 0.0 && v.pressure_pa().is_finite());
    }

    #[test]
    fn transfer_kg_moves_gas_from_high_to_low_pressure_and_conserves_total_mass() {
        let mut hot = DuctVolume::new(0.5, 300_000.0, 480.0);
        let mut cold = DuctVolume::new(0.5, 100_000.0, 288.0);
        let total_before = hot.mass_kg() + cold.mass_kg();
        let moved = transfer_kg(0.1, 0.65, 0.005, &mut hot, &mut cold);
        assert!(moved > 0.0, "flow goes from the higher-pressure hot volume to the lower-pressure cold one");
        assert!((hot.mass_kg() + cold.mass_kg() - total_before).abs() < 1e-9, "mass is conserved across the transfer");
        assert!(hot.pressure_pa() < 300_000.0 && cold.pressure_pa() > 100_000.0);
    }

    #[test]
    fn transfer_kg_never_reverses_the_pressure_gradient_in_one_big_step() {
        let mut hot = DuctVolume::new(0.01, 300_000.0, 480.0); // small volume: easy to overshoot
        let mut cold = DuctVolume::new(0.01, 100_000.0, 288.0);
        transfer_kg(5.0, 0.65, 0.02, &mut hot, &mut cold); // large dt, large area
        assert!(hot.pressure_pa() >= cold.pressure_pa() - 1.0, "the equilibrium clamp must stop a single step from overshooting past equalisation, hot={} cold={}", hot.pressure_pa(), cold.pressure_pa());
    }

    #[test]
    fn insulation_loses_heat_toward_the_zone_and_damage_speeds_it_up() {
        let mut healthy = DuctSection::new("PYLON_1", 0.2, 0.1, 5.0, 300_000.0, 473.15);
        let mut damaged = healthy.clone();
        let ok = DuctSectionFaults::default();
        let bad = DuctSectionFaults { insulation_damage: 1.0, ..Default::default() };
        for _ in 0..600 {
            healthy.step_insulation(288.15, 1.0, &ok);
            damaged.step_insulation(288.15, 1.0, &bad);
        }
        assert!(healthy.gas.temp_k() > 288.15, "still cooling toward, not past, the zone");
        assert!(damaged.gas.temp_k() < healthy.gas.temp_k(), "a torn blanket loses heat faster");
    }

    #[test]
    fn a_healthy_check_valve_blocks_reverse_flow_but_a_failed_one_leaks_backward() {
        let mut low = DuctVolume::new(0.05, 100_000.0, 288.0);
        let mut high = DuctVolume::new(0.05, 300_000.0, 400.0); // downstream now higher pressure than "upstream"
        let moved = one_way_transfer_kg(1.0, 0.7, 0.001, 0.001, &mut low, &mut high, 0.0);
        assert_eq!(moved, 0.0, "a healthy check valve must block reverse flow entirely");
        let mut low2 = DuctVolume::new(0.05, 100_000.0, 288.0);
        let mut high2 = DuctVolume::new(0.05, 300_000.0, 400.0);
        let moved2 = one_way_transfer_kg(1.0, 0.7, 0.001, 0.001, &mut low2, &mut high2, 1.0);
        assert!(moved2 < 0.0, "a fully failed check valve must let the higher-pressure side push mass backward");
    }

    #[test]
    fn a_shut_control_valve_still_gets_check_valve_protection() {
        // The control valve is fully shut (forward_area 0.0): forward flow
        // must be zero even though the source is upstream, and a healthy
        // check valve (fraction 0.0) must still block reverse flow using
        // its own seat area, independent of the shut control valve.
        let mut upstream = DuctVolume::new(0.05, 300_000.0, 400.0);
        let mut downstream = DuctVolume::new(0.05, 100_000.0, 288.0);
        let moved = one_way_transfer_kg(1.0, 0.7, 0.0, 0.001, &mut upstream, &mut downstream, 0.0);
        assert_eq!(moved, 0.0, "a shut control valve must stop forward flow");
        let mut low = DuctVolume::new(0.05, 100_000.0, 288.0);
        let mut high = DuctVolume::new(0.05, 300_000.0, 400.0);
        let moved2 = one_way_transfer_kg(1.0, 0.7, 0.0, 0.001, &mut low, &mut high, 0.0);
        assert_eq!(moved2, 0.0, "a shut control valve with a healthy check valve must still block reverse flow");
    }

    #[test]
    fn no_nan_at_rest_dt_zero() {
        let mut s = DuctSection::new("WING_ROOT", 0.3, 0.15, 4.0, 101_325.0, 288.15);
        let w = s.step_insulation(288.15, 0.0, &DuctSectionFaults::default());
        assert_eq!(w, 0.0);
        assert!(s.gas.pressure_pa().is_finite() && s.gas.temp_k().is_finite());
    }

    #[test]
    fn passive_valve_opens_further_with_more_differential_and_never_goes_negative() {
        assert_eq!(passive_valve_open_fraction(-50_000.0, 6894.757), 0.0, "no reverse opening");
        assert_eq!(passive_valve_open_fraction(0.0, 6894.757), 0.0);
        let half_ish = passive_valve_open_fraction(6894.757, 6894.757); // 1 psi diff, matches FBW's own 45 deg point
        assert!((half_ish - 0.5).abs() < 1e-9);
        let more = passive_valve_open_fraction(3.0 * 6894.757, 6894.757);
        assert!(more > half_ish && more <= 1.0);
    }
}

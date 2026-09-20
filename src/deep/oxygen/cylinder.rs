//! A high-pressure gaseous oxygen cylinder: how much is in it, how warm it
//! is, what a gauge on it reads, and every way it can lose its charge.
//!
//! Three of this area's four registered pressure transducers hang off one
//! of these. The model is a two-node thermal one -- the gas and the
//! cylinder wall -- because that is the smallest model in which the two
//! things a real oxygen gauge does both happen:
//!
//! * the reading falls as gas is drawn off, and
//! * the reading falls when the bottle gets cold, with exactly the same
//!   oxygen still in it.
//!
//! and in which the third thing happens too: the reading *sags further
//! during a discharge than the remaining mass justifies*, then creeps back
//! up over the next few minutes as the heavy steel wall re-warms the gas.
//! That is real, it is what a crew watching the gauge during a long
//! depressurisation actually sees, and it falls out of
//! [`gas::blowdown_cooling_factor`] rather than being scripted.
//!
//! ## Wall mass is derived, not quoted
//!
//! [`hoop_stress_wall_mass_kg`] sizes the cylinder wall from the pressure
//! it has to contain, which is how a real cylinder is sized: the DOT 3AA /
//! ISO 9809 family of high-pressure cylinder standards require a burst
//! pressure of at least three times the service pressure, chrome-moly
//! cylinder steel runs to about 800 MPa ultimate, and steel is 7850 kg/m^3.
//! That gives a wall around 5 mm and a 115 cubic-foot cylinder around 25 kg
//! empty, which is the right order for a real aviation cylinder -- and,
//! more to the point, it is *derived* rather than a number picked to make
//! the thermal lag come out nicely. The wall's heat capacity is four times
//! the gas's, which is why the thermal behaviour above happens at all.
//!
//! ## Sources
//!
//! * DOT 3AA / ISO 9809: minimum burst 3x service pressure for seamless
//!   steel high-pressure gas cylinders.
//! * Chrome-moly cylinder steel (AISI 4130 / AMS 6382 class): ultimate
//!   tensile strength about 800 MPa; density 7850 kg/m^3; specific heat
//!   490 J/(kg K). Standard materials-handbook values.
//! * Everything about the oxygen itself: [`super::gas`].

use super::gas;

/// Ultimate tensile strength of chrome-moly cylinder steel, Pa.
const CYLINDER_STEEL_UTS_PA: f64 = 800e6;
/// Minimum burst pressure as a multiple of service pressure (DOT 3AA /
/// ISO 9809).
const BURST_SAFETY_FACTOR: f64 = 3.0;
/// Density of steel, kg/m^3.
const STEEL_DENSITY_KG_M3: f64 = 7850.0;
/// Specific heat of steel, J/(kg K).
const STEEL_SPECIFIC_HEAT_J_PER_KG_K: f64 = 490.0;
/// Allowance for the domed ends, the neck boss and the valve, as a
/// fraction added to the thin-wall shell mass. **GENERIC**: a real
/// cylinder's heads are thicker than its barrel and it carries a solid
/// brass valve; a quarter again is the modest end of what that adds and
/// only makes the wall's thermal inertia slightly larger.
const CYLINDER_END_MASS_ALLOWANCE: f64 = 0.25;

/// Length-to-diameter ratio assumed when turning a cylinder's volume into
/// its external area. **GENERIC**: aviation oxygen cylinders are visibly
/// long and slim; 3 is the low end of what they look like, and the area it
/// gives is what both heat-transfer coefficients below multiply, so the
/// conservative (smaller-area, slower) choice is the right one.
const CYLINDER_LENGTH_OVER_DIAMETER: f64 = 3.0;

/// Free-convection heat transfer coefficient from a cylinder's outside to
/// still bay air, W/(m^2 K). Textbook free convection of a gas spans about
/// 2 to 25; 5 is the still-air end, which is what a bottle clamped in an
/// unventilated bay sees.
const EXTERNAL_CONVECTION_W_PER_M2_K: f64 = 5.0;

/// Free-convection coefficient from the charged gas to the cylinder wall,
/// W/(m^2 K). **GENERIC**, derived: free convection scales as Ra^(1/4) and
/// the Rayleigh number as density squared, so h grows as the square root
/// of density. At 128 bar oxygen is about 140 times denser than air at one
/// atmosphere, which is a factor near 12 on the external figure above; 30
/// is deliberately well below that, so the gas-to-wall coupling is the
/// pessimistic (slower-recovering) end of the plausible range rather than
/// the flattering one.
const INTERNAL_CONVECTION_W_PER_M2_K: f64 = 30.0;

/// Orifice area a leak failure at magnitude 1.0 opens, m^2.
/// **GENERIC**, sized by what it has to mean: one square millimetre is a
/// hole you could see, and it empties a full crew cylinder in about eight
/// minutes, so the 0..1 magnitude spans everything from a weeping fitting
/// (0.01, most of a day) to a blown line.
pub const LEAK_FULL_SCALE_M2: f64 = 1e-6;

/// Bore of the overpressure discharge (burst) disc, m^2. **GENERIC**: the
/// disc's job is to dump the cylinder faster than any fire can raise its
/// pressure further, so the bore is several times the largest modelled
/// leak; 3 mm diameter is a plausible frangible-disc bore and empties a
/// full cylinder in under a minute, which is the behaviour the green disc
/// on the fuselage announces.
const BURST_DISC_BORE_M2: f64 = 7.07e-6;

/// A cylinder's fixed properties.
#[derive(Clone, Copy, Debug)]
pub struct CylinderSpec {
    /// How many identical cylinders are manifolded together as this
    /// supply. Everything below scales with it; the model carries one
    /// lumped gas volume because manifolded cylinders share a pressure.
    pub count: f64,
    /// Free-air capacity of the whole group, litres at one standard
    /// atmosphere and [`Self::reference_temp_k`].
    pub free_air_liters: f64,
    /// Full charge, Pa *gauge*, at the reference temperature. Cylinder
    /// charge pressures are always quoted gauge, which is why this is not
    /// absolute.
    pub charge_gauge_pa: f64,
    /// The temperature the charge pressure is quoted at, K. Also the
    /// temperature every indicated pressure is corrected back to.
    pub reference_temp_k: f64,
    /// Gauge pressure at which the overpressure discharge disc ruptures.
    pub burst_disc_gauge_pa: f64,
}

impl CylinderSpec {
    /// The internal volume of the whole group, m^3, from its free-air
    /// capacity and charge pressure through the real equation of state.
    pub fn volume_m3(&self) -> f64 {
        let mass = gas::mass_from_free_air_kg(self.free_air_liters, self.reference_temp_k);
        gas::volume_for_charge_m3(mass, self.charge_gauge_pa + 101_325.0, self.reference_temp_k)
    }

    /// Mass of oxygen in the group at full charge, kg.
    pub fn full_mass_kg(&self) -> f64 {
        gas::mass_from_free_air_kg(self.free_air_liters, self.reference_temp_k)
    }

    /// External area of the whole group, m^2.
    pub fn external_area_m2(&self) -> f64 {
        cylinder_external_area_m2(self.volume_m3(), self.count.max(1.0))
    }

    /// Wall mass of the whole group, kg.
    pub fn wall_mass_kg(&self) -> f64 {
        hoop_stress_wall_mass_kg(self.volume_m3(), self.count.max(1.0), self.charge_gauge_pa)
    }
}

/// External area of `count` identical cylinders of total volume
/// `volume_m3`, m^2, at [`CYLINDER_LENGTH_OVER_DIAMETER`].
pub fn cylinder_external_area_m2(volume_m3: f64, count: f64) -> f64 {
    if !(volume_m3 > 0.0) || !(count > 0.0) {
        return 0.0;
    }
    let each = volume_m3 / count;
    let l = CYLINDER_LENGTH_OVER_DIAMETER;
    // V = pi D^2 / 4 * (l D) => D = (4V / (pi l))^(1/3)
    let d = (4.0 * each / (std::f64::consts::PI * l)).cbrt();
    let barrel = std::f64::consts::PI * d * (l * d);
    let ends = 2.0 * std::f64::consts::PI * d * d / 4.0;
    (barrel + ends) * count
}

/// Wall mass of `count` cylinders holding `volume_m3` at
/// `service_gauge_pa`, kg, from the thin-wall hoop stress a cylinder
/// standard's burst factor allows.
pub fn hoop_stress_wall_mass_kg(volume_m3: f64, count: f64, service_gauge_pa: f64) -> f64 {
    if !(volume_m3 > 0.0) || !(count > 0.0) || !(service_gauge_pa > 0.0) {
        return 0.0;
    }
    let each = volume_m3 / count;
    let l = CYLINDER_LENGTH_OVER_DIAMETER;
    let d = (4.0 * each / (std::f64::consts::PI * l)).cbrt();
    let allowable_pa = CYLINDER_STEEL_UTS_PA / BURST_SAFETY_FACTOR;
    // Thin-wall hoop stress: sigma = P D / (2 t) => t = P D / (2 sigma).
    let t = service_gauge_pa * d / (2.0 * allowable_pa);
    let shell = cylinder_external_area_m2(volume_m3, count) * t * STEEL_DENSITY_KG_M3;
    shell * (1.0 + CYLINDER_END_MASS_ALLOWANCE)
}

/// What can be wrong with a cylinder. `Default` is a healthy one.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CylinderFaults {
    /// Leak orifice area as a fraction of [`LEAK_FULL_SCALE_M2`].
    pub leak: f64,
    /// How far the overpressure discharge disc is degraded: the rupture
    /// pressure is the rated one times `1 - disc_weakened`, so 0.5 makes a
    /// disc rated well above full charge rupture below it and blow the
    /// cylinder overboard on the ramp.
    pub disc_weakened: f64,
}

/// One frame of a cylinder's state, all of it real physical quantities.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CylinderOutputs {
    pub absolute_pressure_pa: f64,
    /// What a gauge plumbed into the cylinder reads: absolute minus
    /// whatever is on the other side of its diaphragm.
    pub gauge_pressure_pa: f64,
    /// The same reading corrected back to the cylinder's reference
    /// temperature -- the number an oxygen dispatch table is written
    /// against, and the number the low-pressure logic uses, because a cold
    /// bottle reading low is not a bottle that needs servicing.
    pub corrected_gauge_pressure_pa: f64,
    pub gas_temp_k: f64,
    pub wall_temp_k: f64,
    /// What the system actually drew, kg/s (less than it asked for when
    /// the cylinder cannot supply it).
    pub delivered_kg_s: f64,
    pub leak_kg_s: f64,
    /// Flow out of a ruptured overpressure disc, kg/s.
    pub discharge_kg_s: f64,
    pub disc_ruptured: bool,
    pub mass_kg: f64,
    /// Mass remaining as a fraction of full charge.
    pub quantity_fraction: f64,
}

/// A charged cylinder group, running.
#[derive(Clone, Debug)]
pub struct HpCylinder {
    spec: CylinderSpec,
    volume_m3: f64,
    full_mass_kg: f64,
    gas_heat_capacity_per_kg: f64,
    wall_heat_capacity_j_per_k: f64,
    ua_gas_wall_w_per_k: f64,
    ua_wall_ambient_w_per_k: f64,
    mass_kg: f64,
    gas_temp_k: f64,
    wall_temp_k: f64,
    /// A ruptured disc stays ruptured until the cylinder is serviced --
    /// it is a frangible metal foil, not a valve.
    disc_ruptured: bool,
}

impl HpCylinder {
    /// A cylinder group at full charge, at its reference temperature.
    pub fn new(spec: CylinderSpec) -> Self {
        let volume_m3 = spec.volume_m3();
        let full_mass_kg = spec.full_mass_kg();
        let area = spec.external_area_m2();
        let wall_mass = spec.wall_mass_kg();
        Self {
            spec,
            volume_m3,
            full_mass_kg,
            gas_heat_capacity_per_kg: gas::cv_o2_j_per_kg_k(),
            wall_heat_capacity_j_per_k: (wall_mass * STEEL_SPECIFIC_HEAT_J_PER_KG_K).max(1.0),
            ua_gas_wall_w_per_k: (area * INTERNAL_CONVECTION_W_PER_M2_K).max(1e-6),
            ua_wall_ambient_w_per_k: (area * EXTERNAL_CONVECTION_W_PER_M2_K).max(1e-6),
            mass_kg: full_mass_kg,
            gas_temp_k: spec.reference_temp_k,
            wall_temp_k: spec.reference_temp_k,
            disc_ruptured: false,
        }
    }

    pub fn spec(&self) -> &CylinderSpec {
        &self.spec
    }

    pub fn volume_m3(&self) -> f64 {
        self.volume_m3
    }

    pub fn full_mass_kg(&self) -> f64 {
        self.full_mass_kg
    }

    pub fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    pub fn wall_heat_capacity_j_per_k(&self) -> f64 {
        self.wall_heat_capacity_j_per_k
    }

    /// Servicing: recharged, and a new disc fitted.
    pub fn service(&mut self) {
        self.mass_kg = self.full_mass_kg;
        self.disc_ruptured = false;
    }

    /// Set the charge directly, for tests and for a cold-and-dark start
    /// that begins with a partly-used bottle.
    pub fn set_mass_kg(&mut self, mass_kg: f64) {
        self.mass_kg = mass_kg.clamp(0.0, self.full_mass_kg);
    }

    pub fn set_temperatures_k(&mut self, temp_k: f64) {
        if temp_k > 0.0 {
            self.gas_temp_k = temp_k;
            self.wall_temp_k = temp_k;
        }
    }

    /// Temperature of the gas in the cylinder right now, K.
    pub fn gas_temp_k(&self) -> f64 {
        self.gas_temp_k
    }

    /// Absolute pressure in the cylinder right now, Pa.
    pub fn absolute_pressure_pa(&self) -> f64 {
        gas::pressure_pa(self.mass_kg, self.volume_m3, self.gas_temp_k)
    }

    /// Advance one frame.
    ///
    /// `demand_kg_s` is what the system downstream is asking for,
    /// `surround_temp_k` the temperature of the bay the cylinder is
    /// clamped in, `ambient_pressure_pa` what a leak or a ruptured disc
    /// vents into.
    pub fn step(&mut self, demand_kg_s: f64, ambient_pressure_pa: f64, surround_temp_k: f64, faults: CylinderFaults, dt_s: f64) -> CylinderOutputs {
        let dt = dt_s.max(0.0);
        let ambient = ambient_pressure_pa.max(0.0);
        let surround = if surround_temp_k > 0.0 { surround_temp_k } else { self.spec.reference_temp_k };

        let mut pressure = gas::pressure_pa(self.mass_kg, self.volume_m3, self.gas_temp_k);

        // The overpressure discharge disc. It is a pressure-actuated
        // frangible foil, so it does not care *why* the pressure is high:
        // a bottle heated in a bay fire ruptures it exactly as a bottle
        // overcharged on the ramp does, and that is the coupling that
        // makes the cylinder's own temperature do real work.
        let rupture_at = self.spec.burst_disc_gauge_pa * (1.0 - faults.disc_weakened.clamp(0.0, 1.0)) + ambient;
        if pressure >= rupture_at {
            self.disc_ruptured = true;
        }

        let leak_area = faults.leak.clamp(0.0, 1.0) * LEAK_FULL_SCALE_M2;
        let mut leak = gas::orifice_mass_flow_kg_s(leak_area, pressure, self.gas_temp_k, ambient);
        let mut discharge = if self.disc_ruptured {
            gas::orifice_mass_flow_kg_s(BURST_DISC_BORE_M2, pressure, self.gas_temp_k, ambient)
        } else {
            0.0
        };
        // Nothing can be drawn out of a cylinder that is no higher than
        // what it would be drawn into.
        let mut delivered = if pressure > ambient { demand_kg_s.max(0.0) } else { 0.0 };

        // Conservation: the frame cannot take out more than is in there.
        let total = delivered + leak + discharge;
        if dt > 0.0 && total * dt > self.mass_kg {
            let scale = if total > 0.0 { self.mass_kg / (total * dt) } else { 0.0 };
            delivered *= scale;
            leak *= scale;
            discharge *= scale;
        }
        let total = delivered + leak + discharge;

        // Gas temperature: the blowdown term first (the gas left behind
        // does the work of expelling what leaves), then relaxation toward
        // the cylinder wall.
        if dt > 0.0 && total > 0.0 {
            self.gas_temp_k *= gas::blowdown_cooling_factor(self.mass_kg, total, dt);
        }
        let gas_capacity = (self.mass_kg * self.gas_heat_capacity_per_kg).max(1.0);
        let tau_gas = gas_capacity / self.ua_gas_wall_w_per_k;
        let new_gas_temp = gas::relax(self.gas_temp_k, self.wall_temp_k, tau_gas, dt);

        // Wall: driven by the gas on one side and the bay on the other,
        // relaxed exactly toward the equilibrium of the two.
        let ua_sum = self.ua_gas_wall_w_per_k + self.ua_wall_ambient_w_per_k;
        let wall_target = (self.ua_gas_wall_w_per_k * self.gas_temp_k + self.ua_wall_ambient_w_per_k * surround) / ua_sum;
        let tau_wall = self.wall_heat_capacity_j_per_k / ua_sum;
        self.wall_temp_k = gas::relax(self.wall_temp_k, wall_target, tau_wall, dt);
        self.gas_temp_k = new_gas_temp.max(1.0);

        self.mass_kg = (self.mass_kg - total * dt).max(0.0);
        pressure = gas::pressure_pa(self.mass_kg, self.volume_m3, self.gas_temp_k);
        let corrected = gas::pressure_pa(self.mass_kg, self.volume_m3, self.spec.reference_temp_k);

        CylinderOutputs {
            absolute_pressure_pa: pressure,
            gauge_pressure_pa: (pressure - ambient).max(0.0),
            corrected_gauge_pressure_pa: (corrected - ambient).max(0.0),
            gas_temp_k: self.gas_temp_k,
            wall_temp_k: self.wall_temp_k,
            delivered_kg_s: delivered,
            leak_kg_s: leak,
            discharge_kg_s: discharge,
            disc_ruptured: self.disc_ruptured,
            mass_kg: self.mass_kg,
            quantity_fraction: if self.full_mass_kg > 0.0 { self.mass_kg / self.full_mass_kg } else { 0.0 },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The crew cylinder group, so the numbers below are the ones the area
    /// actually flies with.
    fn spec() -> CylinderSpec {
        CylinderSpec {
            count: 2.0,
            free_air_liters: 6520.0,
            charge_gauge_pa: 1850.0 * gas::PSI_TO_PA,
            reference_temp_k: 294.15,
            burst_disc_gauge_pa: 2775.0 * gas::PSI_TO_PA,
        }
    }

    #[test]
    fn a_full_cylinder_reads_its_rated_charge_at_its_rated_temperature() {
        let c = HpCylinder::new(spec());
        let psi = (c.absolute_pressure_pa() - 101_325.0) / gas::PSI_TO_PA;
        assert!((psi - 1850.0).abs() < 1.0, "{psi} psi");
    }

    #[test]
    fn the_derived_geometry_lands_where_a_real_cylinder_does() {
        let s = spec();
        // Two 115 cubic-foot cylinders: each about 23 litres internal,
        // about a fifth of a metre across, and about 25 kg of steel.
        let v = s.volume_m3();
        assert!(v > 0.035 && v < 0.060, "{v} m^3 for the pair");
        let wall = s.wall_mass_kg();
        assert!(wall > 30.0 && wall < 80.0, "{wall} kg for the pair");
        // The wall's heat capacity must dominate the gas's, or the thermal
        // recovery this model exists to show would not happen.
        let c = HpCylinder::new(s);
        let gas_capacity = c.full_mass_kg() * gas::cv_o2_j_per_kg_k();
        assert!(c.wall_heat_capacity_j_per_k() > 2.0 * gas_capacity, "wall {} gas {}", c.wall_heat_capacity_j_per_k(), gas_capacity);
    }

    #[test]
    fn indicated_pressure_falls_with_use() {
        let mut c = HpCylinder::new(spec());
        let mut out = c.step(0.0, 75_000.0, 294.15, CylinderFaults::default(), 0.0);
        let start = out.gauge_pressure_pa;
        // An hour of four masks at altitude, roughly.
        for _ in 0..3600 {
            out = c.step(1.6e-4, 75_000.0, 294.15, CylinderFaults::default(), 1.0);
        }
        assert!(out.gauge_pressure_pa < start, "{} vs {}", out.gauge_pressure_pa, start);
        assert!(out.quantity_fraction < 1.0 && out.quantity_fraction > 0.7, "{}", out.quantity_fraction);
        // And the fall is real mass, not a display trick: the corrected
        // reading has fallen too.
        assert!(out.corrected_gauge_pressure_pa < start);
    }

    #[test]
    fn indicated_pressure_falls_with_temperature_with_the_same_oxygen_in_it() {
        let mut warm = HpCylinder::new(spec());
        let mut cold = HpCylinder::new(spec());
        // Cold-soak one of them overnight in a -25 C bay; take no gas at
        // all out of either.
        let mut cold_out = cold.step(0.0, 101_325.0, 248.15, CylinderFaults::default(), 0.0);
        let mut warm_out = warm.step(0.0, 101_325.0, 294.15, CylinderFaults::default(), 0.0);
        for _ in 0..(8 * 3600) {
            cold_out = cold.step(0.0, 101_325.0, 248.15, CylinderFaults::default(), 1.0);
            warm_out = warm.step(0.0, 101_325.0, 294.15, CylinderFaults::default(), 1.0);
        }
        assert!((cold_out.mass_kg - warm_out.mass_kg).abs() < 1e-12, "no gas left either bottle");
        let fall_psi = (warm_out.gauge_pressure_pa - cold_out.gauge_pressure_pa) / gas::PSI_TO_PA;
        assert!(fall_psi > 150.0, "a 46 K cold soak should cost several hundred psi, got {fall_psi}");
        // The temperature-corrected reading is the one that does *not*
        // move, which is why a dispatch check uses it.
        let corrected_gap = (warm_out.corrected_gauge_pressure_pa - cold_out.corrected_gauge_pressure_pa).abs() / gas::PSI_TO_PA;
        assert!(corrected_gap < 1.0, "corrected readings should agree, got {corrected_gap} psi apart");
    }

    #[test]
    fn a_hard_discharge_sags_the_gauge_further_than_the_mass_and_it_recovers() {
        let mut c = HpCylinder::new(spec());
        // Five minutes of heavy draw.
        let mut hot = CylinderOutputs::default();
        for _ in 0..300 {
            hot = c.step(2.0e-3, 75_000.0, 294.15, CylinderFaults::default(), 1.0);
        }
        assert!(hot.gas_temp_k < 294.0, "the gas must cool while it is being expelled: {}", hot.gas_temp_k);
        let sagged = hot.gauge_pressure_pa;
        // Now stop drawing and let it sit.
        let mut recovered = hot;
        for _ in 0..3600 {
            recovered = c.step(0.0, 75_000.0, 294.15, CylinderFaults::default(), 1.0);
        }
        assert!((recovered.mass_kg - hot.mass_kg).abs() < 1e-12, "nothing left the bottle while it sat");
        assert!(recovered.gauge_pressure_pa > sagged, "the wall must re-warm the gas: {} vs {}", recovered.gauge_pressure_pa, sagged);
        assert!(recovered.gas_temp_k > hot.gas_temp_k);
    }

    #[test]
    fn a_leak_empties_it_and_conserves_mass_on_the_way() {
        let mut c = HpCylinder::new(spec());
        let faults = CylinderFaults { leak: 1.0, ..Default::default() };
        let start = c.mass_kg();
        let mut lost = 0.0;
        let mut out = CylinderOutputs::default();
        for _ in 0..1200 {
            out = c.step(0.0, 75_000.0, 294.15, faults, 1.0);
            lost += out.leak_kg_s;
        }
        // Choked flow is linear in pressure and pressure is very nearly
        // linear in mass, so the blowdown is exponential with about a
        // seven-and-a-half minute time constant: twenty minutes takes out
        // the great majority of a full crew bottle.
        assert!(out.mass_kg < 0.15 * start, "a 1 mm^2 hole should mostly empty it inside twenty minutes: {} of {start}", out.mass_kg);
        // Conservation: every kilogram that came out of the orifice is a
        // kilogram the cylinder no longer has.
        assert!((lost - (start - out.mass_kg)).abs() < 1e-9, "lost {lost}, missing {}", start - out.mass_kg);
        c.set_mass_kg(0.0);
        assert_eq!(c.step(0.0, 75_000.0, 294.15, faults, 1.0).leak_kg_s, 0.0, "an empty bottle cannot keep leaking");
    }

    #[test]
    fn a_weakened_disc_ruptures_below_the_charge_and_dumps_the_bottle() {
        let mut c = HpCylinder::new(spec());
        let faults = CylinderFaults { disc_weakened: 0.5, ..Default::default() };
        let first = c.step(0.0, 101_325.0, 294.15, faults, 0.1);
        assert!(first.disc_ruptured, "1850 psi is above half of 2775 psi");
        assert!(first.discharge_kg_s > 0.0);
        let mut out = first;
        for _ in 0..600 {
            out = c.step(0.0, 101_325.0, 294.15, faults, 1.0);
        }
        assert!(out.quantity_fraction < 0.01, "{}", out.quantity_fraction);
        // A ruptured disc is a hole, not a valve: clearing the fault does
        // not un-rupture it.
        assert!(c.step(0.0, 101_325.0, 294.15, CylinderFaults::default(), 1.0).disc_ruptured);
        c.service();
        assert!(!c.step(0.0, 101_325.0, 294.15, CylinderFaults::default(), 1.0).disc_ruptured);
    }

    #[test]
    fn a_healthy_disc_ruptures_when_a_bay_fire_heats_the_bottle() {
        // The reason the disc is modelled against pressure rather than
        // against a "bottle overpressure" failure: heat is the real cause.
        let mut c = HpCylinder::new(spec());
        let mut out = CylinderOutputs::default();
        for _ in 0..(4 * 3600) {
            out = c.step(0.0, 101_325.0, 700.0, CylinderFaults::default(), 1.0);
            if out.disc_ruptured {
                break;
            }
        }
        assert!(out.disc_ruptured, "a bottle cooked to a bay fire's temperature must relieve, reached {} K", out.gas_temp_k);
    }

    #[test]
    fn nothing_moves_and_nothing_breaks_at_rest() {
        let mut c = HpCylinder::new(spec());
        let a = c.step(0.0, 101_325.0, 294.15, CylinderFaults::default(), 0.0);
        let b = c.step(0.0, 101_325.0, 294.15, CylinderFaults::default(), 0.0);
        assert_eq!(a, b);
        assert!(a.absolute_pressure_pa.is_finite() && a.gas_temp_k.is_finite() && a.wall_temp_k.is_finite());
        let mut empty = HpCylinder::new(spec());
        empty.set_mass_kg(0.0);
        let e = empty.step(1.0, 101_325.0, 294.15, CylinderFaults { leak: 1.0, disc_weakened: 1.0 }, 1.0);
        assert_eq!(e.mass_kg, 0.0);
        assert!(e.absolute_pressure_pa.is_finite() && e.gas_temp_k > 0.0);
    }
}

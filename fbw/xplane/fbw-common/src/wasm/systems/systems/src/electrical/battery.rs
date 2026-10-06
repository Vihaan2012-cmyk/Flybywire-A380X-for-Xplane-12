use uom::si::{
    electric_charge::ampere_hour, electric_current::ampere, electric_potential::volt,
    electrical_resistance::ohm, f64::*, power::watt, thermodynamic_temperature::degree_celsius,
    time::second,
};

use crate::{
    shared::{ConsumePower, PowerConsumptionReport},
    simulation::{
        InitContext, Read, SimulationElement, SimulatorReader, SimulatorWriter, UpdateContext,
        VariableIdentifier, Write,
    },
};

use super::{
    ElectricalElement, ElectricalElementIdentifier, ElectricalElementIdentifierProvider,
    ElectricalStateWriter, ElectricitySource, Potential, PotentialOrigin, ProvideCurrent,
    ProvidePotential,
};

#[derive(Clone)]
pub struct Battery {
    number: usize,
    identifier: ElectricalElementIdentifier,
    writer: ElectricalStateWriter,
    temperature_id: VariableIdentifier,
    internal_resistance_id: VariableIdentifier,
    charge: ElectricCharge,
    input_potential: ElectricPotential,
    output_potential: ElectricPotential,
    current: ElectricCurrent,
    /// Equivalent-circuit thermal state (electrical/Battery.md documents the
    /// OCV(SOC) curve this shares; the resistance/thermal model is new).
    temperature: ThermodynamicTemperature,
    /// Two-RC-branch (Thevenin) dynamic polarization state: `v1` (fast,
    /// charge-transfer/double-layer) and `v2` (slow, diffusion) voltage
    /// drops the terminal voltage lags its algebraic OCV(SOC)-IR value by,
    /// per the standard second-order equivalent-circuit battery model
    /// (Chen & Rincon-Mora, "A Battery Model ... Dynamic Battery I-V
    /// Performance", IEEE Trans. Energy Conversion, 2006 -- the widely used
    /// 2RC Thevenin model in battery/BMS modelling literature, not this
    /// workstream's own shortcut). No FBW/A380-specific curve fit exists,
    /// so typical published NiCd time constants are used
    /// (derived/typical, electrical.md).
    v1: ElectricPotential,
    v2: ElectricPotential,
    polarization_id: VariableIdentifier,
    /// Continuous cell-aging magnitudes in [0, 1] (0 = new cell), written
    /// by the plugin as plain simulator variables (the breaker-variable
    /// pattern this workstream's other degradation inputs use; both read 0
    /// until the plugin writes them). `resistance_growth` scales
    /// `internal_resistance()` up towards `AGED_RESISTANCE_MULTIPLIER`x its
    /// fresh-cell value (a real Ni-Cd's series resistance climbs markedly
    /// with cycle aging). `capacity_fade` shrinks the *usable* Ah the
    /// OCV(SOC) curve is evaluated against (see `ocv_for_charge`) towards
    /// `MAX_CAPACITY_FADE`, i.e. a faded cell reaches the curve's "high
    /// charge" knee at a lower absolute stored Ah, exactly as a smaller
    /// effective pack would. Will move to `failures::magnitude(id)` once
    /// that lands.
    resistance_growth_id: VariableIdentifier,
    resistance_growth: f64,
    capacity_fade_id: VariableIdentifier,
    capacity_fade: f64,
}
impl Battery {
    const RATED_CAPACITY_AMPERE_HOURS: f64 = 23.;

    /// Cell internal resistance at the 20 C reference point. FBW's own,
    /// previously unused, comment at this file's old `calculate_charging_current`
    /// already cited 0.011 ohm for the cell itself ("Internal resistance =
    /// 0.011 ohm. However that would make current go through the roof."); we
    /// now use that real figure instead of discarding it.
    const CELL_INTERNAL_RESISTANCE_OHM_AT_20C: f64 = 0.011;
    /// Wiring/contactor resistance from the battery to its bus bar. FBW's
    /// code has no figure for this; it is a defensible derived estimate for
    /// a short, heavy-gauge run in an aircraft electrical bay, clearly marked
    /// as such (docs/physics/electrical.md).
    const WIRING_RESISTANCE_OHM: f64 = 0.02;
    /// Ni-Cd internal resistance rises markedly in the cold; 2%/C below the
    /// 20 C reference is a typical published magnitude for aircraft Ni-Cd
    /// packs (derived/typical, no FBW source; electrical.md).
    const RESISTANCE_TEMP_COEFFICIENT_PER_C: f64 = 0.02;
    const RESISTANCE_REFERENCE_TEMP_C: f64 = 20.;
    /// Thermal mass (mass x specific heat) of the cell stack: ~20 kg of
    /// Ni-Cd cells at ~0.3 J/(g K) => ~6000 J/K. Derived estimate (no FBW or
    /// public per-cell mass figure), electrical.md.
    const THERMAL_MASS_J_PER_KELVIN: f64 = 6000.;
    /// Natural-convection heat loss coefficient from the battery case to the
    /// avionics-bay air. Derived estimate, electrical.md.
    const COOLING_W_PER_KELVIN: f64 = 2.5;
    /// A real Ni-Cd fast-charge limit is about the 1C rate (here, the pack's
    /// own rated capacity in Ah numerically as amps) rather than the
    /// arbitrary flat 10 A this replaces.
    const MAX_CHARGE_CURRENT_AMPERES: f64 = Self::RATED_CAPACITY_AMPERE_HOURS;
    /// Below this remaining charge the OCV curve's own output (a fraction of
    /// a volt) is no longer a meaningful equivalent-circuit source (it
    /// predicts sub-milliwatt maximum power transfer); treat the cell as
    /// practically empty rather than asymptotically approaching zero over
    /// simulated hours, matching how a real cell's protection/BMS would
    /// declare it dead.
    const PRACTICALLY_EMPTY_AMPERE_HOURS: f64 = 1e-4;

    /// Peukert exponent for a Ni-Cd cell. Ni-Cd's much flatter rate-capacity
    /// curve gives an exponent far closer to 1 than lead-acid's (~1.2-1.3);
    /// published battery-engineering references for vented/sealed Ni-Cd
    /// cells put it around 1.05-1.10 (e.g. Linden & Reddy, "Handbook of
    /// Batteries", Ni-Cd chapter). No FBW/A380-specific figure exists, so
    /// this is the most defensible derived/typical value (electrical.md).
    const PEUKERT_EXPONENT: f64 = 1.08;
    /// The rate Peukert's law and the rated-capacity figure are both
    /// anchored to: numerically the pack's own rated Ah as amps (the
    /// standard 1C reference rate).
    const PEUKERT_REFERENCE_CURRENT_AMPERES: f64 = Self::RATED_CAPACITY_AMPERE_HOURS;

    /// Thevenin 2RC dynamic-polarization branch parameters (see the `v1`/
    /// `v2` field doc comment for the model and its citation). Typical
    /// published Ni-Cd magnitudes (derived/typical, no FBW/A380-specific
    /// fit, electrical.md): a fast charge-transfer branch with
    /// `tau1 = R1*C1 ~ 2 s`, and a slow diffusion branch with
    /// `tau2 = R2*C2 ~ 60 s`.
    const R1_OHM: f64 = 0.004;
    const C1_FARAD: f64 = 500.;
    const R2_OHM: f64 = 0.006;
    const C2_FARAD: f64 = 10_000.;

    /// Internal-resistance multiplier at maximum modelled aging
    /// (`resistance_growth` magnitude 1.0): roughly 3x a fresh cell's
    /// resistance, a defensible order-of-magnitude estimate for an
    /// end-of-life Ni-Cd cell (published aircraft Ni-Cd maintenance
    /// literature treats a several-fold internal-resistance rise as the
    /// usual end-of-life signature; no FBW/A380-specific figure exists, so
    /// this is derived/typical, docs/physics/electrical.md).
    const AGED_RESISTANCE_MULTIPLIER: f64 = 3.0;
    /// Maximum usable-capacity loss modelled at `capacity_fade` magnitude
    /// 1.0: clamped well short of 100% so `ocv_for_charge`'s rescaling never
    /// divides by (near) zero. Derived/typical (docs/physics/electrical.md).
    const MAX_CAPACITY_FADE: f64 = 0.7;

    pub fn full(context: &mut InitContext, number: usize) -> Battery {
        Battery::new(
            context,
            number,
            ElectricCharge::new::<ampere_hour>(Battery::RATED_CAPACITY_AMPERE_HOURS),
        )
    }

    pub fn half(context: &mut InitContext, number: usize) -> Battery {
        Battery::new(
            context,
            number,
            ElectricCharge::new::<ampere_hour>(Battery::RATED_CAPACITY_AMPERE_HOURS / 2.),
        )
    }

    pub fn empty(context: &mut InitContext, number: usize) -> Battery {
        Battery::new(context, number, ElectricCharge::new::<ampere_hour>(0.))
    }

    pub fn new(context: &mut InitContext, number: usize, charge: ElectricCharge) -> Self {
        Self {
            number,
            identifier: context.next_electrical_identifier(),
            writer: ElectricalStateWriter::new(context, &format!("BAT_{}", number)),
            temperature_id: context.get_identifier(format!("ELEC_BAT_{}_TEMPERATURE", number)),
            internal_resistance_id: context
                .get_identifier(format!("ELEC_BAT_{}_INTERNAL_RESISTANCE", number)),
            charge,
            input_potential: ElectricPotential::new::<volt>(0.),
            output_potential: Battery::calculate_output_potential_for_charge(charge),
            current: ElectricCurrent::new::<ampere>(0.),
            temperature: ThermodynamicTemperature::new::<degree_celsius>(
                Self::RESISTANCE_REFERENCE_TEMP_C,
            ),
            v1: ElectricPotential::new::<volt>(0.),
            v2: ElectricPotential::new::<volt>(0.),
            polarization_id: context
                .get_identifier(format!("ELEC_BAT_{}_POLARIZATION_VOLTAGE", number)),
            resistance_growth_id: context
                .get_identifier(format!("ELEC_BAT_{}_RESISTANCE_GROWTH", number)),
            resistance_growth: 0.,
            capacity_fade_id: context.get_identifier(format!("ELEC_BAT_{}_CAPACITY_FADE", number)),
            capacity_fade: 0.,
        }
    }

    /// The algebraic OCV(SOC) curve's value minus the 2RC branches'
    /// dynamic-polarization drop: the terminal voltage a moment after a
    /// current step lags the eventual steady-state value by exactly this
    /// amount, and relaxes back over each branch's own time constant. Used
    /// in place of the raw OCV curve in both the discharge and charge
    /// Kirchhoff loops below.
    fn effective_ocv(&self, ocv: ElectricPotential) -> ElectricPotential {
        ocv - self.v1 - self.v2
    }

    /// Advances the two RC branches' dynamic state with this tick's current
    /// (sign convention: positive out of the battery, i.e. discharging).
    ///
    /// Uses the exact (zero-order-hold) solution of `dv/dt = (i*R - v)/(RC)`
    /// for a current held constant over the tick, `v(t) = v0*exp(-t/RC) +
    /// i*R*(1 - exp(-t/RC))`, rather than a forward-Euler step: the branches'
    /// time constants (~2 s, ~60 s) are far shorter than some callers' fixed
    /// test/sim steps (this crate's own tests step in whole minutes), and
    /// forward Euler is unconditionally unstable once `dt` exceeds about
    /// `2*tau` for a decay term. The exact exponential update is stable for
    /// any `dt`, matching the brief's "no NaNs or explosions at any sim
    /// rate" requirement, and also converges to the correct steady state
    /// (`v -> i*R`) instead of only approximating it for small steps.
    fn update_polarization(&mut self, dt: f64, discharge_current: ElectricCurrent) {
        let i = discharge_current.get::<ampere>();
        Self::step_rc_branch(&mut self.v1, i, Self::R1_OHM, Self::R1_OHM * Self::C1_FARAD, dt);
        Self::step_rc_branch(&mut self.v2, i, Self::R2_OHM, Self::R2_OHM * Self::C2_FARAD, dt);
    }

    fn step_rc_branch(v: &mut ElectricPotential, i: f64, r: f64, tau: f64, dt: f64) {
        let decay = (-dt / tau).exp();
        let steady_state = i * r;
        *v = ElectricPotential::new::<volt>(v.get::<volt>() * decay + steady_state * (1. - decay));
    }

    /// Peukert-law derating factor (`t = C / I^k`, expressed here as an
    /// equivalent-current multiplier on plain coulomb counting): at
    /// currents above the 1C reference rate a real cell delivers less
    /// usable capacity than a straight Ah count implies. Returns 1 (no
    /// derating) at or below the reference rate, reducing to plain coulomb
    /// counting exactly as before for everyday loads.
    fn peukert_factor(current_amperes: f64) -> f64 {
        if current_amperes <= Self::PEUKERT_REFERENCE_CURRENT_AMPERES {
            1.
        } else {
            (current_amperes / Self::PEUKERT_REFERENCE_CURRENT_AMPERES)
                .powf(Self::PEUKERT_EXPONENT - 1.)
        }
    }

    /// Internal resistance (cell + wiring) at the battery's current
    /// temperature: Kirchhoff-equivalent-circuit series resistance for both
    /// the charge and discharge solves below.
    fn internal_resistance(&self) -> ElectricalResistance {
        let below_reference =
            (Self::RESISTANCE_REFERENCE_TEMP_C - self.temperature.get::<degree_celsius>()).max(0.);
        let temp_factor = 1. + Self::RESISTANCE_TEMP_COEFFICIENT_PER_C * below_reference;
        // Continuous aging: linearly interpolate the resistance multiplier
        // from 1x (new cell) to `AGED_RESISTANCE_MULTIPLIER`x by the
        // plugin-supplied `resistance_growth` magnitude, applied on top of
        // (not instead of) the existing cold-weather factor -- a cold, aged
        // cell is worse than either effect alone. The [0, 1] range is
        // enforced once, with a loud log, at the substrate boundary the
        // plugin writes this quantity through
        // (`fbw-xp-systems/src/invariants.rs::bound_for`'s
        // `DEGRADATION_KEYWORDS` entry) -- not re-clamped silently here.
        let aging_factor =
            1. + self.resistance_growth * (Self::AGED_RESISTANCE_MULTIPLIER - 1.);
        ElectricalResistance::new::<ohm>(
            (Self::CELL_INTERNAL_RESISTANCE_OHM_AT_20C + Self::WIRING_RESISTANCE_OHM)
                * temp_factor
                * aging_factor,
        )
    }

    /// Usable capacity this tick: the nameplate `RATED_CAPACITY_AMPERE_HOURS`
    /// shrunk by the plugin-supplied `capacity_fade` magnitude, scaled by
    /// `MAX_CAPACITY_FADE` so that even full-magnitude fade leaves a
    /// nonzero, physically-still-functioning pack (a design endpoint of
    /// this curve, not a clamp of the input). The magnitude's own [0, 1]
    /// range is enforced once, with a loud log, at the substrate boundary
    /// the plugin writes it through
    /// (`fbw-xp-systems/src/invariants.rs::bound_for`'s
    /// `DEGRADATION_KEYWORDS` entry) -- not re-clamped silently here.
    fn effective_capacity_ah(&self) -> f64 {
        Self::RATED_CAPACITY_AMPERE_HOURS * (1. - self.capacity_fade * Self::MAX_CAPACITY_FADE)
    }

    /// OCV(SOC) lookup that accounts for capacity fade: `calculate_output_
    /// potential_for_charge`'s polynomial/piecewise curve is fit against
    /// absolute stored Ah on a *fresh*, `RATED_CAPACITY_AMPERE_HOURS`-sized
    /// pack. A faded cell reaches the same state of charge (and therefore
    /// the same open-circuit voltage) at a proportionally lower absolute Ah,
    /// so the actual stored charge is rescaled into the fresh pack's own Ah
    /// domain (`charge * rated / effective`) before the curve is evaluated,
    /// and the result is clamped to the curve's own valid domain.
    fn ocv_for_charge(&self, charge: ElectricCharge) -> ElectricPotential {
        let rated = Self::RATED_CAPACITY_AMPERE_HOURS;
        let effective = self.effective_capacity_ah();
        let rescaled_ah = (charge.get::<ampere_hour>() * rated / effective).min(rated).max(0.);
        Battery::calculate_output_potential_for_charge(ElectricCharge::new::<ampere_hour>(
            rescaled_ah,
        ))
    }

    /// First-order energy balance: I^2R self-heating in, convective cooling
    /// to the ambient air out, plus (new) an overcharge gassing heat term,
    /// integrated over the tick with the thermal mass above.
    fn update_temperature(&mut self, context: &UpdateContext, resistance: ElectricalResistance) {
        let heating = self.current.get::<ampere>().powi(2) * resistance.get::<ohm>();
        let cooling = Self::COOLING_W_PER_KELVIN
            * (self.temperature.get::<degree_celsius>()
                - context.ambient_temperature().get::<degree_celsius>());
        let d_temp_dt = (heating + self.overcharge_heating_watt() - cooling)
            / Self::THERMAL_MASS_J_PER_KELVIN;
        self.temperature = ThermodynamicTemperature::new::<degree_celsius>(
            self.temperature.get::<degree_celsius>() + d_temp_dt * context.delta_as_secs_f64(),
        );
    }

    /// Overcharge (gassing-reaction) self-heating: once a Ni-Cd cell is
    /// pushed past its rated capacity while still being charged, the
    /// charging current no longer stores chemical energy -- it electrolyses
    /// the electrolyte (oxygen/hydrogen gassing), and essentially all of
    /// that electrical energy becomes heat directly, on top of ordinary
    /// I^2R. That heat also raises `self.temperature`, which (unlike the
    /// I^2R term) is not bounded by a fixed resistance, so a charger that
    /// keeps forcing current into an already-overcharged, already-hot cell
    /// gets a genuine positive-feedback thermal-runaway loop rather than an
    /// asymptotic steady state -- exactly the accepted Ni-Cd/Ni-MH failure
    /// mode (see e.g. Linden & Reddy, "Handbook of Batteries", Ni-Cd
    /// overcharge/thermal-runaway section). No FBW/A380-specific figure
    /// exists for the gassing reaction's own heat fraction or its thermal
    /// activation, so `GASSING_HEAT_FRACTION`/`RUNAWAY_ONSET_C`/
    /// `RUNAWAY_DOUBLING_C` are derived/typical (docs/physics/electrical.md),
    /// chosen so overcharge alone warms the pack gently while charging
    /// continues (matching the real world's slow, then accelerating,
    /// overcharge temperature rise) rather than trivially exploding.
    fn overcharge_heating_watt(&self) -> f64 {
        let overcharge_ratio = (self.charge.get::<ampere_hour>()
            / Self::RATED_CAPACITY_AMPERE_HOURS
            - 1.)
            .max(0.);
        if overcharge_ratio <= 0. || self.current.get::<ampere>() >= 0. {
            // `current` is negative while charging (this file's own sign
            // convention, see `consume_power`); no gassing heat while
            // discharging or once charge has fallen back to rated.
            return 0.;
        }
        // Essentially all of the overcharge current's own power becomes
        // heat via electrolysis rather than stored charge (derived/typical
        // GASSING_HEAT_FRACTION), scaled up by how far past rated capacity
        // the cell already is, and by an Arrhenius-like exponential in how
        // far the cell's own temperature already sits above the onset
        // point -- the positive-feedback mechanism that makes sustained
        // overcharge of an already-warm pack a runaway rather than a mild
        // warm-up.
        const GASSING_HEAT_FRACTION: f64 = 0.5;
        const RUNAWAY_ONSET_C: f64 = 45.;
        const RUNAWAY_DOUBLING_C: f64 = 10.;
        let overcharge_power = self.current.get::<ampere>().abs()
            * self.output_potential.get::<volt>()
            * GASSING_HEAT_FRACTION
            * overcharge_ratio.min(1.);
        let above_onset = (self.temperature.get::<degree_celsius>() - RUNAWAY_ONSET_C).max(0.);
        let runaway_multiplier = 2f64.powf(above_onset / RUNAWAY_DOUBLING_C);
        overcharge_power * runaway_multiplier
    }

    pub fn needs_charging(&self) -> bool {
        self.charge <= ElectricCharge::new::<ampere_hour>(Battery::RATED_CAPACITY_AMPERE_HOURS - 3.)
    }

    fn is_powered_by_other_potential(&self) -> bool {
        self.input_potential > self.output_potential
    }

    #[cfg(test)]
    fn charge(&self) -> ElectricCharge {
        self.charge
    }

    /// Function for testing purposes.
    fn set_charge(&mut self, charge: ElectricCharge) {
        self.charge = charge;
        self.input_potential = ElectricPotential::new::<volt>(0.);
        self.output_potential = self.ocv_for_charge(self.charge);
    }

    #[cfg(test)]
    pub(crate) fn set_full_charge(&mut self) {
        self.set_charge(ElectricCharge::new::<ampere_hour>(
            Battery::RATED_CAPACITY_AMPERE_HOURS,
        ))
    }

    #[cfg(test)]
    pub(crate) fn set_nearly_empty_battery_charge(&mut self) {
        self.set_charge(ElectricCharge::new::<ampere_hour>(1.))
    }

    /// Function for testing purposes.
    pub fn set_empty_battery_charge(&mut self) {
        self.set_charge(ElectricCharge::new::<ampere_hour>(0.))
    }

    fn calculate_output_potential_for_charge(charge: ElectricCharge) -> ElectricPotential {
        // There are four distinct charges, being:
        // 1. No charge, giving no potential.
        // 2. Low charge, rapidly decreasing from 26.578V.
        // 3. Regular charge, linear from 26.578V to 27.33V.
        // 4. High charge, rapidly increasing from 27.33V to 28.958V.
        // Refer to Battery.md for details.
        let charge = charge.get::<ampere_hour>();
        ElectricPotential::new::<volt>(if charge <= 0. {
            0.
        } else if charge <= 3.488 {
            (13.95303731988 * charge) - 2. * charge.powi(2)
        } else if charge < 22.449 {
            23.85 + 0.14 * charge
        } else {
            8483298.
                + (-2373273.312763873 * charge)
                + (276476.10619333945 * charge.powi(2))
                + (-17167.409762003314 * charge.powi(3))
                + (599.2597390001015 * charge.powi(4))
                + (-11.149802489333474 * charge.powi(5))
                + (0.08638809969727154 * charge.powi(6))
        })
    }

    /// Ohm's law across the real series resistance (cell + wiring) at the
    /// battery's current temperature, clamped to the physical 1C fast-charge
    /// limit real Ni-Cd chargers use (replacing the old flat, admittedly
    /// "fake", 10 A/0.15 ohm placeholder).
    fn calculate_charging_current(
        input: ElectricPotential,
        output: ElectricPotential,
        resistance: ElectricalResistance,
    ) -> ElectricCurrent {
        ((input - output) / resistance)
            .min(ElectricCurrent::new::<ampere>(
                Self::MAX_CHARGE_CURRENT_AMPERES,
            ))
            .max(ElectricCurrent::new::<ampere>(0.))
    }

    /// Terminal voltage under a constant-power discharge load: solves the
    /// Kirchhoff loop V = OCV - I*R with I = P/V, i.e.
    /// V^2 - OCV*V + P*R = 0, taking the physical root that approaches OCV
    /// as P -> 0. When the requested power exceeds what the equivalent
    /// circuit can deliver at any voltage (the classic maximum-power-transfer
    /// limit, P_max = OCV^2/(4R)) the source is power-limited rather than
    /// current-limited: this returns the max-power operating point
    /// (V = OCV/2) and the power actually delivered, instead of solving for
    /// an unreal negative discriminant.
    fn calculate_discharge(
        ocv: ElectricPotential,
        load: Power,
        resistance: ElectricalResistance,
    ) -> (ElectricPotential, Power) {
        let ocv_v = ocv.get::<volt>();
        let r = resistance.get::<ohm>();
        let p = load.get::<watt>();
        let discriminant = ocv_v * ocv_v - 4. * p * r;
        if discriminant < 0. {
            let v_max_power = ocv_v / 2.;
            let p_max = ocv_v * ocv_v / (4. * r);
            (
                ElectricPotential::new::<volt>(v_max_power),
                Power::new::<watt>(p_max),
            )
        } else {
            let v = (ocv_v + discriminant.sqrt()) / 2.;
            (ElectricPotential::new::<volt>(v), load)
        }
    }
}
impl ProvideCurrent for Battery {
    fn current(&self) -> ElectricCurrent {
        self.current
    }

    fn current_normal(&self) -> bool {
        (ElectricCurrent::new::<ampere>(-5.0)..=ElectricCurrent::new::<ampere>(f64::MAX))
            .contains(&self.current)
    }
}
impl ProvidePotential for Battery {
    fn potential(&self) -> ElectricPotential {
        self.output_potential.max(self.input_potential)
    }

    fn potential_normal(&self) -> bool {
        (ElectricPotential::new::<volt>(25.0)..=ElectricPotential::new::<volt>(31.0))
            .contains(&ProvidePotential::potential(self))
    }
}
impl ElectricalElement for Battery {
    fn input_identifier(&self) -> ElectricalElementIdentifier {
        self.identifier
    }

    fn output_identifier(&self) -> ElectricalElementIdentifier {
        self.identifier
    }

    fn is_conductive(&self) -> bool {
        true
    }
}
impl ElectricitySource for Battery {
    fn output_potential(&self) -> Potential {
        if self.output_potential > ElectricPotential::new::<volt>(0.) {
            Potential::new(PotentialOrigin::Battery(self.number), self.output_potential)
        } else {
            Potential::none()
        }
    }
}
impl SimulationElement for Battery {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.resistance_growth = reader.read(&self.resistance_growth_id);
        self.capacity_fade = reader.read(&self.capacity_fade_id);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        self.writer.write_direct(self, writer);
        writer.write(&self.temperature_id, self.temperature);
        writer.write(&self.internal_resistance_id, self.internal_resistance().get::<ohm>());
        writer.write(&self.polarization_id, (self.v1 + self.v2).get::<volt>());
        writer.write(&self.resistance_growth_id, self.resistance_growth);
        writer.write(&self.capacity_fade_id, self.capacity_fade);
    }

    fn consume_power<T: ConsumePower>(&mut self, context: &UpdateContext, consumption: &mut T) {
        self.input_potential = consumption.input_of(self).raw();

        if self.is_powered_by_other_potential() {
            let resistance = self.internal_resistance();
            self.current = Battery::calculate_charging_current(
                self.input_potential,
                self.output_potential,
                resistance,
            );

            let power = self.input_potential * self.current;
            consumption.consume_from_input(self, power);

            let time = Time::new::<second>(context.delta_as_secs_f64());
            self.charge += ((self.input_potential * self.current) * time) / self.input_potential;
            self.update_temperature(context, resistance);
            // Charging current flows into the battery, i.e. out of the
            // battery's own convention is negative.
            self.update_polarization(context.delta_as_secs_f64(), -self.current);
        }
    }

    fn process_power_consumption_report<T: PowerConsumptionReport>(
        &mut self,
        context: &UpdateContext,
        report: &T,
    ) {
        if !self.is_powered_by_other_potential() {
            // The open-circuit voltage is the Kirchhoff source behind the
            // internal resistance; `output_potential` below is instead the
            // *terminal* voltage under the present load, i.e. what the rest
            // of the circuit actually sees this tick.
            let ocv = self.ocv_for_charge(self.charge);
            let demand = report.total_consumption_of(PotentialOrigin::Battery(self.number));
            let resistance = self.internal_resistance();

            if ocv > ElectricPotential::new::<volt>(0.) && demand > Power::new::<watt>(0.) {
                let (terminal_potential, delivered) =
                    Battery::calculate_discharge(self.effective_ocv(ocv), demand, resistance);
                self.current = -(delivered / terminal_potential);

                let time = Time::new::<second>(context.delta_as_secs_f64());
                // Peukert's law: a heavy discharge draws down the cell's
                // usable capacity faster than a plain coulomb count would
                // predict (see `peukert_factor`).
                let peukert = Self::peukert_factor(self.current.get::<ampere>().abs());
                self.charge -=
                    (((delivered * time) / terminal_potential) * peukert).min(self.charge);
                self.output_potential = terminal_potential;
                self.update_polarization(context.delta_as_secs_f64(), -self.current);
            } else {
                self.current = ElectricCurrent::new::<ampere>(0.);
                self.output_potential = ocv;
                self.update_polarization(context.delta_as_secs_f64(), self.current);
            }
            // Always evolve the thermal state, even at rest: a battery with
            // no current still equilibrates towards ambient (cooling term
            // only, since I^2R heating is zero).
            self.update_temperature(context, resistance);
        } else {
            self.output_potential = self.effective_ocv(self.ocv_for_charge(self.charge));
        }

        // A cell this close to empty has a fraction-of-a-volt OCV whose
        // maximum power transfer is no longer meaningful (see
        // PRACTICALLY_EMPTY_AMPERE_HOURS); a real pack's BMS/low-voltage
        // protection would already have declared it dead, so snap to empty
        // instead of asymptotically decaying over simulated hours.
        if self.charge.get::<ampere_hour>() < Self::PRACTICALLY_EMPTY_AMPERE_HOURS {
            self.charge = ElectricCharge::new::<ampere_hour>(0.);
            self.output_potential = ElectricPotential::new::<volt>(0.);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(test)]
    mod battery_tests {
        use more_asserts::*;

        use super::*;
        use crate::simulation::test::ReadByName;
        use crate::simulation::InitContext;
        use crate::{
            electrical::{
                consumption::PowerConsumer, test::TestElectricitySource, Contactor, ElectricalBus,
                ElectricalBusType, Electricity,
            },
            simulation::{
                test::{SimulationTestBed, TestBed},
                Aircraft, SimulationElementVisitor, UpdateContext,
            },
        };
        use std::time::Duration;
        use uom::si::power::watt;

        struct BatteryTestBed {
            test_bed: SimulationTestBed<TestAircraft>,
        }
        impl BatteryTestBed {
            fn with_full_batteries() -> Self {
                Self {
                    test_bed: SimulationTestBed::new(|context| {
                        TestAircraft::new(
                            Battery::full(context, 1),
                            Battery::full(context, 2),
                            context,
                        )
                    }),
                }
            }

            fn with_half_charged_batteries() -> Self {
                Self {
                    test_bed: SimulationTestBed::new(|context| {
                        TestAircraft::new(
                            Battery::half(context, 1),
                            Battery::half(context, 2),
                            context,
                        )
                    }),
                }
            }

            fn with_nearly_empty_batteries() -> Self {
                Self {
                    test_bed: SimulationTestBed::new(|context| {
                        TestAircraft::new(
                            Battery::new(context, 1, ElectricCharge::new::<ampere_hour>(0.001)),
                            Battery::new(context, 2, ElectricCharge::new::<ampere_hour>(0.001)),
                            context,
                        )
                    }),
                }
            }

            fn with_nearly_empty_dissimilarly_charged_batteries() -> Self {
                Self {
                    test_bed: SimulationTestBed::new(|context| {
                        TestAircraft::new(
                            Battery::new(context, 1, ElectricCharge::new::<ampere_hour>(0.002)),
                            Battery::new(context, 2, ElectricCharge::new::<ampere_hour>(0.001)),
                            context,
                        )
                    }),
                }
            }

            fn with_full_and_empty_battery() -> Self {
                Self {
                    test_bed: SimulationTestBed::new(|context| {
                        TestAircraft::new(
                            Battery::full(context, 1),
                            Battery::empty(context, 2),
                            context,
                        )
                    }),
                }
            }

            fn with_empty_batteries() -> Self {
                Self {
                    test_bed: SimulationTestBed::new(|context| {
                        TestAircraft::new(
                            Battery::empty(context, 1),
                            Battery::empty(context, 2),
                            context,
                        )
                    }),
                }
            }

            fn current_is_normal(&mut self, number: usize) -> bool {
                self.read_by_name(&format!("ELEC_BAT_{}_CURRENT_NORMAL", number))
            }

            fn current(&mut self, number: usize) -> ElectricCurrent {
                self.read_by_name(&format!("ELEC_BAT_{}_CURRENT", number))
            }

            fn potential_is_normal(&mut self, number: usize) -> bool {
                self.read_by_name(&format!("ELEC_BAT_{}_POTENTIAL_NORMAL", number))
            }

            fn potential(&mut self, number: usize) -> ElectricPotential {
                self.read_by_name(&format!("ELEC_BAT_{}_POTENTIAL", number))
            }
        }
        impl TestBed for BatteryTestBed {
            type Aircraft = TestAircraft;

            fn test_bed(&self) -> &SimulationTestBed<TestAircraft> {
                &self.test_bed
            }

            fn test_bed_mut(&mut self) -> &mut SimulationTestBed<TestAircraft> {
                &mut self.test_bed
            }
        }

        struct TestAircraft {
            electricity_source: TestElectricitySource,
            bat_bus: ElectricalBus,
            battery_1: Battery,
            battery_1_contactor: Contactor,
            battery_2: Battery,
            battery_2_contactor: Contactor,
            consumer: PowerConsumer,
            battery_consumption: Power,
        }
        impl TestAircraft {
            fn new(battery_1: Battery, battery_2: Battery, context: &mut InitContext) -> Self {
                let mut aircraft = Self {
                    electricity_source: TestElectricitySource::unpowered(
                        context,
                        PotentialOrigin::TransformerRectifier(1),
                    ),
                    battery_1,
                    battery_2,
                    bat_bus: ElectricalBus::new(context, ElectricalBusType::DirectCurrentBattery),
                    battery_1_contactor: Contactor::new(context, "BAT1"),
                    battery_2_contactor: Contactor::new(context, "BAT2"),
                    consumer: PowerConsumer::from(ElectricalBusType::DirectCurrentBattery),
                    battery_consumption: Power::new::<watt>(0.),
                };

                aircraft.battery_1_contactor.close_when(true);

                aircraft
            }

            fn supply_input_potential(&mut self, potential: ElectricPotential) {
                self.electricity_source.set_potential(potential);
            }

            fn close_battery_2_contactor(&mut self) {
                self.battery_2_contactor.close_when(true);
            }

            fn power_demand(&mut self, power: Power) {
                self.consumer.demand(power);
            }

            fn battery_1_charge(&self) -> ElectricCharge {
                self.battery_1.charge()
            }

            fn battery_2_charge(&self) -> ElectricCharge {
                self.battery_2.charge()
            }

        }
        impl Aircraft for TestAircraft {
            fn update_before_power_distribution(
                &mut self,
                _: &UpdateContext,
                electricity: &mut Electricity,
            ) {
                electricity.supplied_by(&self.battery_1);
                electricity.supplied_by(&self.battery_2);
                electricity.flow(&self.battery_1, &self.battery_1_contactor);
                electricity.flow(&self.battery_2, &self.battery_2_contactor);

                electricity.supplied_by(&self.electricity_source);
                electricity.flow(&self.electricity_source, &self.bat_bus);
                electricity.flow(&self.battery_1_contactor, &self.bat_bus);
                electricity.flow(&self.battery_2_contactor, &self.bat_bus);
            }
        }
        impl SimulationElement for TestAircraft {
            fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
                self.bat_bus.accept(visitor);
                self.battery_1.accept(visitor);
                self.battery_1_contactor.accept(visitor);
                self.battery_2.accept(visitor);
                self.battery_2_contactor.accept(visitor);
                self.consumer.accept(visitor);

                visitor.visit(self);
            }

            fn process_power_consumption_report<T: PowerConsumptionReport>(
                &mut self,
                _: &UpdateContext,
                report: &T,
            ) {
                self.battery_consumption = report.total_consumption_of(PotentialOrigin::Battery(1));
            }
        }

        #[test]
        fn when_full_has_potential() {
            let mut test_bed = BatteryTestBed::with_full_batteries();

            test_bed.run();

            assert_gt!(test_bed.potential(1), ElectricPotential::new::<volt>(0.));
        }

        #[test]
        fn when_full_potential_is_normal() {
            let mut test_bed = BatteryTestBed::with_full_batteries();

            test_bed.run();

            assert!(test_bed.potential_is_normal(1));
        }

        #[test]
        fn when_empty_has_no_potential() {
            let mut test_bed = BatteryTestBed::with_empty_batteries();

            test_bed.run();

            assert_eq!(test_bed.potential(1), ElectricPotential::new::<volt>(0.));
        }

        #[test]
        fn when_empty_potential_is_abnormal() {
            let mut test_bed = BatteryTestBed::with_empty_batteries();

            test_bed.run();

            assert!(!test_bed.potential_is_normal(1));
        }

        #[test]
        fn when_input_potential_is_greater_than_output_potential_returns_input_potential_for_ecam_and_overhead_indication(
        ) {
            let mut test_bed = BatteryTestBed::with_half_charged_batteries();

            test_bed.run();

            let input_potential = ElectricPotential::new::<volt>(28.);
            assert!(test_bed.potential(1) < input_potential,
                "This test assumes the battery's potential is lower than the given input potential.");

            test_bed.command(|a| a.supply_input_potential(input_potential));

            test_bed.run();

            assert_eq!(test_bed.potential(1), input_potential);
        }

        #[test]
        fn when_input_potential_is_less_than_output_potential_returns_output_potential_for_ecam_and_overhead_indication(
        ) {
            let mut test_bed = BatteryTestBed::with_full_batteries();

            test_bed.run();

            let input_potential = ElectricPotential::new::<volt>(26.);
            assert!(input_potential < test_bed.potential(1),
                "This test assumes the battery's potential is higher than the given input potential.");

            test_bed.command(|a| a.supply_input_potential(input_potential));
            test_bed.run();

            assert_lt!(input_potential, test_bed.potential(1));
        }

        #[test]
        fn when_charging_current_is_normal() {
            let mut test_bed = BatteryTestBed::with_empty_batteries();

            test_bed.command(|a| a.supply_input_potential(ElectricPotential::new::<volt>(28.)));
            test_bed.run();

            assert!(test_bed.current_is_normal(1));
        }

        #[test]
        fn when_charging_battery_current_is_charge_current() {
            let mut test_bed = BatteryTestBed::with_half_charged_batteries();

            test_bed.command(|a| a.supply_input_potential(ElectricPotential::new::<volt>(28.)));
            test_bed.run();

            assert_gt!(test_bed.current(1), ElectricCurrent::new::<ampere>(0.));
        }

        #[test]
        fn when_discharging_slowly_current_is_normal() {
            let mut test_bed = BatteryTestBed::with_full_batteries();

            test_bed.command(|a| a.power_demand(Power::new::<watt>(40.)));
            test_bed.run();

            assert!(test_bed.current_is_normal(1));
        }

        #[test]
        fn when_discharging_quickly_current_is_abnormal() {
            let mut test_bed = BatteryTestBed::with_full_batteries();

            test_bed.command(|a| a.power_demand(Power::new::<watt>(500.)));
            test_bed.run();

            assert!(!test_bed.current_is_normal(1));
        }

        #[test]
        fn when_discharging_battery_current_is_discharge_current() {
            let mut test_bed = BatteryTestBed::with_full_batteries();

            test_bed.command(|a| a.power_demand(Power::new::<watt>(100.)));
            test_bed.run();

            assert!(test_bed.current(1) < ElectricCurrent::new::<ampere>(0.))
        }

        #[test]
        fn when_discharging_loses_charge() {
            let mut test_bed = BatteryTestBed::with_full_batteries();

            let charge_prior_to_run = test_bed.query(|a| a.battery_1_charge());

            test_bed.command(|a| a.power_demand(Power::new::<watt>(28. * 5.)));
            test_bed.run_with_delta(Duration::from_secs(60));

            assert_lt!(
                test_bed.query(|a| a.battery_1_charge()),
                charge_prior_to_run
            );
        }

        #[test]
        fn when_charging_gains_charge() {
            let mut test_bed = BatteryTestBed::with_empty_batteries();

            let charge_prior_to_run = test_bed.query(|a| a.battery_1_charge());

            test_bed.command(|a| a.supply_input_potential(ElectricPotential::new::<volt>(28.)));
            test_bed.run_with_delta(Duration::from_secs(60));

            assert_gt!(
                test_bed.query(|a| a.battery_1_charge()),
                charge_prior_to_run
            );
        }

        #[test]
        fn can_charge_beyond_rated_capacity() {
            let mut test_bed = BatteryTestBed::with_full_batteries();

            let charge_prior_to_run = test_bed.query(|a| a.battery_1_charge());

            test_bed.command(|a| a.supply_input_potential(ElectricPotential::new::<volt>(28.)));
            test_bed.run_with_delta(Duration::from_secs(1_000));

            assert_gt!(
                test_bed.query(|a| a.battery_1_charge()),
                charge_prior_to_run
            );
        }

        #[test]
        fn does_not_charge_when_input_potential_lower_than_battery_potential() {
            let mut test_bed = BatteryTestBed::with_half_charged_batteries();

            let charge_prior_to_run = test_bed.query(|a| a.battery_1_charge());

            test_bed.command(|a| a.supply_input_potential(ElectricPotential::new::<volt>(10.)));
            test_bed.run_with_delta(Duration::from_secs(1_000));

            assert_eq!(
                test_bed.query(|a| a.battery_1_charge()),
                charge_prior_to_run
            );
        }

        #[test]
        fn when_neither_charging_nor_discharging_charge_remains_equal() {
            let mut test_bed = BatteryTestBed::with_half_charged_batteries();

            let charge_prior_to_run = test_bed.query(|a| a.battery_1_charge());

            test_bed.run_with_delta(Duration::from_secs(1_000));

            assert_eq!(
                test_bed.query(|a| a.battery_1_charge()),
                charge_prior_to_run
            );
        }

        #[test]
        fn when_neither_charging_nor_discharging_current_is_zero() {
            let mut test_bed = BatteryTestBed::with_half_charged_batteries();

            test_bed.run_with_delta(Duration::from_secs(1_000));

            assert_eq!(test_bed.current(1), ElectricCurrent::new::<ampere>(0.));
        }

        #[test]
        fn cannot_discharge_below_zero() {
            let mut test_bed = BatteryTestBed::with_nearly_empty_batteries();

            test_bed.command(|a| a.power_demand(Power::new::<watt>(5000.)));
            test_bed.run_with_delta(Duration::from_secs(50));

            assert_eq!(
                test_bed.query(|a| a.battery_1_charge()),
                ElectricCharge::new::<ampere_hour>(0.)
            );
        }

        #[test]
        fn dissimilar_charged_batteries_in_parallel_deplete() {
            // With a real series resistance, two nearly-empty, dissimilarly
            // charged cells in parallel first equalise with each other (the
            // fuller one both feeds the load and trickle-charges the
            // weaker one, exactly as `batteries_charge_each_other...` shows
            // for two large cells) before the pair's tiny combined OCV
            // self-limits how much of the requested 10 W they can actually
            // deliver (P_max = OCV^2/(4R)). This is a real, and slower,
            // failure mode than the old model's unbounded low-voltage
            // current, which used to drain both to zero in 15 simulated
            // seconds; this test now checks the physically meaningful
            // properties instead of that arbitrary timing.
            let mut test_bed = BatteryTestBed::with_nearly_empty_dissimilarly_charged_batteries();
            let original_total = test_bed.query(|a| a.battery_1_charge())
                + test_bed.query(|a| a.battery_2_charge());

            test_bed.command(|a| a.power_demand(Power::new::<watt>(10.)));
            test_bed.command(|a| a.close_battery_2_contactor());

            for _ in 0..180 {
                test_bed.run_with_delta(Duration::from_secs(1));
            }

            let charge_1 = test_bed.query(|a| a.battery_1_charge());
            let charge_2 = test_bed.query(|a| a.battery_2_charge());
            // No free energy: the pair never ends up with more charge than
            // they, plus the (negligible at this scale) load, started with.
            assert!(charge_1 + charge_2 <= original_total + ElectricCharge::new::<ampere_hour>(0.0001));
            // They converge towards each other rather than diverging.
            assert!((charge_1 - charge_2).abs() < ElectricCharge::new::<ampere_hour>(0.001));
        }

        #[test]
        fn batteries_charge_each_other_until_relatively_equal_charge() {
            let mut test_bed = BatteryTestBed::with_full_and_empty_battery();

            let original_charge = test_bed.query(|a| a.battery_1_charge());

            test_bed.command(|a| a.close_battery_2_contactor());

            for _ in 0..100 {
                test_bed.run_with_delta(Duration::from_secs(120));
            }

            // Charging through a real series resistance (internal + wiring)
            // dissipates some energy as I^2R heat, so the transfer is no
            // longer loss-free: the combined charge can drop below the
            // original, but conservation of energy still forbids it rising
            // above it (no free charge created).
            assert!(
                (test_bed.query(|a| a.battery_1_charge())
                    - test_bed.query(|a| a.battery_2_charge()))
                .abs()
                    < ElectricCharge::new::<ampere_hour>(0.1)
            );
            let combined = test_bed.query(|a| a.battery_1_charge())
                + test_bed.query(|a| a.battery_2_charge());
            assert!(combined <= original_charge + ElectricCharge::new::<ampere_hour>(0.001));
            assert!(combined > original_charge - ElectricCharge::new::<ampere_hour>(2.));
        }

        #[test]
        fn internal_resistance_is_higher_in_the_cold() {
            let mut test_bed = BatteryTestBed::with_full_batteries();
            test_bed.set_ambient_temperature(ThermodynamicTemperature::new::<degree_celsius>(20.));
            test_bed.run_with_delta(Duration::from_secs(120));
            let warm_resistance: f64 =
                test_bed.read_by_name("ELEC_BAT_1_INTERNAL_RESISTANCE");

            let mut cold_test_bed = BatteryTestBed::with_full_batteries();
            cold_test_bed
                .set_ambient_temperature(ThermodynamicTemperature::new::<degree_celsius>(-40.));
            cold_test_bed.run_with_delta(Duration::from_secs(120));
            let cold_resistance: f64 =
                cold_test_bed.read_by_name("ELEC_BAT_1_INTERNAL_RESISTANCE");

            assert_gt!(cold_resistance, warm_resistance);
        }

        #[test]
        fn discharge_terminal_voltage_sags_with_load_like_a_real_cell() {
            // Kirchhoff's loop (V = OCV - I*R) means a heavier constant-power
            // load must sag the terminal voltage further below the
            // open-circuit voltage than a light one. Two ticks: the
            // published potential is `max(input, output)` for ECAM/overhead
            // indication purposes, and on the very first tick `input`
            // still holds the constructor's unloaded OCV.
            let mut light_load = BatteryTestBed::with_full_batteries();
            light_load.command(|a| a.power_demand(Power::new::<watt>(20.)));
            light_load.run();
            light_load.run();
            let light_load_potential = light_load.potential(1);

            let mut heavy_load = BatteryTestBed::with_full_batteries();
            heavy_load.command(|a| a.power_demand(Power::new::<watt>(400.)));
            heavy_load.run();
            heavy_load.run();
            let heavy_load_potential = heavy_load.potential(1);

            assert_lt!(heavy_load_potential, light_load_potential);
        }

        #[test]
        fn an_overload_beyond_maximum_power_transfer_is_power_limited_not_infinite() {
            // A fixed-resistance source can never deliver more than
            // OCV^2/(4R) (the maximum power transfer theorem); demanding
            // more than that must not produce runaway/NaN current.
            let mut test_bed = BatteryTestBed::with_full_batteries();
            test_bed.command(|a| a.power_demand(Power::new::<watt>(1_000_000.)));
            test_bed.run();

            let current = test_bed.current(1).get::<ampere>();
            assert!(current.is_finite());
            // With OCV ~29 V and R ~0.03 ohm, P_max ~ 29^2/(4*0.03) ~ 7 kW,
            // i.e. well under 100 kA; generous bound just guards against a
            // regression back to an unbounded division.
            assert_lt!(current.abs(), 100_000.);
        }
    }
}

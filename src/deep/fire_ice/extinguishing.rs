//! Fire extinguishing: Halon 1301 bottles and their squibs and
//! distribution piping, zone agent-concentration decay against
//! ventilation, cross-feed between bottle pairs, cargo-compartment optical
//! smoke detection and two-stage (high-rate knockdown, then metered)
//! suppression, and lavatory smoke detection with a self-contained
//! fusible-link extinguisher.
//!
//! ## Sources
//! - Halon 1301 (bromotrifluoromethane, CBrF3) is stored as a liquefied gas
//!   super-pressurized with dry nitrogen so it discharges as a fine mist
//!   rather than boiling off slowly: this is the standard total-flooding
//!   fire-suppression bottle design, NFPA 12A *Standard on Halon 1301 Fire
//!   Extinguishing Systems* (public standard); a super-pressurized bottle's
//!   internal pressure rises with temperature at essentially constant
//!   volume (Amontons' law for the nitrogen headspace), the same public
//!   figures (~360 psi at 21 C / ~600 psi super-pressurized variants) NFPA
//!   12A's own pressure-vs-temperature charts show -- GENERIC here as
//!   "~=360 psi at 21 C", the lower of the two common charge pressures, not
//!   an A380-specific number.
//! - Design (minimum) total-flooding concentration for Halon 1301 against
//!   a Class B (flammable liquid) fire: 5% v/v, NFPA 12A Table 1 (public,
//!   the most commonly cited figure for aircraft engine/APU/cargo
//!   applications). Halon 1301 molar mass 148.9 g/mol (public, from its
//!   CBrF3 formula).
//! - Extended-duration cargo compartment suppression (initial high-rate
//!   knockdown, then a slower metered discharge to hold the design
//!   concentration against compartment leakage) sized for the diversion
//!   time after a cargo fire warning: 14 CFR/EASA CS-25.858 requires
//!   suppression effective for the time needed to land and evacuate after
//!   an indication, up to 195 minutes for extended-range operations away
//!   from an adequate airport (public FAA/EASA regulation) -- the reason
//!   large, long-range aircraft carry a metered/extended system rather
//!   than a single knockdown-only charge.
//! - Cargo smoke detector: photoelectric (light-scatter/obscuration)
//!   optical technology, alarm sensitivity in the 0.5-4%/ft obscuration
//!   range (UL 217 *Standard for Smoke Alarms*, public); this module uses
//!   2%/ft, the middle of that public range, GENERIC. Specific extinction
//!   coefficient of flaming-combustion smoke ~=8.7 m^2/g (Mulholland, G.W.,
//!   in the SFPE *Handbook of Fire Protection Engineering*, a widely cited
//!   public research figure for smoke optical density per unit soot mass).
//!   Soot yield for kerosene/Jet-A-class combustion ~=0.05 kg soot per kg
//!   fuel burned (public SFPE handbook order-of-magnitude figure for
//!   under-ventilated hydrocarbon flames) -- GENERIC.
//! - Lavatory smoke detector and its automatic fire extinguisher: 14 CFR/
//!   EASA CS-25.854 mandates a smoke detector and an automatic (not
//!   crew-actuated) extinguisher in each lavatory trash receptacle,
//!   triggered by a fusible thermal link (public regulation; the fusible
//!   link itself is a standard passive thermal fuse, melting once and not
//!   resettable, at a typical rating around 77 C / 170 F -- GENERIC,
//!   representative of common fusible-link fire-suppression hardware
//!   ratings, not an A380-specific figure).

use super::util::{clamp01, orifice_mass_flow_kg_s, PSI_TO_PA};

// ---------------------------------------------------------------------------
// Bottles, squibs, distribution
// ---------------------------------------------------------------------------

/// Halon 1301 molar mass, kg/mol (public, from CBrF3).
const HALON_MOLAR_MASS_KG_MOL: f64 = 0.1489;
/// Universal gas constant, J/(mol K) (CODATA).
const R_UNIVERSAL: f64 = 8.314462618;
/// GENERIC charge pressure at 21 C (module doc): the lower of the two
/// common Halon 1301 super-pressurization charges.
const CHARGE_PRESSURE_21C_PA: f64 = 360.0 * PSI_TO_PA;
const CHARGE_REF_TEMP_K: f64 = 294.15; // 21 C
/// NFPA 12A Table 1 minimum design concentration for a Class B fire,
/// volume fraction (module doc).
pub const DESIGN_CONCENTRATION_VOLUME_FRACTION: f64 = 0.05;

/// Fractional faults on one bottle/squib, 0 healthy .. 1 fully failed.
#[derive(Clone, Copy, Debug, Default)]
pub struct BottleFaults {
    /// A small continuous leak from the bottle body/valve seat: fraction of
    /// [`Bottle::LEAK_AREA_MAX_M2`].
    pub leak: f64,
    /// The pyrotechnic squib fails to fully rupture its discharge disc:
    /// fraction reduction of the achieved discharge orifice area (1.0 = no
    /// discharge at all even when fired).
    pub squib_failure: f64,
}

pub struct Bottle {
    /// Agent (liquid Halon) mass remaining, kg.
    agent_mass_kg: f64,
    design_charge_kg: f64,
    /// Current stored pressure, Pa.
    pressure_pa: f64,
    volume_m3: f64,
    discharged: bool,
}

/// A leak orifice area representative of a slowly failing bottle valve
/// seat/body seal at full-severity fault, m^2. **GENERIC**: no public
/// per-bottle figure exists; sized small enough that a full-severity leak
/// empties a bottle over roughly 9-10 hours, not seconds -- consistent
/// with a maintenance-detectable slow leak rather than an instantaneous
/// rupture.
const LEAK_AREA_MAX_M2: f64 = 4.0e-8;
const LEAK_DISCHARGE_COEFFICIENT: f64 = 0.62;

/// The squib+distribution-pipe combined discharge orifice area at full
/// severity, m^2. **GENERIC**: sized so a full bottle empties in a few
/// seconds through its distribution pipe into the zone, matching the
/// "one-second" order-of-magnitude discharge timescale total-flooding
/// bottles are designed around (NFPA 12A's own discharge-time guidance for
/// engine-nacelle applications, public).
const DISCHARGE_AREA_MAX_M2: f64 = 8.0e-5;

/// Pressure switch threshold below which the bottle is flagged low
/// (a real, commonly fitted feature -- public halon-system description,
/// e.g. any transport-aircraft fire-protection system training text):
/// GENERIC, 80% of the nominal 21 C charge pressure.
pub fn low_pressure_threshold_pa() -> f64 {
    CHARGE_PRESSURE_21C_PA * 0.8
}

impl Bottle {
    pub fn new(design_charge_kg: f64, volume_m3: f64) -> Self {
        Self { agent_mass_kg: design_charge_kg, design_charge_kg, pressure_pa: CHARGE_PRESSURE_21C_PA, volume_m3, discharged: false }
    }

    pub fn agent_mass_kg(&self) -> f64 {
        self.agent_mass_kg
    }
    pub fn pressure_pa(&self) -> f64 {
        self.pressure_pa
    }
    pub fn is_discharged(&self) -> bool {
        self.discharged
    }
    pub fn is_low_pressure(&self) -> bool {
        self.pressure_pa < low_pressure_threshold_pa()
    }

    /// Fraction of design charge below which the bottle is treated as
    /// "mostly vapour, little liquid left": below this point pressure
    /// falls off with remaining charge; above it, pressure holds at the
    /// full super-pressurized value regardless of fill level. **GENERIC**
    /// figure, but the qualitative behaviour is real and public: a
    /// two-phase liquid-Halon/super-pressurizing-nitrogen bottle's
    /// pressure is set by liquid/vapour equilibrium at the ambient
    /// temperature and stays essentially flat with fill level while any
    /// liquid remains (the same reason NFPA 12A's pressure-vs-temperature
    /// bottle charts are indexed by temperature alone, not by fill level)
    /// -- it only collapses once the liquid is exhausted and the
    /// remaining nitrogen itself must expand to fill the bottle.
    const RESIDUAL_LIQUID_FRACTION: f64 = 0.05;

    /// Pressure-vs-temperature: Amontons' law for the nitrogen
    /// super-pressurization headspace at essentially constant volume
    /// (module doc) while liquid agent remains; once the charge falls
    /// below [`Self::RESIDUAL_LIQUID_FRACTION`] (liquid exhausted, only
    /// vapour left) pressure instead falls off proportionally with the
    /// remaining charge, the free-expansion regime of a nearly empty
    /// bottle.
    fn retarget_pressure(&mut self, ambient_c: f64) {
        let charge_fraction = (self.agent_mass_kg / self.design_charge_kg.max(1e-6)).clamp(0.0, 1.0);
        let temp_k = (ambient_c + 273.15).max(1.0);
        let full_pressure = CHARGE_PRESSURE_21C_PA * (temp_k / CHARGE_REF_TEMP_K);
        self.pressure_pa = if charge_fraction > Self::RESIDUAL_LIQUID_FRACTION {
            full_pressure
        } else {
            full_pressure * (charge_fraction / Self::RESIDUAL_LIQUID_FRACTION)
        };
    }

    /// One tick. `fire_command` is the squib fire signal (fire pushbutton
    /// released + agent pushbutton pressed, or an automatic system such as
    /// APU-on-ground). `zone_pressure_pa` is what the bottle discharges
    /// against. Returns the mass flow delivered into the zone this tick,
    /// kg/s.
    pub fn step(&mut self, ambient_c: f64, fire_command: bool, zone_pressure_pa: f64, faults: &BottleFaults, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);

        // Slow continuous leak, independent of fire command: the bottle
        // vents to ambient (assumed sea-level for the leak's downstream
        // side, a fitting in the bay) through a tiny orifice.
        let leak_area = LEAK_AREA_MAX_M2 * clamp01(faults.leak);
        let leak_kg_s = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, leak_area, self.pressure_pa, (ambient_c + 273.15).max(1.0), 101_325.0);
        self.agent_mass_kg = (self.agent_mass_kg - leak_kg_s * dt).max(0.0);

        // A pyrotechnic squib ruptures a burst disc: a one-shot, one-way
        // event. Once fired (and the disc actually ruptures -- a fully
        // failed squib never opens it at all), the valve cannot reclose,
        // so the bottle keeps draining under its own pressure on every
        // subsequent tick regardless of whether `fire_command` is still
        // asserted, until it is genuinely empty. This is why real systems
        // describe the bottle as "discharged" the instant the squib fires,
        // not only once it happens to reach zero mass.
        if fire_command && !self.discharged && self.agent_mass_kg > 0.0 && clamp01(faults.squib_failure) < 1.0 {
            self.discharged = true;
        }
        let discharge_kg_s = if self.discharged && self.agent_mass_kg > 0.0 {
            let effective_area = DISCHARGE_AREA_MAX_M2 * (1.0 - clamp01(faults.squib_failure));
            let flow = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, effective_area, self.pressure_pa, (ambient_c + 273.15).max(1.0), zone_pressure_pa);
            let delivered = flow.min(self.agent_mass_kg / dt.max(1e-6));
            self.agent_mass_kg = (self.agent_mass_kg - delivered * dt).max(0.0);
            delivered
        } else {
            0.0
        };

        self.retarget_pressure(ambient_c);
        discharge_kg_s
    }
}

/// One zone's extinguishing-agent concentration, tracking discharge inflow
/// against ventilation washout: `dC/dt = m_dot_agent/(rho_zone_air*V) -
/// (vent_m3_s/V)*C`, the standard first-order gas-dispersion/washout
/// equation used in clean-agent total-flooding design guides (e.g. NFPA
/// 12A's own room-integrity/concentration-hold-time treatment, public).
pub struct ZoneConcentration {
    volume_fraction: f64,
    volume_m3: f64,
}

impl ZoneConcentration {
    pub fn new(volume_m3: f64) -> Self {
        Self { volume_fraction: 0.0, volume_m3 }
    }

    pub fn volume_fraction(&self) -> f64 {
        self.volume_fraction
    }

    /// Fraction of [`DESIGN_CONCENTRATION_VOLUME_FRACTION`] currently
    /// present, clamped to 1 -- the value `combustion.rs`'s
    /// `ZoneSupply::suppression_fraction` expects.
    pub fn suppression_fraction(&self) -> f64 {
        clamp01(self.volume_fraction / DESIGN_CONCENTRATION_VOLUME_FRACTION)
    }

    pub fn step(&mut self, agent_inflow_kg_s: f64, ambient_c: f64, ambient_pressure_pa: f64, ventilation_m3_s: f64, dt_s: f64) {
        let dt = dt_s.max(0.0);
        let temp_k = (ambient_c + 273.15).max(1.0);
        // Total gas moles in the zone (ideal gas, ignoring the still-small
        // agent partial pressure at these low design concentrations).
        let total_moles = ambient_pressure_pa.max(1000.0) * self.volume_m3 / (R_UNIVERSAL * temp_k);
        let agent_moles = self.volume_fraction * total_moles;
        let inflow_moles_s = agent_inflow_kg_s.max(0.0) / HALON_MOLAR_MASS_KG_MOL;
        let washout_moles_s = (ventilation_m3_s.max(0.0) / self.volume_m3.max(1e-6)) * agent_moles;
        let new_moles = (agent_moles + (inflow_moles_s - washout_moles_s) * dt).max(0.0);
        self.volume_fraction = clamp01(new_moles / total_moles.max(1e-6));
    }
}

/// Cross-feed: a valve joining two zones' bottle manifolds so a healthy
/// bottle pair can serve a neighbouring zone whose own bottles are spent
/// or failed -- the real redundancy purpose of the cross-feed line fitted
/// between adjacent engine fire-extinguishing bottle groups on multi-
/// engine transports (public fire-protection-system description, e.g. any
/// type-training airframe fire-protection chapter).
#[derive(Clone, Copy, Debug, Default)]
pub struct CrossFeed {
    pub valve_open: bool,
}

impl CrossFeed {
    /// Split a discharge request across the two zones' own bottle
    /// capability plus, if open, the cross-feed path: returns
    /// `(deliver_from_own_zone, deliver_via_cross_feed)` as booleans for
    /// the caller's own bottle `step` calls.
    pub fn routing(&self, own_zone_bottles_available: bool, other_zone_bottles_available: bool) -> (bool, bool) {
        if own_zone_bottles_available {
            (true, false)
        } else {
            (false, self.valve_open && other_zone_bottles_available)
        }
    }
}

// ---------------------------------------------------------------------------
// Cargo optical smoke detection
// ---------------------------------------------------------------------------

/// GENERIC specific extinction coefficient of flaming-combustion smoke,
/// m^2/kg (module doc, Mulholland ~=8.7 m^2/g = 8700 m^2/kg).
const SMOKE_SPECIFIC_EXTINCTION_M2_KG: f64 = 8700.0;
/// GENERIC soot yield, kg soot per kg fuel burned (module doc).
const SOOT_YIELD_KG_PER_KG_FUEL: f64 = 0.05;
/// GENERIC alarm threshold, obscuration fraction per metre of light path
/// (module doc: 2%/ft = 0.02/0.3048 m).
const ALARM_OBSCURATION_PER_M: f64 = 0.02 / 0.3048;

/// Fractional fault on one optical smoke detector, 0 healthy .. 1 fully
/// failed.
#[derive(Clone, Copy, Debug, Default)]
pub struct SmokeDetectorFaults {
    /// The optical chamber/lens is dirty or physically obscured (dust,
    /// cargo residue): a real, commonly logged smoke-detector maintenance
    /// fault. Modelled as a loss of effective sensitivity -- the detector
    /// needs proportionally more real smoke to produce the same measured
    /// obscuration change, delaying or (at full severity) preventing
    /// alarm entirely, rather than a scripted "detector disabled" flag.
    pub lens_obscured: f64,
}

pub struct OpticalSmokeDetector {
    path_length_m: f64,
    smoke_density_kg_m3: f64,
    volume_m3: f64,
}

impl OpticalSmokeDetector {
    pub fn new(path_length_m: f64, volume_m3: f64) -> Self {
        Self { path_length_m, smoke_density_kg_m3: 0.0, volume_m3 }
    }

    pub fn smoke_density_kg_m3(&self) -> f64 {
        self.smoke_density_kg_m3
    }

    /// One tick: soot mass balance (yield from `burn_rate_kg_s`, washed out
    /// by `ventilation_m3_s`) then Beer-Lambert obscuration over the
    /// detector's actual (sensitivity-derated by `faults.lens_obscured`)
    /// optical path, `1 - exp(-k*rho*L)`.
    pub fn step(&mut self, burn_rate_kg_s: f64, ventilation_m3_s: f64, faults: &SmokeDetectorFaults, dt_s: f64) -> bool {
        let dt = dt_s.max(0.0);
        let soot_in_kg_s = burn_rate_kg_s.max(0.0) * SOOT_YIELD_KG_PER_KG_FUEL;
        let washout_per_s = ventilation_m3_s.max(0.0) / self.volume_m3.max(1e-6);
        let mass_kg = self.smoke_density_kg_m3 * self.volume_m3;
        let new_mass = (mass_kg + (soot_in_kg_s - washout_per_s * mass_kg) * dt).max(0.0);
        self.smoke_density_kg_m3 = new_mass / self.volume_m3.max(1e-6);

        // Total obscuration over this detector's actual optical path
        // (Beer-Lambert), compared against the public %/ft alarm rating
        // compounded over that same path length (`(1-x)^L`, the standard
        // way to scale a per-unit-length obscuration rate to an actual
        // path -- valid for any path length, not just a small-optical-
        // depth approximation). A dirty/obscured chamber effectively sees
        // less contrast from the same real smoke density.
        let effective_density = self.smoke_density_kg_m3 * (1.0 - clamp01(faults.lens_obscured));
        let total_obscuration = 1.0 - (-SMOKE_SPECIFIC_EXTINCTION_M2_KG * effective_density * self.path_length_m).exp();
        let alarm_threshold_total = 1.0 - (1.0 - ALARM_OBSCURATION_PER_M).powf(self.path_length_m);
        total_obscuration >= alarm_threshold_total
    }
}

/// Fractional faults on one hold's cargo suppression system, 0 healthy ..
/// 1 fully failed (ECAM completeness pass, E-FIRE §F).
///
/// `knockdown_squib_fault`/`extended_squib_fault` split what used to be a
/// single `squib_failure` shared by both discharge stages: the general
/// two-bottle cargo fire-suppression architecture (US Patent 9,248,326/
/// 8,925,642, "Scalable cargo fire-suppression agent distribution system")
/// has bottle 1 discharge rapidly for initial knockdown, and bottle 2 be
/// timed to discharge later for extended suppression as concentration
/// decays -- exactly the two-stage shape [`CargoSuppressionSystem::step`]
/// already implements internally, but until this pass gated with one
/// fault covering both stages. Splitting it makes the existing physics
/// correspond to two physically distinct, independently-faultable
/// initiators, matching real hardware (two bottles, two squibs).
///
/// `distribution_fault` is new: the line/valve between the bottle manifold
/// and this hold, independent of the bottle/squib itself -- gates
/// `effective_area` the same way a squib fault already does.
#[derive(Clone, Copy, Debug, Default)]
pub struct CargoSuppressionFaults {
    pub leak: f64,
    pub knockdown_squib_fault: f64,
    pub extended_squib_fault: f64,
    pub distribution_fault: f64,
}

/// Two-stage cargo suppression: an initial high-rate "knockdown" discharge
/// to reach design concentration quickly, then a flow-restricted "metered"
/// discharge whose small orifice makes the remaining agent last far
/// longer, offsetting compartment leakage for the extended diversion time
/// CS-25.858 requires (module doc). The metered duration is not a
/// scripted timer: it falls out of remaining agent mass divided by the
/// metered flow rate, exactly like every other mass-balance quantity in
/// this crate.
pub struct CargoSuppressionSystem {
    pub bottle: Bottle,
    knockdown_area_m2: f64,
    metered_area_m2: f64,
    knocked_down: bool,
}

impl CargoSuppressionSystem {
    /// GENERIC: the metered stage's orifice area, sized two orders of
    /// magnitude below the knockdown discharge area so a bottle sized for
    /// a fast few-second knockdown instead metres out over hours once
    /// switched -- the qualitative two-rate behaviour CS-25.858-compliant
    /// systems use, not a literal type-certified flow schedule.
    const METERED_AREA_FRACTION: f64 = 0.01;

    pub fn new(design_charge_kg: f64, volume_m3: f64) -> Self {
        Self { bottle: Bottle::new(design_charge_kg, volume_m3), knockdown_area_m2: DISCHARGE_AREA_MAX_M2, metered_area_m2: DISCHARGE_AREA_MAX_M2 * Self::METERED_AREA_FRACTION, knocked_down: false }
    }

    /// One tick. Switches from knockdown to metered once the zone reaches
    /// design concentration (a real, output-driven switch, not a fixed
    /// timer). `knockdown_squib_fault` gates only the knockdown stage,
    /// `extended_squib_fault` only the metered stage -- the same
    /// independence a real second bottle/squib would have; `distribution_
    /// fault` gates both, the same way each stage's own squib fault does,
    /// since a distribution-path fault sits downstream of either bottle.
    pub fn step(&mut self, ambient_c: f64, fire_command: bool, zone_pressure_pa: f64, zone_concentration: &ZoneConcentration, faults: &CargoSuppressionFaults, dt_s: f64) -> f64 {
        if fire_command && zone_concentration.suppression_fraction() >= 1.0 {
            self.knocked_down = true;
        }
        let (area, stage_squib_fault) = if self.knocked_down { (self.metered_area_m2, faults.extended_squib_fault) } else { (self.knockdown_area_m2, faults.knockdown_squib_fault) };
        let effective_area = area * (1.0 - clamp01(stage_squib_fault)) * (1.0 - clamp01(faults.distribution_fault));
        let dt = dt_s.max(0.0);
        let leak_area = LEAK_AREA_MAX_M2 * clamp01(faults.leak);
        let leak_kg_s = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, leak_area, self.bottle.pressure_pa, (ambient_c + 273.15).max(1.0), 101_325.0);
        self.bottle.agent_mass_kg = (self.bottle.agent_mass_kg - leak_kg_s * dt).max(0.0);

        let discharge_kg_s = if fire_command && self.bottle.agent_mass_kg > 0.0 {
            let flow = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, effective_area, self.bottle.pressure_pa, (ambient_c + 273.15).max(1.0), zone_pressure_pa);
            let delivered = flow.min(self.bottle.agent_mass_kg / dt.max(1e-6));
            self.bottle.agent_mass_kg = (self.bottle.agent_mass_kg - delivered * dt).max(0.0);
            delivered
        } else {
            0.0
        };
        self.bottle.discharged = self.bottle.agent_mass_kg <= 1e-6;
        self.bottle.retarget_pressure(ambient_c);
        discharge_kg_s
    }

    pub fn is_metering(&self) -> bool {
        self.knocked_down
    }
}

// ---------------------------------------------------------------------------
// Lavatory smoke detection and automatic fusible-link extinguisher
// ---------------------------------------------------------------------------

/// GENERIC fusible-link rating, deg C (module doc: ~77 C/170 F, a common
/// passive-thermal-fuse rating).
pub const FUSIBLE_LINK_MELT_C: f64 = 77.0;
/// GENERIC: how much higher an aged/corroded fusible link's actual melt
/// point can drift at full-severity degradation, deg C -- a real,
/// documented aging mechanism for thermal fuses (corrosion/oxidation of
/// the fusible alloy raises its effective melting behaviour), delaying
/// (not preventing outright) an automatic discharge.
const FUSIBLE_LINK_DEGRADED_MARGIN_C: f64 = 50.0;

/// Fractional fault on the lavatory's fusible-link extinguisher, 0 healthy
/// .. 1 fully failed.
#[derive(Clone, Copy, Debug, Default)]
pub struct LavatoryFaults {
    /// Aged/corroded fusible link: raises the temperature actually needed
    /// to trigger discharge (module doc), up to
    /// [`FUSIBLE_LINK_DEGRADED_MARGIN_C`] above the design rating.
    pub link_degraded: f64,
}

pub struct LavatoryProtection {
    pub smoke_detector: OpticalSmokeDetector,
    link_melted: bool,
}

impl LavatoryProtection {
    pub fn new(volume_m3: f64) -> Self {
        Self { smoke_detector: OpticalSmokeDetector::new(0.3, volume_m3), link_melted: false }
    }

    pub fn is_discharged(&self) -> bool {
        self.link_melted
    }

    /// One tick: the fusible link is a passive, irreversible thermal fuse
    /// (once melted it stays melted, exactly like `Bottle`'s own "can't be
    /// recharged" behaviour) -- it does not need power or crew action, the
    /// real point of 14 CFR/EASA CS-25.854's requirement for an *automatic*
    /// trash-bin extinguisher.
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, local_temp_c: f64, burn_rate_kg_s: f64, ventilation_m3_s: f64, smoke_faults: &SmokeDetectorFaults, link_faults: &LavatoryFaults, dt_s: f64) -> bool {
        let smoke_alarm = self.smoke_detector.step(burn_rate_kg_s, ventilation_m3_s, smoke_faults, dt_s);
        let melt_c = FUSIBLE_LINK_MELT_C + FUSIBLE_LINK_DEGRADED_MARGIN_C * clamp01(link_faults.link_degraded);
        if local_temp_c >= melt_c {
            self.link_melted = true;
        }
        let _ = smoke_alarm;
        self.link_melted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bottle_pressure_rises_with_temperature_and_falls_with_lost_charge() {
        let mut b = Bottle::new(5.0, 0.005);
        b.retarget_pressure(21.0);
        let cold_full = b.pressure_pa();
        b.retarget_pressure(50.0);
        let hot_full = b.pressure_pa();
        assert!(hot_full > cold_full);

        b.agent_mass_kg = 0.1; // below the residual-liquid fraction: vapour-only regime
        b.retarget_pressure(21.0);
        assert!(b.pressure_pa() < cold_full);
    }

    #[test]
    fn firing_a_healthy_squib_opens_the_valve_which_then_drains_on_its_own_until_empty() {
        let mut b = Bottle::new(5.0, 0.005);
        b.step(20.0, true, 101_325.0, &BottleFaults::default(), 0.1);
        assert!(b.is_discharged(), "the squib firing must immediately show the bottle as discharged (valve open)");
        let mass_just_after_firing = b.agent_mass_kg();
        assert!(mass_just_after_firing < 5.0);

        // A pyrotechnically-opened rupture disc cannot partially reseat: the
        // bottle keeps emptying under its own pressure even once
        // `fire_command` is no longer asserted, purely from the physics,
        // not from a repeated command.
        for _ in 0..200 {
            b.step(20.0, false, 101_325.0, &BottleFaults::default(), 0.1);
        }
        assert!(b.agent_mass_kg() < mass_just_after_firing, "an open valve must keep draining the bottle");
        assert!(b.agent_mass_kg() < 0.05, "should be essentially empty after several seconds at full discharge area, got {} kg", b.agent_mass_kg());
    }

    #[test]
    fn a_fully_failed_squib_prevents_any_discharge() {
        let mut b = Bottle::new(5.0, 0.005);
        let mut delivered_total = 0.0;
        for _ in 0..20 {
            delivered_total += b.step(20.0, true, 101_325.0, &BottleFaults { squib_failure: 1.0, ..Default::default() }, 0.1);
        }
        assert_eq!(delivered_total, 0.0);
        assert!((b.agent_mass_kg() - 5.0).abs() < 1e-6, "no leak fault active, mass should be unchanged");
    }

    #[test]
    fn a_leaking_bottle_loses_mass_and_pressure_over_time_even_unfired() {
        let mut b = Bottle::new(5.0, 0.005);
        for _ in 0..36000 {
            // 10 hours at 1 s steps
            b.step(20.0, false, 101_325.0, &BottleFaults { leak: 1.0, ..Default::default() }, 1.0);
        }
        assert!(b.agent_mass_kg() < 5.0, "a full-severity leak must lose mass over hours");
        assert!(b.is_low_pressure() || b.agent_mass_kg() < 4.0, "should show meaningfully depleted after 10h leaking");
    }

    #[test]
    fn a_leaky_bottle_delivers_less_punch_when_actually_needed() {
        // Leak long enough (~9.3 h) to run the remaining charge below the
        // residual-liquid fraction, so the leaky bottle is genuinely
        // discharging from a weaker (lower-pressure, nearly-vapour-only)
        // state, not just starting from a smaller but still fully
        // pressurized reservoir.
        let mut healthy = Bottle::new(5.0, 0.005);
        let mut leaky = Bottle::new(5.0, 0.005);
        for _ in 0..33_500 {
            leaky.step(20.0, false, 101_325.0, &BottleFaults { leak: 1.0, ..Default::default() }, 1.0);
        }
        assert!(leaky.agent_mass_kg() < 5.0 * Bottle::RESIDUAL_LIQUID_FRACTION, "setup: leak must have driven the bottle below the residual-liquid fraction, got {} kg", leaky.agent_mass_kg());

        let mut healthy_delivered = 0.0;
        let mut leaky_delivered = 0.0;
        for _ in 0..20 {
            healthy_delivered += healthy.step(20.0, true, 101_325.0, &BottleFaults::default(), 0.1);
            leaky_delivered += leaky.step(20.0, true, 101_325.0, &BottleFaults::default(), 0.1);
        }
        assert!(leaky_delivered < healthy_delivered, "a bottle weakened by a slow leak must deliver less agent when fired: leaky {leaky_delivered} vs healthy {healthy_delivered}");
    }

    #[test]
    fn zone_concentration_rises_with_discharge_and_decays_with_ventilation() {
        let mut zone = ZoneConcentration::new(20.0);
        for _ in 0..50 {
            zone.step(0.05, 20.0, 101_325.0, 0.0, 0.1); // discharging, no ventilation
        }
        let peak = zone.volume_fraction();
        assert!(peak > 0.0);
        for _ in 0..500 {
            zone.step(0.0, 20.0, 101_325.0, 2.0, 0.1); // no more discharge, strong ventilation
        }
        assert!(zone.volume_fraction() < peak, "ventilation must wash the agent back out");
    }

    #[test]
    fn suppression_fraction_saturates_at_one_once_design_concentration_is_reached() {
        let mut zone = ZoneConcentration::new(5.0);
        for _ in 0..2000 {
            zone.step(0.05, 20.0, 101_325.0, 0.0, 0.1);
        }
        assert_eq!(zone.suppression_fraction(), 1.0);
    }

    #[test]
    fn cross_feed_routes_to_the_neighbour_only_when_own_bottles_are_gone_and_valve_open() {
        let cf_open = CrossFeed { valve_open: true };
        let cf_closed = CrossFeed { valve_open: false };
        assert_eq!(cf_open.routing(true, true), (true, false), "own bottles available: use them, no need for cross-feed");
        assert_eq!(cf_open.routing(false, true), (false, true));
        assert_eq!(cf_closed.routing(false, true), (false, false), "valve closed: no cross-feed even if the neighbour has agent");
    }

    #[test]
    fn optical_smoke_detector_alarms_once_soot_accumulates_and_not_at_rest() {
        let mut d = OpticalSmokeDetector::new(1.0, 10.0);
        assert!(!d.step(0.0, 0.0, &SmokeDetectorFaults::default(), 1.0), "no fire, no smoke");
        let mut alarmed = false;
        for _ in 0..600 {
            if d.step(0.01, 0.05, &SmokeDetectorFaults::default(), 1.0) {
                alarmed = true;
                break;
            }
        }
        assert!(alarmed, "sustained burning must eventually alarm the smoke detector");
    }

    #[test]
    fn a_fully_obscured_lens_never_alarms_no_matter_how_much_smoke() {
        let mut d = OpticalSmokeDetector::new(1.0, 10.0);
        let faults = SmokeDetectorFaults { lens_obscured: 1.0 };
        let mut alarmed = false;
        for _ in 0..2000 {
            if d.step(0.02, 0.02, &faults, 1.0) {
                alarmed = true;
                break;
            }
        }
        assert!(!alarmed, "a fully obscured detector must never alarm regardless of real smoke present");
        assert!(d.smoke_density_kg_m3() > 0.0, "smoke must still genuinely be accumulating -- the fault blinds the detector, not the fire");
    }

    #[test]
    fn cargo_suppression_switches_from_knockdown_to_metered_once_design_concentration_is_reached() {
        let mut system = CargoSuppressionSystem::new(10.0, 30.0);
        let mut zone = ZoneConcentration::new(30.0);
        for _ in 0..600 {
            let delivered = system.step(20.0, true, 101_325.0, &zone, &CargoSuppressionFaults::default(), 0.1);
            zone.step(delivered, 20.0, 101_325.0, 0.05, 0.1);
            if system.is_metering() {
                break;
            }
        }
        assert!(system.is_metering(), "must reach design concentration and switch to metered discharge within the test window");
        let mass_at_switch = system.bottle.agent_mass_kg();
        assert!(mass_at_switch > 0.0, "must still have agent left for the extended metered phase");
    }

    /// E-FIRE §F: the knockdown and extended squib faults are independent
    /// -- each silences only its own stage.
    #[test]
    fn knockdown_and_extended_squib_faults_are_independent_stages() {
        let mut zone = ZoneConcentration::new(30.0);
        let mut system = CargoSuppressionSystem::new(10.0, 30.0);
        let faults = CargoSuppressionFaults { knockdown_squib_fault: 1.0, ..Default::default() };
        // The knockdown stage is fully failed: no delivery at all while
        // still in that stage, and the zone concentration never climbs
        // enough to switch to metered.
        let mut total = 0.0;
        for _ in 0..200 {
            let delivered = system.step(20.0, true, 101_325.0, &zone, &faults, 0.1);
            zone.step(delivered, 20.0, 101_325.0, 0.05, 0.1);
            total += delivered;
        }
        assert_eq!(total, 0.0, "a fully failed knockdown squib must deliver nothing in the knockdown stage");
        assert!(!system.is_metering());

        // A healthy knockdown stage reaches metering; the extended fault
        // then silences only that later stage.
        let mut zone2 = ZoneConcentration::new(30.0);
        let mut system2 = CargoSuppressionSystem::new(10.0, 30.0);
        let extended_fault = CargoSuppressionFaults { extended_squib_fault: 1.0, ..Default::default() };
        for _ in 0..600 {
            let delivered = system2.step(20.0, true, 101_325.0, &zone2, &extended_fault, 0.1);
            zone2.step(delivered, 20.0, 101_325.0, 0.05, 0.1);
            if system2.is_metering() {
                break;
            }
        }
        assert!(system2.is_metering(), "the knockdown stage must be unaffected by the extended-stage fault");
        let delivered_while_metering = system2.step(20.0, true, 101_325.0, &zone2, &extended_fault, 0.1);
        assert_eq!(delivered_while_metering, 0.0, "a fully failed extended squib must deliver nothing once metering");
    }

    /// E-FIRE §F: a distribution-path fault (the line/valve between the
    /// bottle manifold and the hold) reduces delivered agent independently
    /// of either squib.
    #[test]
    fn a_distribution_fault_reduces_delivered_agent_with_a_healthy_squib() {
        let zone = ZoneConcentration::new(30.0);
        let mut healthy = CargoSuppressionSystem::new(10.0, 30.0);
        let mut faulted = CargoSuppressionSystem::new(10.0, 30.0);
        let healthy_delivered = healthy.step(20.0, true, 101_325.0, &zone, &CargoSuppressionFaults::default(), 0.1);
        let faulted_delivered = faulted.step(20.0, true, 101_325.0, &zone, &CargoSuppressionFaults { distribution_fault: 1.0, ..Default::default() }, 0.1);
        assert!(healthy_delivered > 0.0);
        assert_eq!(faulted_delivered, 0.0, "a fully failed distribution path must deliver nothing even with a healthy bottle/squib");
    }

    #[test]
    fn lavatory_fusible_link_melts_once_and_stays_melted() {
        let mut lav = LavatoryProtection::new(2.0);
        assert!(!lav.step(20.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &LavatoryFaults::default(), 1.0));
        assert!(lav.step(FUSIBLE_LINK_MELT_C + 5.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &LavatoryFaults::default(), 1.0));
        assert!(lav.is_discharged());
        // Cooling back down does not un-melt it.
        assert!(lav.step(20.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &LavatoryFaults::default(), 1.0));
    }

    #[test]
    fn a_degraded_fusible_link_delays_discharge_past_the_design_temperature() {
        let mut lav = LavatoryProtection::new(2.0);
        let faults = LavatoryFaults { link_degraded: 1.0 };
        // At exactly the design melt temperature, a fully degraded link
        // must not yet have triggered.
        assert!(!lav.step(FUSIBLE_LINK_MELT_C + 5.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &faults, 1.0));
        // Past the degraded threshold, it does.
        assert!(lav.step(FUSIBLE_LINK_MELT_C + FUSIBLE_LINK_DEGRADED_MARGIN_C + 5.0, 0.0, 0.01, &SmokeDetectorFaults::default(), &faults, 1.0));
        assert!(lav.is_discharged());
    }

    #[test]
    fn no_nan_at_rest() {
        let mut b = Bottle::new(5.0, 0.005);
        let out = b.step(0.0, false, 101_325.0, &BottleFaults::default(), 0.0);
        assert!(!out.is_nan());
        assert!(!b.pressure_pa().is_nan());
        let mut zone = ZoneConcentration::new(10.0);
        zone.step(0.0, 0.0, 101_325.0, 0.0, 0.0);
        assert!(!zone.volume_fraction().is_nan());
    }
}

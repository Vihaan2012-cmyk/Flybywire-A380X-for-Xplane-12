//! #12 (OXY-001): oxygen quantity, consumption, low-pressure caution and
//! passenger mask deployment. No FBW source exists to port: neither
//! `fbw-common/src/wasm/systems/systems/src` nor `fbw-a380x/.../a380_systems/
//! src` has an `oxygen` module (docs/analysis/systems.md OXY-001's own
//! directory search confirms this, and this port's own search of both trees
//! for `oxygen`/`OXYGEN`/`O2` came up empty too), so this is a small native
//! addition rather than a port, matching how `fuel_network.rs` had to be
//! built from scratch for MSFS's fuel system rather than translated from
//! FlyByWire's own code.
//!
//! hyperrealism.md physics workstream 5 (fluids) replaced this module's
//! generic percent-based figures with real gas physics
//! (`physics::gas::ideal_gas_pressure_pa`/`ideal_gas_mass_kg`): the crew
//! bottle's pressure now comes from its actual mass of oxygen, physical
//! volume and temperature (the ideal gas law), depleted by a modelled
//! regulator's mass flow (mask minute ventilation x altitude-dependent
//! dilution, `physics::gas::diluter_demand_o2_fraction`), not a flat
//! percent-per-hour drain. The passenger system's chemical-generator
//! duration and output are tied to published A380 generator figures. See
//! `docs/physics/fluids.md` for every constant's derivation.
//!
//! **Sourcing:**
//! - Passenger mask automatic deployment at cabin altitude ~14,000 ft is a
//!   widely published Airbus trigger threshold (docs/analysis/systems.md
//!   OXY-001 itself already cites "roughly 14,000 ft"; it matches the
//!   FAA/EASA requirement for automatic oxygen presentation on large
//!   transport aircraft). Cabin altitude comes from the systems' own output,
//!   `PRESS_CABIN_ALTITUDE_B1` (a380_systems air_conditioning/cpiom_b.rs:861,
//!   one of the four CPIOM-B channels; B1 is used the way other modules in
//!   this plugin already pick one channel of a redundant set, e.g.
//!   prim.rs's `A32NX_RA_1_RADIO_ALTITUDE`).
//! - The crew bottle's full-charge pressure (1850 psig) and nominal free-air
//!   capacity (3260 L / 115 cubic feet) are the commonly published A320
//!   crew oxygen cylinder figures (repeated across multiple ATA-35 aviation
//!   training references, e.g. "ATA 35: Airbus A320", AviationHunt); no
//!   public A380-specific AMM figure was found. The cylinder's *physical*
//!   volume is derived from those two numbers via Boyle's law (free-air
//!   volume at 1 atm equals the charged volume at 1850 psig, same
//!   temperature): see [`crew_bottle_volume_m3`]. The A380's flight deck
//!   normally seats more occupants than the A320's two (two pilots plus up
//!   to two observer seats on long-haul augmented crews), so the A320
//!   cylinder volume is scaled by `CREW_COUNT`/2 -- a documented
//!   derivation, not a sourced A380 bottle size, flagged at
//!   [`crew_bottle_volume_m3`].
//! - Crew minute ventilation at rest under mask (`MINUTE_VENTILATION_LPM`)
//!   is a commonly cited resting adult respiratory minute-volume figure from
//!   aeromedical/respiratory-physiology references (typically quoted in the
//!   6-10 L/min range; 8 L/min is the midpoint used here), not an A380- or
//!   even aviation-specific number -- the closest defensible figure without
//!   a cited FAA/EASA aeromedical table in hand.
//! - The diluter-demand regulator's oxygen fraction schedule
//!   (`physics::gas::diluter_demand_o2_fraction`) is the standard
//!   qualitative aviation-physiology diluter-demand curve (ambient at sea
//!   level, ramping to 100% oxygen by approximately FL340), not a specific
//!   A380 regulator's proprietary calibration.
//! - The passenger chemical oxygen generator duration (15 minutes) and
//!   output (a two-person generator yields at least 42 L of oxygen, a
//!   three-person at least 62 L, a four-person at least 84 L, all over the
//!   15-minute decomposition) are the commonly published TSO-style figures
//!   for this class of generator (e.g. Transportation Safety Board of
//!   Canada A98H0003 supporting technical information; FlyByWire's own
//!   systems.md analysis notes A380 passenger oxygen may be chemical or
//!   gaseous depending on build -- this models the chemical-generator case,
//!   the more common one). The three-person size is used as the
//!   representative unit for the Study panel's flow-rate figure
//!   ([`PAX_GENERATOR_OUTPUT_LITERS`]), since the model does not track
//!   individual seat-row generators.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::physics::gas;
use crate::xp::Xplm;
use crate::Vars;

/// Commonly published A320 crew oxygen cylinder full charge (see module
/// doc); not a confirmed A380 AMM figure.
const CREW_FULL_PSI: f64 = 1850.;
/// A quarter of full charge, a common low-pressure caution margin.
const CREW_LOW_PRESSURE_FRACTION: f64 = 0.25;
/// Commonly published A320 crew cylinder free-air (1 atm) capacity, litres
/// (115 cubic feet); see module doc.
const CREW_FREE_AIR_LITERS: f64 = 3260.;
/// How many flight-deck occupants the modelled bottle serves when masks are
/// donned: two pilots plus two observer seats, a typical A380 augmented
/// long-haul crew complement (derived assumption, not a cited AMM crew
/// count).
const CREW_COUNT: f64 = 4.;
/// The A320's own crew count, for the bottle-volume scaling in the module
/// doc.
const A320_CREW_COUNT: f64 = 2.;
/// Assumed cylinder/regulator temperature: the plugin does not currently
/// read a flight-deck interior air temperature, so a fixed ambient-cabin
/// figure (20 C) stands in -- flagged as a simplification, not a sourced
/// cockpit temperature.
const CREW_BOTTLE_TEMP_K: f64 = 293.15;
/// Physical cylinder volume, derived via Boyle's law from the sourced A320
/// free-air capacity and charge pressure, then scaled by crew count (module
/// doc): `V_320 = V_free_air * P_atm / P_charge`, `V_380 = V_320 *
/// CREW_COUNT / A320_CREW_COUNT`.
fn crew_bottle_volume_m3() -> f64 {
    const SEA_LEVEL_PSI: f64 = 14.696;
    let v320_liters = CREW_FREE_AIR_LITERS * SEA_LEVEL_PSI / CREW_FULL_PSI;
    let v380_liters = v320_liters * CREW_COUNT / A320_CREW_COUNT;
    v380_liters / 1000.
}
/// Resting adult minute ventilation under mask, litres/minute (module doc):
/// a generic aeromedical/respiratory-physiology figure, not aviation- or
/// A380-specific.
const MINUTE_VENTILATION_LPM: f64 = 8.;

/// Published A380-class three-person chemical oxygen generator output
/// (module doc), litres of O2 over its full [`PAX_SUPPLY_MINUTES`] duration;
/// used only to report a representative flow rate, not the whole cabin
/// fleet's aggregate output (the model does not track individual seat-row
/// generators).
const PAX_GENERATOR_OUTPUT_LITERS: f64 = 62.;
/// Published chemical oxygen generator duration (module doc): the midpoint
/// of the commonly cited 13/15/22-minute family and the figure the sourced
/// A380 generator outputs above are quoted against.
const PAX_SUPPLY_MINUTES: f64 = 15.;
/// Widely published Airbus automatic mask deployment threshold.
const MASK_DEPLOY_CABIN_ALT_FT: f64 = 14_000.;

/// Pascals per psi (exact).
const PSI_TO_PA: f64 = 6894.757;

struct Ids {
    /// New here: no FBW L:var exists for a crew oxygen mask switch.
    crew_mask_on: VariableIdentifier,
    crew_quantity_percent: VariableIdentifier,
    crew_pressure_psi: VariableIdentifier,
    crew_low_pressure: VariableIdentifier,
    /// hyperrealism.md physics workstream 5: the regulator's own delivered
    /// mass flow and the altitude-driven dilution fraction driving it
    /// (Study panel quantities; previously only quantity/pressure/caution
    /// were published, with no flow visible at all).
    crew_flow_kg_s: VariableIdentifier,
    crew_dilution_fraction: VariableIdentifier,
    pax_quantity_percent: VariableIdentifier,
    pax_masks_deployed: VariableIdentifier,
    pax_flow_lpm: VariableIdentifier,
    cabin_altitude_ft: VariableIdentifier,
}

/// Servicing asked for from the Study panel, done on the next update.
static SERVICE_REQUESTED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Ask for the bottles to be refilled and the masks stowed.
pub fn request_service() {
    SERVICE_REQUESTED.store(true, std::sync::atomic::Ordering::Relaxed);
}

pub struct Oxygen {
    ids: Ids,
    /// hyperrealism.md physics workstream 5: the crew bottle's actual
    /// oxygen mass, kg (the ideal-gas state variable); pressure is derived
    /// from this, the bottle's fixed volume and `CREW_BOTTLE_TEMP_K`, rather
    /// than being tracked as its own percentage.
    crew_mass_kg: f64,
    crew_full_mass_kg: f64,
    pax_percent: f64,
    pax_deployed: bool,
}

impl Oxygen {
    pub fn new(vars: &mut Vars, _xplm: &Xplm) -> Self {
        let ids = Ids {
            crew_mask_on: vars.get("OXYGEN CREW MASK ON".into()),
            crew_quantity_percent: vars.get("OXYGEN_CREW_QUANTITY_PERCENT".into()),
            crew_pressure_psi: vars.get("OXYGEN_CREW_PRESSURE_PSI".into()),
            crew_low_pressure: vars.get("OXYGEN_CREW_LOW_PRESSURE".into()),
            crew_flow_kg_s: vars.get("OXYGEN_CREW_FLOW_KG_S".into()),
            crew_dilution_fraction: vars.get("OXYGEN_CREW_DILUTION_FRACTION".into()),
            pax_quantity_percent: vars.get("OXYGEN_PAX_QUANTITY_PERCENT".into()),
            pax_masks_deployed: vars.get("OXYGEN_PAX_MASKS_DEPLOYED".into()),
            pax_flow_lpm: vars.get("OXYGEN_PAX_FLOW_LPM".into()),
            // a380_systems air_conditioning/cpiom_b.rs:861.
            cabin_altitude_ft: vars.get("PRESS_CABIN_ALTITUDE_B1".into()),
        };
        let crew_full_mass_kg = gas::ideal_gas_mass_kg(CREW_FULL_PSI * PSI_TO_PA, crew_bottle_volume_m3(), CREW_BOTTLE_TEMP_K);
        Self { ids, crew_mass_kg: crew_full_mass_kg, crew_full_mass_kg, pax_percent: 100., pax_deployed: false }
    }

    /// After the systems tick, so `PRESS_CABIN_ALTITUDE_B1` is this tick's.
    /// `_xplm` is unused today (kept for signature symmetry with the other
    /// slot modules, e.g. `Lights::update`, and in case a future X-Plane-only
    /// input is needed); the real logic is in [`Self::step`], which the
    /// tests below call directly so they need no `Xplm` at all (matching how
    /// `lights.rs`'s own tests avoid needing one, see that file's test doc).
    pub fn update<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, _xplm: &Xplm, delta: f64) {
        self.step(vars, delta);
    }

    fn step<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, delta: f64) {
        if SERVICE_REQUESTED.swap(false, std::sync::atomic::Ordering::Relaxed) {
            self.service();
        }
        let cabin_alt = vars.read(&self.ids.cabin_altitude_ft);
        let masks_on = vars.read(&self.ids.crew_mask_on) != 0.;

        // hyperrealism.md physics workstream 5: crew consumption is now a
        // real regulator mass flow -- minute ventilation per person, the
        // fraction of it a diluter-demand regulator draws as pure oxygen at
        // this cabin altitude, times how many masks are donned -- instead of
        // a flat "100%/CREW_ENDURANCE_HOURS per hour" drain. Depleting the
        // bottle's own mass, with pressure following from the ideal gas law.
        let dilution = gas::diluter_demand_o2_fraction(cabin_alt);
        let flow_kg_s = if masks_on && self.crew_mass_kg > 0. {
            CREW_COUNT * gas::o2_mass_flow_kg_s(MINUTE_VENTILATION_LPM * dilution)
        } else {
            0.
        };
        self.crew_mass_kg = (self.crew_mass_kg - flow_kg_s * delta).max(0.);

        // Passenger masks drop automatically above the cabin-altitude
        // threshold and, once dropped, the chemical generators start
        // depleting and cannot be reset (matching how a real generator, once
        // fired, runs to exhaustion -- a real property of the exothermic
        // sodium-chlorate/iron reaction, not a simplification).
        if cabin_alt >= MASK_DEPLOY_CABIN_ALT_FT {
            self.pax_deployed = true;
        }
        if self.pax_deployed && self.pax_percent > 0. {
            self.pax_percent = (self.pax_percent - 100. * delta / 60. / PAX_SUPPLY_MINUTES).max(0.);
        }

        let crew_pressure_pa = gas::ideal_gas_pressure_pa(self.crew_mass_kg, crew_bottle_volume_m3(), CREW_BOTTLE_TEMP_K);
        let crew_psi = crew_pressure_pa / PSI_TO_PA;
        let crew_percent = 100. * self.crew_mass_kg / self.crew_full_mass_kg.max(1e-9);
        vars.write(&self.ids.crew_quantity_percent, crew_percent);
        vars.write(&self.ids.crew_pressure_psi, crew_psi);
        vars.write(&self.ids.crew_low_pressure, (crew_percent <= CREW_LOW_PRESSURE_FRACTION * 100.) as i32 as f64);
        vars.write(&self.ids.crew_flow_kg_s, flow_kg_s);
        vars.write(&self.ids.crew_dilution_fraction, dilution);
        vars.write(&self.ids.pax_quantity_percent, self.pax_percent);
        vars.write(&self.ids.pax_masks_deployed, self.pax_deployed as i32 as f64);
        vars.write(&self.ids.pax_flow_lpm, if self.pax_deployed && self.pax_percent > 0. { PAX_GENERATOR_OUTPUT_LITERS / PAX_SUPPLY_MINUTES } else { 0. });
    }

    /// Ground servicing (refill) for the Study Ground Services page
    /// (STUDY-003): tops the crew bottle back up, and — since a fired
    /// chemical generator cannot be refilled, only replaced between flights —
    /// resets the passenger supply and retracts the "deployed" state, the
    /// way a real turnaround replaces spent generator packs.
    ///
    /// Not called anywhere yet: the Study panel's Ground Services page
    /// (STUDY-003) is the lead's file to add the button to; the L:vars this
    /// module already publishes (`OXYGEN_CREW_QUANTITY_PERCENT` etc.) are
    /// also readable directly without this method.
    #[allow(dead_code)]
    pub fn service(&mut self) {
        self.crew_mass_kg = self.crew_full_mass_kg;
        self.pax_percent = 100.;
        self.pax_deployed = false;
    }

    /// The Study panel: crew bottle quantity/pressure/low-pressure caution,
    /// passenger supply remaining and whether the masks have dropped. Not
    /// called anywhere yet, for the same reason as `service` above; the
    /// published L:vars are the other way to read the same numbers.
    #[allow(dead_code)]
    pub fn crew_quantity_percent(&self) -> f64 {
        100. * self.crew_mass_kg / self.crew_full_mass_kg.max(1e-9)
    }
    #[allow(dead_code)]
    pub fn pax_quantity_percent(&self) -> f64 {
        self.pax_percent
    }
    #[allow(dead_code)]
    pub fn pax_masks_deployed(&self) -> bool {
        self.pax_deployed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;

    #[test]
    fn the_cabin_altitude_variable_this_module_reads_is_one_the_systems_register() {
        // Guards the citation at the top of this file: if FlyByWire ever
        // renames CpiomId::B1's variable, this fails instead of the plugin
        // silently reading a variable nothing writes.
        #[derive(Default)]
        struct Names(Vec<String>);
        impl VariableRegistry for Names {
            fn get(&mut self, name: String) -> VariableIdentifier {
                self.0.push(name);
                VariableIdentifier::new(0usize)
            }
            fn get_unprefixed(&mut self, name: String) -> VariableIdentifier {
                self.0.push(name);
                VariableIdentifier::new(0usize)
            }
        }
        let mut names = Names::default();
        let _ = systems::simulation::Simulation::new(systems::simulation::StartState::Apron, a380_systems::A380::new, &mut names);
        assert!(names.0.iter().any(|n| n == "PRESS_CABIN_ALTITUDE_B1"), "{:?}", names.0.iter().filter(|n| n.contains("CABIN_ALTITUDE")).collect::<Vec<_>>());
    }

    fn oxygen_with(vars: &mut TestVars) -> Oxygen {
        let ids = Ids {
            crew_mask_on: vars.get("OXYGEN CREW MASK ON".into()),
            crew_quantity_percent: vars.get("OXYGEN_CREW_QUANTITY_PERCENT".into()),
            crew_pressure_psi: vars.get("OXYGEN_CREW_PRESSURE_PSI".into()),
            crew_low_pressure: vars.get("OXYGEN_CREW_LOW_PRESSURE".into()),
            crew_flow_kg_s: vars.get("OXYGEN_CREW_FLOW_KG_S".into()),
            crew_dilution_fraction: vars.get("OXYGEN_CREW_DILUTION_FRACTION".into()),
            pax_quantity_percent: vars.get("OXYGEN_PAX_QUANTITY_PERCENT".into()),
            pax_masks_deployed: vars.get("OXYGEN_PAX_MASKS_DEPLOYED".into()),
            pax_flow_lpm: vars.get("OXYGEN_PAX_FLOW_LPM".into()),
            cabin_altitude_ft: vars.get("PRESS_CABIN_ALTITUDE_B1".into()),
        };
        let crew_full_mass_kg = gas::ideal_gas_mass_kg(CREW_FULL_PSI * PSI_TO_PA, crew_bottle_volume_m3(), CREW_BOTTLE_TEMP_K);
        Oxygen { ids, crew_mass_kg: crew_full_mass_kg, crew_full_mass_kg, pax_percent: 100., pax_deployed: false }
    }

    #[test]
    fn crew_oxygen_only_depletes_with_a_mask_donned() {
        let mut vars = TestVars::default();
        let mut o = oxygen_with(&mut vars);
        vars.write(&o.ids.cabin_altitude_ft, 8000.);
        vars.write(&o.ids.crew_mask_on, 0.);
        o.step(&mut vars, 3600.);
        assert_eq!(o.crew_quantity_percent(), 100., "no mask, no consumption");

        vars.write(&o.ids.crew_mask_on, 1.);
        o.step(&mut vars, 3600.);
        assert!(o.crew_quantity_percent() < 100., "a donned mask should consume oxygen");
    }

    #[test]
    fn crew_bottle_pressure_follows_the_ideal_gas_law() {
        let mut vars = TestVars::default();
        let mut o = oxygen_with(&mut vars);
        vars.write(&o.ids.cabin_altitude_ft, 35_000.);
        vars.write(&o.ids.crew_mask_on, 1.);
        // Deplete halfway.
        o.crew_mass_kg = o.crew_full_mass_kg / 2.;
        o.step(&mut vars, 0.001);
        let pressure = vars.read(&o.ids.crew_pressure_psi);
        // Half the mass at the same volume/temperature should read close to
        // half the full pressure (ideal gas: P proportional to m).
        assert!((pressure - CREW_FULL_PSI / 2.).abs() / (CREW_FULL_PSI / 2.) < 0.01, "{pressure}");
    }

    #[test]
    fn higher_cabin_altitude_drains_the_crew_bottle_faster() {
        let mut vars_low = TestVars::default();
        let mut low = oxygen_with(&mut vars_low);
        vars_low.write(&low.ids.cabin_altitude_ft, 1000.);
        vars_low.write(&low.ids.crew_mask_on, 1.);
        low.step(&mut vars_low, 600.);

        let mut vars_high = TestVars::default();
        let mut high = oxygen_with(&mut vars_high);
        vars_high.write(&high.ids.cabin_altitude_ft, 40_000.);
        vars_high.write(&high.ids.crew_mask_on, 1.);
        high.step(&mut vars_high, 600.);

        assert!(high.crew_mass_kg < low.crew_mass_kg, "more dilution-demand oxygen should be drawn at higher cabin altitude");
    }

    #[test]
    fn low_pressure_fires_at_a_quarter_full() {
        let mut vars = TestVars::default();
        let mut o = oxygen_with(&mut vars);
        o.crew_mass_kg = o.crew_full_mass_kg * 0.10;
        vars.write(&o.ids.crew_mask_on, 0.);
        vars.write(&o.ids.cabin_altitude_ft, 0.);
        o.step(&mut vars, 0.001);
        assert_eq!(vars.read(&o.ids.crew_low_pressure), 1.);

        o.crew_mass_kg = o.crew_full_mass_kg * 0.90;
        o.step(&mut vars, 0.001);
        assert_eq!(vars.read(&o.ids.crew_low_pressure), 0.);
    }

    #[test]
    fn passenger_masks_stay_up_below_the_threshold_and_drop_above_it() {
        let mut vars = TestVars::default();
        let mut o = oxygen_with(&mut vars);
        vars.write(&o.ids.cabin_altitude_ft, 9000.);
        o.step(&mut vars, 60.);
        assert!(!o.pax_masks_deployed());
        assert_eq!(o.pax_quantity_percent(), 100.);

        vars.write(&o.ids.cabin_altitude_ft, 14_500.);
        o.step(&mut vars, 60.);
        assert!(o.pax_masks_deployed());
    }

    #[test]
    fn once_deployed_the_generator_runs_to_exhaustion_even_if_altitude_drops_back() {
        let mut vars = TestVars::default();
        let mut o = oxygen_with(&mut vars);
        vars.write(&o.ids.cabin_altitude_ft, 15_000.);
        o.step(&mut vars, 1.);
        assert!(o.pax_masks_deployed());

        // Cabin altitude comes back down (the crew descends); the fired
        // generators do not un-fire.
        vars.write(&o.ids.cabin_altitude_ft, 6000.);
        o.step(&mut vars, PAX_SUPPLY_MINUTES * 60.);
        assert!(o.pax_masks_deployed());
        assert!(o.pax_quantity_percent() < 1., "should be exhausted after a full supply duration: {}", o.pax_quantity_percent());
    }

    #[test]
    fn pax_generator_flow_matches_the_published_output_over_its_duration() {
        let mut vars = TestVars::default();
        let mut o = oxygen_with(&mut vars);
        vars.write(&o.ids.cabin_altitude_ft, 15_000.);
        o.step(&mut vars, 1.);
        let flow_lpm = vars.read(&o.ids.pax_flow_lpm);
        assert!((flow_lpm - PAX_GENERATOR_OUTPUT_LITERS / PAX_SUPPLY_MINUTES).abs() < 1e-9);
        // Total volume over the whole duration should match the published
        // generator output.
        assert!((flow_lpm * PAX_SUPPLY_MINUTES - PAX_GENERATOR_OUTPUT_LITERS).abs() < 1e-6);
    }

    #[test]
    fn servicing_restores_both_bottles_and_retracts_the_masks() {
        let mut vars = TestVars::default();
        let mut o = oxygen_with(&mut vars);
        o.crew_mass_kg = o.crew_full_mass_kg * 0.05;
        o.pax_percent = 0.;
        o.pax_deployed = true;
        o.service();
        assert_eq!(o.crew_quantity_percent(), 100.);
        assert_eq!(o.pax_quantity_percent(), 100.);
        assert!(!o.pax_masks_deployed());
    }

    #[test]
    fn ideal_and_real_gas_pressure_differ_by_the_expected_amount_for_the_crew_bottle() {
        // Documents why `ideal_gas_pressure_pa` (not `van_der_waals_pressure_pa`)
        // drives the displayed crew pressure: the sourced 1850 psi full-charge
        // figure is itself a round nominal spec, and a full bottle's ideal-gas
        // mass at that nominal pressure is what `crew_full_mass_kg` is defined
        // as (module doc). At this bottle's actual charge density the two laws
        // do *not* nearly agree -- real O2 at ~127 bar/165+ kg/m^3 departs from
        // ideal by roughly 10% (see `physics::gas`'s own test, and its module
        // doc's derivation) -- so this test pins that real, non-trivial gap
        // instead of asserting the two coincide.
        let volume = crew_bottle_volume_m3();
        let mass = gas::ideal_gas_mass_kg(CREW_FULL_PSI * PSI_TO_PA, volume, CREW_BOTTLE_TEMP_K);
        let ideal = gas::ideal_gas_pressure_pa(mass, volume, CREW_BOTTLE_TEMP_K);
        let real = gas::van_der_waals_pressure_pa(mass, volume, CREW_BOTTLE_TEMP_K);
        let rel_dev = (real - ideal).abs() / ideal;
        assert!(rel_dev > 0.05 && rel_dev < 0.15, "{rel_dev}");
    }

    #[test]
    fn crew_bottle_volume_scales_with_crew_count_from_the_sourced_a320_figure() {
        // At CREW_COUNT == A320_CREW_COUNT this should reproduce the A320
        // cylinder's own physical volume via Boyle's law.
        let v320_liters = CREW_FREE_AIR_LITERS * 14.696 / CREW_FULL_PSI;
        let expected_m3 = v320_liters / 1000. * CREW_COUNT / A320_CREW_COUNT;
        assert!((crew_bottle_volume_m3() - expected_m3).abs() < 1e-9);
    }
}

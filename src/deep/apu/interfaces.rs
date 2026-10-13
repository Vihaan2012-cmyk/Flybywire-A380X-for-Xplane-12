//! Cross-area interfaces as plain structs -- item 5. This directory does
//! not import `deep::pneumatic_ducts` or `deep::electrical` (self-
//! containment; those are other areas' own directories, and nothing
//! outside this directory references this one yet either), so the contract
//! at each boundary is expressed here as a plain data struct with no
//! behaviour: whoever wires the areas together on both sides fills/reads
//! these, and each side can evolve its own internals without the other
//! needing to change.
//!
//! - `BleedOutput`: what this APU delivers to `deep::pneumatic_ducts` --
//!   real, causal outputs of `load_compressor.rs` (pressure and mass flow
//!   the load compressor is actually delivering this tick, plus its
//!   temperature so a duct-side heat balance has something real to use),
//!   not a demand or a setpoint.
//! - `GeneratorOutput`: what each generator delivers to `deep::electrical`
//!   -- real electrical power output and the shaft power it cost
//!   (`generators.rs`), so an electrical-side model can do its own bus/load
//!   accounting without needing to re-derive the mechanical side.
//! - `BatteryInput`: what `deep::electrical`'s own battery model hands this
//!   APU's starter (`starter.rs`) -- a plain Thevenin-equivalent source
//!   (open-circuit voltage and internal resistance) rather than a single
//!   voltage number, so a cold-soaked or depleted battery's *effect*
//!   (more sag under the same starter current) is a real consequence of
//!   the resistance electrical's own model reports, not something this
//!   directory has to know how to compute for someone else's battery.

/// Real bleed air this APU is delivering this tick -- `load_compressor.rs`'s
/// own output, not a request.
#[derive(Clone, Copy, Debug, Default)]
pub struct BleedOutput {
    pub mass_flow_kg_s: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
}

/// Real electrical output one generator is delivering this tick, plus what
/// it cost mechanically -- `generators.rs`'s own numbers.
#[derive(Clone, Copy, Debug, Default)]
pub struct GeneratorOutput {
    pub real_power_w: f64,
    pub shaft_power_w: f64,
    pub overloaded: bool,
}

/// A Thevenin-equivalent battery source, as `deep::electrical`'s own
/// battery model would report it to the starter: open-circuit voltage and
/// internal resistance. A depleted or cold-soaked battery shows up here as
/// a lower `open_circuit_v` and/or a higher `internal_resistance_ohm` --
/// `starter.rs` reacts to whichever combination it is handed, it does not
/// itself model *why* the battery is weak.
#[derive(Clone, Copy, Debug)]
pub struct BatteryInput {
    pub open_circuit_v: f64,
    pub internal_resistance_ohm: f64,
    pub available: bool,
}

impl BatteryInput {
    /// A healthy, fully charged, room-temperature main battery -- the
    /// default a caller with no real `deep::electrical` model wired up yet
    /// can use, and this file's own tests' baseline.
    pub fn healthy() -> Self {
        Self {
            open_circuit_v: super::params::BATTERY_NOMINAL_OPEN_CIRCUIT_V,
            internal_resistance_ohm: super::params::BATTERY_INTERNAL_RESISTANCE_OHM,
            available: true,
        }
    }
}

impl Default for BatteryInput {
    fn default() -> Self {
        Self::healthy()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::params;

    #[test]
    fn the_healthy_default_matches_the_nominal_battery_constants() {
        let b = BatteryInput::healthy();
        assert_eq!(b.open_circuit_v, params::BATTERY_NOMINAL_OPEN_CIRCUIT_V);
        assert!(b.available);
    }

    #[test]
    fn plain_structs_carry_exactly_what_they_are_given() {
        let bleed = BleedOutput { mass_flow_kg_s: 1.1, pressure_pa: 300_000.0, temperature_k: 460.0 };
        assert_eq!(bleed.mass_flow_kg_s, 1.1);
        let gen = GeneratorOutput { real_power_w: 50_000.0, shaft_power_w: 58_000.0, overloaded: false };
        assert!(!gen.overloaded);
    }
}

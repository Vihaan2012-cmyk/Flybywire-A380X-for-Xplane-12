//! Plugin-side glue for physics workstream 3 (air), `docs/briefs/hyperrealism.md`.
//!
//! Almost all of this workstream's physics lives in FlyByWire's own Rust systems crate
//! (`D:\fbw-aircraft`, patched in `patches/fbw-rust/air.patch`): a real bootstrap air cycle
//! machine model (`a380_systems/src/air_conditioning/air_cycle_machine.rs`) replaces the common
//! crate's `AirConditioningPack`, which was explicitly documented as "a placeholder until the
//! packs are modelled". The cabin thermal model, cabin pressure control volume (compressible
//! orifice flow through the outflow valves) and mixer/trim-air distribution were already real
//! physics in FlyByWire's own crate (energy balance, Reynolds/Nusselt convection, subsonic/
//! choked flow) and are unchanged; see `docs/physics/air.md` for the full audit.
//!
//! The one piece that has to live in the plugin is the shared engine-load contract: FlyByWire's
//! systems now compute each engine's real bleed extraction mass flow (the pressure regulating
//! valve's own flow, `EngineBleedAirSystem::bleed_extraction_flow`, pneumatic.rs) and publish it
//! under FBW's own naming (`PNEU_ENG_<n>_BLEED_EXTRACTION_FLOW`); this module republishes it
//! under the contract name the engine workstream reads (`ENGINE_BLEED_EXTRACTION_KG_S:<n>`),
//! exactly as `physics::electrical::EngineLoads` does for the electrical load term.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::Vars;

pub struct EngineBleedLoads {
    engines: [(VariableIdentifier, VariableIdentifier); 4],
}

impl EngineBleedLoads {
    pub fn new(vars: &mut Vars) -> Self {
        let engines = std::array::from_fn(|i| {
            let n = i + 1;
            (
                vars.get(format!("PNEU_ENG_{n}_BLEED_EXTRACTION_FLOW")),
                vars.get(format!("ENGINE_BLEED_EXTRACTION_KG_S:{n}")),
            )
        });
        Self { engines }
    }

    /// Reads the tick's bleed extraction flow FlyByWire's pneumatic system just computed and
    /// republishes it under the shared contract name. Call after the systems tick, same as
    /// `EngineLoads::update`.
    pub fn update(&self, vars: &mut Vars) {
        for (fbw_flow, contract_flow) in &self.engines {
            let flow = vars.read(fbw_flow);
            vars.write(contract_flow, flow);
        }
    }
}

//! Plugin-side glue for the shared engine-load contract's hydraulic term
//! (`docs/briefs/hyperrealism.md`): `ENGINE_GEARBOX_HYD_LOAD_W:n`, "the
//! engine-driven hydraulic pump shaft power (written by hydraulics: pressure
//! x flow divided by efficiency)".
//!
//! FlyByWire's own `hydraulic/mod.rs` (a380_systems, unmodified except for
//! this workstream's patch, `patches/fbw-rust/fluids.patch`) already runs
//! the A380's engine-driven pumps as real physics: pressure-vs-displacement
//! curves sourced from the real 5000 psi A380 hydraulic system
//! (`PumpCharacteristics::a380_edp`, `pumps.rs`), a flow solve against the
//! circuit's actual demand. What it never did was turn that into a
//! mechanical load back on the engine -- nothing in the systems crate or
//! this plugin read the pump's own pressure/flow to compute the shaft power
//! the engine gearbox must supply. This workstream added
//! `EngineDrivenPump::shaft_power()` (pressure x flow / 0.90, an aviation
//! axial-piston-pump efficiency figure, see the module doc there) and a new
//! `HYD_<id>_EDPUMP_SHAFT_POWER_W` simulator variable per pump; this module
//! is the plugin-side half, summing each engine's two pumps and republishing
//! the contract name the engine workstream reads.
//!
//! Each A380 engine drives two engine-driven pumps
//! (`AirbusEngineDrivenPumpId::Edp{n}a`/`Edp{n}b`, `shared/mod.rs`): engines
//! 1-2 feed the Green circuit (`HYD_GREEN_{n}A`/`HYD_GREEN_{n}B` variable
//! names, from that enum's `Display` impl), engines 3-4 the Yellow circuit
//! (`HYD_YELLOW_{n}A`/`HYD_YELLOW_{n}B`).

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::Vars;

struct EnginePumps {
    a: VariableIdentifier,
    b: VariableIdentifier,
    load: VariableIdentifier,
}

/// Sums FlyByWire's per-pump engine-driven-pump shaft power into the shared
/// `ENGINE_GEARBOX_HYD_LOAD_W:n` contract variable, once per engine.
pub struct Hydraulics {
    engines: [EnginePumps; 4],
}

impl Hydraulics {
    pub fn new(vars: &mut Vars) -> Self {
        let engines = std::array::from_fn(|i| {
            let n = i + 1;
            let (color, half) = if n <= 2 { ("GREEN", n) } else { ("YELLOW", n) };
            EnginePumps {
                a: vars.get(format!("HYD_{color}_{half}A_EDPUMP_SHAFT_POWER_W")),
                b: vars.get(format!("HYD_{color}_{half}B_EDPUMP_SHAFT_POWER_W")),
                load: vars.get(format!("ENGINE_GEARBOX_HYD_LOAD_W:{n}")),
            }
        });
        Self { engines }
    }

    /// Reads the tick's two per-pump shaft powers FlyByWire's systems just
    /// wrote and republishes their sum. Call after the systems tick.
    pub fn update(&self, vars: &mut Vars) {
        for e in &self.engines {
            let total = vars.read(&e.a) + vars.read(&e.b);
            vars.write(&e.load, total);
        }
    }
}

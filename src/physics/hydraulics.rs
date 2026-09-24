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
    /// `FBW_HYD_STATS` only: everything the engine-driven pumps depend on,
    /// so a circuit that will not pressurise can be read rather than
    /// guessed at.
    stats: Stats,
}

/// The inputs an EDP needs, in the order they gate each other: the engine
/// has to be turning, its pushbutton has to be in AUTO, its fire pushbutton
/// has to be stowed, and only then does the pump make pressure.
struct Stats {
    at: Option<std::time::Instant>,
    n3: [VariableIdentifier; 4],
    pump_auto: [VariableIdentifier; 4],
    fire_released: [VariableIdentifier; 4],
    green_psi: VariableIdentifier,
    yellow_psi: VariableIdentifier,
}

impl Stats {
    fn new(vars: &mut Vars) -> Self {
        Self {
            at: None,
            n3: std::array::from_fn(|i| vars.get(format!("ENGINE_N3:{}", i + 1))),
            // `A380EngineDrivenPumpController::update`'s own gate: the "a"
            // pump of each engine stands for the pair here.
            pump_auto: std::array::from_fn(|i| {
                let n = i + 1;
                vars.get(format!("OVHD_HYD_ENG_{n}A_PUMP_PB_IS_AUTO"))
            }),
            fire_released: std::array::from_fn(|i| vars.get(format!("FIRE_BUTTON_ENG{}", i + 1))),
            green_psi: vars.get("HYD_GREEN_SYSTEM_1_SECTION_PRESSURE".to_owned()),
            yellow_psi: vars.get("HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE".to_owned()),
        }
    }
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
        let stats = Stats::new(vars);
        Self { engines, stats }
    }

    /// Reads the tick's two per-pump shaft powers FlyByWire's systems just
    /// wrote and republishes their sum. Call after the systems tick.
    pub fn update(&mut self, vars: &mut Vars) {
        for e in &self.engines {
            let total = vars.read(&e.a) + vars.read(&e.b);
            vars.write(&e.load, total);
        }
        self.log_stats(vars);
    }

    /// `FBW_HYD_STATS=1`: every 5 s, one line giving each engine's N3, its
    /// pump pushbutton and fire pushbutton, the shaft power its two pumps
    /// are drawing, and both circuits' pressure.
    ///
    /// A circuit that will not pressurise has a short list of causes and
    /// they are strictly ordered -- the Trent drives its pumps from N3
    /// (`trent_engine.rs`: `hydraulic_pump_output_speed` is
    /// `uncorrected_n3` geared down, read from `ENGINE_N3:n`), the
    /// controller needs the pushbutton in AUTO and the fire pushbutton
    /// stowed, and the pump then makes shaft power. Printing all four
    /// together says which link is the broken one instead of leaving it to
    /// inference.
    fn log_stats(&mut self, vars: &mut Vars) {
        use std::sync::OnceLock;
        static ON: OnceLock<bool> = OnceLock::new();
        if !*ON.get_or_init(|| std::env::var("FBW_HYD_STATS").is_ok_and(|v| v.trim() != "0" && !v.trim().is_empty())) {
            return;
        }
        let now = std::time::Instant::now();
        if self.stats.at.is_some_and(|t| now - t < std::time::Duration::from_secs(5)) {
            return;
        }
        self.stats.at = Some(now);
        let mut line = String::from("hyd:");
        for i in 0..4 {
            let n3 = vars.read(&self.stats.n3[i]);
            let auto = vars.read(&self.stats.pump_auto[i]) != 0.;
            let fire = vars.read(&self.stats.fire_released[i]) != 0.;
            let shaft = vars.read(&self.engines[i].a) + vars.read(&self.engines[i].b);
            line.push_str(&format!(" eng{}: N3 {n3:.0}% pb {} fire {} shaft {shaft:.0}W;", i + 1, if auto { "AUTO" } else { "off" }, if fire { "RELEASED" } else { "in" }));
        }
        line.push_str(&format!(
            " green {:.0} psi, yellow {:.0} psi",
            vars.read(&self.stats.green_psi),
            vars.read(&self.stats.yellow_psi)
        ));
        crate::log(&line);
    }
}

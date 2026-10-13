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
    /// Per pump, in `AirbusEngineDrivenPumpId` order (1A, 1B, 2A, ... 4B):
    /// whether the controller has it pressurising, the flow it is moving,
    /// and whether it is cavitating.
    pump: [PumpStats; 8],
    /// Per circuit, green then yellow.
    reservoir: [ReservoirStats; 2],
}

struct PumpStats {
    name: &'static str,
    active: VariableIdentifier,
    flow: VariableIdentifier,
    /// `HYD_<id>_EDPUMP_CAVITATION` is an *efficiency ratio*, not a flag:
    /// 1.0 is a pump with a full, pressurised, cool reservoir behind it,
    /// and `EngineDrivenPump::is_heating_this_tick` treats anything below
    /// `CAVITATION_OVERHEAT_EFFICIENCY_RATIO` (0.3) as severe enough to
    /// start heating the pump toward damage.
    cavitation: VariableIdentifier,
    overheat: VariableIdentifier,
    damaged: VariableIdentifier,
}

struct ReservoirStats {
    name: &'static str,
    level: VariableIdentifier,
    low_level: VariableIdentifier,
    low_air_pressure: VariableIdentifier,
    overheating: VariableIdentifier,
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
            pump: PUMP_NAMES.map(|name| PumpStats {
                name,
                active: vars.get(format!("HYD_{name}_EDPUMP_ACTIVE")),
                flow: vars.get(format!("HYD_{name}_EDPUMP_FLOW")),
                cavitation: vars.get(format!("HYD_{name}_EDPUMP_CAVITATION")),
                overheat: vars.get(format!("HYD_{name}_EDPUMP_OVERHEAT")),
                damaged: vars.get(format!("HYD_{name}_EDPUMP_DAMAGED")),
            }),
            reservoir: ["GREEN", "YELLOW"].map(|name| ReservoirStats {
                name,
                level: vars.get(format!("HYD_{name}_RESERVOIR_LEVEL")),
                low_level: vars.get(format!("HYD_{name}_RESERVOIR_LEVEL_IS_LOW")),
                low_air_pressure: vars.get(format!("HYD_{name}_RESERVOIR_AIR_PRESSURE_IS_LOW")),
                overheating: vars.get(format!("HYD_{name}_RESERVOIR_OVHT")),
            }),
        }
    }
}

/// `AirbusEngineDrivenPumpId`'s own `Display` spellings: engines 1 and 2
/// drive the green circuit, 3 and 4 the yellow, two pumps each.
const PUMP_NAMES: [&str; 8] =
    ["GREEN_1A", "GREEN_1B", "GREEN_2A", "GREEN_2B", "YELLOW_3A", "YELLOW_3B", "YELLOW_4A", "YELLOW_4B"];

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

    /// `FBW_HYD_STATS=1`: every 5 s, three lines covering the whole chain
    /// from the engine to the circuit.
    ///
    /// A circuit that will not pressurise has a short list of causes and
    /// they are strictly ordered -- the Trent drives its pumps from N3
    /// (`trent_engine.rs`: `hydraulic_pump_output_speed` is
    /// `uncorrected_n3` geared down, read from `ENGINE_N3:n`), the
    /// controller needs the pushbutton in AUTO and the fire pushbutton
    /// stowed, and the pump then makes shaft power. Printing all four
    /// together says which link is the broken one instead of leaving it to
    /// inference.
    ///
    /// The first line is that chain. It is not enough on its own, because
    /// the case actually seen -- pumps making 5311 W at 50 % N3 and 0 W at
    /// 76 %, with the circuit down to 16 psi -- is the wrong way round for
    /// every link in it: pump speed is a *monotone* function of N3, so more
    /// N3 cannot mean less pump. Something downstream is taking the pump
    /// out, and there are exactly three candidates in `hydraulic/mod.rs`,
    /// all of them published:
    ///
    /// * the controller stops commanding it (`..._EDPUMP_ACTIVE` false),
    /// * the reservoir cannot feed it -- low level, low air pressure, or
    ///   overheating (`HYD_<circuit>_RESERVOIR_*`), which is what makes a
    ///   pump cavitate, and
    /// * the pump has overheated into damage, after which
    ///   `EngineDrivenPump::update` stops applying `pump_speed` at all
    ///   (`!self.is_damaged()`) -- so it reads as a pump spinning at zero
    ///   while the engine it is bolted to is at take-off power.
    ///
    /// Each of those is printed, so the three are distinguishable rather
    /// than all presenting the same way -- zero flow at zero pressure --
    /// from outside. Note that shaft power is *derived* from pressure
    /// (`EngineDrivenPump::shaft_power` is `last_pressure * flow / 0.90`),
    /// so "shaft 0 W" is a restatement of "the circuit is at 16 psi", not
    /// independent evidence about the pump.
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

        let mut pumps = String::from("hyd pumps:");
        for p in &self.stats.pump {
            pumps.push_str(&format!(
                " {} {} {:.2} gal/s cav {:.2}{}{};",
                p.name,
                if vars.read(&p.active) != 0. { "active" } else { "off" },
                vars.read(&p.flow),
                vars.read(&p.cavitation),
                if vars.read(&p.overheat) != 0. { " OVERHEAT" } else { "" },
                if vars.read(&p.damaged) != 0. { " DAMAGED" } else { "" },
            ));
        }
        crate::log(&pumps);

        let mut res = String::from("hyd reservoirs:");
        for r in &self.stats.reservoir {
            res.push_str(&format!(
                " {} {:.1} gal{}{}{};",
                r.name,
                vars.read(&r.level),
                if vars.read(&r.low_level) != 0. { " LOW LEVEL" } else { "" },
                if vars.read(&r.low_air_pressure) != 0. { " LOW AIR PRESS" } else { "" },
                if vars.read(&r.overheating) != 0. { " OVHT" } else { "" },
            ));
        }
        crate::log(&res);
    }
}

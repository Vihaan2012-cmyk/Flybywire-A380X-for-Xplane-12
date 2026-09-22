//! A port of Microsoft Flight Simulator's modular fuel system ("fuel system
//! v2", the `[FUEL_SYSTEM]` section of `flight_model.cfg`).
//!
//! In MSFS the A380X's fuel is moved by the simulator, not by FlyByWire's
//! code: FBW only commands pumps, valves, junctions and triggers through
//! `FUELSYSTEM_*` key events and reads `FUELSYSTEM *` simvars back. X-Plane has
//! nothing like it, so this module parses the same definition and simulates
//! it. It depends on `std` only.
//!
//! # What the SDK documents, and what this port does with it
//!
//! Components (`Tank`, `Pump`, `Valve`, `Junction`, `Engine`, `APU`) are
//! connected only by `Line`s, by name.
//!
//! * **Direction.** A line's Source/Destination only fixes its direction when
//!   it has `GravityBasedFuelFlow` (Asobo, MSFS DevSupport topic 14092:
//!   "The directionality of the line is determined by source and destination
//!   only for gravity based lines. Otherwise, the pressure will determine the
//!   direction"). Every other line may carry fuel either way, limited by its
//!   end components: a tank's `InputOnlyLines` only fill it and its
//!   `OutputOnlyLines` only drain it, the same for a junction; a valve with a
//!   `DestinationLine` only lets fuel out through that line; a pump takes fuel
//!   in through any other line and pushes it out through its
//!   `DestinationLine`; engines and APUs only take fuel in.
//! * **Pressure.** Active pumps push their `Pressure` into their destination
//!   line (an engine-driven pump scales it by its `PressureCurve` of engine
//!   RPM). A tank with a `PressureCurve` pushes curve(level) into the lines it
//!   may drain through. Junctions and open valves pass on the highest incoming
//!   pressure (the SDK's Fuel System Debug page shows a junction's "highest
//!   incoming pressure"); pumps in series do not add up, the outlet carries the
//!   higher of the pump and inlet pressure (DevSupport topic 17962 reports
//!   exactly that). Pressure does not pass through tanks.
//! * **Line pressure** is signed along Source→Destination. A two-way line
//!   pushed equally from both ends reads 0 psi and carries nothing (Asobo,
//!   topic 14092). The pressure an end pushes is worked out with that line
//!   removed, so a line never pushes against its own echo.
//! * **Flow.** A line carries at most `FuelFlowAt1PSI` (lb/s per psi, default
//!   0.1) times its pressure, and a gravity line also carries
//!   `GravityBasedFuelFlow` (gal/h) without pressure. Gravity flow only moves
//!   fuel between tanks, never to an engine, as the SDK says. Fuel leaves
//!   tanks down to their `UnusableCapacity` and fills tanks up to `Capacity`.
//! * **Engines and APUs** take the fuel the caller demands when the network
//!   can supply it. MSFS gives an engine its whole demand as long as its feed
//!   line is pressurised (DevSupport topic 10130). Engine and APU demand is
//!   served first, and tank-to-tank transfers get what is left of each line.
//! * **Junction options.** A junction option opens the lines it lists and
//!   closes the rest. The SDK does not say how options treat input lines. One
//!   rule fits both the SDK's P-51 selector and FBW's A380X cfg: when an
//!   option lists none of the junction's `InputOnlyLines`, all input lines
//!   stay open. The A380X's forward gallery options 1 and 2 list only outputs,
//!   yet FBW's inner-tank transfers run through them.
//! * **Pumps.** A pump is active when its switch is on (or on AUTO with its
//!   `AutoCondition` engine pressure below the threshold), its drive is
//!   available (electric: its `CIRCUIT_FUEL_PUMP:<Index>` circuit is powered;
//!   engine-driven: the engine turns; APU-driven: the APU runs), and its
//!   `TankFuelRequired` tank still has usable fuel. An inactive pump neither
//!   pushes pressure nor lets fuel through.
//! * **Valves.** The valve's position runs linearly towards its switch over
//!   `OpeningTime` (default 0.5 s) while its `CIRCUIT_FUEL_VALVE:<Circuit>`
//!   circuit is powered. A partly open valve scales the capacity of the
//!   lines on either side by how far it is open.
//! * **Triggers.** A trigger's condition is checked every update. After
//!   `DelayTrue`/`DelayFalse` its status changes, and `EffectTrue` or
//!   `EffectFalse` is applied once on that change. A `Manual` trigger only
//!   changes status through the `FUELSYSTEM_TRIGGER_*` events, and the same
//!   status-change rule applies its effects.
//! * **Tank `Priority`** only orders the fuel UI and time skips, per the SDK.
//!   See [`FuelNetwork::set_total_fuel_by_priority`].
//!
//! # Undocumented MSFS behaviour and the choices made here
//!
//! 1. **Flow gain.** Taken at the SDK's word (lb/s per psi), FBW's A380X
//!    transfer lines (`FuelFlowAt1PSI:0.00175` at 36.8 psi) would move about
//!    70 gal/h. FBW's PR #9045 tuned them to beat cruise fuel burn (about
//!    1000 gal/h per feed tank). MSFS's effective gain is therefore far
//!    higher. That fits fuel system versions up to 5 integrating per frame
//!    (version 6 notes say it "limits the framerate of the fuel system to
//!    30fps to provide a more stable time interval"). This port applies
//!    [`DEFAULT_LINE_FLOW_GAIN`] = 60, which is the documented unit per frame
//!    at 60 fps. Change it with [`FuelNetwork::set_line_flow_gain`].
//! 2. **Line contents.** A line's `Volume` (the fuel inside it) is not part of
//!    the fuel balance. `line_level` reads `Volume` while the line is
//!    pressurised or flowing, otherwise 0. The A380X's 172 lines hold about
//!    41 gal together, and its 1-gallon "Extra" tanks already give each
//!    engine the start-up buffer.
//! 3. **Sharing.** When several sinks share a bottleneck, the split comes
//!    from a max-flow solution, not from any documented rule.
//! 4. **Imbalance conditions.** `TankImbalance*` compares the signed
//!    difference (first target minus second) with the threshold, and
//!    `TankAbsImbalance*` compares the absolute difference. Both thresholds
//!    are in gallons, as the SDK states.
//! 5. **Start-up triggers.** On the first update a trigger whose condition is
//!    already true applies `EffectTrue`. `EffectFalse` is never applied at
//!    start-up.
//! 6. **Effects on other triggers.** `StartTrigger`/`StopTrigger` effects set
//!    the target trigger's status and apply its effects in turn, up to 8
//!    levels deep. Effects that name a component that does not exist are
//!    ignored and listed in [`FuelNetwork::warnings`]. The A380X cfg has
//!    `TrimLineIsolationValveAft_1` and `_2`.
//! 7. **Anemometer pumps** take their pressure fraction from
//!    [`FuelNetwork::set_anemometer_fraction`], default 0. Engines never send
//!    fuel back (no return lines).
//!
//! # Units
//! Gallons (US), gallons per hour and psi, as FBW requests them.
//! `FUELSYSTEM ENGINE PRESSURE` and `FUELSYSTEM LINE FUEL PRESSURE` default to
//! kPa in MSFS, and FBW reads them in psi.

use std::collections::{HashMap, HashSet, VecDeque};

use crate::failures;

/// SDK default for `Line.FuelFlowAt1PSI` (lb/s per psi).
pub const DEFAULT_LINE_FUEL_FLOW_AT_1PSI: f64 = 0.1;
/// SDK default for `Line.Volume` (gallons).
pub const DEFAULT_LINE_VOLUME_GAL: f64 = 0.24;
/// SDK default for `Valve.OpeningTime` (seconds).
pub const DEFAULT_VALVE_OPENING_TIME_S: f64 = 0.5;
/// SDK default for `Pump.PressureDecreaseRate` (fraction per second).
pub const DEFAULT_PRESSURE_DECREASE_RATE: f64 = 0.5;
/// Multiplier on the documented `FuelFlowAt1PSI` unit. See the module docs.
pub const DEFAULT_LINE_FLOW_GAIN: f64 = 60.0;
/// Jet A weight per US gallon, in pounds.
pub const JET_A_LBS_PER_GAL: f64 = 6.7;

const EPS: f64 = 1e-9;
const MAX_SUBSTEP_S: f64 = 0.1;
const MAX_EFFECT_DEPTH: u32 = 8;

// ---------------------------------------------------------------------------
// Definition (parsed cfg)
// ---------------------------------------------------------------------------

/// A 1-D piecewise linear curve (`Curve.N = x:y, x:y, ...`).
#[derive(Debug, Clone, PartialEq)]
pub struct Curve {
    pub points: Vec<(f64, f64)>,
}

impl Curve {
    /// Linear interpolation, clamped to the end points.
    pub fn eval(&self, x: f64) -> f64 {
        let p = &self.points;
        if p.is_empty() {
            return 0.0;
        }
        if x <= p[0].0 {
            return p[0].1;
        }
        for w in p.windows(2) {
            let (x0, y0) = w[0];
            let (x1, y1) = w[1];
            if x <= x1 {
                if (x1 - x0).abs() < EPS {
                    return y1;
                }
                return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
            }
        }
        p[p.len() - 1].1
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ApuDef {
    pub index: usize,
    pub name: String,
    pub title: Option<String>,
    /// Gallons per hour.
    pub fuel_burn_rate: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EngineDef {
    pub index: usize,
    pub name: String,
    pub title: Option<String>,
    /// The simulator engine index (1-based) this component stands for.
    pub engine_index: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TankDef {
    pub index: usize,
    pub name: String,
    pub title: Option<String>,
    pub capacity: f64,
    pub unusable_capacity: f64,
    pub position: Option<[f64; 3]>,
    pub priority: i64,
    pub pressure_curve: Option<usize>,
    pub input_only_lines: Vec<String>,
    pub output_only_lines: Vec<String>,
    pub drop_timer: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LineDef {
    pub index: usize,
    pub name: String,
    pub title: Option<String>,
    pub source: String,
    pub destination: String,
    /// lb/s per psi.
    pub fuel_flow_at_1psi: f64,
    /// Gallons.
    pub volume: f64,
    /// Gallons per hour.
    pub gravity_based_fuel_flow: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JunctionDef {
    pub index: usize,
    pub name: String,
    pub title: Option<String>,
    /// `Option` keys in definition order, option 1 first.
    pub options: Vec<Vec<String>>,
    pub input_only_lines: Vec<String>,
    pub output_only_lines: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ValveDef {
    pub index: usize,
    pub name: String,
    pub title: Option<String>,
    pub destination_line: Option<String>,
    pub opening_time: f64,
    /// Index N of `CIRCUIT_FUEL_VALVE:N`.
    pub circuit: Option<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PumpType {
    Electric,
    EngineDriven,
    ApuDriven,
    Manual,
    Anemometer,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PumpDef {
    pub index: usize,
    pub name: String,
    pub title: Option<String>,
    pub pressure: f64,
    pub pressure_curve: Option<usize>,
    pub tank_fuel_required: Option<String>,
    pub destination_line: String,
    pub pump_type: PumpType,
    /// Electric: `CIRCUIT_FUEL_PUMP:N`; engine-driven: engine index; anemometer index.
    pub type_index: Option<usize>,
    pub auto_condition: Option<(String, f64)>,
    pub pressure_decrease_rate: f64,
    pub pct_pressure_per_pump: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TriggerCondition {
    TankQuantityBelow,
    TankQuantityAbove,
    CgAboveLimit,
    CgBelowLimit,
    CgBetweenLimits,
    AutostartEnabled,
    AutoshutdownEnabled,
    Manual,
    TankImbalanceAbove,
    TankImbalanceBelow,
    TankAbsImbalanceAbove,
    TankAbsImbalanceBelow,
    JunctionOptionChanged,
    Unknown(String),
}

#[derive(Debug, Clone, PartialEq)]
pub enum TriggerEffect {
    OpenValve(String),
    CloseValve(String),
    StartPump(String),
    StopPump(String),
    SetJunction(String, usize),
    StartTrigger(String),
    StopTrigger(String),
    Unknown(String),
}

#[derive(Debug, Clone, PartialEq)]
pub struct TriggerDef {
    pub index: usize,
    pub name: String,
    pub title: Option<String>,
    pub target: Vec<String>,
    pub threshold: Vec<f64>,
    pub target_index: Option<usize>,
    pub delay_true: f64,
    pub delay_false: f64,
    pub condition: TriggerCondition,
    pub effect_true: Vec<TriggerEffect>,
    pub effect_false: Vec<TriggerEffect>,
}

/// Everything in a `[FUEL_SYSTEM]` section, each element list sorted by its N.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FuelSystemDef {
    pub version: Option<String>,
    /// `fuel_type` from `[FUEL_SYSTEM]` (version 6+) or `[FUEL]`.
    pub fuel_type: Option<i64>,
    pub apus: Vec<ApuDef>,
    pub engines: Vec<EngineDef>,
    pub tanks: Vec<TankDef>,
    pub lines: Vec<LineDef>,
    pub junctions: Vec<JunctionDef>,
    pub valves: Vec<ValveDef>,
    pub pumps: Vec<PumpDef>,
    pub triggers: Vec<TriggerDef>,
    pub curves: HashMap<usize, Curve>,
    /// Parse problems that were tolerated.
    pub warnings: Vec<String>,
}

fn strip_comment(line: &str) -> &str {
    let t = line.trim_start();
    if t.starts_with("//") {
        return "";
    }
    match line.find(';') {
        Some(i) => &line[..i],
        None => line,
    }
}

/// Reads the key/value pairs of one cfg section, case-insensitively.
/// Lines starting with `#` continue the previous value.
fn section_entries(text: &str, section: &str) -> Vec<(String, String)> {
    let want = section.to_ascii_lowercase();
    let mut in_section = false;
    let mut out: Vec<(String, String)> = Vec::new();
    for raw in text.lines() {
        let line = strip_comment(raw).trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('[') {
            let name = line.trim_start_matches('[').trim_end_matches(']').trim();
            in_section = name.eq_ignore_ascii_case(&want);
            continue;
        }
        if !in_section {
            continue;
        }
        if line.starts_with('#') {
            if let Some(last) = out.last_mut() {
                last.1.push_str(line);
            }
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            out.push((k.trim().to_ascii_lowercase(), v.trim().to_string()));
        }
    }
    out
}

/// Splits a `Key:Value#Key:Value` hash map. Keys are lower-cased.
fn parse_map(v: &str) -> Vec<(String, String)> {
    v.split('#')
        .filter_map(|part| {
            let part = part.trim();
            if part.is_empty() {
                return None;
            }
            match part.split_once(':') {
                Some((k, val)) => Some((k.trim().to_ascii_lowercase(), val.trim().to_string())),
                None => Some((part.to_ascii_lowercase(), String::new())),
            }
        })
        .collect()
}

fn map_get<'a>(m: &'a [(String, String)], key: &str) -> Option<&'a str> {
    m.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
}

fn map_list(m: &[(String, String)], key: &str) -> Vec<String> {
    map_get(m, key).map(split_list).unwrap_or_default()
}

fn split_list(v: &str) -> Vec<String> {
    v.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()).collect()
}

fn parse_f64(v: Option<&str>) -> Option<f64> {
    v.and_then(|s| s.trim().parse::<f64>().ok())
}

fn parse_usize(v: Option<&str>) -> Option<usize> {
    v.and_then(|s| {
        let s = s.trim();
        s.parse::<usize>().ok().or_else(|| s.parse::<f64>().ok().map(|f| f as usize))
    })
}

fn parse_effects(v: Option<&str>, warnings: &mut Vec<String>, trigger: &str) -> Vec<TriggerEffect> {
    let Some(v) = v else { return Vec::new() };
    split_list(v)
        .into_iter()
        .map(|e| {
            let Some((kind, rest)) = e.split_once('.') else {
                warnings.push(format!("trigger {trigger}: malformed effect '{e}'"));
                return TriggerEffect::Unknown(e);
            };
            let rest = rest.trim().to_string();
            match kind.trim().to_ascii_lowercase().as_str() {
                "openvalve" => TriggerEffect::OpenValve(rest),
                "closevalve" => TriggerEffect::CloseValve(rest),
                "startpump" => TriggerEffect::StartPump(rest),
                "stoppump" => TriggerEffect::StopPump(rest),
                "starttrigger" => TriggerEffect::StartTrigger(rest),
                "stoptrigger" => TriggerEffect::StopTrigger(rest),
                "setjunction" => match rest.rsplit_once('.') {
                    Some((name, opt)) if opt.trim().parse::<usize>().is_ok() => {
                        TriggerEffect::SetJunction(name.trim().to_string(), opt.trim().parse().unwrap())
                    }
                    _ => {
                        warnings.push(format!("trigger {trigger}: SetJunction without option '{e}'"));
                        TriggerEffect::Unknown(e)
                    }
                },
                _ => {
                    warnings.push(format!("trigger {trigger}: unknown effect '{e}'"));
                    TriggerEffect::Unknown(e)
                }
            }
        })
        .collect()
}

fn parse_condition(s: &str) -> TriggerCondition {
    match s.trim().to_ascii_lowercase().as_str() {
        "tankquantitybelow" => TriggerCondition::TankQuantityBelow,
        "tankquantityabove" => TriggerCondition::TankQuantityAbove,
        "cgabovelimit" => TriggerCondition::CgAboveLimit,
        "cgbelowlimit" => TriggerCondition::CgBelowLimit,
        "cgbetweenlimits" => TriggerCondition::CgBetweenLimits,
        "autostart_enabled" => TriggerCondition::AutostartEnabled,
        "autoshutdown_enabled" => TriggerCondition::AutoshutdownEnabled,
        "manual" => TriggerCondition::Manual,
        "tankimbalanceabove" => TriggerCondition::TankImbalanceAbove,
        "tankimbalancebelow" => TriggerCondition::TankImbalanceBelow,
        "tankabsimbalanceabove" => TriggerCondition::TankAbsImbalanceAbove,
        "tankabsimbalancebelow" => TriggerCondition::TankAbsImbalanceBelow,
        "junctionoptionchanged" => TriggerCondition::JunctionOptionChanged,
        _ => TriggerCondition::Unknown(s.trim().to_string()),
    }
}

/// Parses the `[FUEL_SYSTEM]` section (and `fuel_type` from `[FUEL]`) of a
/// `flight_model.cfg`.
pub fn parse_fuel_system(text: &str) -> Result<FuelSystemDef, String> {
    let entries = section_entries(text, "FUEL_SYSTEM");
    if entries.is_empty() {
        return Err("no [FUEL_SYSTEM] section".to_string());
    }
    let mut def = FuelSystemDef::default();
    let mut w = Vec::new();

    for (k, v) in section_entries(text, "FUEL") {
        if k == "fuel_type" {
            def.fuel_type = v.trim().parse().ok();
        }
    }

    for (key, value) in &entries {
        if key == "version" {
            def.version = Some(value.clone());
            continue;
        }
        if key == "fuel_type" {
            def.fuel_type = value.trim().parse().ok();
            continue;
        }
        let Some((kind, n)) = key.rsplit_once('.') else {
            w.push(format!("ignored key '{key}'"));
            continue;
        };
        let Ok(index) = n.trim().parse::<usize>() else {
            w.push(format!("ignored key '{key}': bad index"));
            continue;
        };
        if kind == "curve" {
            let mut points = Vec::new();
            for pair in value.split(',') {
                let pair = pair.trim();
                if pair.is_empty() {
                    continue;
                }
                match pair.split_once(':') {
                    Some((x, y)) => match (x.trim().parse::<f64>(), y.trim().parse::<f64>()) {
                        (Ok(x), Ok(y)) => points.push((x, y)),
                        _ => w.push(format!("curve {index}: bad point '{pair}'")),
                    },
                    None => w.push(format!("curve {index}: bad point '{pair}'")),
                }
            }
            points.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            def.curves.insert(index, Curve { points });
            continue;
        }
        let m = parse_map(value);
        let name = map_get(&m, "name").unwrap_or("").to_string();
        let title = map_get(&m, "title").map(str::to_string);
        if name.is_empty() && kind != "trigger" {
            w.push(format!("{kind}.{index} has no Name"));
        }
        match kind {
            "apu" => def.apus.push(ApuDef {
                index,
                name,
                title,
                fuel_burn_rate: parse_f64(map_get(&m, "fuelburnrate")).unwrap_or(0.0),
            }),
            "engine" => def.engines.push(EngineDef {
                index,
                name,
                title,
                engine_index: parse_usize(map_get(&m, "index")).unwrap_or(index),
            }),
            "tank" => {
                let position = map_get(&m, "position").and_then(|p| {
                    let v: Vec<f64> = p.split(',').filter_map(|s| s.trim().parse().ok()).collect();
                    (v.len() == 3).then(|| [v[0], v[1], v[2]])
                });
                let capacity = parse_f64(map_get(&m, "capacity")).unwrap_or_else(|| {
                    w.push(format!("tank {index}: no Capacity"));
                    0.0
                });
                def.tanks.push(TankDef {
                    index,
                    name,
                    title,
                    capacity,
                    unusable_capacity: parse_f64(map_get(&m, "unusablecapacity")).unwrap_or(0.0),
                    position,
                    priority: map_get(&m, "priority").and_then(|p| p.trim().parse().ok()).unwrap_or(0),
                    pressure_curve: parse_usize(map_get(&m, "pressurecurve")),
                    input_only_lines: map_list(&m, "inputonlylines"),
                    output_only_lines: map_list(&m, "outputonlylines"),
                    drop_timer: parse_f64(map_get(&m, "droptimer")),
                })
            }
            "line" => def.lines.push(LineDef {
                index,
                name,
                title,
                source: map_get(&m, "source").unwrap_or("").to_string(),
                destination: map_get(&m, "destination").unwrap_or("").to_string(),
                fuel_flow_at_1psi: parse_f64(map_get(&m, "fuelflowat1psi")).unwrap_or(DEFAULT_LINE_FUEL_FLOW_AT_1PSI),
                volume: parse_f64(map_get(&m, "volume")).unwrap_or(DEFAULT_LINE_VOLUME_GAL),
                gravity_based_fuel_flow: parse_f64(map_get(&m, "gravitybasedfuelflow")),
            }),
            "junction" => def.junctions.push(JunctionDef {
                index,
                name,
                title,
                options: m.iter().filter(|(k, _)| k == "option").map(|(_, v)| split_list(v)).collect(),
                input_only_lines: map_list(&m, "inputonlylines"),
                output_only_lines: map_list(&m, "outputonlylines"),
            }),
            "valve" => def.valves.push(ValveDef {
                index,
                name,
                title,
                destination_line: map_get(&m, "destinationline").filter(|s| !s.is_empty()).map(str::to_string),
                opening_time: parse_f64(map_get(&m, "openingtime")).unwrap_or(DEFAULT_VALVE_OPENING_TIME_S),
                circuit: parse_usize(map_get(&m, "circuit")),
            }),
            "pump" => {
                let t = map_get(&m, "type").unwrap_or("Electric").to_ascii_lowercase();
                let pump_type = match t.as_str() {
                    "electric" => PumpType::Electric,
                    "enginedriven" => PumpType::EngineDriven,
                    "apudriven" => PumpType::ApuDriven,
                    "manual" => PumpType::Manual,
                    "anemometer" => PumpType::Anemometer,
                    other => {
                        w.push(format!("pump {index}: unknown Type '{other}', treated as Electric"));
                        PumpType::Electric
                    }
                };
                let auto_condition = map_get(&m, "autocondition").and_then(|a| {
                    let (e, p) = a.split_once(',')?;
                    Some((e.trim().to_string(), p.trim().parse().ok()?))
                });
                def.pumps.push(PumpDef {
                    index,
                    name,
                    title,
                    pressure: parse_f64(map_get(&m, "pressure")).unwrap_or(0.0).max(0.0),
                    pressure_curve: parse_usize(map_get(&m, "pressurecurve")),
                    tank_fuel_required: map_get(&m, "tankfuelrequired").filter(|s| !s.is_empty()).map(str::to_string),
                    destination_line: map_get(&m, "destinationline").unwrap_or("").to_string(),
                    pump_type,
                    type_index: parse_usize(map_get(&m, "index")),
                    auto_condition,
                    pressure_decrease_rate: parse_f64(map_get(&m, "pressuredecreaserate"))
                        .unwrap_or(DEFAULT_PRESSURE_DECREASE_RATE),
                    pct_pressure_per_pump: parse_f64(map_get(&m, "pctpressureperpump")).unwrap_or(1.0),
                })
            }
            "trigger" => {
                let tname = if name.is_empty() { format!("Trigger.{index}") } else { name };
                let effect_true = parse_effects(map_get(&m, "effecttrue"), &mut w, &tname);
                let effect_false = parse_effects(map_get(&m, "effectfalse"), &mut w, &tname);
                def.triggers.push(TriggerDef {
                    index,
                    title,
                    target: map_list(&m, "target"),
                    threshold: map_get(&m, "threshold")
                        .map(|t| t.split(',').filter_map(|s| s.trim().parse().ok()).collect())
                        .unwrap_or_default(),
                    target_index: parse_usize(map_get(&m, "index")),
                    delay_true: parse_f64(map_get(&m, "delaytrue")).unwrap_or(0.0),
                    delay_false: parse_f64(map_get(&m, "delayfalse")).unwrap_or(0.0),
                    condition: parse_condition(map_get(&m, "condition").unwrap_or("")),
                    effect_true,
                    effect_false,
                    name: tname,
                })
            }
            other => w.push(format!("ignored element kind '{other}.{index}'")),
        }
    }
    def.apus.sort_by_key(|e| e.index);
    def.engines.sort_by_key(|e| e.index);
    def.tanks.sort_by_key(|e| e.index);
    def.lines.sort_by_key(|e| e.index);
    def.junctions.sort_by_key(|e| e.index);
    def.valves.sort_by_key(|e| e.index);
    def.pumps.sort_by_key(|e| e.index);
    def.triggers.sort_by_key(|e| e.index);
    def.warnings = w;
    Ok(def)
}

// ---------------------------------------------------------------------------
// Simulator
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Node {
    Tank(usize),
    Pump(usize),
    Valve(usize),
    Junction(usize),
    Engine(usize),
    Apu(usize),
    Missing,
}

#[derive(Debug, Clone, Default)]
struct JunctionTopo {
    inputs: HashSet<usize>,
    outputs: HashSet<usize>,
    /// Lines per option, plus whether the option names an input line.
    options: Vec<(HashSet<usize>, bool)>,
}

/// Max-flow network (Edmonds-Karp) with float capacities.
struct FlowNet {
    adj: Vec<Vec<usize>>,
    to: Vec<usize>,
    cap: Vec<f64>,
    orig: Vec<f64>,
}

impl FlowNet {
    fn new(n: usize) -> Self {
        Self { adj: vec![Vec::new(); n], to: Vec::new(), cap: Vec::new(), orig: Vec::new() }
    }
    fn add(&mut self, u: usize, v: usize, c: f64) -> usize {
        let e = self.to.len();
        self.to.push(v);
        self.cap.push(c.max(0.0));
        self.orig.push(c.max(0.0));
        self.adj[u].push(e);
        self.to.push(u);
        self.cap.push(0.0);
        self.orig.push(0.0);
        self.adj[v].push(e + 1);
        e
    }
    fn flow(&self, e: usize) -> f64 {
        (self.orig[e] - self.cap[e]).max(0.0)
    }
    fn run(&mut self, s: usize, t: usize) {
        let n = self.adj.len();
        loop {
            let mut prev: Vec<usize> = vec![usize::MAX; n];
            let mut seen = vec![false; n];
            seen[s] = true;
            let mut q = VecDeque::from([s]);
            while let Some(u) = q.pop_front() {
                if u == t {
                    break;
                }
                for &e in &self.adj[u] {
                    let v = self.to[e];
                    if !seen[v] && self.cap[e] > EPS {
                        seen[v] = true;
                        prev[v] = e;
                        q.push_back(v);
                    }
                }
            }
            if !seen[t] {
                return;
            }
            let mut bottleneck = f64::INFINITY;
            let mut v = t;
            while v != s {
                let e = prev[v];
                bottleneck = bottleneck.min(self.cap[e]);
                v = self.to[e ^ 1];
            }
            if bottleneck <= EPS || !bottleneck.is_finite() {
                return;
            }
            let mut v = t;
            while v != s {
                let e = prev[v];
                self.cap[e] -= bottleneck;
                self.cap[e ^ 1] += bottleneck;
                v = self.to[e ^ 1];
            }
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct LineSolve {
    /// +1 source to destination, -1 reverse, 0 no flow possible.
    dir: i8,
    pressure: f64,
    cap_pressure_gph: f64,
    cap_gravity_gph: f64,
}

/// The simulated fuel system. Every index this API takes is the 1-based `N`
/// of the matching `Tank.N`, `Pump.N`, `Valve.N`, `Line.N`, `Junction.N` or
/// `Trigger.N` in the cfg, which is also the index of the MSFS simvars and
/// events. An index that does not exist reads 0 or false, and commands to it
/// are ignored.
pub struct FuelNetwork {
    /// Each pump's own pressure and each node's own pushed pressure (a
    /// tank's head, an active pump's), fixed for one `solve_lines`:
    /// computed once there instead of once per line and relaxation pass.
    solve_pump_p: Vec<f64>,
    solve_base_out: Vec<f64>,
    def: FuelSystemDef,
    warnings: Vec<String>,
    nodes: Vec<Node>,
    incident: Vec<Vec<usize>>,
    line_src: Vec<usize>,
    line_dst: Vec<usize>,
    tank_in_only: Vec<HashSet<usize>>,
    tank_out_only: Vec<HashSet<usize>>,
    junction_topo: Vec<JunctionTopo>,
    valve_dest: Vec<Option<usize>>,
    pump_dest: Vec<Option<usize>>,
    pump_tank: Vec<Option<usize>>,
    pump_auto_engine: Vec<Option<usize>>,
    names: HashMap<String, usize>,
    tank_n: HashMap<usize, usize>,
    pump_n: HashMap<usize, usize>,
    valve_n: HashMap<usize, usize>,
    line_n: HashMap<usize, usize>,
    junction_n: HashMap<usize, usize>,
    trigger_n: HashMap<usize, usize>,
    trigger_names: HashMap<String, usize>,

    // state
    tank_qty: Vec<f64>,
    pump_switch: Vec<u8>,
    pump_active: Vec<bool>,
    pump_manual_fraction: Vec<f64>,
    valve_switch: Vec<bool>,
    valve_open: Vec<f64>,
    junction_setting: Vec<usize>,
    trigger_status: Vec<bool>,
    trigger_timer: Vec<f64>,
    line_flow: Vec<f64>,
    line_pressure: Vec<f64>,
    line_level: Vec<f64>,
    engine_pressure: Vec<f64>,
    engine_delivered_gph: Vec<f64>,
    engine_demand_gph: Vec<f64>,
    engine_fed: Vec<bool>,
    /// hyperrealism.md physics workstream (fuel second pass): the APU feed
    /// line's own pressure, tracked the same way [`Self::engine_pressure`]
    /// already is (highest-pressure line entering the `Node::Apu` node this
    /// solve), where before the APU feed had no pressure concept at all --
    /// only a demand/delivered gallon flow. Exposed via
    /// [`FuelNetwork::apu_feed_pressure_psi`] for the APU workstream's own
    /// feed-pressure consumer.
    apu_pressure: Vec<f64>,
    apu_delivered_gph: Vec<f64>,
    apu_demand_gph: Vec<f64>,
    apu_fed: Vec<bool>,
    pump_circuits: HashMap<usize, bool>,
    valve_circuits: HashMap<usize, bool>,
    engine_rpm: HashMap<usize, f64>,
    anemometer: HashMap<usize, f64>,
    apu_running: bool,
    cg_percent: f64,
    autostart: bool,
    autoshutdown: bool,
    density: f64,
    gain: f64,
    /// hyperrealism.md physics workstream 5 (fluids): a multiplier (0-1) on
    /// every line's flow-per-psi conductance, from the fuel's own kinematic
    /// viscosity (`physics::fluids::viscosity_flow_derate`, set once per tick
    /// by `fuel.rs` from its tank temperature model). 1.0 (no derate, the
    /// reference-viscosity default) until `fuel.rs` calls
    /// [`FuelNetwork::set_viscosity_derate`]. Applying it uniformly to every
    /// line's `k` is the network-model equivalent of Hagen-Poiseuille pipe
    /// flow (`Q = k * dP`, with real pipe conductance `k` proportional to
    /// `1/mu`): this network already treats each line as a linear
    /// pressure-to-flow conductance (`fuel_flow_at_1psi`) rather than solving
    /// a full sqrt(dP) orifice per line, so a viscosity derate on that same
    /// conductance is the physically consistent way to plumb temperature-
    /// dependent thickening into it, not a new shortcut.
    viscosity_derate: f64,
    /// hyperrealism.md physics workstream 5 (fluids): a per-tank multiplier
    /// (0-1) on every pump drawing from that tank, from
    /// `physics::fluids::unporting_factor` (set once per tick by `fuel.rs`
    /// from the tank's own fill level and the aircraft's pitch/bank). 1.0
    /// (no derate) for every tank until `fuel.rs` calls
    /// [`FuelNetwork::set_tank_pump_derate`]. Indexed by the same internal
    /// tank index `pump_tank` already stores, so `pump_own_pressure` can look
    /// it up directly without a tank-number round-trip.
    pump_tank_derate: Vec<f64>,
    /// The `failures.rs` `extra::fuel` catalogue (ids `28_000`-`28_011`,
    /// `Effect::Hook { var: "FAIL_FUEL_HOOK", .. }`): nothing previously
    /// consumed that hook, so every one of these entries armed in the EFB
    /// failure page did nothing. [`Self::refresh_catalogue_failures`] reads
    /// `failures::magnitude` for each id once per [`Self::update`] and
    /// stores the continuous fraction against the real pump/valve here by
    /// internal index (`0.0` healthy/not stuck, up to `1.0` fully failed),
    /// so a feed/trim pump failure derates that one pump's own delivery
    /// pressure ([`Self::pump_own_pressure`]) proportionally along its
    /// curve without touching its sibling pump on the same tank, and a
    /// cross-feed/jettison valve failure sticks that valve at a fraction of
    /// the way from wherever it was when it seized towards its commanded
    /// position -- `1.0` freezes it completely (it stops tracking its
    /// switch at all in [`Self::substep`]), matching "sticks; no longer
    /// opens or closes" in the catalogue description, while a lower
    /// fraction leaves it partial authority, i.e. "sticks at y%".
    pump_fail: Vec<f64>,
    valve_fail: Vec<f64>,
    /// The valve's own `valve_open` fraction captured the instant its
    /// [`Self::valve_fail`] magnitude last went from `0.0` to positive --
    /// the position it is stuck relative to. See `valve_fail`'s doc.
    valve_stuck_at: Vec<f64>,
    started: bool,
    total_burnt_gal: f64,
}

fn fuel_type_density(fuel_type: Option<i64>) -> f64 {
    // Approximate US-gallon weights of the MSFS fuel types.
    match fuel_type {
        Some(1) | Some(3) | Some(4) => 6.0, // avgas 100, avgas 80, autogas
        Some(5) => 6.5,                     // Jet B
        Some(6) => 4.2,                     // liquid propane
        _ => JET_A_LBS_PER_GAL,             // Jet A and unknown
    }
}

impl FuelNetwork {
    /// Parses `flight_model.cfg` text and builds the simulator. Every switch
    /// starts off, every junction on option 1, every trigger off and every
    /// tank empty, which is MSFS's state before the `.flt` file is applied.
    /// See [`FuelNetwork::apply_flt_state`].
    pub fn from_cfg(text: &str) -> Result<Self, String> {
        Self::from_def(parse_fuel_system(text)?)
    }

    /// Builds the simulator from an already parsed definition.
    pub fn from_def(def: FuelSystemDef) -> Result<Self, String> {
        let mut warnings = def.warnings.clone();
        let mut nodes = Vec::new();
        let mut names: HashMap<String, usize> = HashMap::new();
        let mut add_node = |name: &str, node: Node, nodes: &mut Vec<Node>, warnings: &mut Vec<String>| {
            let id = nodes.len();
            nodes.push(node);
            if names.insert(name.to_string(), id).is_some() {
                warnings.push(format!("duplicate component name '{name}'"));
            }
        };
        for (i, t) in def.tanks.iter().enumerate() {
            add_node(&t.name, Node::Tank(i), &mut nodes, &mut warnings);
        }
        for (i, p) in def.pumps.iter().enumerate() {
            add_node(&p.name, Node::Pump(i), &mut nodes, &mut warnings);
        }
        for (i, v) in def.valves.iter().enumerate() {
            add_node(&v.name, Node::Valve(i), &mut nodes, &mut warnings);
        }
        for (i, j) in def.junctions.iter().enumerate() {
            add_node(&j.name, Node::Junction(i), &mut nodes, &mut warnings);
        }
        for (i, e) in def.engines.iter().enumerate() {
            add_node(&e.name, Node::Engine(i), &mut nodes, &mut warnings);
        }
        for (i, a) in def.apus.iter().enumerate() {
            add_node(&a.name, Node::Apu(i), &mut nodes, &mut warnings);
        }
        let lower: HashMap<String, usize> = names.iter().map(|(k, v)| (k.to_ascii_lowercase(), *v)).collect();
        let missing = nodes.len();
        nodes.push(Node::Missing);

        let resolve = |name: &str, what: &str, warnings: &mut Vec<String>| -> usize {
            if let Some(&id) = names.get(name) {
                return id;
            }
            if let Some(&id) = lower.get(&name.to_ascii_lowercase()) {
                warnings.push(format!("{what}: '{name}' matched case-insensitively"));
                return id;
            }
            warnings.push(format!("{what}: unknown component '{name}'"));
            missing
        };

        let n_lines = def.lines.len();
        let mut line_names: HashMap<String, usize> = HashMap::new();
        let mut line_src = Vec::with_capacity(n_lines);
        let mut line_dst = Vec::with_capacity(n_lines);
        let mut incident = vec![Vec::new(); nodes.len()];
        for (i, l) in def.lines.iter().enumerate() {
            if line_names.insert(l.name.clone(), i).is_some() {
                warnings.push(format!("duplicate line name '{}'", l.name));
            }
            let s = resolve(&l.source, &format!("line {} source", l.index), &mut warnings);
            let d = resolve(&l.destination, &format!("line {} destination", l.index), &mut warnings);
            line_src.push(s);
            line_dst.push(d);
            if s != missing {
                incident[s].push(i);
            }
            if d != missing && d != s {
                incident[d].push(i);
            }
        }
        let line_lower: HashMap<String, usize> =
            line_names.iter().map(|(k, v)| (k.to_ascii_lowercase(), *v)).collect();
        let line_id = |name: &str, what: &str, warnings: &mut Vec<String>| -> Option<usize> {
            if let Some(&i) = line_names.get(name) {
                return Some(i);
            }
            if let Some(&i) = line_lower.get(&name.to_ascii_lowercase()) {
                warnings.push(format!("{what}: line '{name}' matched case-insensitively"));
                return Some(i);
            }
            warnings.push(format!("{what}: unknown line '{name}'"));
            None
        };
        let line_set = |list: &[String], what: &str, warnings: &mut Vec<String>| -> HashSet<usize> {
            list.iter().filter_map(|n| line_id(n, what, warnings)).collect()
        };

        let tank_in_only: Vec<_> = def
            .tanks
            .iter()
            .map(|t| line_set(&t.input_only_lines, &format!("tank {}", t.index), &mut warnings))
            .collect();
        let tank_out_only: Vec<_> = def
            .tanks
            .iter()
            .map(|t| line_set(&t.output_only_lines, &format!("tank {}", t.index), &mut warnings))
            .collect();
        let junction_topo: Vec<_> = def
            .junctions
            .iter()
            .map(|j| {
                let what = format!("junction {}", j.index);
                let inputs = line_set(&j.input_only_lines, &what, &mut warnings);
                let outputs = line_set(&j.output_only_lines, &what, &mut warnings);
                let options = j
                    .options
                    .iter()
                    .map(|o| {
                        let set = line_set(o, &what, &mut warnings);
                        let lists_input = set.iter().any(|l| inputs.contains(l));
                        (set, lists_input)
                    })
                    .collect();
                JunctionTopo { inputs, outputs, options }
            })
            .collect();
        let valve_dest: Vec<_> = def
            .valves
            .iter()
            .map(|v| v.destination_line.as_deref().and_then(|d| line_id(d, &format!("valve {}", v.index), &mut warnings)))
            .collect();
        let pump_dest: Vec<_> = def
            .pumps
            .iter()
            .map(|p| line_id(&p.destination_line, &format!("pump {}", p.index), &mut warnings))
            .collect();
        let tank_by_name: HashMap<&str, usize> = def.tanks.iter().enumerate().map(|(i, t)| (t.name.as_str(), i)).collect();
        let pump_tank: Vec<_> = def
            .pumps
            .iter()
            .map(|p| {
                p.tank_fuel_required.as_deref().and_then(|t| {
                    let r = tank_by_name.get(t).copied();
                    if r.is_none() {
                        warnings.push(format!("pump {}: unknown TankFuelRequired '{t}'", p.index));
                    }
                    r
                })
            })
            .collect();
        let pump_auto_engine: Vec<_> = def
            .pumps
            .iter()
            .map(|p| {
                p.auto_condition.as_ref().and_then(|(e, _)| {
                    let r = def.engines.iter().position(|en| en.name == *e);
                    if r.is_none() {
                        warnings.push(format!("pump {}: unknown AutoCondition engine '{e}'", p.index));
                    }
                    r
                })
            })
            .collect();
        for p in &def.pumps {
            if let Some(c) = p.pressure_curve {
                if !def.curves.contains_key(&c) {
                    warnings.push(format!("pump {}: unknown PressureCurve {c}", p.index));
                }
            }
        }
        for t in &def.tanks {
            if let Some(c) = t.pressure_curve {
                if !def.curves.contains_key(&c) {
                    warnings.push(format!("tank {}: unknown PressureCurve {c}", t.index));
                }
            }
        }
        let trigger_names: HashMap<String, usize> =
            def.triggers.iter().enumerate().map(|(i, t)| (t.name.clone(), i)).collect();
        let valve_names: HashSet<&str> = def.valves.iter().map(|v| v.name.as_str()).collect();
        let pump_names: HashSet<&str> = def.pumps.iter().map(|v| v.name.as_str()).collect();
        let junction_names: HashSet<&str> = def.junctions.iter().map(|v| v.name.as_str()).collect();
        for t in &def.triggers {
            for e in t.effect_true.iter().chain(t.effect_false.iter()) {
                let ok = match e {
                    TriggerEffect::OpenValve(n) | TriggerEffect::CloseValve(n) => valve_names.contains(n.as_str()),
                    TriggerEffect::StartPump(n) | TriggerEffect::StopPump(n) => pump_names.contains(n.as_str()),
                    TriggerEffect::SetJunction(n, _) => junction_names.contains(n.as_str()),
                    TriggerEffect::StartTrigger(n) | TriggerEffect::StopTrigger(n) => trigger_names.contains_key(n),
                    TriggerEffect::Unknown(_) => true,
                };
                if !ok {
                    warnings.push(format!("trigger {} ({}): effect {:?} targets an unknown component", t.index, t.name, e));
                }
            }
            if let TriggerCondition::Unknown(c) = &t.condition {
                warnings.push(format!("trigger {}: unknown condition '{c}', never true", t.index));
            }
        }

        let idx = |v: Vec<usize>| -> HashMap<usize, usize> { v.into_iter().enumerate().map(|(i, n)| (n, i)).collect() };
        let nt = def.tanks.len();
        let np = def.pumps.len();
        let nv = def.valves.len();
        let nj = def.junctions.len();
        let ntr = def.triggers.len();
        let ne = def.engines.len();
        let na = def.apus.len();
        Ok(Self {
            solve_pump_p: Vec::new(),
            solve_base_out: Vec::new(),
            tank_n: idx(def.tanks.iter().map(|t| t.index).collect()),
            pump_n: idx(def.pumps.iter().map(|t| t.index).collect()),
            valve_n: idx(def.valves.iter().map(|t| t.index).collect()),
            line_n: idx(def.lines.iter().map(|t| t.index).collect()),
            junction_n: idx(def.junctions.iter().map(|t| t.index).collect()),
            trigger_n: idx(def.triggers.iter().map(|t| t.index).collect()),
            density: fuel_type_density(def.fuel_type),
            def,
            warnings,
            nodes,
            incident,
            line_src,
            line_dst,
            tank_in_only,
            tank_out_only,
            junction_topo,
            valve_dest,
            pump_dest,
            pump_tank,
            pump_auto_engine,
            names,
            trigger_names,
            tank_qty: vec![0.0; nt],
            pump_switch: vec![0; np],
            pump_active: vec![false; np],
            pump_manual_fraction: vec![0.0; np],
            valve_switch: vec![false; nv],
            valve_open: vec![0.0; nv],
            junction_setting: vec![1; nj],
            trigger_status: vec![false; ntr],
            trigger_timer: vec![0.0; ntr],
            line_flow: vec![0.0; n_lines],
            line_pressure: vec![0.0; n_lines],
            line_level: vec![0.0; n_lines],
            engine_pressure: vec![0.0; ne],
            engine_delivered_gph: vec![0.0; ne],
            engine_demand_gph: vec![0.0; ne],
            engine_fed: vec![false; ne],
            apu_pressure: vec![0.0; na],
            apu_delivered_gph: vec![0.0; na],
            apu_demand_gph: vec![0.0; na],
            apu_fed: vec![false; na],
            pump_circuits: HashMap::new(),
            valve_circuits: HashMap::new(),
            engine_rpm: HashMap::new(),
            anemometer: HashMap::new(),
            apu_running: false,
            cg_percent: 0.0,
            autostart: false,
            autoshutdown: false,
            gain: DEFAULT_LINE_FLOW_GAIN,
            viscosity_derate: 1.0,
            pump_tank_derate: vec![1.0; nt],
            pump_fail: vec![0.0; np],
            valve_fail: vec![0.0; nv],
            valve_stuck_at: vec![0.0; nv],
            started: false,
            total_burnt_gal: 0.0,
        })
    }

    /// The parsed definition.
    pub fn definition(&self) -> &FuelSystemDef {
        &self.def
    }

    /// Parse and topology problems that were tolerated.
    pub fn warnings(&self) -> &[String] {
        &self.warnings
    }

    /// Applies an MSFS `.flt` file's `[FuelSystem.0]` section: `Valve.N` and
    /// `Pump.N` switches (valves jump straight to position), `Tank.N` levels
    /// (fraction of capacity), `Junction.N` options and `Trigger.N` statuses.
    /// Trigger effects are not applied, since the saved state already includes
    /// them. Returns how many entries were applied.
    pub fn apply_flt_state(&mut self, flt_text: &str) -> usize {
        let mut in_section = false;
        let mut applied = 0;
        for raw in flt_text.lines() {
            let line = strip_comment(raw).trim();
            if line.starts_with('[') {
                let name = line.trim_start_matches('[').trim_end_matches(']').trim().to_ascii_lowercase();
                in_section = name.starts_with("fuelsystem");
                continue;
            }
            if !in_section {
                continue;
            }
            let Some((k, v)) = line.split_once('=') else { continue };
            let Some((kind, n)) = k.trim().rsplit_once('.') else { continue };
            let Ok(n) = n.trim().parse::<usize>() else { continue };
            let v = v.trim();
            let as_bool = || v.eq_ignore_ascii_case("true") || v.parse::<f64>().map(|f| f != 0.0).unwrap_or(false);
            match kind.trim().to_ascii_lowercase().as_str() {
                "valve" => {
                    if let Some(&i) = self.valve_n.get(&n) {
                        let b = as_bool();
                        self.valve_switch[i] = b;
                        self.valve_open[i] = if b { 1.0 } else { 0.0 };
                        applied += 1;
                    }
                }
                "pump" => {
                    if let Some(&i) = self.pump_n.get(&n) {
                        self.pump_switch[i] = if v.eq_ignore_ascii_case("true") {
                            1
                        } else {
                            v.parse::<f64>().map(|f| f.clamp(0.0, 2.0) as u8).unwrap_or(0)
                        };
                        applied += 1;
                    }
                }
                "tank" => {
                    if let (Some(&i), Ok(f)) = (self.tank_n.get(&n), v.parse::<f64>()) {
                        self.tank_qty[i] = (f.clamp(0.0, 1.0)) * self.def.tanks[i].capacity;
                        applied += 1;
                    }
                }
                "junction" => {
                    if let (Some(&i), Ok(o)) = (self.junction_n.get(&n), v.parse::<f64>()) {
                        self.set_junction_pos(i, o as usize);
                        applied += 1;
                    }
                }
                "trigger" => {
                    if let Some(&i) = self.trigger_n.get(&n) {
                        self.trigger_status[i] = as_bool();
                        applied += 1;
                    }
                }
                _ => {}
            }
        }
        applied
    }

    // ----- tuning / environment -------------------------------------------

    /// Fuel weight, used to turn `FuelFlowAt1PSI` (lb/s) into gal/h.
    pub fn set_fuel_density_lbs_per_gal(&mut self, lbs_per_gal: f64) {
        if lbs_per_gal > 0.0 {
            self.density = lbs_per_gal;
        }
    }
    pub fn fuel_density_lbs_per_gal(&self) -> f64 {
        self.density
    }
    /// See [`DEFAULT_LINE_FLOW_GAIN`].
    pub fn set_line_flow_gain(&mut self, gain: f64) {
        if gain > 0.0 {
            self.gain = gain;
        }
    }
    /// hyperrealism.md physics workstream 5: `fuel.rs` calls this once per
    /// tick with `physics::fluids::viscosity_flow_derate` of the current
    /// (feed-tank) fuel temperature's kinematic viscosity, so cold/thickened
    /// fuel genuinely throttles every line's flow capacity here (see the
    /// `viscosity_derate` field doc for why a uniform multiplier on the
    /// linear line conductance is the physically consistent hook, not the
    /// full per-line sqrt(dP) orifice form). Clamped to the same [0.05, 1.0]
    /// range `viscosity_flow_derate` itself returns, so a bad caller value
    /// can never zero out or invert flow.
    pub fn set_viscosity_derate(&mut self, derate: f64) {
        self.viscosity_derate = derate.clamp(0.05, 1.0);
    }
    pub fn viscosity_derate(&self) -> f64 {
        self.viscosity_derate
    }
    /// hyperrealism.md physics workstream 5: `fuel.rs` calls this once per
    /// tick per tank with `physics::fluids::unporting_factor`, so a boost
    /// pump's own delivery pressure genuinely falls as its tank's fuel level
    /// drops and as pitch/bank tilt the fuel surface away from the inlet
    /// (`pump_own_pressure`'s own doc), instead of the pump running at full
    /// rated pressure right up until the tank's `unusablecapacity` cliff.
    /// `tank` is the external tank number (as `set_tank_gallons` takes);
    /// unknown tank numbers are silently ignored, matching this module's
    /// other per-tank setters. Clamped to 0-1 so a bad caller value can never
    /// invert or boost pump pressure.
    pub fn set_tank_pump_derate(&mut self, tank: usize, factor: f64) {
        if let Some(&i) = self.tank_n.get(&tank) {
            self.pump_tank_derate[i] = factor.clamp(0.0, 1.0);
        }
    }
    /// Breaker-coupling workstream: the same per-tank inlet-coverage derate
    /// [`set_tank_pump_derate`] already applies to a pump's own delivery
    /// *pressure* also implies extra motor *current* -- a cavitating pump
    /// (an uncovered inlet drawing vapour/air) loads its motor unevenly,
    /// not a quiet, undriven, zero-current one. Returns that pump's extra
    /// mechanical-load fraction (0.0 fully covered/no extra load, 1.0 fully
    /// uncovered/unported -- the complement of the coverage factor
    /// `set_tank_pump_derate` stores) for the `PumpType::Electric` pump
    /// whose `type_index` (`CIRCUIT_FUEL_PUMP:N`, the same number
    /// `fuel.rs`'s `power_circuits` already has per circuit) is
    /// `circuit_type_index`. `fuel.rs` reads this once per tick per fuel
    /// pump circuit and publishes it to `physics::motor`'s registry keyed
    /// by that circuit's own `breakers.rs` catalogue id
    /// (`"sys-<circuit.number>"`), so the feed pump breaker's estimated
    /// current genuinely rises with fuel geometry (tank level, pitch/bank),
    /// not a flag. `0.0` (no extra load) for any circuit index with no
    /// matching electric pump, no tank requirement, or an unknown tank --
    /// a coupling that finds nothing real must never invent a load.
    pub fn pump_cavitation_load(&self, circuit_type_index: usize) -> f64 {
        let Some(p) = self.def.pumps.iter().find(|p| p.pump_type == PumpType::Electric && p.type_index == Some(circuit_type_index)) else {
            return 0.0;
        };
        let Some(tank_name) = p.tank_fuel_required.as_deref() else { return 0.0 };
        let Some(n) = self.tank_index(tank_name) else { return 0.0 };
        let Some(&i) = self.tank_n.get(&n) else { return 0.0 };
        (1.0 - self.pump_tank_derate[i]).clamp(0.0, 1.0)
    }

    /// This electric pump's *delivered hydraulic power*, as a fraction of
    /// its rated hydraulic power (`physics::motor::hydraulic_power_current_
    /// multiplier`'s own input) -- the physically correct cavitation
    /// coupling (fbw-xp-systems' motor-coupling fix, see that function's own
    /// doc for why an earlier pass's "cavitation as extra mechanical load"
    /// coupling was backwards).
    ///
    /// `pump_own_pressure` (above) already derates a cavitating electric
    /// pump's *delivery pressure* linearly by inlet coverage: `delta_p =
    /// rated_pressure * derate`. The standard centrifugal-pump shaft-power
    /// relation is `P_shaft = delta_p * Q / eta` (Karassik et al., *Pump
    /// Handbook*, ch. 2); in the same network's resistive flow model, flow
    /// through a fixed downstream resistance is itself driven by (and, in
    /// the normal partial-cavitation operating range, tracks) that same
    /// delivery pressure, so `Q` derates by approximately the same factor.
    /// With both `delta_p` and `Q` scaling by `coverage`, `P_shaft` scales by
    /// `coverage^2` -- a *derived* consequence of this module's own already-
    /// tested pressure model (`pump_pressure_psi`'s doc/tests), not an
    /// independently authored number, and explicitly labelled an
    /// approximation (real `Q` is the network's own full nonlinear solve,
    /// which this does not re-run) rather than claimed as a second
    /// measurement.
    pub fn pump_hydraulic_power_fraction(&self, circuit_type_index: usize) -> f64 {
        let coverage = 1.0 - self.pump_cavitation_load(circuit_type_index);
        coverage * coverage
    }
    /// Powers `CIRCUIT_FUEL_PUMP:<circuit>`. Every circuit starts powered.
    pub fn set_fuel_pump_circuit_powered(&mut self, circuit: usize, powered: bool) {
        self.pump_circuits.insert(circuit, powered);
    }
    /// Powers `CIRCUIT_FUEL_VALVE:<circuit>`. Every circuit starts powered.
    pub fn set_fuel_valve_circuit_powered(&mut self, circuit: usize, powered: bool) {
        self.valve_circuits.insert(circuit, powered);
    }
    /// Engine RPM as a fraction of its maximum, for engine-driven pumps.
    pub fn set_engine_rpm_fraction(&mut self, engine_index: usize, fraction: f64) {
        self.engine_rpm.insert(engine_index, fraction.clamp(0.0, 1.0));
    }
    /// Drive fraction (0..1) of an anemometer pump, by its `Index`.
    pub fn set_anemometer_fraction(&mut self, anemometer_index: usize, fraction: f64) {
        self.anemometer.insert(anemometer_index, fraction.clamp(0.0, 1.0));
    }
    /// Whether the APU runs, for APU-driven pumps.
    pub fn set_apu_running(&mut self, running: bool) {
        self.apu_running = running;
    }
    /// `CG PERCENT`, for the CG trigger conditions.
    pub fn set_cg_percent(&mut self, cg_percent: f64) {
        self.cg_percent = cg_percent;
    }
    pub fn set_autostart_enabled(&mut self, on: bool) {
        self.autostart = on;
    }
    pub fn set_autoshutdown_enabled(&mut self, on: bool) {
        self.autoshutdown = on;
    }

    // ----- tanks -----------------------------------------------------------

    pub fn tank_count(&self) -> usize {
        self.def.tanks.len()
    }
    /// The 1-based `N` of the tank with this name.
    pub fn tank_index(&self, name: &str) -> Option<usize> {
        self.def.tanks.iter().find(|t| t.name == name).map(|t| t.index)
    }
    /// `FUELSYSTEM TANK QUANTITY`: usable gallons, without unusable fuel.
    pub fn tank_gallons(&self, n: usize) -> f64 {
        self.tank_n.get(&n).map_or(0.0, |&i| (self.tank_qty[i] - self.def.tanks[i].unusable_capacity).max(0.0))
    }
    /// `FUELSYSTEM TANK TOTAL QUANTITY`: gallons, with unusable fuel.
    pub fn tank_total_gallons(&self, n: usize) -> f64 {
        self.tank_n.get(&n).map_or(0.0, |&i| self.tank_qty[i])
    }
    /// Sets `FUELSYSTEM TANK QUANTITY` (usable gallons), clamped to capacity.
    pub fn set_tank_gallons(&mut self, n: usize, gallons: f64) {
        if let Some(&i) = self.tank_n.get(&n) {
            let t = &self.def.tanks[i];
            let g = if gallons.is_finite() { gallons } else { 0.0 };
            self.tank_qty[i] = if g <= 0.0 { 0.0 } else { (g + t.unusable_capacity).min(t.capacity) };
        }
    }
    /// `FUELSYSTEM TANK CAPACITY` in gallons.
    pub fn tank_capacity(&self, n: usize) -> f64 {
        self.tank_n.get(&n).map_or(0.0, |&i| self.def.tanks[i].capacity)
    }
    pub fn tank_unusable_capacity(&self, n: usize) -> f64 {
        self.tank_n.get(&n).map_or(0.0, |&i| self.def.tanks[i].unusable_capacity)
    }
    /// `FUELSYSTEM TANK LEVEL`: total quantity over capacity, 0..1.
    pub fn tank_level(&self, n: usize) -> f64 {
        self.tank_n.get(&n).map_or(0.0, |&i| self.level(i))
    }
    /// `FUELSYSTEM TANK WEIGHT` in pounds.
    pub fn tank_weight_lbs(&self, n: usize) -> f64 {
        self.tank_gallons(n) * self.density
    }
    /// Total gallons in all tanks, unusable fuel included.
    pub fn total_fuel_gallons(&self) -> f64 {
        self.tank_qty.iter().sum()
    }
    /// Gallons delivered to engines and APUs since the network was built.
    pub fn total_burnt_gallons(&self) -> f64 {
        self.total_burnt_gal
    }
    /// Spreads `total` gallons over the tanks the way the MSFS fuel UI does.
    /// Higher `Priority` tanks fill first, and tanks of equal priority fill
    /// to the same fraction of their capacity. Returns the gallons that did
    /// not fit.
    pub fn set_total_fuel_by_priority(&mut self, total: f64) -> f64 {
        let mut remaining = total.max(0.0);
        for q in &mut self.tank_qty {
            *q = 0.0;
        }
        let mut prios: Vec<i64> = self.def.tanks.iter().map(|t| t.priority).collect();
        prios.sort_unstable();
        prios.dedup();
        for p in prios.into_iter().rev() {
            let group: Vec<usize> = (0..self.def.tanks.len()).filter(|&i| self.def.tanks[i].priority == p).collect();
            let cap: f64 = group.iter().map(|&i| self.def.tanks[i].capacity).sum();
            if cap <= 0.0 {
                continue;
            }
            let frac = (remaining / cap).min(1.0);
            for &i in &group {
                self.tank_qty[i] = self.def.tanks[i].capacity * frac;
            }
            remaining -= cap * frac;
            if remaining <= EPS {
                return 0.0;
            }
        }
        remaining
    }

    fn level(&self, t: usize) -> f64 {
        let c = self.def.tanks[t].capacity;
        if c > 0.0 {
            (self.tank_qty[t] / c).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }
    fn usable(&self, t: usize) -> f64 {
        (self.tank_qty[t] - self.def.tanks[t].unusable_capacity).max(0.0)
    }

    // ----- pumps -----------------------------------------------------------

    pub fn pump_count(&self) -> usize {
        self.def.pumps.len()
    }
    pub fn pump_index(&self, name: &str) -> Option<usize> {
        self.def.pumps.iter().find(|p| p.name == name).map(|p| p.index)
    }
    /// `FUELSYSTEM_PUMP_SET`: 0 = off, 1 = on, 2 = auto.
    pub fn set_pump(&mut self, n: usize, state: u8) {
        if let Some(&i) = self.pump_n.get(&n) {
            self.pump_switch[i] = state.min(2);
        }
    }
    /// `FUELSYSTEM_PUMP_ON`.
    pub fn pump_on(&mut self, n: usize) {
        self.set_pump(n, 1);
    }
    /// `FUELSYSTEM_PUMP_OFF`.
    pub fn pump_off(&mut self, n: usize) {
        self.set_pump(n, 0);
    }
    /// `FUELSYSTEM_PUMP_TOGGLE`: off becomes on, on or auto becomes off.
    pub fn toggle_pump(&mut self, n: usize) {
        if let Some(&i) = self.pump_n.get(&n) {
            self.pump_switch[i] = if self.pump_switch[i] == 0 { 1 } else { 0 };
        }
    }
    /// `FUELSYSTEM PUMP SWITCH` (0 off, 1 on, 2 auto).
    pub fn pump_switch(&self, n: usize) -> u8 {
        self.pump_n.get(&n).map_or(0, |&i| self.pump_switch[i])
    }
    /// `FUELSYSTEM PUMP ACTIVE`, as of the last update or command.
    pub fn pump_active(&self, n: usize) -> bool {
        self.pump_n.get(&n).is_some_and(|&i| self.pump_active[i])
    }
    /// hyperrealism.md physics workstream 5 (fluids): the pump's own
    /// delivery pressure this tick (`pump_own_pressure`, psi), 0 when it is
    /// not being driven even if its switch is on. Not previously exposed;
    /// `fuel.rs` uses it for the pump's hydraulic power/current draw.
    pub fn pump_pressure_psi(&self, n: usize) -> f64 {
        self.pump_n.get(&n).map_or(0.0, |&i| self.pump_own_pressure(i))
    }
    /// hyperrealism.md physics workstream 5: the flow through the pump's own
    /// destination line (gal/h), an approximation of the pump's own flow --
    /// exact for a pump that is the sole driver of its destination line,
    /// the common case for this network's boost/transfer pumps.
    pub fn pump_flow_gph(&self, n: usize) -> f64 {
        self.pump_n
            .get(&n)
            .and_then(|&i| self.line_index(&self.def.pumps[i].destination_line))
            .map_or(0.0, |li| self.line_flow_gph(li))
    }
    /// One full stroke of a manual pump's handle.
    pub fn manual_pump_stroke(&mut self, n: usize) {
        if let Some(&i) = self.pump_n.get(&n) {
            let p = &self.def.pumps[i];
            self.pump_manual_fraction[i] = (self.pump_manual_fraction[i] + p.pct_pressure_per_pump).min(1.0);
        }
    }

    // ----- valves ----------------------------------------------------------

    pub fn valve_count(&self) -> usize {
        self.def.valves.len()
    }
    pub fn valve_index(&self, name: &str) -> Option<usize> {
        self.def.valves.iter().find(|p| p.name == name).map(|p| p.index)
    }
    /// `FUELSYSTEM_VALVE_SET`.
    pub fn set_valve(&mut self, n: usize, open: bool) {
        if let Some(&i) = self.valve_n.get(&n) {
            self.valve_switch[i] = open;
        }
    }
    /// `FUELSYSTEM_VALVE_OPEN`.
    pub fn open_valve(&mut self, n: usize) {
        self.set_valve(n, true);
    }
    /// `FUELSYSTEM_VALVE_CLOSE`.
    pub fn close_valve(&mut self, n: usize) {
        self.set_valve(n, false);
    }
    /// `FUELSYSTEM_VALVE_TOGGLE`.
    pub fn toggle_valve(&mut self, n: usize) {
        if let Some(&i) = self.valve_n.get(&n) {
            self.valve_switch[i] = !self.valve_switch[i];
        }
    }
    /// `FUELSYSTEM VALVE SWITCH`.
    pub fn valve_switch(&self, n: usize) -> bool {
        self.valve_n.get(&n).is_some_and(|&i| self.valve_switch[i])
    }
    /// `FUELSYSTEM VALVE OPEN`: position, 0 closed to 1 open.
    pub fn valve_open(&self, n: usize) -> f64 {
        self.valve_n.get(&n).map_or(0.0, |&i| self.valve_open[i])
    }

    /// Opens or shuts each of the four cross-feed valves (`CrossFeedValve1
    /// ..4`, `Valve.46..49`) from the crew's own per-valve selection --
    /// `fuel.rs::Crossfeed`'s `FUEL CROSSFEED SWITCH:1..4` on the real
    /// plugin, a plain `[bool; 4]` here so this network stays testable
    /// without `Vars`/`Xplm`. Four independent calls, not one combined
    /// switch or a per-pair one: the real aircraft's own SD `FuelPage.tsx`
    /// reads and draws all four valves independently
    /// (`FUELSYSTEM VALVE OPEN:46..49`) and `ata28.ts`'s abnormal-sensed
    /// checklist raises a separate "FUEL CROSSFEED VLV {n} FAULT" for each
    /// one -- the real indications never gang two valves together, so
    /// anything coarser would be a fabrication the fault list itself
    /// contradicts. (The MSFS failure catalogue's own 28_009/28_010
    /// pairing above, `refresh_catalogue_failures`'s own comment, is a two-slot
    /// granularity limit on failure *injection* only, not evidence about
    /// the cockpit control.)
    ///
    /// This is the one place either fuel model's crossfeed selection
    /// reaches the real valves: `deep::fuel::live.rs`'s own
    /// `crossfeed_open` and this network's valve state both derive from
    /// the same `Truth::controls::crossfeed_valve_selected` /
    /// `FUEL CROSSFEED SWITCH:n` reading (`deep/plugin.rs`'s sourcing
    /// table, `fuel.rs::Fuel::crossfeed`), so the two models can never
    /// disagree about whether the valves are actually open.
    pub fn set_crossfeed_selection(&mut self, selected: [bool; 4]) {
        for (i, &open) in selected.iter().enumerate() {
            let Some(v) = self.valve_index(&format!("CrossFeedValve{}", i + 1)) else { continue };
            if open {
                self.open_valve(v);
            } else {
                self.close_valve(v);
            }
        }
    }

    // ----- triggers --------------------------------------------------------

    pub fn trigger_count(&self) -> usize {
        self.def.triggers.len()
    }
    pub fn trigger_index(&self, name: &str) -> Option<usize> {
        self.trigger_names.get(name).map(|&i| self.def.triggers[i].index)
    }
    /// `FUELSYSTEM_TRIGGER_SET`. The effects of a status change apply at once.
    pub fn set_trigger(&mut self, n: usize, on: bool) {
        if let Some(&i) = self.trigger_n.get(&n) {
            self.set_trigger_pos(i, on, 0);
            self.refresh_pumps();
        }
    }
    /// `FUELSYSTEM_TRIGGER_ON`.
    pub fn trigger_on(&mut self, n: usize) {
        self.set_trigger(n, true);
    }
    /// `FUELSYSTEM_TRIGGER_OFF`.
    pub fn trigger_off(&mut self, n: usize) {
        self.set_trigger(n, false);
    }
    /// `FUELSYSTEM_TRIGGER_TOGGLE`.
    pub fn toggle_trigger(&mut self, n: usize) {
        if let Some(&i) = self.trigger_n.get(&n) {
            let on = !self.trigger_status[i];
            self.set_trigger_pos(i, on, 0);
            self.refresh_pumps();
        }
    }
    /// `FUELSYSTEM TRIGGER STATUS`.
    pub fn trigger_status(&self, n: usize) -> bool {
        self.trigger_n.get(&n).is_some_and(|&i| self.trigger_status[i])
    }

    // ----- junctions -------------------------------------------------------

    pub fn junction_count(&self) -> usize {
        self.def.junctions.len()
    }
    pub fn junction_index(&self, name: &str) -> Option<usize> {
        self.def.junctions.iter().find(|p| p.name == name).map(|p| p.index)
    }
    /// `FUELSYSTEM_JUNCTION_SET`. Options that do not exist are ignored.
    pub fn set_junction(&mut self, n: usize, option: usize) {
        if let Some(&i) = self.junction_n.get(&n) {
            self.set_junction_pos(i, option);
        }
    }
    /// `FUELSYSTEM JUNCTION SETTING` (1-based option).
    pub fn junction_setting(&self, n: usize) -> usize {
        self.junction_n.get(&n).map_or(0, |&i| self.junction_setting[i])
    }
    fn set_junction_pos(&mut self, i: usize, option: usize) {
        let count = self.def.junctions[i].options.len();
        if option >= 1 && (count == 0 || option <= count) {
            self.junction_setting[i] = option;
        }
    }

    // ----- lines, engines, APU ---------------------------------------------

    pub fn line_count(&self) -> usize {
        self.def.lines.len()
    }
    pub fn line_index(&self, name: &str) -> Option<usize> {
        self.def.lines.iter().find(|p| p.name == name).map(|p| p.index)
    }
    /// `FUELSYSTEM LINE FUEL FLOW` in gal/h, never negative.
    pub fn line_flow_gph(&self, n: usize) -> f64 {
        self.line_n.get(&n).map_or(0.0, |&i| self.line_flow[i])
    }
    /// `FUELSYSTEM LINE FUEL PRESSURE` in psi, negative when pushed from
    /// destination to source.
    pub fn line_pressure_psi(&self, n: usize) -> f64 {
        self.line_n.get(&n).map_or(0.0, |&i| self.line_pressure[i])
    }
    /// `FUELSYSTEM LINE FUEL LEVEL` in gallons (see module docs, point 2).
    pub fn line_level_gallons(&self, n: usize) -> f64 {
        self.line_n.get(&n).map_or(0.0, |&i| self.line_level[i])
    }
    pub fn engine_count(&self) -> usize {
        self.def.engines.len()
    }
    fn engine_pos(&self, engine_index: usize) -> Option<usize> {
        self.def.engines.iter().position(|e| e.engine_index == engine_index)
    }
    /// `FUELSYSTEM ENGINE PRESSURE` (psi) of simulator engine `engine_index` (1-based).
    pub fn engine_pressure_psi(&self, engine_index: usize) -> f64 {
        self.engine_pos(engine_index).map_or(0.0, |i| self.engine_pressure[i])
    }
    /// hyperrealism.md physics workstream (fuel second pass): the APU feed
    /// line's own continuous pressure (psi), the [`Self::engine_pressure_psi`]
    /// equivalent for the APU -- previously the network had no pressure
    /// concept for the APU feed at all. 0 with no APU defined.
    pub fn apu_feed_pressure_psi(&self, apu_index: usize) -> f64 {
        self.apu_pressure.get(apu_index).copied().unwrap_or(0.0)
    }
    /// Fuel actually delivered to the engine over the last update, in gal/h.
    pub fn engine_fuel_flow_gph(&self, engine_index: usize) -> f64 {
        self.engine_pos(engine_index).map_or(0.0, |i| self.engine_delivered_gph[i])
    }
    /// Whether the engine got its whole demand throughout the last update, or
    /// with no demand, whether its feed is pressurised.
    pub fn engine_fed(&self, engine_index: usize) -> bool {
        self.engine_pos(engine_index).is_some_and(|i| self.engine_fed[i])
    }
    /// `APU.N FuelBurnRate` (gal/h) of the first APU, 0 without one.
    pub fn apu_burn_rate_gph(&self) -> f64 {
        self.def.apus.first().map_or(0.0, |a| a.fuel_burn_rate)
    }
    pub fn apu_fuel_flow_gph(&self) -> f64 {
        self.apu_delivered_gph.first().copied().unwrap_or(0.0)
    }
    pub fn apu_fed(&self) -> bool {
        self.apu_fed.first().copied().unwrap_or(false)
    }

    // ----- MSFS names ------------------------------------------------------

    /// Handles an MSFS `FUELSYSTEM_*` key event by name (`K:` and `KEY_`
    /// prefixes allowed). `p0` is the element index and `p1` the value.
    /// Returns false for other events.
    pub fn handle_key_event(&mut self, name: &str, p0: u32, p1: u32) -> bool {
        let n = name.trim();
        let n = n.strip_prefix("K:").unwrap_or(n);
        let n = n.strip_prefix("KEY_").unwrap_or(n).to_ascii_uppercase();
        let i = p0 as usize;
        match n.as_str() {
            "FUELSYSTEM_PUMP_ON" => self.pump_on(i),
            "FUELSYSTEM_PUMP_OFF" => self.pump_off(i),
            "FUELSYSTEM_PUMP_SET" => self.set_pump(i, p1.min(2) as u8),
            "FUELSYSTEM_PUMP_TOGGLE" => self.toggle_pump(i),
            "FUELSYSTEM_VALVE_OPEN" => self.open_valve(i),
            "FUELSYSTEM_VALVE_CLOSE" => self.close_valve(i),
            "FUELSYSTEM_VALVE_SET" => self.set_valve(i, p1 != 0),
            "FUELSYSTEM_VALVE_TOGGLE" => self.toggle_valve(i),
            "FUELSYSTEM_TRIGGER_ON" => self.trigger_on(i),
            "FUELSYSTEM_TRIGGER_OFF" => self.trigger_off(i),
            "FUELSYSTEM_TRIGGER_SET" => self.set_trigger(i, p1 != 0),
            "FUELSYSTEM_TRIGGER_TOGGLE" => self.toggle_trigger(i),
            "FUELSYSTEM_JUNCTION_SET" => self.set_junction(i, p1 as usize),
            _ => return false,
        }
        true
    }

    /// Reads a `FUELSYSTEM ...` simvar by name (`A:` prefix and underscores
    /// allowed) in this module's units: gallons, gal/h, psi, pounds,
    /// 0..1 fractions, and 0/1 for booleans.
    pub fn read_simvar(&self, name: &str, index: usize) -> Option<f64> {
        let n = name.trim();
        let n = n.strip_prefix("A:").unwrap_or(n).replace('_', " ").to_ascii_uppercase();
        let b = |v: bool| if v { 1.0 } else { 0.0 };
        Some(match n.as_str() {
            "FUELSYSTEM TANK QUANTITY" => self.tank_gallons(index),
            "FUELSYSTEM TANK TOTAL QUANTITY" => self.tank_total_gallons(index),
            "FUELSYSTEM TANK CAPACITY" => self.tank_capacity(index),
            "FUELSYSTEM TANK LEVEL" => self.tank_level(index),
            "FUELSYSTEM TANK WEIGHT" => self.tank_weight_lbs(index),
            "FUELSYSTEM PUMP ACTIVE" => b(self.pump_active(index)),
            "FUELSYSTEM PUMP SWITCH" => self.pump_switch(index) as f64,
            "FUELSYSTEM VALVE OPEN" => self.valve_open(index),
            "FUELSYSTEM VALVE SWITCH" => b(self.valve_switch(index)),
            "FUELSYSTEM LINE FUEL FLOW" => self.line_flow_gph(index),
            "FUELSYSTEM LINE FUEL PRESSURE" => self.line_pressure_psi(index),
            "FUELSYSTEM LINE FUEL LEVEL" => self.line_level_gallons(index),
            "FUELSYSTEM ENGINE PRESSURE" => self.engine_pressure_psi(index),
            "FUELSYSTEM TRIGGER STATUS" => b(self.trigger_status(index)),
            "FUELSYSTEM JUNCTION SETTING" => self.junction_setting(index) as f64,
            _ => return None,
        })
    }

    // ----- simulation ------------------------------------------------------

    /// Advances the network by `delta_seconds`.
    ///
    /// `engine_demand_gal_per_hour[k]` is what the engine with `Index` k+1
    /// wants to burn, drawn through its `Engine.N` component.
    /// `apu_demand_gal_per_hour` goes to the first APU; pass
    /// [`FuelNetwork::apu_burn_rate_gph`] while the APU runs. With FBW's
    /// A380X both are normally 0, because FBW's FADEC burns straight from the
    /// "Extra" tanks 12 to 15 (see the integration notes). The step is split
    /// into sub-steps of at most 0.1 s.
    /// Reads the `extra::fuel` catalogue (`failures.rs` ids `28_000`-
    /// `28_011`) and stores each id's continuous `failures::magnitude`
    /// against the pump/valve it names, by internal index, in
    /// [`Self::pump_fail`]/[`Self::valve_fail`] (and captures
    /// [`Self::valve_stuck_at`] on the frame a valve's magnitude first goes
    /// positive). Called once per [`Self::update`] rather than every
    /// substep since `active_ids`/magnitudes cannot change mid-frame; a
    /// name this network's `.cfg` doesn't define is silently skipped, the
    /// same tolerance every other by-name setter here already has.
    fn refresh_catalogue_failures(&mut self) {
        for f in self.pump_fail.iter_mut() {
            *f = 0.0;
        }
        // Needed to detect a 0.0 -> positive transition below without a
        // second parallel bool array: a valve only gets a fresh
        // `valve_stuck_at` capture the instant it starts sticking, not on
        // every frame it stays stuck.
        let prev_valve_fail = self.valve_fail.clone();
        for f in self.valve_fail.iter_mut() {
            *f = 0.0;
        }
        let mut fail_pump = |name: &str, mag: f64| {
            if let Some(i) = self.def.pumps.iter().position(|p| p.name == name) {
                self.pump_fail[i] = self.pump_fail[i].max(mag);
            }
        };
        // 28_000-28_007: "Tank N feed pump A/B" -- FeedNTankPump1/2 in the
        // A380's own flight_model.cfg naming, one id per physical pump. A
        // degraded pump (magnitude < 1.0) loses that fraction of its rated
        // head, and with it a proportional fraction of the flow the linear
        // Q = k*dP network solve derives from that head -- see
        // `pump_own_pressure`.
        for tank in 1..=4u32 {
            for pump in 1..=2u32 {
                let k = (tank - 1) * 2 + (pump - 1);
                let mag = failures::magnitude(28_000 + k as u64);
                if mag > 0.0 {
                    fail_pump(&format!("Feed{tank}TankPump{pump}"), mag);
                }
            }
        }
        // 28_008: "Trim tank transfer pump" -- both the left and right trim
        // pumps feed the same aft gallery, so the catalogue's single item
        // takes out both real pumps, at the same magnitude.
        let trim_mag = failures::magnitude(28_008);
        if trim_mag > 0.0 {
            fail_pump("TrimTankPumpLeft", trim_mag);
            fail_pump("TrimTankPumpRight", trim_mag);
        }
        let mut fail_valve = |slf: &mut Self, name: &str, mag: f64| {
            if let Some(i) = slf.def.valves.iter().position(|v| v.name == name) {
                if prev_valve_fail[i] <= 0.0 {
                    // Freshly stuck this frame: lock in the position it
                    // seized at before `substep` moves it any further.
                    slf.valve_stuck_at[i] = slf.valve_open[i];
                }
                slf.valve_fail[i] = slf.valve_fail[i].max(mag);
            }
        };
        // 28_009/28_010: the cross-feed valve pair joining engines 1-2 and
        // 3-4 to the cross-feed manifold (CrossFeedValve1/2 and
        // CrossFeedValve3/4 in flight_model.cfg's own Junction/Line
        // topology).
        let cf12_mag = failures::magnitude(28_009);
        if cf12_mag > 0.0 {
            fail_valve(self, "CrossFeedValve1", cf12_mag);
            fail_valve(self, "CrossFeedValve2", cf12_mag);
        }
        let cf34_mag = failures::magnitude(28_010);
        if cf34_mag > 0.0 {
            fail_valve(self, "CrossFeedValve3", cf34_mag);
            fail_valve(self, "CrossFeedValve4", cf34_mag);
        }
        // 28_011: "A wing jettison valve sticks open or closed" -- both
        // nozzle valves, since the catalogue doesn't distinguish left/right.
        let jett_mag = failures::magnitude(28_011);
        if jett_mag > 0.0 {
            fail_valve(self, "JettisonNozzleValveLeft", jett_mag);
            fail_valve(self, "JettisonNozzleValveRight", jett_mag);
        }
        // 28_100..: every further pump and valve, by its own element name.
        for &(id, _, pump, elements) in failures::extra::FUEL_ELEMENTS {
            let mag = failures::magnitude(id);
            if mag <= 0.0 {
                continue;
            }
            for name in elements {
                if pump {
                    if let Some(i) = self.def.pumps.iter().position(|p| p.name == *name) {
                        self.pump_fail[i] = self.pump_fail[i].max(mag);
                    }
                } else {
                    fail_valve(self, name, mag);
                }
            }
        }
    }

    pub fn update(&mut self, delta_seconds: f64, engine_demand_gal_per_hour: [f64; 4], apu_demand_gal_per_hour: f64) {
        self.refresh_catalogue_failures();
        for (i, e) in self.def.engines.iter().enumerate() {
            let k = e.engine_index.wrapping_sub(1);
            self.engine_demand_gph[i] = if k < 4 { engine_demand_gal_per_hour[k].max(0.0) } else { 0.0 };
        }
        for (i, d) in self.apu_demand_gph.iter_mut().enumerate() {
            *d = if i == 0 { apu_demand_gal_per_hour.max(0.0) } else { 0.0 };
        }
        if !(delta_seconds > 0.0) || !delta_seconds.is_finite() {
            self.refresh_pumps();
            return;
        }
        let steps = (delta_seconds / MAX_SUBSTEP_S).ceil().max(1.0) as usize;
        let h = delta_seconds / steps as f64;
        let ne = self.engine_count();
        let na = self.def.apus.len();
        let mut eng_gal = vec![0.0; ne];
        let mut eng_fed = vec![true; ne];
        let mut apu_gal = vec![0.0; na];
        let mut apu_fed = vec![true; na];
        let mut flow_acc = vec![0.0; self.line_flow.len()];
        for _ in 0..steps {
            self.substep(h);
            for i in 0..ne {
                eng_gal[i] += self.engine_delivered_gph[i] * h / 3600.0;
                eng_fed[i] &= self.engine_fed[i];
            }
            for i in 0..na {
                apu_gal[i] += self.apu_delivered_gph[i] * h / 3600.0;
                apu_fed[i] &= self.apu_fed[i];
            }
            for (a, f) in flow_acc.iter_mut().zip(&self.line_flow) {
                *a += f * h;
            }
        }
        for i in 0..ne {
            self.engine_delivered_gph[i] = eng_gal[i] * 3600.0 / delta_seconds;
            self.engine_fed[i] = eng_fed[i];
        }
        for i in 0..na {
            self.apu_delivered_gph[i] = apu_gal[i] * 3600.0 / delta_seconds;
            self.apu_fed[i] = apu_fed[i];
        }
        for (f, a) in self.line_flow.iter_mut().zip(flow_acc) {
            *f = a / delta_seconds;
        }
    }

    fn substep(&mut self, dt: f64) {
        // Valves move towards their switch while powered.
        for i in 0..self.def.valves.len() {
            let v = &self.def.valves[i];
            let powered = v.circuit.map_or(true, |c| *self.valve_circuits.get(&c).unwrap_or(&true));
            if !powered {
                continue;
            }
            let raw_target = if self.valve_switch[i] { 1.0 } else { 0.0 };
            // 28_009-28_011: a stuck valve loses `valve_fail[i]` of its
            // authority to reach the commanded position, measured from
            // `valve_stuck_at` (its position the instant it seized). At
            // magnitude 1.0 (a full failure, matching legacy binary
            // behaviour) `target == valve_stuck_at` always, so it never
            // moves again -- "sticks; no longer opens or closes". A lower
            // magnitude still lets it travel part of the way, i.e. "sticks
            // at y%" of full travel from where it seized.
            let target = if self.valve_fail[i] > 0.0 {
                self.valve_stuck_at[i] + (1.0 - self.valve_fail[i]) * (raw_target - self.valve_stuck_at[i])
            } else {
                raw_target
            };
            let rate = if v.opening_time > 0.0 { dt / v.opening_time } else { 1.0 };
            let cur = self.valve_open[i];
            self.valve_open[i] = if cur < target { (cur + rate).min(target) } else { (cur - rate).max(target) };
        }
        // Manual pump pressure decays.
        for i in 0..self.def.pumps.len() {
            if self.def.pumps[i].pump_type == PumpType::Manual {
                let r = self.def.pumps[i].pressure_decrease_rate;
                self.pump_manual_fraction[i] = (self.pump_manual_fraction[i] * (1.0 - r * dt).max(0.0)).max(0.0);
            }
        }
        self.refresh_pumps();

        let solve = self.solve_lines();
        self.run_flows(&solve, dt);

        self.evaluate_triggers(dt);
        self.refresh_pumps();
        self.started = true;
    }

    /// hyperrealism.md physics workstream 5: every pump's rated pressure is
    /// also derated by its own tank's [`pump_tank_derate`] (unporting from
    /// low fuel level and pitch/bank tilt, `physics::fluids::unporting_factor`)
    /// -- a pump submerged in its tank cannot deliver rated pressure once its
    /// inlet is uncovered, whatever type of pump it is (electric, engine- or
    /// APU-driven, manual, or anemometer-driven all sit in the same tank and
    /// share the same inlet). A pump with no known tank (`pump_tank[p] ==
    /// None`, e.g. a line-mounted transfer/jet pump with no
    /// `TankFuelRequired`) is unaffected (derate 1.0), matching how
    /// `refresh_pumps`'s own `tank_ok` check already treats that case.
    fn pump_own_pressure(&self, p: usize) -> f64 {
        let d = &self.def.pumps[p];
        let curve = |x: f64| d.pressure_curve.and_then(|c| self.def.curves.get(&c)).map_or(x, |c| c.eval(x));
        let rated = match d.pump_type {
            PumpType::Electric | PumpType::ApuDriven => d.pressure,
            PumpType::EngineDriven => {
                let rpm = d.type_index.and_then(|e| self.engine_rpm.get(&e).copied()).unwrap_or(0.0);
                d.pressure * curve(rpm).max(0.0)
            }
            PumpType::Manual => d.pressure * self.pump_manual_fraction[p],
            PumpType::Anemometer => {
                d.pressure * d.type_index.and_then(|a| self.anemometer.get(&a).copied()).unwrap_or(0.0)
            }
        };
        let derate = self.pump_tank[p].map_or(1.0, |t| self.pump_tank_derate.get(t).copied().unwrap_or(1.0));
        // 28_000-28_008: a degraded pump loses `pump_fail[p]` of its rated
        // head; at magnitude 1.0 (a full failure, matching legacy binary
        // behaviour) it delivers exactly 0.0. Under this network's linear
        // Q = k*dP solve that same fraction of flow is lost along with the
        // head, which is the "loses head and flow along its curve" this
        // module's docs ask for without solving a real centrifugal-pump
        // curve. `pump_fail[p]` is always in `0.0..=1.0` already --
        // `failures::magnitude` clamps at the source -- so no further clamp
        // belongs here (see invariants.rs for the shared-accessor guard the
        // substrate agent is wiring in).
        let health = 1.0 - self.pump_fail[p];
        rated * derate * health
    }

    fn refresh_pumps(&mut self) {
        for p in 0..self.def.pumps.len() {
            let d = &self.def.pumps[p];
            let tank_ok = self.pump_tank[p].map_or(true, |t| self.usable(t) > EPS);
            let drive_ok = match d.pump_type {
                PumpType::Electric => d.type_index.map_or(true, |c| *self.pump_circuits.get(&c).unwrap_or(&true)),
                PumpType::EngineDriven => {
                    d.type_index.and_then(|e| self.engine_rpm.get(&e).copied()).unwrap_or(0.0) > 0.0
                }
                PumpType::ApuDriven => self.apu_running,
                PumpType::Manual | PumpType::Anemometer => true,
            };
            let switch_ok = match d.pump_type {
                // A hand pump works whenever its handle is pumped.
                PumpType::Manual => true,
                _ => match self.pump_switch[p] {
                    0 => false,
                    1 => true,
                    _ => match (&d.auto_condition, self.pump_auto_engine[p]) {
                        (Some((_, threshold)), Some(e)) => self.engine_pressure[e] < *threshold,
                        _ => true,
                    },
                },
            };
            self.pump_active[p] = tank_ok && drive_ok && switch_ok && self.pump_own_pressure(p) > 0.0;
        }
    }

    fn junction_line_open(&self, j: usize, line: usize) -> bool {
        let topo = &self.junction_topo[j];
        if topo.options.is_empty() {
            return true;
        }
        let setting = self.junction_setting[j];
        let Some((set, lists_input)) = topo.options.get(setting.wrapping_sub(1)) else { return true };
        if set.contains(&line) {
            return true;
        }
        // Input-only lines stay open unless the option names one of them.
        topo.inputs.contains(&line) && !lists_input
    }

    /// Whether `node` lets fuel through `line`, entering it or leaving it.
    fn port_allows(&self, node: usize, line: usize, entering: bool) -> bool {
        match self.nodes[node] {
            Node::Tank(t) => {
                if entering {
                    !self.tank_out_only[t].contains(&line)
                } else {
                    !self.tank_in_only[t].contains(&line)
                }
            }
            Node::Junction(j) => {
                let topo = &self.junction_topo[j];
                if entering && topo.outputs.contains(&line) {
                    return false;
                }
                if !entering && topo.inputs.contains(&line) {
                    return false;
                }
                self.junction_line_open(j, line)
            }
            Node::Valve(v) => {
                if self.valve_open[v] <= EPS {
                    return false;
                }
                match self.valve_dest[v] {
                    Some(d) => entering != (line == d),
                    None => true,
                }
            }
            Node::Pump(p) => {
                if !self.pump_active[p] {
                    return false;
                }
                match self.pump_dest[p] {
                    Some(d) => entering != (line == d),
                    None => false,
                }
            }
            Node::Engine(_) | Node::Apu(_) => entering,
            Node::Missing => false,
        }
    }

    fn line_dirs(&self) -> Vec<(bool, bool)> {
        (0..self.def.lines.len())
            .map(|l| {
                let (s, d) = (self.line_src[l], self.line_dst[l]);
                if s == d {
                    return (false, false);
                }
                let fwd = self.port_allows(s, l, false) && self.port_allows(d, l, true);
                let gravity = self.def.lines[l].gravity_based_fuel_flow.is_some();
                let bwd = !gravity && self.port_allows(d, l, false) && self.port_allows(s, l, true);
                (fwd, bwd)
            })
            .collect()
    }

    fn is_pass_through(&self, n: usize) -> bool {
        match self.nodes[n] {
            Node::Junction(_) | Node::Valve(_) => true,
            Node::Pump(p) => self.pump_active[p],
            _ => false,
        }
    }

    /// Pressure each node pushes (`out`) and receives (`inn`), and the pump
    /// suction reaching each junction or valve from downstream (`pull`), with
    /// `excluded` taken out of the network. Suction goes upstream from an
    /// active pump's inlet through junctions and open valves, so a pump
    /// behind a selector junction still draws from its tanks. It does not
    /// pass through another pump.
    fn pressures(
        &self,
        dirs: &[(bool, bool)],
        excluded: Option<usize>,
        inn: &mut [f64],
        out: &mut [f64],
        pull: &mut [f64],
    ) {
        for n in 0..self.nodes.len() {
            inn[n] = 0.0;
            pull[n] = 0.0;
        }
        out.copy_from_slice(&self.solve_base_out);
        for _ in 0..=self.nodes.len() {
            let mut changed = false;
            for n in 0..self.nodes.len() {
                if !self.is_pass_through(n) {
                    continue;
                }
                let mut m: f64 = 0.0;
                let mut suction: f64 = 0.0;
                for &l in &self.incident[n] {
                    if Some(l) == excluded {
                        continue;
                    }
                    let (fwd, bwd) = dirs[l];
                    if self.line_dst[l] == n && fwd {
                        m = m.max(out[self.line_src[l]]);
                    }
                    if self.line_src[l] == n && bwd {
                        m = m.max(out[self.line_dst[l]]);
                    }
                    if !matches!(self.nodes[n], Node::Pump(_)) {
                        if self.line_src[l] == n && fwd {
                            suction = suction.max(self.pull_at(self.line_dst[l], l, pull));
                        }
                        if self.line_dst[l] == n && bwd {
                            suction = suction.max(self.pull_at(self.line_src[l], l, pull));
                        }
                    }
                }
                inn[n] = m;
                if suction > pull[n] + 1e-12 {
                    pull[n] = suction;
                    changed = true;
                }
                let new_out = match self.nodes[n] {
                    Node::Pump(p) => m.max(self.solve_pump_p[p]),
                    _ => m,
                };
                if new_out > out[n] + 1e-12 {
                    out[n] = new_out;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    }

    fn valve_factor(&self, node: usize) -> f64 {
        match self.nodes[node] {
            Node::Valve(v) => self.valve_open[v].clamp(0.0, 1.0),
            _ => 1.0,
        }
    }

    /// Suction on `line` from its end at `node`.
    fn pull_at(&self, node: usize, line: usize, pull: &[f64]) -> f64 {
        match self.nodes[node] {
            Node::Pump(p) if self.pump_active[p] && self.pump_dest[p] != Some(line) => self.solve_pump_p[p],
            Node::Junction(_) | Node::Valve(_) => pull[node],
            _ => 0.0,
        }
    }

    fn solve_lines(&mut self) -> Vec<LineSolve> {
        self.solve_pump_p = (0..self.def.pumps.len()).map(|p| self.pump_own_pressure(p)).collect();
        self.solve_base_out = (0..self.nodes.len())
            .map(|n| match self.nodes[n] {
                Node::Tank(t) => self.def.tanks[t]
                    .pressure_curve
                    .and_then(|c| self.def.curves.get(&c))
                    .map_or(0.0, |c| c.eval(self.level(t)).max(0.0)),
                Node::Pump(p) if self.pump_active[p] => self.solve_pump_p[p],
                _ => 0.0,
            })
            .collect();
        let dirs = self.line_dirs();
        let nn = self.nodes.len();
        let mut inn = vec![0.0; nn];
        let mut out = vec![0.0; nn];
        let mut pull = vec![0.0; nn];
        // hyperrealism.md physics workstream 5: the fuel-viscosity derate
        // (`set_viscosity_derate`, 1.0 until `fuel.rs` sets it from the tank
        // temperature model) scales every line's conductance uniformly, the
        // Hagen-Poiseuille sense in which this network's linear
        // flow-per-psi form already behaves (see the field's own doc).
        let gph_per_psi_unit = self.gain * 3600.0 / self.density * self.viscosity_derate;
        let mut solve = vec![LineSolve::default(); self.def.lines.len()];
        for l in 0..self.def.lines.len() {
            let (fwd, bwd) = dirs[l];
            if !fwd && !bwd {
                continue;
            }
            self.pressures(&dirs, Some(l), &mut inn, &mut out, &mut pull);
            let (a, b) = (self.line_src[l], self.line_dst[l]);
            let def = &self.def.lines[l];
            let k = def.fuel_flow_at_1psi.max(0.0) * gph_per_psi_unit * self.valve_factor(a) * self.valve_factor(b);
            let pf = if fwd { out[a].max(self.pull_at(b, l, &pull)) } else { 0.0 };
            let pb = if bwd { out[b].max(self.pull_at(a, l, &pull)) } else { 0.0 };
            let s = &mut solve[l];
            if fwd && bwd {
                let net = pf - pb;
                s.pressure = net;
                if net > EPS {
                    s.dir = 1;
                    s.cap_pressure_gph = k * net;
                } else if net < -EPS {
                    s.dir = -1;
                    s.cap_pressure_gph = -k * net;
                }
            } else if fwd {
                // One-way: a higher pressure already at the far end shuts it.
                let back = if self.is_pass_through(b) { inn[b] } else { 0.0 };
                s.dir = 1;
                if back > pf + 1e-6 {
                    s.pressure = 0.0;
                } else {
                    s.pressure = pf;
                    s.cap_pressure_gph = k * pf;
                }
            } else {
                let back = if self.is_pass_through(a) { inn[a] } else { 0.0 };
                s.dir = -1;
                if back > pb + 1e-6 {
                    s.pressure = 0.0;
                } else {
                    s.pressure = -pb;
                    s.cap_pressure_gph = k * pb;
                }
            }
            if fwd {
                if let Some(g) = def.gravity_based_fuel_flow {
                    s.dir = 1;
                    s.cap_gravity_gph = g.max(0.0) * self.valve_factor(a) * self.valve_factor(b);
                }
            }
        }
        for p in self.engine_pressure.iter_mut() {
            *p = 0.0;
        }
        for p in self.apu_pressure.iter_mut() {
            *p = 0.0;
        }
        for l in 0..self.def.lines.len() {
            self.line_pressure[l] = solve[l].pressure;
            if let Node::Engine(e) = self.nodes[self.line_dst[l]] {
                if solve[l].dir == 1 {
                    self.engine_pressure[e] = self.engine_pressure[e].max(solve[l].pressure);
                }
            }
            // hyperrealism.md physics workstream (fuel second pass): the APU
            // feed line gets the same continuous pressure tracking the
            // engine feed lines already had -- before this, `apu_pressure`
            // did not exist at all, so the APU's own feed line had no
            // pressure concept, only a demand/delivered gallon flow.
            if let Node::Apu(a) = self.nodes[self.line_dst[l]] {
                if solve[l].dir == 1 {
                    self.apu_pressure[a] = self.apu_pressure[a].max(solve[l].pressure);
                }
            }
        }
        solve
    }

    fn run_flows(&mut self, solve: &[LineSolve], dt: f64) {
        let dt_h = dt / 3600.0;
        let nn = self.nodes.len();
        let nt = self.def.tanks.len();
        let s = nn + nt;
        let t = s + 1;
        let tank_node: Vec<usize> = (0..nn)
            .filter_map(|n| if let Node::Tank(tk) = self.nodes[n] { Some((tk, n)) } else { None })
            .fold(vec![0; nt], |mut v, (tk, n)| {
                v[tk] = n;
                v
            });
        let tank_in = |tk: usize| nn + tk;
        // Arc ends per line in its flow direction; fuel enters a tank at its
        // separate "in" node, so it cannot pass through the tank.
        let ends: Vec<(usize, usize)> = (0..solve.len())
            .map(|l| {
                let (u, v) = if solve[l].dir > 0 {
                    (self.line_src[l], self.line_dst[l])
                } else {
                    (self.line_dst[l], self.line_src[l])
                };
                let v = match self.nodes[v] {
                    Node::Tank(tk) => tank_in(tk),
                    _ => v,
                };
                (u, v)
            })
            .collect();
        let arc_ends = |l: usize, _dir: i8| ends[l];

        // Phase 1: engines and APUs, on pressure only.
        let mut net = FlowNet::new(t + 1);
        let src1: Vec<usize> = (0..nt).map(|tk| net.add(s, tank_node[tk], self.usable(tk) / dt_h)).collect();
        let arcs1: Vec<Option<usize>> = (0..solve.len())
            .map(|l| {
                let ls = solve[l];
                if ls.dir == 0 || ls.cap_pressure_gph <= EPS {
                    return None;
                }
                let (u, v) = arc_ends(l, ls.dir);
                Some(net.add(u, v, ls.cap_pressure_gph))
            })
            .collect();
        let mut eng_arc = vec![None; self.def.engines.len()];
        let mut apu_arc = vec![None; self.def.apus.len()];
        for n in 0..nn {
            match self.nodes[n] {
                Node::Engine(e) if self.engine_demand_gph[e] > EPS => {
                    eng_arc[e] = Some(net.add(n, t, self.engine_demand_gph[e]));
                }
                Node::Apu(a) if self.apu_demand_gph[a] > EPS => {
                    apu_arc[a] = Some(net.add(n, t, self.apu_demand_gph[a]));
                }
                _ => {}
            }
        }
        net.run(s, t);
        let f1: Vec<f64> = arcs1.iter().map(|a| a.map_or(0.0, |e| net.flow(e))).collect();
        let out1: Vec<f64> = src1.iter().map(|&e| net.flow(e) * dt_h).collect();
        let mut burnt = 0.0;
        for e in 0..self.def.engines.len() {
            let d = eng_arc[e].map_or(0.0, |a| net.flow(a));
            self.engine_delivered_gph[e] = d;
            burnt += d * dt_h;
            let demand = self.engine_demand_gph[e];
            self.engine_fed[e] = if demand > EPS {
                d >= demand * (1.0 - 1e-6) - 1e-6
            } else {
                self.engine_pressure[e] > EPS
            };
        }
        for a in 0..self.def.apus.len() {
            let d = apu_arc[a].map_or(0.0, |x| net.flow(x));
            self.apu_delivered_gph[a] = d;
            burnt += d * dt_h;
            let demand = self.apu_demand_gph[a];
            let pressurised = self
                .incident
                .iter()
                .enumerate()
                .find(|(n, _)| self.nodes[*n] == Node::Apu(a))
                .map_or(false, |(_, ls)| ls.iter().any(|&l| solve[l].pressure.abs() > EPS));
            self.apu_fed[a] = if demand > EPS { d >= demand * (1.0 - 1e-6) - 1e-6 } else { pressurised };
        }

        // Phase 2: tank to tank, on what pressure capacity is left plus gravity.
        let mut net2 = FlowNet::new(t + 1);
        let src2: Vec<usize> = (0..nt)
            .map(|tk| net2.add(s, tank_node[tk], (self.usable(tk) - out1[tk]).max(0.0) / dt_h))
            .collect();
        let sink2: Vec<usize> = (0..nt)
            .map(|tk| {
                let room = self.def.tanks[tk].capacity - (self.tank_qty[tk] - out1[tk]);
                net2.add(tank_in(tk), t, room.max(0.0) / dt_h)
            })
            .collect();
        let arcs2: Vec<Option<usize>> = (0..solve.len())
            .map(|l| {
                let ls = solve[l];
                if ls.dir == 0 {
                    return None;
                }
                let cap = (ls.cap_pressure_gph - f1[l]).max(0.0) + ls.cap_gravity_gph;
                if cap <= EPS {
                    return None;
                }
                let (u, v) = arc_ends(l, ls.dir);
                Some(net2.add(u, v, cap))
            })
            .collect();
        net2.run(s, t);

        for tk in 0..nt {
            let out2 = net2.flow(src2[tk]) * dt_h;
            let in2 = net2.flow(sink2[tk]) * dt_h;
            let q = self.tank_qty[tk] - out1[tk] - out2 + in2;
            let cap = self.def.tanks[tk].capacity;
            self.tank_qty[tk] = if q.abs() < 1e-12 { 0.0 } else { q.clamp(0.0, cap.max(0.0)) };
        }
        self.total_burnt_gal += burnt;
        for l in 0..solve.len() {
            let f = f1[l] + arcs2[l].map_or(0.0, |e| net2.flow(e));
            self.line_flow[l] = f;
            self.line_level[l] = if f > EPS || solve[l].pressure.abs() > EPS { self.def.lines[l].volume } else { 0.0 };
        }
    }

    fn condition(&self, i: usize) -> bool {
        let tr = &self.def.triggers[i];
        let tank = |k: usize| -> Option<f64> {
            let name = tr.target.get(k)?;
            let t = self.def.tanks.iter().position(|t| &t.name == name)?;
            Some(self.usable(t))
        };
        let th = tr.threshold.first().copied().unwrap_or(0.0);
        match tr.condition {
            TriggerCondition::TankQuantityBelow => tank(0).is_some_and(|q| q < th),
            TriggerCondition::TankQuantityAbove => tank(0).is_some_and(|q| q > th),
            TriggerCondition::CgAboveLimit => self.cg_percent > th,
            TriggerCondition::CgBelowLimit => self.cg_percent < th,
            TriggerCondition::CgBetweenLimits => {
                let hi = tr.threshold.get(1).copied().unwrap_or(th);
                self.cg_percent >= th.min(hi) && self.cg_percent <= th.max(hi)
            }
            TriggerCondition::AutostartEnabled => self.autostart,
            TriggerCondition::AutoshutdownEnabled => self.autoshutdown,
            TriggerCondition::Manual => self.trigger_status[i],
            TriggerCondition::TankImbalanceAbove => matches!((tank(0), tank(1)), (Some(a), Some(b)) if a - b > th),
            TriggerCondition::TankImbalanceBelow => matches!((tank(0), tank(1)), (Some(a), Some(b)) if a - b < th),
            TriggerCondition::TankAbsImbalanceAbove => {
                matches!((tank(0), tank(1)), (Some(a), Some(b)) if (a - b).abs() > th)
            }
            TriggerCondition::TankAbsImbalanceBelow => {
                matches!((tank(0), tank(1)), (Some(a), Some(b)) if (a - b).abs() < th)
            }
            TriggerCondition::JunctionOptionChanged => {
                let j = tr.target.first().and_then(|n| self.def.junctions.iter().position(|j| &j.name == n));
                match (j, tr.target_index) {
                    (Some(j), Some(opt)) => self.junction_setting[j] == opt,
                    _ => false,
                }
            }
            TriggerCondition::Unknown(_) => false,
        }
    }

    fn evaluate_triggers(&mut self, dt: f64) {
        for i in 0..self.def.triggers.len() {
            if self.def.triggers[i].condition == TriggerCondition::Manual {
                continue;
            }
            let raw = self.condition(i);
            if raw == self.trigger_status[i] {
                self.trigger_timer[i] = 0.0;
                continue;
            }
            self.trigger_timer[i] += dt;
            let delay = if raw { self.def.triggers[i].delay_true } else { self.def.triggers[i].delay_false };
            if self.trigger_timer[i] + 1e-12 >= delay {
                self.trigger_timer[i] = 0.0;
                self.set_trigger_pos(i, raw, 0);
            }
        }
    }

    fn set_trigger_pos(&mut self, i: usize, on: bool, depth: u32) {
        if self.trigger_status[i] == on || depth > MAX_EFFECT_DEPTH {
            return;
        }
        self.trigger_status[i] = on;
        let effects = if on { self.def.triggers[i].effect_true.clone() } else { self.def.triggers[i].effect_false.clone() };
        for e in effects {
            self.apply_effect(&e, depth + 1);
        }
    }

    fn apply_effect(&mut self, e: &TriggerEffect, depth: u32) {
        let node = |name: &str| self.names.get(name).map(|&n| self.nodes[n]);
        match e {
            TriggerEffect::OpenValve(n) | TriggerEffect::CloseValve(n) => {
                if let Some(Node::Valve(v)) = node(n) {
                    self.valve_switch[v] = matches!(e, TriggerEffect::OpenValve(_));
                }
            }
            TriggerEffect::StartPump(n) | TriggerEffect::StopPump(n) => {
                if let Some(Node::Pump(p)) = node(n) {
                    self.pump_switch[p] = if matches!(e, TriggerEffect::StartPump(_)) { 1 } else { 0 };
                }
            }
            TriggerEffect::SetJunction(n, opt) => {
                if let Some(Node::Junction(j)) = node(n) {
                    self.set_junction_pos(j, *opt);
                }
            }
            TriggerEffect::StartTrigger(n) | TriggerEffect::StopTrigger(n) => {
                if let Some(&t) = self.trigger_names.get(n) {
                    self.set_trigger_pos(t, matches!(e, TriggerEffect::StartTrigger(_)), depth);
                }
            }
            TriggerEffect::Unknown(_) => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const CFG: &str = r"D:\fbw-aircraft\fbw-a380x\src\base\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380X\common\config\flight_model.cfg";
    const TAXI_FLT: &str = r"D:\fbw-aircraft\fbw-a380x\src\base\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380X\common\flt\taxi.flt";

    // A380X element indices (flight_model.cfg).
    const LEFT_OUTER: usize = 1;
    const FEED: [usize; 4] = [2, 5, 6, 9];
    const LEFT_MID: usize = 3;
    const LEFT_INNER: usize = 4;
    const RIGHT_INNER: usize = 7;
    const RIGHT_MID: usize = 8;
    const RIGHT_OUTER: usize = 10;
    const TRIM: usize = 11;
    const EXTRA: [usize; 4] = [12, 13, 14, 15];
    const EXTRA_APU: usize = 16;

    fn a380() -> Option<FuelNetwork> {
        match std::fs::read_to_string(CFG) {
            Ok(text) => Some(FuelNetwork::from_cfg(&text).expect("A380X cfg parses")),
            Err(_) => {
                eprintln!("skipping: {CFG} not found");
                None
            }
        }
    }

    /// taxi.flt switch states (engine LP valves, feed and gravity pumps, APU
    /// valves open, junctions on option 1), or the same by hand.
    fn a380_taxi() -> Option<FuelNetwork> {
        let mut net = a380()?;
        match std::fs::read_to_string(TAXI_FLT) {
            Ok(flt) => {
                assert!(net.apply_flt_state(&flt) > 0);
            }
            Err(_) => {
                for v in [1, 2, 3, 4, 37, 40, 50, 51] {
                    net.open_valve(v);
                }
                for p in [1, 2, 3, 4, 5, 6, 7, 8, 22, 23, 24, 25] {
                    net.pump_on(p);
                }
                net.update(5.0, [0.0; 4], 0.0);
            }
        }
        for (t, g) in [
            (LEFT_OUTER, 1000.0),
            (LEFT_MID, 4000.0),
            (LEFT_INNER, 5000.0),
            (RIGHT_INNER, 5000.0),
            (RIGHT_MID, 4000.0),
            (RIGHT_OUTER, 1000.0),
            (TRIM, 2000.0),
        ] {
            net.set_tank_gallons(t, g);
        }
        for f in FEED {
            net.set_tank_gallons(f, 5000.0);
        }
        for e in EXTRA {
            net.set_tank_gallons(e, 1.0);
        }
        net.set_tank_gallons(EXTRA_APU, 1.0);
        Some(net)
    }

    fn run(net: &mut FuelNetwork, seconds: f64, dt: f64, demand: [f64; 4], apu: f64) {
        let mut t = 0.0;
        while t < seconds - 1e-9 {
            let h = dt.min(seconds - t);
            net.update(h, demand, apu);
            t += h;
        }
    }

    fn all_quantities(net: &FuelNetwork) -> Vec<f64> {
        (1..=net.tank_count()).map(|i| net.tank_total_gallons(i)).collect()
    }

    #[test]
    fn parses_every_a380_element() {
        let Some(net) = a380() else { return };
        let d = net.definition();
        assert_eq!(d.version.as_deref(), Some("5"));
        assert_eq!(d.fuel_type, Some(2));
        assert_eq!(d.apus.len(), 1);
        assert_eq!(d.engines.len(), 4);
        assert_eq!(d.tanks.len(), 16);
        assert_eq!(d.lines.len(), 172);
        assert_eq!(d.junctions.len(), 17);
        assert_eq!(d.valves.len(), 59);
        assert_eq!(d.pumps.len(), 25);
        assert_eq!(d.triggers.len(), 46);
        assert_eq!(d.curves.len(), 1);
        assert_eq!(net.tank_capacity(2), 7299.6);
        // "InputOnlylines" (lower-case l) on Tank.8.
        assert_eq!(d.tanks[7].input_only_lines.len(), 2);
        assert_eq!(d.junctions[6].options.len(), 3);
        assert_eq!(d.lines[54].fuel_flow_at_1psi, 0.00175);
        assert_eq!(d.lines[141].gravity_based_fuel_flow, Some(500.0));
        assert_eq!(d.valves[0].opening_time, 3.0);
        assert_eq!(d.valves[4].opening_time, DEFAULT_VALVE_OPENING_TIME_S);
        assert_eq!(d.valves[58].circuit, Some(60));
        assert_eq!(d.pumps[12].type_index, Some(17));
        assert_eq!(
            d.triggers[23].effect_true.last(),
            Some(&TriggerEffect::SetJunction("AftGalleryJunction2".into(), 1))
        );
        assert_eq!(d.triggers[10].effect_false.len(), 4);
        assert_eq!(d.curves[&1].eval(0.01), 15.0);
        assert_eq!(net.apu_burn_rate_gph(), 33.0);
        // The only unresolved references are two names that are not defined
        // in FBW's cfg.
        for w in net.warnings() {
            assert!(w.contains("TrimLineIsolationValveAft_"), "unexpected warning: {w}");
        }
        assert_eq!(net.warnings().len(), 2);
    }

    #[test]
    fn parser_tolerates_case_comments_spaces_and_continuations() {
        let text = "
[fuel]
FUEL_TYPE = 2
[Fuel_System] ; comment
version = Latest
engine.1 = name:Eng # index:1
TANK.1 = Name:Main #capacity: 100 # unusablecapacity:2 #outputonlylines: MainToPump ; trailing
#Priority:3
line.1 = Name:MainToPump#Source:Main#Destination:Pump
Line.2 = Name : PumpToValve # Source:Pump # Destination:Valve
Line.3 = Name:ValveToEng#Source:Valve#Destination:Eng
Pump.1 = Name:Pump#Pressure:20#DestinationLine:PumpToValve#TankFuelRequired:Main#type:electric#index:1
Valve.1 = Name:Valve#DestinationLine:ValveToEng
[OTHER]
Tank.2 = Name:NotFuel#Capacity:1
";
        let mut net = FuelNetwork::from_cfg(text).unwrap();
        assert_eq!(net.tank_count(), 1);
        assert_eq!(net.definition().tanks[0].priority, 3);
        assert_eq!(net.definition().tanks[0].capacity, 100.0);
        assert!(net.warnings().is_empty(), "{:?}", net.warnings());
        net.set_tank_gallons(1, 50.0);
        assert_eq!(net.tank_total_gallons(1), 52.0);
        net.pump_on(1);
        net.open_valve(1);
        net.update(1.0, [0.0; 4], 0.0);
        assert_eq!(net.valve_open(1), 1.0);
        run(&mut net, 3600.0, 1.0, [10.0, 0.0, 0.0, 0.0], 0.0);
        assert!((net.tank_gallons(1) - 40.0).abs() < 1e-6, "{}", net.tank_gallons(1));
        assert!(net.engine_fed(1));
        // Down to the unusable fuel, then the pump stops and the engine starves.
        run(&mut net, 5.0 * 3600.0, 1.0, [10.0, 0.0, 0.0, 0.0], 0.0);
        assert!(net.tank_gallons(1).abs() < 1e-9);
        assert!((net.tank_total_gallons(1) - 2.0).abs() < 1e-9);
        assert!(!net.pump_active(1));
        assert!(!net.engine_fed(1));
        assert!(FuelNetwork::from_cfg("[FUEL]\nfuel_type=2\n").is_err());
    }

    /// hyperrealism.md physics workstream 5: a minimal one-tank/one-pump/
    /// one-valve/one-engine network (same shape as
    /// `parser_tolerates_case_comments_spaces_and_continuations`'s cfg) to
    /// exercise [`FuelNetwork::set_tank_pump_derate`] without depending on
    /// the real A380X cfg file being present on disk.
    const DERATE_TEST_CFG: &str = "
[fuel]
FUEL_TYPE = 2
[Fuel_System]
version = Latest
engine.1 = name:Eng # index:1
TANK.1 = Name:Main #capacity: 100 # unusablecapacity:2
line.1 = Name:MainToPump#Source:Main#Destination:Pump
Line.2 = Name:PumpToValve#Source:Pump#Destination:Valve
Line.3 = Name:ValveToEng#Source:Valve#Destination:Eng
Pump.1 = Name:Pump#Pressure:20#DestinationLine:PumpToValve#TankFuelRequired:Main#type:electric#index:1
Valve.1 = Name:Valve#DestinationLine:ValveToEng
";

    #[test]
    fn tank_pump_derate_reduces_pump_pressure_and_flow() {
        let mut net = FuelNetwork::from_cfg(DERATE_TEST_CFG).unwrap();
        net.set_tank_gallons(1, 50.0);
        net.pump_on(1);
        net.open_valve(1);
        // A demand far beyond what this test line's own conductance can
        // ever supply (`fuel_flow_at_1psi`'s default is 0.1, two lines in
        // series -- nowhere near the 500,000 gph asked for here), so the
        // delivered flow stays conductance-(and so pressure-)limited at
        // every derate step below, rather than sitting flat at a small
        // demand the pump could always meet regardless of pressure.
        const DEMAND_GPH: f64 = 500_000.0;
        net.update(1.0, [DEMAND_GPH, 0.0, 0.0, 0.0], 0.0);
        assert_eq!(net.pump_pressure_psi(1), 20.0);
        let full_flow = net.pump_flow_gph(1);
        assert!(full_flow > 0.0, "expected the pump to be delivering flow before any derate");

        // Half-uncovered: pressure halves, flow drops, but the pump keeps
        // running at the reduced pressure (a progressive derate, not a
        // cliff).
        net.set_tank_pump_derate(1, 0.5);
        net.update(1.0, [DEMAND_GPH, 0.0, 0.0, 0.0], 0.0);
        assert_eq!(net.pump_pressure_psi(1), 10.0);
        assert!(net.pump_flow_gph(1) < full_flow, "flow {} should fall below the full-pressure flow {full_flow}", net.pump_flow_gph(1));
        assert!(net.pump_active(1), "a partially uncovered inlet keeps running at reduced pressure");

        // Fully uncovered: no pressure, and the pump goes inactive exactly
        // as it already does when the tank itself runs dry.
        net.set_tank_pump_derate(1, 0.0);
        net.update(1.0, [DEMAND_GPH, 0.0, 0.0, 0.0], 0.0);
        assert_eq!(net.pump_pressure_psi(1), 0.0);
        assert_eq!(net.pump_flow_gph(1), 0.0);
        assert!(!net.pump_active(1), "a fully uncovered inlet delivers no pressure and goes inactive");
    }

    /// Breaker-coupling workstream: `pump_cavitation_load` is the exact
    /// complement of `set_tank_pump_derate`'s own coverage factor --
    /// independent prediction, not re-derived from the accessor under test:
    /// coverage 0.5 (half-uncovered) must read back as extra load 0.5,
    /// coverage 0.0 (fully uncovered/cavitating) as extra load 1.0.
    /// Decouple check: raising coverage back to 1.0 (fully covered) removes
    /// the extra load entirely (reads back 0.0) -- the load is a real
    /// function of inlet coverage, not a constant left over from the fault.
    #[test]
    fn pump_cavitation_load_is_the_complement_of_inlet_coverage_and_decouples() {
        let mut net = FuelNetwork::from_cfg(DERATE_TEST_CFG).unwrap();
        // Pump.1's own `index:1` is its `CIRCUIT_FUEL_PUMP:N` type index.
        assert_eq!(net.pump_cavitation_load(1), 0.0, "full coverage (the default) must add no extra load");

        net.set_tank_pump_derate(1, 0.5);
        let got = net.pump_cavitation_load(1);
        assert!((got - 0.5).abs() < 1e-9, "coverage 0.5 -> predicted extra load 0.5, got {got}");

        net.set_tank_pump_derate(1, 0.0);
        let got = net.pump_cavitation_load(1);
        assert!((got - 1.0).abs() < 1e-9, "fully uncovered -> predicted extra load 1.0, got {got}");

        // Decouple: covering the inlet again removes the load.
        net.set_tank_pump_derate(1, 1.0);
        assert_eq!(net.pump_cavitation_load(1), 0.0, "restoring full coverage must remove the extra load");

        // No matching electric pump for a circuit index that doesn't exist:
        // must read 0.0, never invent a load.
        assert_eq!(net.pump_cavitation_load(999), 0.0);
    }

    /// `pump_hydraulic_power_fraction`'s own derivation (coverage^2, from the
    /// pressure-derate relation `pump_own_pressure` already implements and
    /// `tank_pump_derate_reduces_pump_pressure_and_flow` above already
    /// tests): an independent check of its two endpoints and its
    /// monotonic/bounded shape in between, plus the decouple.
    #[test]
    fn pump_hydraulic_power_fraction_is_the_coverage_squared_and_decouples() {
        let mut net = FuelNetwork::from_cfg(DERATE_TEST_CFG).unwrap();
        assert_eq!(net.pump_hydraulic_power_fraction(1), 1.0, "full coverage (the default) delivers exactly rated hydraulic power");

        net.set_tank_pump_derate(1, 0.5);
        // coverage = 1 - cavitation_load = 1 - 0.5 = 0.5; fraction = 0.5^2 = 0.25.
        let got = net.pump_hydraulic_power_fraction(1);
        assert!((got - 0.25).abs() < 1e-9, "coverage 0.5 -> predicted delivered-power fraction 0.5^2=0.25, got {got}");

        net.set_tank_pump_derate(1, 0.0);
        assert_eq!(net.pump_hydraulic_power_fraction(1), 0.0, "fully uncovered -> coverage 0.0 -> predicted fraction 0.0^2=0.0");

        // Decouple: covering the inlet again restores full rated delivery.
        net.set_tank_pump_derate(1, 1.0);
        assert_eq!(net.pump_hydraulic_power_fraction(1), 1.0, "restoring full coverage must restore full rated delivered power");

        // No matching electric pump for a circuit index that doesn't exist:
        // `pump_cavitation_load` itself defaults to 0.0 (no reported load)
        // for an unmatched pump, so this must read back 1.0 (full/neutral
        // delivery, no derate) -- the same "never invent a fault, default to
        // no-effect" convention `pump_cavitation_load(999)` already uses.
        assert_eq!(net.pump_hydraulic_power_fraction(999), 1.0);
    }

    #[test]
    fn tank_pump_derate_is_clamped_and_ignores_unknown_tanks() {
        let mut net = FuelNetwork::from_cfg(DERATE_TEST_CFG).unwrap();
        net.set_tank_gallons(1, 50.0);
        net.pump_on(1);
        net.update(1.0, [0.0; 4], 0.0);
        assert_eq!(net.pump_pressure_psi(1), 20.0);

        // Above 1.0 clamps to no boost, never exceeding rated pressure.
        net.set_tank_pump_derate(1, 5.0);
        assert_eq!(net.pump_pressure_psi(1), 20.0);
        // Below 0.0 clamps to a full cut, never inverting to negative pressure.
        net.set_tank_pump_derate(1, -3.0);
        assert_eq!(net.pump_pressure_psi(1), 0.0);
        net.set_tank_pump_derate(1, 1.0);
        assert_eq!(net.pump_pressure_psi(1), 20.0);

        // An unknown tank number is silently ignored, matching this module's
        // other per-tank setters (e.g. `set_tank_gallons`) -- no panic, and
        // no effect on the real tank's own pump.
        net.set_tank_pump_derate(999, 0.0);
        assert_eq!(net.pump_pressure_psi(1), 20.0);
    }

    #[test]
    fn engines_drain_their_feed_tanks_at_the_demanded_rate() {
        let Some(mut net) = a380_taxi() else { return };
        let before = all_quantities(&net);
        let demand = [1000.0, 1500.0, 2000.0, 3000.0];
        run(&mut net, 600.0, 1.0 / 30.0, demand, 0.0);
        for (k, f) in FEED.iter().enumerate() {
            let used = before[f - 1] - net.tank_total_gallons(*f);
            let want = demand[k] * 600.0 / 3600.0;
            assert!((used - want).abs() < 1e-6, "feed {k}: used {used}, want {want}");
            assert!(net.engine_fed(k + 1));
            assert!((net.engine_fuel_flow_gph(k + 1) - demand[k]).abs() < 1e-6);
            assert!((net.engine_pressure_psi(k + 1) - 30.0).abs() < 1e-9);
            // The Extra tank is kept full through its LP valve line.
            assert!((net.tank_total_gallons(EXTRA[k]) - 1.0).abs() < 1e-6);
        }
        for t in [LEFT_OUTER, LEFT_MID, LEFT_INNER, RIGHT_INNER, RIGHT_MID, RIGHT_OUTER, TRIM] {
            assert_eq!(net.tank_total_gallons(t), before[t - 1], "tank {t} moved");
        }
        assert!(net.pump_active(1) && net.pump_active(22));
        // The gravity feed pump (20 psi) is shut out by the 30 psi main pumps.
        assert_eq!(net.line_flow_gph(160), 0.0);
        assert!(net.line_flow_gph(17) > 999.0);
    }

    #[test]
    fn fbw_style_extra_tank_draw_is_refilled_through_lp_valve_line() {
        // FBW's FADEC burns straight from tanks 12..15 and passes no demand.
        let Some(mut net) = a380_taxi() else { return };
        let feed_before = net.tank_total_gallons(FEED[0]);
        for _ in 0..300 {
            let e = net.tank_gallons(EXTRA[0]);
            net.set_tank_gallons(EXTRA[0], e - 3000.0 / 3600.0 / 30.0);
            net.update(1.0 / 30.0, [0.0; 4], 0.0);
            assert!((net.tank_gallons(EXTRA[0]) - 1.0).abs() < 1e-9);
            assert!((net.line_flow_gph(21) - 3000.0).abs() < 1e-6);
        }
        let used = feed_before - net.tank_total_gallons(FEED[0]);
        assert!((used - 3000.0 * 10.0 / 3600.0).abs() < 1e-6, "{used}");
    }

    #[test]
    fn closing_an_lp_valve_starves_that_engine() {
        let Some(mut net) = a380_taxi() else { return };
        let demand = [3000.0; 4];
        run(&mut net, 5.0, 0.05, demand, 0.0);
        assert!(net.engine_fed(1));
        net.handle_key_event("FUELSYSTEM_VALVE_CLOSE", 1, 0);
        assert!(!net.valve_switch(1));
        run(&mut net, 1.0, 0.05, demand, 0.0);
        assert!(net.valve_open(1) > 0.6 && net.valve_open(1) < 0.7, "valve animates over 3 s");
        run(&mut net, 10.0, 0.05, demand, 0.0);
        assert_eq!(net.valve_open(1), 0.0);
        assert!(!net.engine_fed(1));
        assert_eq!(net.engine_fuel_flow_gph(1), 0.0);
        assert_eq!(net.tank_total_gallons(EXTRA[0]), 0.0);
        for e in 2..=4 {
            assert!(net.engine_fed(e));
        }
        let feed1 = net.tank_total_gallons(FEED[0]);
        run(&mut net, 10.0, 0.05, demand, 0.0);
        assert_eq!(net.tank_total_gallons(FEED[0]), feed1);
        // Reopen: the Extra tank refills and the engine is fed again.
        net.open_valve(1);
        run(&mut net, 4.0, 0.05, demand, 0.0);
        assert!(net.engine_fed(1));
    }

    #[test]
    fn inner_tank_transfer_pump_fills_feed_tank() {
        let Some(mut net) = a380_taxi() else { return };
        let before = all_quantities(&net);
        net.pump_on(12); // LeftInnerTankPumpFwd
        net.open_valve(11); // FeedTank1FwdTransferValve2
        net.set_junction(7, 2); // forward gallery to the ..Valve2 group
        run(&mut net, 60.0, 0.1, [0.0; 4], 0.0);
        let gained = net.tank_total_gallons(FEED[0]) - before[FEED[0] - 1];
        let lost = before[LEFT_INNER - 1] - net.tank_total_gallons(LEFT_INNER);
        assert!(gained > 1.0, "feed 1 gained {gained}");
        assert!((gained - lost).abs() < 1e-6);
        for f in &FEED[1..] {
            assert_eq!(net.tank_total_gallons(*f), before[f - 1]);
        }
        // Without the junction on option 2 nothing reaches the valve.
        let Some(mut net) = a380_taxi() else { return };
        net.pump_on(12);
        net.open_valve(11);
        run(&mut net, 10.0, 0.1, [0.0; 4], 0.0);
        assert_eq!(net.tank_total_gallons(FEED[0]), before[FEED[0] - 1]);
    }

    #[test]
    fn fbw_trigger_1_starts_the_inner_transfer() {
        let Some(mut net) = a380_taxi() else { return };
        let before = net.tank_total_gallons(FEED[0]);
        net.handle_key_event("K:FUELSYSTEM_TRIGGER_TOGGLE", 1, 0);
        assert!(net.trigger_status(1));
        assert!(net.valve_switch(11));
        assert_eq!(net.pump_switch(12), 1);
        assert_eq!(net.pump_switch(13), 1);
        assert_eq!(net.junction_setting(7), 2);
        run(&mut net, 60.0, 0.1, [0.0; 4], 0.0);
        let rate = (net.tank_total_gallons(FEED[0]) - before) * 60.0;
        // Lines 62 and 81 (FuelFlowAt1PSI 0.00175) at 36.8 psi, times the gain.
        assert!(rate > 1000.0, "transfer rate {rate} gal/h");
        // Toggling off has no EffectFalse: the valve stays open.
        net.toggle_trigger(1);
        assert!(!net.trigger_status(1));
        assert!(net.valve_switch(11));
        net.trigger_on(7); // InnerandMidTanksXferFeed1End
        assert!(!net.valve_switch(11));
    }

    #[test]
    fn manual_trigger_effect_false_reverses() {
        let Some(mut net) = a380_taxi() else { return };
        assert!(net.valve_switch(37) && !net.valve_switch(38));
        net.handle_key_event("FUELSYSTEM_TRIGGER_SET", 11, 1);
        assert!(!net.valve_switch(37) && net.valve_switch(38));
        assert_eq!(net.pump_switch(10), 1);
        net.handle_key_event("FUELSYSTEM_TRIGGER_SET", 11, 0);
        assert!(net.valve_switch(37) && !net.valve_switch(38));
        assert_eq!(net.pump_switch(10), 0);
        assert_eq!(net.pump_switch(12), 1);
        assert_eq!(net.read_simvar("FUELSYSTEM TRIGGER STATUS", 11), Some(0.0));
    }

    #[test]
    fn outer_tank_emergency_transfer_runs_by_gravity() {
        let Some(mut net) = a380_taxi() else { return };
        let before = all_quantities(&net);
        net.open_valve(52); // LeftOuterEmerTransferValve
        run(&mut net, 3.0, 0.1, [0.0; 4], 0.0);
        let start_outer = net.tank_total_gallons(LEFT_OUTER);
        let start_feed = net.tank_total_gallons(FEED[0]);
        run(&mut net, 360.0, 0.5, [0.0; 4], 0.0);
        let moved = start_outer - net.tank_total_gallons(LEFT_OUTER);
        assert!((moved - 50.0).abs() < 1e-6, "moved {moved} gal in 6 min at 500 gal/h");
        assert!((net.tank_total_gallons(FEED[0]) - start_feed - moved).abs() < 1e-9);
        assert!((net.line_flow_gph(142) - 500.0).abs() < 1e-6);
        assert!((net.line_flow_gph(144) - 500.0).abs() < 1e-6);
        assert_eq!(net.tank_total_gallons(RIGHT_OUTER), before[RIGHT_OUTER - 1]);
        // Stops when the feed tank is full.
        net.set_tank_gallons(FEED[0], net.tank_capacity(FEED[0]));
        run(&mut net, 60.0, 0.5, [0.0; 4], 0.0);
        assert_eq!(net.tank_total_gallons(FEED[0]), net.tank_capacity(FEED[0]));
    }

    #[test]
    fn outer_tank_pump_transfer_via_trigger_35() {
        let Some(mut net) = a380_taxi() else { return };
        let before = all_quantities(&net);
        net.trigger_on(35);
        run(&mut net, 60.0, 0.1, [0.0; 4], 0.0);
        let to1 = net.tank_total_gallons(FEED[0]) - before[FEED[0] - 1];
        let to4 = net.tank_total_gallons(FEED[3]) - before[FEED[3] - 1];
        let from = before[LEFT_OUTER - 1] - net.tank_total_gallons(LEFT_OUTER) + before[RIGHT_OUTER - 1]
            - net.tank_total_gallons(RIGHT_OUTER);
        assert!(to1 > 1.0 && to4 > 1.0);
        assert!((to1 + to4 - from).abs() < 1e-6);
    }

    #[test]
    fn trim_tank_transfers_to_feed_tank() {
        let Some(mut net) = a380_taxi() else { return };
        let before = all_quantities(&net);
        net.trigger_on(24); // TrimTankTransferToFeedTank1
        run(&mut net, 5.0, 0.1, [0.0; 4], 0.0);
        let gained = net.tank_total_gallons(FEED[0]) - before[FEED[0] - 1];
        let lost = before[TRIM - 1] - net.tank_total_gallons(TRIM);
        assert!(gained > 1.0, "{gained}");
        assert!((gained - lost).abs() < 1e-6);
        for f in &FEED[1..] {
            assert_eq!(net.tank_total_gallons(*f), before[f - 1]);
        }
        assert!(net.pump_active(19) && net.pump_active(20));
        // Trim tank empty: its pumps stop (TankFuelRequired).
        net.set_tank_gallons(TRIM, 0.0);
        net.update(0.1, [0.0; 4], 0.0);
        assert!(!net.pump_active(19));
        assert_eq!(net.pump_switch(19), 1);
    }

    #[test]
    fn crossfeed_feeds_an_engine_whose_feed_tank_is_empty() {
        let Some(mut net) = a380_taxi() else { return };
        net.set_tank_gallons(FEED[1], 0.0);
        let demand = [1000.0; 4];
        run(&mut net, 10.0, 0.1, demand, 0.0);
        assert!(!net.engine_fed(2));
        assert!(!net.pump_active(3) && !net.pump_active(4) && !net.pump_active(23));
        net.open_valve(46);
        net.open_valve(47);
        run(&mut net, 10.0, 0.1, demand, 0.0);
        assert!(net.engine_fed(2));
        let f1 = net.tank_total_gallons(FEED[0]);
        run(&mut net, 36.0, 0.1, demand, 0.0);
        // Feed 1 now supplies engines 1 and 2.
        assert!((f1 - net.tank_total_gallons(FEED[0]) - 20.0).abs() < 1e-6);
        // With both sides pressurised, the crossfeed carries nothing.
        net.set_tank_gallons(FEED[1], 3000.0);
        run(&mut net, 1.0, 0.1, demand, 0.0);
        assert_eq!(net.line_flow_gph(129), 0.0);
        assert_eq!(net.line_pressure_psi(129), 0.0);    }

    #[test]
    fn apu_feed_needs_its_pump() {
        let Some(mut net) = a380_taxi() else { return };
        let burn = net.apu_burn_rate_gph();
        net.handle_key_event("FUELSYSTEM_PUMP_ON", 21, 0);
        run(&mut net, 5.0, 0.1, [0.0; 4], burn);
        assert!(net.apu_fed());
        assert!((net.line_flow_gph(141) - burn).abs() < 1e-6);
        net.handle_key_event("FUELSYSTEM_PUMP_OFF", 21, 0);
        run(&mut net, 120.0, 0.1, [0.0; 4], burn);
        assert!(!net.apu_fed());
        assert_eq!(net.line_flow_gph(141), 0.0);
    }

    #[test]
    fn unpowered_circuits_stop_pumps_and_freeze_valves() {
        let Some(mut net) = a380_taxi() else { return };
        net.set_fuel_pump_circuit_powered(1, false);
        net.update(0.1, [0.0; 4], 0.0);
        assert!(!net.pump_active(1) && net.pump_active(2));
        net.set_fuel_valve_circuit_powered(46, false);
        net.open_valve(46);
        run(&mut net, 5.0, 0.1, [0.0; 4], 0.0);
        assert!(net.valve_switch(46));
        assert_eq!(net.valve_open(46), 0.0);
    }

    #[test]
    fn long_run_conserves_fuel_and_respects_limits() {
        let Some(mut net) = a380_taxi() else { return };
        let initial = net.total_fuel_gallons();
        net.trigger_on(1);
        net.trigger_on(4);
        net.trigger_on(35);
        net.open_valve(52);
        net.open_valve(53);
        net.pump_on(21);
        let dt = 0.5;
        let mut burnt = 0.0;
        for step in 0..(3 * 3600 * 2) {
            let t = step as f64 * dt;
            let demand = [
                1200.0 + 600.0 * (t / 900.0).sin().abs(),
                1400.0,
                1400.0 + 300.0 * (t / 500.0).cos(),
                1200.0,
            ];
            if step == 3600 {
                net.trigger_on(24);
            }
            if step == 9000 {
                net.close_valve(3);
            }
            net.update(dt, demand, 33.0);
            burnt += (0..4).map(|k| net.engine_fuel_flow_gph(k + 1)).sum::<f64>() * dt / 3600.0;
            burnt += net.apu_fuel_flow_gph() * dt / 3600.0;
            if step % 600 == 0 {
                for i in 1..=net.tank_count() {
                    let q = net.tank_total_gallons(i);
                    assert!(q >= 0.0, "tank {i} negative: {q}");
                    assert!(q <= net.tank_capacity(i) + 1e-9, "tank {i} overfilled: {q}");
                }
            }
        }
        let now = net.total_fuel_gallons();
        assert!(
            ((initial - now) - burnt).abs() < 1e-6 * initial,
            "fuel lost {} vs burnt {burnt}",
            initial - now
        );
        assert!((net.total_burnt_gallons() - burnt).abs() < 1e-6 * initial);
        assert!(!net.engine_fed(3));
        for i in 1..=net.tank_count() {
            let q = net.tank_total_gallons(i);
            assert!(q >= 0.0 && q <= net.tank_capacity(i) + 1e-9);
        }
    }

    #[test]
    fn junction_option_listing_an_input_closes_other_inputs() {
        // The SDK's P-51 selector: each option names one tank's input line.
        let text = "
[FUEL_SYSTEM]
Version = Latest
Engine.1 = Name:Engine#Index:1
Tank.1 = Name:LeftMain#Capacity:92#UnusableCapacity:0#OutputOnlyLines:LTankToFuelSelector
Tank.2 = Name:RightMain#Capacity:92#UnusableCapacity:0#OutputOnlyLines:RTankToFuelSelector
Line.1 = Name:LTankToFuelSelector#Source:LeftMain#Destination:FuelSelector
Line.2 = Name:RTankToFuelSelector#Source:RightMain#Destination:FuelSelector
Line.3 = Name:FuelSelectorToBoosterPump#Source:FuelSelector#Destination:BoosterPump
Line.4 = Name:BoosterPumpToEngine#Source:BoosterPump#Destination:Engine
Junction.1 = Name:FuelSelector#InputOnlyLines:LTankToFuelSelector,RTankToFuelSelector#OutputOnlyLines:FuelSelectorToBoosterPump#Option:LTankToFuelSelector,FuelSelectorToBoosterPump#Option:RTankToFuelSelector,FuelSelectorToBoosterPump#Option:FuelSelectorToBoosterPump
Pump.1 = Name:BoosterPump#Pressure:11#DestinationLine:BoosterPumpToEngine#Type:Electric#Index:1
";
        let mut net = FuelNetwork::from_cfg(text).unwrap();
        net.set_tank_gallons(1, 50.0);
        net.set_tank_gallons(2, 50.0);
        net.pump_on(1);
        run(&mut net, 360.0, 1.0, [10.0, 0.0, 0.0, 0.0], 0.0);
        assert!((net.tank_gallons(1) - 49.0).abs() < 1e-9, "left {}", net.tank_gallons(1));
        assert_eq!(net.tank_gallons(2), 50.0);
        net.set_junction(1, 2);
        run(&mut net, 360.0, 1.0, [10.0, 0.0, 0.0, 0.0], 0.0);
        assert!((net.tank_gallons(1) - 49.0).abs() < 1e-9, "left {}", net.tank_gallons(1));
        assert!((net.tank_gallons(2) - 49.0).abs() < 1e-9);
        // Option 3 names no input line, so both tanks feed.
        net.set_junction(1, 3);
        run(&mut net, 360.0, 1.0, [10.0, 0.0, 0.0, 0.0], 0.0);
        assert!((net.tank_gallons(1) + net.tank_gallons(2) - 97.0).abs() < 1e-9);
        net.set_junction(1, 9);
        assert_eq!(net.junction_setting(1), 3);
    }

    #[test]
    fn automatic_triggers_with_delay_and_priority_fill() {
        let text = "
[FUEL_SYSTEM]
Version = 5
Engine.1 = Name:Eng#Index:1
Tank.1 = Name:Aux#Capacity:100#UnusableCapacity:0#Priority:1#OutputOnlyLines:AuxToValve
Tank.2 = Name:Main#Capacity:50#UnusableCapacity:0#Priority:2#InputOnlyLines:ValveToMain#OutputOnlyLines:MainToPump
Line.1 = Name:AuxToValve#Source:Aux#Destination:Xfer#GravityBasedFuelFlow:360
Line.2 = Name:ValveToMain#Source:Xfer#Destination:Main#GravityBasedFuelFlow:360
Line.3 = Name:MainToPump#Source:Main#Destination:Pump
Line.4 = Name:PumpToEng#Source:Pump#Destination:Eng
Valve.1 = Name:Xfer#OpeningTime:0
Pump.1 = Name:Pump#Pressure:10#DestinationLine:PumpToEng#TankFuelRequired:Main
Trigger.1 = Name:MainLow#Target:Main#Threshold:20#Condition:TankQuantityBelow#DelayTrue:10#EffectTrue:OpenValve.Xfer#EffectFalse:CloseValve.Xfer
Trigger.2 = Name:Start#Condition:Autostart_Enabled#EffectTrue:StartPump.Pump
";
        let mut net = FuelNetwork::from_cfg(text).unwrap();
        assert_eq!(net.set_total_fuel_by_priority(120.0), 0.0);
        assert_eq!(net.tank_gallons(2), 50.0);
        assert_eq!(net.tank_gallons(1), 70.0);
        net.set_autostart_enabled(true);
        net.update(0.1, [0.0; 4], 0.0);
        assert!(net.trigger_status(2) && net.pump_switch(1) == 1);
        // Burn 3600 gal/h = 1 gal/s: Main drops below 20 after 30 s, and the
        // transfer valve opens 10 s later.
        run(&mut net, 30.5, 0.1, [3600.0, 0.0, 0.0, 0.0], 0.0);
        assert!(!net.trigger_status(1));
        run(&mut net, 9.0, 0.1, [3600.0, 0.0, 0.0, 0.0], 0.0);
        assert!(!net.trigger_status(1) && !net.valve_switch(1));
        run(&mut net, 1.5, 0.1, [3600.0, 0.0, 0.0, 0.0], 0.0);
        assert!(net.trigger_status(1) && net.valve_switch(1));
        let aux = net.tank_gallons(1);
        run(&mut net, 10.0, 0.1, [0.0; 4], 0.0);
        assert!((aux - net.tank_gallons(1) - 1.0).abs() < 1e-6, "360 gal/h gravity");
    }

    /// `failures.rs` `extra::fuel` 28_000 ("Tank 1 feed pump A"): the hook
    /// was previously unconsumed. Confirms `refresh_catalogue_failures`
    /// zeroes only Feed1TankPump1's own delivery pressure, leaving its
    /// sibling Feed1TankPump2 (pump 2) unaffected, matching the catalogue's
    /// per-pump wording ("A main feed pump fails to prime its tank's feed
    /// line", singular).
    #[test]
    fn catalogue_feed_pump_failure_kills_one_pump_not_its_sibling() {
        let _g = failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        failures::Failures::new();
        let Some(mut net) = a380_taxi() else { return };
        net.update(1.0, [0.0; 4], 0.0);
        assert!(net.pump_pressure_psi(1) > 0.0, "pump 1 primes normally before the failure");
        assert!(net.pump_pressure_psi(2) > 0.0, "pump 2 primes normally before the failure");
        failures::set_active(28_000, true);
        net.update(1.0, [0.0; 4], 0.0);
        assert_eq!(net.pump_pressure_psi(1), 0.0, "28_000 kills pump 1's own pressure");
        assert!(net.pump_pressure_psi(2) > 0.0, "28_000 leaves pump 2 (a separate id) alone");
        failures::set_active(28_000, false);
    }

    /// Every further pump and valve failure (28_100..) names real elements
    /// of the A380's own fuel system, and a failed outer tank transfer pump
    /// loses its own pressure while its opposite number keeps working.
    #[test]
    fn every_fuel_element_failure_acts_on_a_real_element() {
        let _g = failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        failures::Failures::new();
        let Some(mut net) = a380_taxi() else { return };
        for &(id, name, pump, elements) in failures::extra::FUEL_ELEMENTS {
            for e in elements {
                let found = if pump { net.def.pumps.iter().any(|p| p.name == *e) } else { net.def.valves.iter().any(|v| v.name == *e) };
                assert!(found, "{id} {name}: no element {e} in flight_model.cfg");
            }
        }
        let left = net.pump_index("LeftOuterTankPump").unwrap();
        let right = net.pump_index("RightOuterTankPump").unwrap();
        net.pump_on(left);
        net.pump_on(right);
        net.update(1.0, [0.0; 4], 0.0);
        assert!(net.pump_pressure_psi(left) > 0.0 && net.pump_pressure_psi(right) > 0.0);
        failures::set_active(28_100, true);
        net.update(1.0, [0.0; 4], 0.0);
        assert_eq!(net.pump_pressure_psi(left), 0.0, "28_100 kills the left outer tank pump");
        assert!(net.pump_pressure_psi(right) > 0.0, "and leaves the right one alone");
        failures::set_active(28_100, false);
    }

    /// 28_008 ("Trim tank transfer pump") takes out both real trim pumps
    /// (TrimTankPumpLeft/Right, pumps 19/20), since the catalogue's single
    /// item doesn't distinguish left/right.
    #[test]
    fn catalogue_trim_pump_failure_kills_both_trim_pumps() {
        let _g = failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        failures::Failures::new();
        let Some(mut net) = a380_taxi() else { return };
        net.pump_on(19);
        net.pump_on(20);
        net.update(1.0, [0.0; 4], 0.0);
        assert!(net.pump_pressure_psi(19) > 0.0);
        assert!(net.pump_pressure_psi(20) > 0.0);
        failures::set_active(28_008, true);
        net.update(1.0, [0.0; 4], 0.0);
        assert_eq!(net.pump_pressure_psi(19), 0.0);
        assert_eq!(net.pump_pressure_psi(20), 0.0);
        failures::set_active(28_008, false);
    }

    /// 28_009 ("Cross-feed valve 1-2"): a stuck valve stops tracking its
    /// switch and freezes at its current position instead of snapping open
    /// or closed.
    #[test]
    fn catalogue_crossfeed_valve_failure_freezes_position() {
        let _g = failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        failures::Failures::new();
        let Some(mut net) = a380_taxi() else { return };
        assert_eq!(net.valve_open(46), 0.0, "CrossFeedValve1 starts closed");
        failures::set_active(28_009, true);
        net.open_valve(46);
        run(&mut net, 10.0, 0.1, [0.0; 4], 0.0);
        assert_eq!(net.valve_open(46), 0.0, "stuck valve ignores the open command");
        failures::set_active(28_009, false);
        run(&mut net, 10.0, 0.1, [0.0; 4], 0.0);
        assert!(net.valve_open(46) > 0.9, "clearing the failure lets it move again");
    }

    /// FUEL-0XX (`set_crossfeed_selection`'s own doc): selecting a
    /// cross-feed valve opens it and deselecting shuts it, and the four
    /// valves are independent of each other -- selecting 1 and 3 must never
    /// move 2 or 4.
    #[test]
    fn set_crossfeed_selection_opens_and_shuts_each_valve_independently() {
        // `run`/`net.update` read the process-global failure catalogue
        // every tick (`refresh_catalogue_failures`), so any test exercising
        // a real network must serialise against every other test that can
        // set 28_009/28_010 active, the same guard
        // `catalogue_crossfeed_valve_failure_freezes_position` above takes.
        let _g = failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        failures::Failures::new();
        let Some(mut net) = a380_taxi() else { return };
        // The valves' starting position depends on the taxi flight state;
        // force a known closed baseline before asserting anything about a
        // selection driving them.
        for v in [46, 47, 48, 49] {
            net.close_valve(v);
        }
        run(&mut net, 5.0, 0.1, [0.0; 4], 0.0);
        assert_eq!(
            [net.valve_open(46), net.valve_open(47), net.valve_open(48), net.valve_open(49)],
            [0.0; 4],
            "closed baseline"
        );

        net.set_crossfeed_selection([true, false, true, false]);
        run(&mut net, 5.0, 0.1, [0.0; 4], 0.0);
        assert!(net.valve_open(46) > 0.9, "CrossFeedValve1 selected: must open");
        assert_eq!(net.valve_open(47), 0.0, "CrossFeedValve2 not selected: must stay shut");
        assert!(net.valve_open(48) > 0.9, "CrossFeedValve3 selected: must open");
        assert_eq!(net.valve_open(49), 0.0, "CrossFeedValve4 not selected: must stay shut");

        net.set_crossfeed_selection([false; 4]);
        run(&mut net, 5.0, 0.1, [0.0; 4], 0.0);
        assert_eq!(
            [net.valve_open(46), net.valve_open(47), net.valve_open(48), net.valve_open(49)],
            [0.0; 4],
            "deselecting every valve shuts them all again"
        );
    }

    // ------------------------------------------------------------------
    // Three-way intersection: continuous catalogue failures interacting
    // through the ordinary network solve.
    // ------------------------------------------------------------------

    /// A minimal synthetic `.cfg`, not the real A380X, built so every
    /// constant in the derivation on
    /// [`intersection_pump_degradation_and_stuck_crossfeed_valve_starve_engine`]
    /// is exact. `fuel_type=1` gives density 6.0 lb/gal -- not physically
    /// avgas, picked only so the flow constant below is a whole number.
    /// Two independent paths feed Engine1: Tank1 through the catalogue's
    /// own `Feed1TankPump1` directly, and Tank2 through `XfeedPump` and the
    /// catalogue's own `CrossFeedValve1`. The tank-to-pump and
    /// pump-to-valve feeder lines (`FuelFlowAt1PSI:10`) and the tank
    /// capacities (1,000,000 gal) are sized so neither fuel supply nor
    /// those feeder segments is ever the bottleneck -- only the two lines'
    /// own small `FuelFlowAt1PSI` (into Engine1) are.
    const INTERSECTION_CFG: &str = "\
[FUEL]
fuel_type = 1

[FUEL_SYSTEM]
version = 5
Tank.1 = Name:Tank1#Capacity:1000000
Tank.2 = Name:Tank2#Capacity:1000000
Engine.1 = Name:Engine1#Index:1
Pump.1 = Name:Feed1TankPump1#Type:Electric#Pressure:30#DestinationLine:Line2#TankFuelRequired:Tank1
Pump.2 = Name:XfeedPump#Type:Electric#Pressure:30#DestinationLine:Line4#TankFuelRequired:Tank2
Valve.1 = Name:CrossFeedValve1#OpeningTime:1
Line.1 = Name:Line1#Source:Tank1#Destination:Feed1TankPump1#FuelFlowAt1PSI:10
Line.2 = Name:Line2#Source:Feed1TankPump1#Destination:Engine1#FuelFlowAt1PSI:0.001
Line.3 = Name:Line3#Source:Tank2#Destination:XfeedPump#FuelFlowAt1PSI:10
Line.4 = Name:Line4#Source:XfeedPump#Destination:CrossFeedValve1#FuelFlowAt1PSI:10
Line.5 = Name:Line5#Source:CrossFeedValve1#Destination:Engine1#FuelFlowAt1PSI:0.00075
";

    /// The decoupling control for the same test: identical network except
    /// Line 5 (CrossFeedValve1 -> Engine1) is removed. CrossFeedValve1
    /// still exists and the 28_009 failure still lands on it, but nothing
    /// carries its position to Engine1 any more -- the physical coupling
    /// the intersection depends on is cut.
    const INTERSECTION_CFG_DECOUPLED: &str = "\
[FUEL]
fuel_type = 1

[FUEL_SYSTEM]
version = 5
Tank.1 = Name:Tank1#Capacity:1000000
Tank.2 = Name:Tank2#Capacity:1000000
Engine.1 = Name:Engine1#Index:1
Pump.1 = Name:Feed1TankPump1#Type:Electric#Pressure:30#DestinationLine:Line2#TankFuelRequired:Tank1
Pump.2 = Name:XfeedPump#Type:Electric#Pressure:30#DestinationLine:Line4#TankFuelRequired:Tank2
Valve.1 = Name:CrossFeedValve1#OpeningTime:1
Line.1 = Name:Line1#Source:Tank1#Destination:Feed1TankPump1#FuelFlowAt1PSI:10
Line.2 = Name:Line2#Source:Feed1TankPump1#Destination:Engine1#FuelFlowAt1PSI:0.001
Line.3 = Name:Line3#Source:Tank2#Destination:XfeedPump#FuelFlowAt1PSI:10
Line.4 = Name:Line4#Source:XfeedPump#Destination:CrossFeedValve1#FuelFlowAt1PSI:10
";

    fn intersection_net(cfg: &str) -> FuelNetwork {
        let mut net = FuelNetwork::from_cfg(cfg).expect("synthetic intersection cfg parses");
        net.set_tank_gallons(1, 500_000.0);
        net.set_tank_gallons(2, 500_000.0);
        net.pump_on(1);
        net.pump_on(2);
        net
    }

    /// Hand-derived before any `update` call: flow gph = `FuelFlowAt1PSI *
    /// gain * 3600 / density * dP(psi)` (module docs, "Undocumented MSFS
    /// behaviour" #1), with `gain` = [`DEFAULT_LINE_FLOW_GAIN`] = 60 and
    /// `density` = 6.0 (`fuel_type=1` in the cfg above), so the flow
    /// constant is `C = 60 * 3600 / 6.0 = 36000` exactly.
    ///
    /// * Feed path (`Feed1TankPump1`, 30 psi, `Line2` FuelFlowAt1PSI
    ///   0.001): healthy capacity = `0.001 * 36000 * 30` = 1080 gal/h. At
    ///   catalogue id 28_000 magnitude 0.6, `pump_own_pressure`'s
    ///   `health = 1 - magnitude` = 0.4, so degraded capacity =
    ///   `1080 * 0.4` = **432 gal/h**.
    /// * Crossfeed path (`XfeedPump`, 30 psi, through `CrossFeedValve1`,
    ///   `Line5` FuelFlowAt1PSI 0.00075): fully-open capacity =
    ///   `0.00075 * 36000 * 30` = 810 gal/h. `CrossFeedValve1` is armed
    ///   with 28_009 while still closed (so it seizes at
    ///   `valve_stuck_at` = 0) and only commanded open afterwards, so per
    ///   `substep`'s stuck-valve blend it settles at
    ///   `0 + (1 - 0.5) * (1 - 0)` = 0.5 open: restricted capacity =
    ///   `810 * 0.5` = **405 gal/h**.
    ///
    /// Both paths terminate on the same `Engine1` node, so the network's
    /// max-flow solve sums their capacities (module docs #3, "Sharing"):
    /// pump-alone 432 + 810 = 1242, valve-alone 1080 + 405 = 1485, both
    /// together 432 + 405 = **837**. Against an engine-1 demand of 1000
    /// gal/h: pump-alone and valve-alone both clear it (delivered flow
    /// saturates at the demand itself, 1000), but together the network can
    /// only supply 837 -- a 163 gal/h shortfall neither failure alone
    /// produces. None of 1080, 1242, 810, 1485, 432, 405, 837 come from
    /// running the sim; they are computed above from the network's
    /// documented flow formula and this module's own pump-health/
    /// valve-stuck fractions before `update` is ever called.
    #[test]
    fn intersection_pump_degradation_and_stuck_crossfeed_valve_starve_engine() {
        let _g = failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        failures::Failures::new();
        failures::reset_all();
        let demand = [1000.0, 0.0, 0.0, 0.0];

        // Case 1: pump degraded alone; the valve is healthy and opened from
        // the start, well before the run, so it is fully open throughout.
        let mut net = intersection_net(INTERSECTION_CFG);
        net.open_valve(1);
        failures::set_magnitude(28_000, 0.6);
        run(&mut net, 3.0, 0.1, demand, 0.0);
        assert!(
            (net.engine_fuel_flow_gph(1) - 1000.0).abs() < 1.0,
            "pump alone still meets demand: {}",
            net.engine_fuel_flow_gph(1)
        );
        assert!(net.engine_fed(1), "pump-alone: engine still fed");
        failures::reset_all();

        // Case 2: crossfeed valve stuck at 50% alone, pump healthy. The
        // failure is armed one frame before the valve is commanded open, so
        // it seizes at position 0 (matching the derivation above) rather
        // than wherever an arbitrary earlier open command left it.
        let mut net = intersection_net(INTERSECTION_CFG);
        failures::set_magnitude(28_009, 0.5);
        net.update(0.1, [0.0; 4], 0.0);
        net.open_valve(1);
        run(&mut net, 3.0, 0.1, demand, 0.0);
        assert!((net.valve_open(1) - 0.5).abs() < 1e-6, "valve settles at 50% open: {}", net.valve_open(1));
        assert!(
            (net.engine_fuel_flow_gph(1) - 1000.0).abs() < 1.0,
            "valve alone still meets demand: {}",
            net.engine_fuel_flow_gph(1)
        );
        assert!(net.engine_fed(1), "valve-alone: engine still fed");
        failures::reset_all();

        // Case 3: both together. Neither alone starves Engine1 (cases 1-2
        // above); the network solve summing 432 + 405 = 837 gal/h does.
        let mut net = intersection_net(INTERSECTION_CFG);
        failures::set_magnitude(28_009, 0.5);
        net.update(0.1, [0.0; 4], 0.0);
        net.open_valve(1);
        failures::set_magnitude(28_000, 0.6);
        run(&mut net, 3.0, 0.1, demand, 0.0);
        assert!((net.valve_open(1) - 0.5).abs() < 1e-6);
        assert!(
            (net.engine_fuel_flow_gph(1) - 837.0).abs() < 1.0,
            "combined: expected 837 gal/h, got {}",
            net.engine_fuel_flow_gph(1)
        );
        assert!(!net.engine_fed(1), "combined: engine starved by both together");
        failures::reset_all();
    }

    /// Decoupling control for the test above (per the emergence-testing
    /// directive: prove the interaction by cutting the physical coupling it
    /// depends on and showing the effect vanishes). With Line 5 removed
    /// (`INTERSECTION_CFG_DECOUPLED`), CrossFeedValve1's position can no
    /// longer reach Engine1 at all, so its own 28_009 failure -- applied at
    /// the very same magnitude as the starving case above -- must make
    /// zero difference to Engine1's delivered flow, which should instead
    /// track the degraded feed pump alone (432 gal/h, from the same
    /// derivation above; still short of the 1000 gal/h demand, but for a
    /// single-cause reason, not the three-way interaction).
    #[test]
    fn intersection_starvation_vanishes_when_crossfeed_link_to_engine_is_cut() {
        let _g = failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        failures::Failures::new();
        failures::reset_all();
        let demand = [1000.0, 0.0, 0.0, 0.0];

        // Same combined failures as case 3 above, but on the decoupled
        // network.
        let mut net = intersection_net(INTERSECTION_CFG_DECOUPLED);
        failures::set_magnitude(28_009, 0.5);
        net.update(0.1, [0.0; 4], 0.0);
        net.open_valve(1);
        failures::set_magnitude(28_000, 0.6);
        run(&mut net, 3.0, 0.1, demand, 0.0);
        let with_valve_failed = net.engine_fuel_flow_gph(1);
        failures::reset_all();

        // Identical run, but the valve is left healthy (28_009 never
        // armed). If the crossfeed path still reached Engine1, this would
        // differ from the run above by the 405 gal/h the healthy valve adds
        // (as it does in the coupled network, case 1 vs. case 3 of the test
        // above). On the decoupled network it must not differ at all.
        let mut net = intersection_net(INTERSECTION_CFG_DECOUPLED);
        net.open_valve(1);
        failures::set_magnitude(28_000, 0.6);
        run(&mut net, 3.0, 0.1, demand, 0.0);
        let with_valve_healthy = net.engine_fuel_flow_gph(1);

        assert!(
            (with_valve_failed - with_valve_healthy).abs() < 1.0,
            "cutting the crossfeed-to-engine link should erase the valve's effect: \
             failed={with_valve_failed}, healthy={with_valve_healthy}"
        );
        // Both equal the degraded feed pump's own capacity, 432 gal/h --
        // the pump-only figure from the derivation above, not 837: this
        // confirms the coupling was really cut, not just coincidentally
        // equal.
        assert!(
            (with_valve_failed - 432.0).abs() < 1.0,
            "decoupled delivered flow should equal the feed pump alone: {with_valve_failed}"
        );
        assert!(!net.engine_fed(1), "still short of demand, but only from the pump, not the interaction");
        failures::reset_all();
    }
}

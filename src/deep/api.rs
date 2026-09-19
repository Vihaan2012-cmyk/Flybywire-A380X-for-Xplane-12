//! The registration API for the deep systems: every area registers its
//! failures, components and ECAM alerts here, in code, from its own
//! `registry.rs` (`pub fn register(r: &mut Registry)`), so no area edits a
//! shared file and nothing is described only in prose.
//!
//! Ids: `failure_id(area, ata, n)` = area code × 1 000 000 + ATA × 1 000 + n,
//! so areas never collide with each other or with the existing catalogue
//! (`failures.rs`, ids below 100 000).
//!
//! ECAM: an alert is data — level, aural, trigger condition over published
//! variables, confirmation delay, inhibits, and procedure lines whose
//! completion is itself a condition over the variable the real cockpit
//! control writes. Pressing the real button changes that variable and the
//! line completes; nothing is ticked by hand. `Ecam` below evaluates alerts
//! against any variable source each tick.

use std::collections::HashMap;

/// The area codes: the high part of every new failure id.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Area {
    Electrical = 1,
    EngineAccessories = 2,
    Hydraulics = 3,
    FlightControls = 4,
    GearStructure = 5,
    Sensors = 6,
    Apu = 7,
    FireIce = 8,
    AvionicsNetwork = 9,
    Cabin = 10,
    ThermalZones = 11,
    Wiring = 12,
    FlightModel = 13,
    Environment = 14,
    PneumaticDucts = 15,
    EngineCore = 16,
    Breakers = 17,
    Integration = 18,
    Fuel = 19,
}

pub const fn failure_id(area: Area, ata: u16, n: u16) -> u64 {
    area as u64 * 1_000_000 + ata as u64 * 1_000 + n as u64
}

/// One single failure: a physical fault on one modelled element, with a
/// magnitude 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Debug)]
pub struct FailureDef {
    pub id: u64,
    pub area: Area,
    pub ata: u16,
    /// As a maintenance crew would name it, e.g. "Green hydraulic line 3 leak".
    pub name: String,
    /// The component it lives on (`ComponentDef::id`).
    pub component: String,
    /// The model field the magnitude drives, `file::Struct.field`.
    pub model_field: String,
    /// What magnitude 0..1 means physically, e.g. "leak orifice area, 0..20 mm²".
    pub magnitude: String,
    /// The physical effect, briefly.
    pub effect: String,
}

/// One health parameter of a component, 0..1.
#[derive(Clone, Debug)]
pub struct ParamDef {
    pub name: String,
    /// What 0 and 1 mean physically.
    pub meaning: String,
    /// Healthy value (usually 0 for a loss, 1 for an efficiency fraction).
    pub healthy: f64,
}

/// A physical part with health parameters (wear, degradation), persisted
/// between flights and shown on the Components page.
#[derive(Clone, Debug)]
pub struct ComponentDef {
    /// Stable id, `NN_system.part`, e.g. "29_hyd.green_edp_1".
    pub id: String,
    pub area: Area,
    pub ata: u16,
    pub name: String,
    pub params: Vec<ParamDef>,
    /// Failures that act on this component.
    pub failures: Vec<u64>,
}

// ---------------------------------------------------------------------------
// Conditions over published variables.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cmp {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
}

/// A condition over published variables (the plugin's Vars / datarefs, by
/// name, e.g. `ENGINE_OIL_PRESSURE_PSI:2`).
#[derive(Clone, Debug)]
pub enum Cond {
    Always,
    Var { name: String, cmp: Cmp, value: f64 },
    /// Two variables compared, e.g. N1 of engine 1 against engine 2.
    VarVar { a: String, cmp: Cmp, b: String },
    And(Vec<Cond>),
    Or(Vec<Cond>),
    Not(Box<Cond>),
}

/// Builder: `var("ENGINE_OIL_PRESSURE_PSI:2").lt(25.0)`.
pub fn var(name: &str) -> VarRef {
    VarRef(name.to_string())
}
pub struct VarRef(String);
impl VarRef {
    fn cmp(self, cmp: Cmp, value: f64) -> Cond {
        Cond::Var { name: self.0, cmp, value }
    }
    pub fn lt(self, v: f64) -> Cond {
        self.cmp(Cmp::Lt, v)
    }
    pub fn le(self, v: f64) -> Cond {
        self.cmp(Cmp::Le, v)
    }
    pub fn gt(self, v: f64) -> Cond {
        self.cmp(Cmp::Gt, v)
    }
    pub fn ge(self, v: f64) -> Cond {
        self.cmp(Cmp::Ge, v)
    }
    pub fn eq(self, v: f64) -> Cond {
        self.cmp(Cmp::Eq, v)
    }
    pub fn ne(self, v: f64) -> Cond {
        self.cmp(Cmp::Ne, v)
    }
    /// True when the variable is nonzero (a switch on, a valve open).
    pub fn on(self) -> Cond {
        self.cmp(Cmp::Ne, 0.0)
    }
    pub fn off(self) -> Cond {
        self.cmp(Cmp::Eq, 0.0)
    }
}
pub fn all(conds: Vec<Cond>) -> Cond {
    Cond::And(conds)
}
pub fn any(conds: Vec<Cond>) -> Cond {
    Cond::Or(conds)
}
pub fn not(c: Cond) -> Cond {
    Cond::Not(Box::new(c))
}

impl Cond {
    /// Evaluate against a variable source; a missing variable reads 0.
    pub fn eval(&self, read: &dyn Fn(&str) -> f64) -> bool {
        let test = |x: f64, cmp: Cmp, y: f64| match cmp {
            Cmp::Lt => x < y,
            Cmp::Le => x <= y,
            Cmp::Gt => x > y,
            Cmp::Ge => x >= y,
            Cmp::Eq => (x - y).abs() < 1e-9,
            Cmp::Ne => (x - y).abs() >= 1e-9,
        };
        match self {
            Cond::Always => true,
            Cond::Var { name, cmp, value } => test(read(name), *cmp, *value),
            Cond::VarVar { a, cmp, b } => test(read(a), *cmp, read(b)),
            Cond::And(v) => v.iter().all(|c| c.eval(read)),
            Cond::Or(v) => v.iter().any(|c| c.eval(read)),
            Cond::Not(c) => !c.eval(read),
        }
    }
}

// ---------------------------------------------------------------------------
// ECAM alerts.
// ---------------------------------------------------------------------------

/// Alert level, with the A380's colour and attention getters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Level 3: red, continuous repetitive chime, MASTER WARN.
    Warning,
    /// Level 2: amber, single chime, MASTER CAUT.
    Caution,
    /// Level 1: amber, no aural, no master light.
    Advisory,
    /// Green/white memo.
    Memo,
}

impl Level {
    pub fn colour(self) -> &'static str {
        match self {
            Level::Warning => "red",
            Level::Caution | Level::Advisory => "amber",
            Level::Memo => "green",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Aural {
    ContinuousRepetitiveChime,
    SingleChime,
    Cavalry,
    /// A specific synthetic voice / sound, by the name the sound system uses.
    Named(&'static str),
    None,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MasterLight {
    Warning,
    Caution,
    None,
}

/// Flight phases (the A380 FWS's), for inhibits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    ElecPower,
    FirstEngineStarted,
    FirstEngineTakeoffPower,
    Above80Kt,
    LiftOff,
    Above1500Ft,
    Below800Ft,
    Touchdown,
    Below80Kt,
    SecondEngineShutdown,
}

/// One line of a procedure.
#[derive(Clone, Debug)]
pub struct ProcLine {
    /// As shown, e.g. "ENG 2 MASTER" with `action_text` "OFF".
    pub text: String,
    pub action_text: String,
    /// cyan for actions, white/green for information, amber for limitations.
    pub colour: &'static str,
    /// The line applies only while this holds (e.g. "IF OIL PR < MIN").
    pub applies_if: Cond,
    /// The line is done when this holds — a condition over the variable the
    /// real cockpit control writes (e.g. `var("ENGINE_MASTER:2").off()`).
    /// `None` for information lines that are never "done".
    pub done_when: Option<Cond>,
    /// Shown only after this many seconds since the alert appeared (e.g. an
    /// "AFTER 30 S" line).
    pub after_s: f64,
}

/// One ECAM alert.
#[derive(Clone, Debug)]
pub struct EcamAlert {
    /// Unique key, e.g. "ENG_2_OIL_LO_PR".
    pub key: String,
    pub ata: u16,
    /// The title as shown, e.g. "ENG 2 OIL LO PR".
    pub title: String,
    pub level: Level,
    pub aural: Aural,
    pub master: MasterLight,
    pub trigger: Cond,
    /// The trigger must hold this long before the alert shows.
    pub confirm_s: f64,
    pub inhibited_in: Vec<Phase>,
    pub procedure: Vec<ProcLine>,
    /// STATUS page lines and INOP SYS entries.
    pub status: Vec<String>,
    pub inop: Vec<String>,
    /// The failures that can raise it (for the Components/Failures pages).
    pub failures: Vec<u64>,
}

/// Builder for a procedure line: `line("ENG 2 MASTER", "OFF").done(var(...).off())`.
pub fn line(text: &str, action: &str) -> ProcLine {
    ProcLine { text: text.to_string(), action_text: action.to_string(), colour: "cyan", applies_if: Cond::Always, done_when: None, after_s: 0.0 }
}
impl ProcLine {
    pub fn done(mut self, c: Cond) -> Self {
        self.done_when = Some(c);
        self
    }
    pub fn only_if(mut self, c: Cond) -> Self {
        self.applies_if = c;
        self
    }
    pub fn after(mut self, s: f64) -> Self {
        self.after_s = s;
        self
    }
    pub fn colour(mut self, c: &'static str) -> Self {
        self.colour = c;
        self
    }
}

impl EcamAlert {
    /// A new alert with the A380's aural and master light for its level.
    pub fn new(key: &str, ata: u16, title: &str, level: Level, trigger: Cond) -> Self {
        let (aural, master) = match level {
            Level::Warning => (Aural::ContinuousRepetitiveChime, MasterLight::Warning),
            Level::Caution => (Aural::SingleChime, MasterLight::Caution),
            Level::Advisory | Level::Memo => (Aural::None, MasterLight::None),
        };
        Self {
            key: key.to_string(),
            ata,
            title: title.to_string(),
            level,
            aural,
            master,
            trigger,
            confirm_s: 0.0,
            inhibited_in: Vec::new(),
            procedure: Vec::new(),
            status: Vec::new(),
            inop: Vec::new(),
            failures: Vec::new(),
        }
    }
    pub fn confirm(mut self, s: f64) -> Self {
        self.confirm_s = s;
        self
    }
    pub fn inhibit(mut self, phases: &[Phase]) -> Self {
        self.inhibited_in.extend_from_slice(phases);
        self
    }
    pub fn step(mut self, l: ProcLine) -> Self {
        self.procedure.push(l);
        self
    }
    pub fn status_line(mut self, s: &str) -> Self {
        self.status.push(s.to_string());
        self
    }
    pub fn inop_sys(mut self, s: &str) -> Self {
        self.inop.push(s.to_string());
        self
    }
    pub fn raised_by(mut self, ids: &[u64]) -> Self {
        self.failures.extend_from_slice(ids);
        self
    }
    pub fn aural(mut self, a: Aural) -> Self {
        self.aural = a;
        self
    }
}

// ---------------------------------------------------------------------------
// The registry every area fills.
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct Registry {
    pub failures: Vec<FailureDef>,
    pub components: Vec<ComponentDef>,
    pub alerts: Vec<EcamAlert>,
    pub errors: Vec<String>,
}

impl Registry {
    pub fn failure(&mut self, f: FailureDef) -> u64 {
        if self.failures.iter().any(|x| x.id == f.id) {
            self.errors.push(format!("duplicate failure id {} ({})", f.id, f.name));
        }
        if f.id / 1_000 % 1_000 != f.ata as u64 {
            self.errors.push(format!("failure {} id does not carry its ATA {}", f.id, f.ata));
        }
        let id = f.id;
        self.failures.push(f);
        id
    }

    pub fn component(&mut self, c: ComponentDef) {
        if self.components.iter().any(|x| x.id == c.id) {
            self.errors.push(format!("duplicate component {}", c.id));
        }
        self.components.push(c);
    }

    pub fn alert(&mut self, a: EcamAlert) {
        if self.alerts.iter().any(|x| x.key == a.key) {
            self.errors.push(format!("duplicate ECAM alert {}", a.key));
        }
        self.alerts.push(a);
    }

    /// Cross-checks once everything is registered: every failure's component
    /// exists, every alert's failures exist.
    pub fn validate(&self) -> Vec<String> {
        let mut e = self.errors.clone();
        for f in &self.failures {
            if !self.components.iter().any(|c| c.id == f.component) {
                e.push(format!("failure {} names unknown component {}", f.id, f.component));
            }
        }
        for a in &self.alerts {
            for id in &a.failures {
                if !self.failures.iter().any(|f| f.id == *id) {
                    e.push(format!("alert {} names unknown failure {}", a.key, id));
                }
            }
        }
        e
    }
}

// ---------------------------------------------------------------------------
// Runtime: which alerts show, which lines are done.
// ---------------------------------------------------------------------------

/// One alert's live state for the displays.
#[derive(Clone, Debug, PartialEq)]
pub struct ActiveAlert {
    pub key: String,
    pub level: Level,
    pub title: String,
    /// Each applicable, visible line with whether it is done.
    pub lines: Vec<(String, String, &'static str, bool)>,
    pub since_s: f64,
}

/// Evaluates registered alerts against a variable source each tick.
#[derive(Default)]
pub struct Ecam {
    /// How long each trigger has held, s.
    held_s: HashMap<String, f64>,
    /// How long each alert has been shown, s.
    shown_s: HashMap<String, f64>,
}

impl Ecam {
    pub fn update(&mut self, alerts: &[EcamAlert], phase: Option<Phase>, read: &dyn Fn(&str) -> f64, dt_s: f64) -> Vec<ActiveAlert> {
        let mut active = Vec::new();
        for a in alerts {
            let triggered = a.trigger.eval(read);
            let held = self.held_s.entry(a.key.clone()).or_insert(0.0);
            *held = if triggered { *held + dt_s.max(0.0) } else { 0.0 };
            let inhibited = phase.map_or(false, |p| a.inhibited_in.contains(&p));
            let shows = triggered && *held >= a.confirm_s && !inhibited;
            let shown = self.shown_s.entry(a.key.clone()).or_insert(0.0);
            if !shows {
                *shown = 0.0;
                continue;
            }
            *shown += dt_s.max(0.0);
            let since = *shown;
            let lines = a
                .procedure
                .iter()
                .filter(|l| l.applies_if.eval(read) && since >= l.after_s)
                .map(|l| (l.text.clone(), l.action_text.clone(), l.colour, l.done_when.as_ref().map_or(false, |c| c.eval(read))))
                .collect();
            active.push(ActiveAlert { key: a.key.clone(), level: a.level, title: a.title.clone(), lines, since_s: since });
        }
        active.sort_by(|x, y| x.level.cmp(&y.level));
        active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn oil_alert() -> EcamAlert {
        EcamAlert::new("ENG_2_OIL_LO_PR", 79, "ENG 2 OIL LO PR", Level::Warning, var("ENGINE_OIL_PRESSURE_PSI:2").lt(25.0))
            .confirm(2.0)
            .inhibit(&[Phase::LiftOff])
            .step(line("THR LEVER 2", "IDLE").done(var("AUTOTHRUST_TLA:2").le(0.0)))
            .step(line("ENG 2 MASTER", "OFF").done(var("ENGINE_MASTER:2").off()).after(30.0))
    }

    #[test]
    fn an_alert_shows_after_its_confirmation_delay_and_lines_complete_from_the_real_controls() {
        let mut vars: HashMap<&str, f64> = HashMap::from([("ENGINE_OIL_PRESSURE_PSI:2", 10.0), ("AUTOTHRUST_TLA:2", 25.0), ("ENGINE_MASTER:2", 1.0)]);
        let alerts = vec![oil_alert()];
        let mut ecam = Ecam::default();
        fn run(ecam: &mut Ecam, alerts: &[EcamAlert], vars: &HashMap<&str, f64>, dt: f64) -> Vec<ActiveAlert> {
            ecam.update(alerts, None, &|n: &str| *vars.get(n).unwrap_or(&0.0), dt)
        }
        assert!(run(&mut ecam, &alerts, &vars, 1.0).is_empty());
        let shown = run(&mut ecam, &alerts, &vars, 1.5);
        assert_eq!(shown.len(), 1);
        assert_eq!(shown[0].lines.len(), 1, "the 30 s line is not shown yet");
        assert!(!shown[0].lines[0].3);
        vars.insert("AUTOTHRUST_TLA:2", 0.0);
        let shown = run(&mut ecam, &alerts, &vars, 30.0);
        assert!(shown[0].lines[0].3, "moving the real lever completes the line");
        assert_eq!(shown[0].lines.len(), 2);
        vars.insert("ENGINE_OIL_PRESSURE_PSI:2", 40.0);
        assert!(run(&mut ecam, &alerts, &vars, 0.1).is_empty());
    }

    #[test]
    fn an_inhibited_phase_hides_it() {
        let vars: HashMap<&str, f64> = HashMap::from([("ENGINE_OIL_PRESSURE_PSI:2", 10.0)]);
        let mut ecam = Ecam::default();
        let read = |n: &str| *vars.get(n).unwrap_or(&0.0);
        assert!(ecam.update(&[oil_alert()], Some(Phase::LiftOff), &read, 5.0).is_empty());
    }

    #[test]
    fn the_registry_catches_collisions_and_dangling_references() {
        let mut r = Registry::default();
        let id = failure_id(Area::Hydraulics, 29, 1);
        assert_eq!(id, 3_029_001);
        let f = FailureDef { id, area: Area::Hydraulics, ata: 29, name: "Green line 1 leak".into(), component: "29_hyd.green_line_1".into(), model_field: "network::Line.leak_area".into(), magnitude: "0..20 mm2".into(), effect: "fluid loss".into() };
        r.failure(f.clone());
        r.failure(f);
        r.alert(oil_alert().raised_by(&[42]));
        let errors = r.validate();
        assert!(errors.iter().any(|e| e.contains("duplicate failure")));
        assert!(errors.iter().any(|e| e.contains("unknown component")));
        assert!(errors.iter().any(|e| e.contains("unknown failure 42")));
    }
}

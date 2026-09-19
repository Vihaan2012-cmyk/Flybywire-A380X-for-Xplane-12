//! FlyByWire's MSFS "aspects", the bridging actions their glue runs around
//! every systems tick, reproduced for X-Plane.
//!
//! The generic part mirrors `systems_wasm/src/aspects.rs` action for action:
//!
//! - `copy` / `map` / `map_many` / `reduce` (aspects.rs:100-162, 376-509):
//!   read inputs, write one output, at PreTick or PostTick.
//! - `variables_to_object` (aspects.rs:165-172, 534-566): read variables
//!   at PostTick and, when the object says so, write simulator variables
//!   (MSFS's `set_data_on_sim_object`; here the simulator variables are this
//!   plugin's own simvar slots).
//! - `event_to_variable` (aspects.rs:178-202, 581-843): a key event sets a
//!   variable through one of FlyByWire's mappings, with the optional leading
//!   debounce, `afterwards_reset_to` and the SmoothPress ramp run in pre_tick.
//!   `mask` is recorded but has no meaning in X-Plane (nothing else receives
//!   the event).
//! - `variable_to_event` (aspects.rs:206-238, 845-941): after the tick, send
//!   a variable as an event, every tick or only when it changed. Events go to
//!   an outgoing queue the plugin turns into X-Plane effects.
//! - `on_change` / `on_change_with_starting_values` (aspects.rs:241-266,
//!   943-991): run a function when any observed value changed, with the
//!   previous and current values; starting values are read at registration.
//!
//! Ordering is FlyByWire's (systems_wasm lib.rs:250-262, 290-332; aspects.rs
//! 320-347): each aspect's event handlers pre_tick (SmoothPress), then its
//! PreTick actions, all before the systems tick; after it, each aspect's
//! PostTick actions, then its handlers' post_tick (debounce resets). Actions
//! run in declaration order, aspects in registration order.
//!
//! One deviation: FlyByWire's leading debounce times with `Instant::now()`
//! (wall clock, aspects.rs:642-656); here the clock is the simulation time
//! handed to `pre_tick`, which is the same while the sim runs and freezes
//! with it when paused (MSFS does not tick aspects when paused either).

#![allow(dead_code)]

use std::time::Duration;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

/// Variable access the aspects need, for any of the plugin's registries.
pub trait VarIo {
    fn io_id(&mut self, variable: &Variable) -> VariableIdentifier;
    fn io_read(&mut self, id: &VariableIdentifier) -> f64;
    fn io_write(&mut self, id: &VariableIdentifier, value: f64);
}

impl<T: VariableRegistry + SimulatorReaderWriter> VarIo for T {
    fn io_id(&mut self, variable: &Variable) -> VariableIdentifier {
        match variable {
            // MSFS simvars and FlyByWire's aspect variables are both looked
            // up by bare name: the registry keeps simvars (with spaces) plain
            // and prefixes the rest, as FlyByWire's registry resolves an
            // aspect variable asked for by `get` (systems_wasm lib.rs:633-640).
            Variable::Aircraft(name, index) => self.get(Variable::indexed(name, *index)),
            Variable::Named(name) | Variable::Aspect(name) => self.get(name.clone()),
            Variable::Unprefixed(name) => self.get_unprefixed(name.clone()),
        }
    }

    fn io_read(&mut self, id: &VariableIdentifier) -> f64 {
        self.read(id)
    }

    fn io_write(&mut self, id: &VariableIdentifier, value: f64) {
        self.write(id, value)
    }
}

/// FlyByWire's `Variable` (systems_wasm lib.rs:377-452).
#[derive(Clone, Debug, PartialEq)]
pub enum Variable {
    /// An MSFS simulation variable, `NAME:index` (index 0 means no suffix).
    Aircraft(String, usize),
    /// An L:var with FlyByWire's `A32NX_` prefix.
    Named(String),
    /// An L:var without the prefix (`provides_named_variable`).
    Unprefixed(String),
    /// An aspect variable, visible to the systems by its bare name.
    Aspect(String),
}

impl Variable {
    pub fn aircraft(name: &str, index: usize) -> Self {
        Self::Aircraft(name.into(), index)
    }
    pub fn named(name: &str) -> Self {
        Self::Named(name.into())
    }
    pub fn unprefixed(name: &str) -> Self {
        Self::Unprefixed(name.into())
    }
    pub fn aspect(name: &str) -> Self {
        Self::Aspect(name.into())
    }

    fn indexed(name: &str, index: usize) -> String {
        if index > 0 {
            format!("{name}:{index}")
        } else {
            name.into()
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ExecuteOn {
    PreTick,
    PostTick,
}

/// How an event becomes a value (aspects.rs:704-732). Event data is MSFS's
/// DWORD.
#[derive(Clone, Copy)]
pub enum EventToVariableMapping {
    Value(f64),
    EventDataRaw,
    EventData32kPosition,
    EventData32kPositionInverted,
    EventDataToValue(fn(u32) -> f64),
    CurrentValueToValue(fn(f64) -> f64),
    EventDataAndCurrentValueToValue(fn(u32, f64) -> f64),
    /// Press factor, release factor, per second.
    SmoothPress(f64, f64),
}

#[derive(Clone, Copy, Default)]
pub struct EventToVariableOptions {
    pub mask: bool,
    pub leading_debounce: Option<Duration>,
    pub reset_to: Option<f64>,
}

impl EventToVariableOptions {
    pub fn mask(mut self) -> Self {
        self.mask = true;
        self
    }
    pub fn leading_debounce(mut self, duration: Duration) -> Self {
        self.leading_debounce = Some(duration);
        self
    }
    pub fn afterwards_reset_to(mut self, value: f64) -> Self {
        self.reset_to = Some(value);
        self
    }
}

#[derive(Clone, Copy)]
pub enum VariableToEventMapping {
    EventDataRaw,
    EventData32kPosition,
}

#[derive(Clone, Copy)]
pub enum VariableToEventWriteOn {
    EveryTick,
    Change,
}

/// systems_wasm lib.rs:655-679.
pub fn sim_connect_32k_pos_to_f64(v: u32) -> f64 {
    (((v as i32) as f64 + 16384.) / 32768.).clamp(0., 1.)
}
pub fn sim_connect_32k_pos_inv_to_f64(v: u32) -> f64 {
    ((-((v as i32) as f64) + 16384.) / 32768.).clamp(0., 1.)
}
pub fn f64_to_sim_connect_32k_pos(v: f64) -> u32 {
    ((v * 32768.) - 16384.) as i32 as u32
}

/// What an `on_change` function may do: read and write variables by name,
/// and send key events (MSFS's `trigger_key_event`).
pub struct Effects<'a> {
    vars: &'a mut dyn VarIo,
    events: &'a mut Vec<(String, u32)>,
}

impl Effects<'_> {
    pub fn read(&mut self, variable: &Variable) -> f64 {
        let id = self.vars.io_id(variable);
        self.vars.io_read(&id)
    }
    pub fn write(&mut self, variable: &Variable, value: f64) {
        let id = self.vars.io_id(variable);
        self.vars.io_write(&id, value)
    }
    pub fn key_event(&mut self, name: &str, data: u32) {
        self.events.push((name.to_owned(), data));
    }
}

pub type OnChangeFn = Box<dyn FnMut(&[f64], &[f64], &mut Effects)>;
/// Values in, simulator variable writes out, or `None` for ObjectWrite::Ignore.
pub type ToObjectFn = Box<dyn FnMut(&[f64]) -> Option<Vec<(Variable, f64)>>>;

enum Action {
    Map { input: VariableIdentifier, func: fn(f64) -> f64, output: VariableIdentifier },
    MapMany { inputs: Vec<VariableIdentifier>, func: fn(&[f64]) -> f64, output: VariableIdentifier },
    Reduce { inputs: Vec<VariableIdentifier>, init: f64, func: fn(f64, f64) -> f64, output: VariableIdentifier },
    ToObject { inputs: Vec<VariableIdentifier>, func: ToObjectFn },
    ToEvent {
        input: VariableIdentifier,
        mapping: VariableToEventMapping,
        write_on: VariableToEventWriteOn,
        event: String,
        last_written: Option<f64>,
    },
    OnChange { observed: Vec<VariableIdentifier>, previous: Vec<f64>, func: OnChangeFn },
}

fn read_many(vars: &mut dyn VarIo, ids: &[VariableIdentifier]) -> Vec<f64> {
    ids.iter().map(|id| vars.io_read(id)).collect()
}

impl Action {
    fn execute(&mut self, vars: &mut dyn VarIo, events: &mut Vec<(String, u32)>) {
        match self {
            Action::Map { input, func, output } => {
                let v = vars.io_read(input);
                vars.io_write(output, func(v));
            }
            Action::MapMany { inputs, func, output } => {
                let values = read_many(vars, inputs);
                vars.io_write(output, func(&values));
            }
            Action::Reduce { inputs, init, func, output } => {
                let result = read_many(vars, inputs).into_iter().fold(*init, *func);
                vars.io_write(output, result);
            }
            Action::ToObject { inputs, func } => {
                let values = read_many(vars, inputs);
                if let Some(writes) = func(&values) {
                    for (variable, value) in writes {
                        let id = vars.io_id(&variable);
                        vars.io_write(&id, value);
                    }
                }
            }
            Action::ToEvent { input, mapping, write_on, event, last_written } => {
                let value = vars.io_read(input);
                #[allow(clippy::float_cmp)]
                let should_write = match write_on {
                    VariableToEventWriteOn::EveryTick => true,
                    VariableToEventWriteOn::Change => last_written.map_or(true, |last| value != last),
                };
                if should_write {
                    let data = match mapping {
                        VariableToEventMapping::EventDataRaw => value as u32,
                        VariableToEventMapping::EventData32kPosition => f64_to_sim_connect_32k_pos(value),
                    };
                    events.push((event.clone(), data));
                    *last_written = Some(value);
                }
            }
            Action::OnChange { observed, previous, func } => {
                let current = read_many(vars, observed);
                #[allow(clippy::float_cmp)]
                let changed = previous.iter().zip(&current).any(|(p, c)| p != c);
                if changed {
                    let mut effects = Effects { vars, events };
                    func(previous, &current, &mut effects);
                }
                *previous = current;
            }
        }
    }
}

enum Debounce {
    None,
    Leading { duration: f64, handled_at: Option<f64> },
}

struct EventToVariable {
    event: String,
    mask: bool,
    handled_before_tick: bool,
    target: VariableIdentifier,
    mapping: EventToVariableMapping,
    debounce: Debounce,
    reset_to: Option<f64>,
}

impl EventToVariable {
    fn should_handle(&self, now: f64) -> bool {
        match self.debounce {
            Debounce::None => true,
            Debounce::Leading { duration, handled_at } => handled_at.map_or(true, |t| now - t > duration),
        }
    }

    fn handle(&mut self, vars: &mut dyn VarIo, data: u32, now: f64) {
        if !self.should_handle(now) {
            return;
        }
        let value = match self.mapping {
            EventToVariableMapping::Value(v) => v,
            EventToVariableMapping::EventDataRaw => data as f64,
            EventToVariableMapping::EventData32kPosition => sim_connect_32k_pos_to_f64(data),
            EventToVariableMapping::EventData32kPositionInverted => sim_connect_32k_pos_inv_to_f64(data),
            EventToVariableMapping::EventDataToValue(f) => f(data),
            EventToVariableMapping::CurrentValueToValue(f) => f(vars.io_read(&self.target)),
            EventToVariableMapping::EventDataAndCurrentValueToValue(f) => f(data, vars.io_read(&self.target)),
            EventToVariableMapping::SmoothPress(..) => vars.io_read(&self.target),
        };
        vars.io_write(&self.target, value);
        if let Debounce::Leading { handled_at, .. } = &mut self.debounce {
            *handled_at = Some(now);
        }
        self.handled_before_tick = true;
    }

    fn pre_tick(&mut self, vars: &mut dyn VarIo, delta: f64) {
        if let EventToVariableMapping::SmoothPress(press, release) = self.mapping {
            let mut value = vars.io_read(&self.target);
            if self.handled_before_tick {
                value += delta * press;
            } else {
                value -= delta * release;
            }
            vars.io_write(&self.target, value.clamp(0., 1.));
        }
    }

    fn post_tick(&mut self, vars: &mut dyn VarIo, now: f64) {
        match &mut self.debounce {
            Debounce::None => {
                if let Some(v) = self.reset_to {
                    vars.io_write(&self.target, v);
                }
            }
            Debounce::Leading { duration, handled_at } => {
                if let Some(t) = *handled_at {
                    if now - t > *duration {
                        if let Some(v) = self.reset_to {
                            vars.io_write(&self.target, v);
                        }
                        *handled_at = None;
                    }
                }
            }
        }
        self.handled_before_tick = false;
    }
}

/// One aspect: its event handlers and actions, in declaration order.
#[derive(Default)]
pub struct Aspect {
    handlers: Vec<EventToVariable>,
    actions: Vec<(Action, ExecuteOn)>,
}

/// FlyByWire's `MsfsAspectBuilder`, registering against the plugin's
/// variables.
pub struct AspectBuilder<'a> {
    vars: &'a mut dyn VarIo,
    aspect: Aspect,
}

impl<'a> AspectBuilder<'a> {
    pub fn new(vars: &'a mut dyn VarIo) -> Self {
        Self { vars, aspect: Aspect::default() }
    }

    pub fn build(self) -> Aspect {
        self.aspect
    }

    fn id(&mut self, v: &Variable) -> VariableIdentifier {
        self.vars.io_id(v)
    }

    fn ids(&mut self, vs: &[Variable]) -> Vec<VariableIdentifier> {
        vs.iter().map(|v| self.vars.io_id(v)).collect()
    }

    pub fn init_variable(&mut self, variable: Variable, value: f64) {
        let id = self.id(&variable);
        self.vars.io_write(&id, value);
    }

    /// Always PreTick (aspects.rs:100-110).
    pub fn copy(&mut self, input: Variable, output: Variable) {
        let input = self.id(&input);
        let output = self.id(&output);
        self.aspect.actions.push((Action::Map { input, func: |v| v, output }, ExecuteOn::PreTick));
    }

    pub fn map(&mut self, on: ExecuteOn, input: Variable, func: fn(f64) -> f64, output: Variable) {
        let input = self.id(&input);
        let output = self.id(&output);
        self.aspect.actions.push((Action::Map { input, func, output }, on));
    }

    pub fn map_many(&mut self, on: ExecuteOn, inputs: Vec<Variable>, func: fn(&[f64]) -> f64, output: Variable) {
        let inputs = self.ids(&inputs);
        let output = self.id(&output);
        self.aspect.actions.push((Action::MapMany { inputs, func, output }, on));
    }

    pub fn reduce(&mut self, on: ExecuteOn, inputs: Vec<Variable>, init: f64, func: fn(f64, f64) -> f64, output: Variable) {
        let inputs = self.ids(&inputs);
        let output = self.id(&output);
        self.aspect.actions.push((Action::Reduce { inputs, init, func, output }, on));
    }

    /// Always PostTick (aspects.rs:165-172).
    pub fn variables_to_object(&mut self, inputs: Vec<Variable>, func: ToObjectFn) {
        let inputs = self.ids(&inputs);
        self.aspect.actions.push((Action::ToObject { inputs, func }, ExecuteOn::PostTick));
    }

    pub fn event_to_variable(
        &mut self,
        event: &str,
        mapping: EventToVariableMapping,
        target: Variable,
        options: fn(EventToVariableOptions) -> EventToVariableOptions,
    ) {
        let target = self.id(&target);
        let options = options(EventToVariableOptions::default());
        self.aspect.handlers.push(EventToVariable {
            event: event.to_owned(),
            mask: options.mask,
            handled_before_tick: false,
            target,
            mapping,
            debounce: match options.leading_debounce {
                Some(d) => Debounce::Leading { duration: d.as_secs_f64(), handled_at: None },
                None => Debounce::None,
            },
            reset_to: options.reset_to,
        });
    }

    /// Always PostTick (aspects.rs:206-221).
    pub fn variable_to_event(&mut self, input: Variable, mapping: VariableToEventMapping, write_on: VariableToEventWriteOn, event: &str) {
        let input = self.id(&input);
        self.aspect.actions.push((
            Action::ToEvent { input, mapping, write_on, event: event.to_owned(), last_written: None },
            ExecuteOn::PostTick,
        ));
    }

    pub fn on_change(&mut self, on: ExecuteOn, observed: Vec<Variable>, func: OnChangeFn) {
        let observed = self.ids(&observed);
        let previous = read_many(self.vars, &observed);
        self.aspect.actions.push((Action::OnChange { observed, previous, func }, on));
    }

    pub fn on_change_with_starting_values(&mut self, on: ExecuteOn, observed: Vec<Variable>, starting: Vec<f64>, func: OnChangeFn) {
        let observed = self.ids(&observed);
        self.aspect.actions.push((Action::OnChange { observed, previous: starting, func }, on));
    }
}

/// Every aspect, run in FlyByWire's order.
#[derive(Default)]
pub struct Aspects {
    aspects: Vec<Aspect>,
    now: f64,
    /// Key events the aspects sent, oldest first, for the plugin to act on.
    outgoing: Vec<(String, u32)>,
}

impl Aspects {
    pub fn with_aspect(&mut self, vars: &mut dyn VarIo, configure: impl FnOnce(&mut AspectBuilder)) {
        let mut builder = AspectBuilder::new(vars);
        configure(&mut builder);
        self.aspects.push(builder.build());
    }

    /// A key event arriving between ticks: the first handler for it takes it
    /// (MsfsHandler::handle_message, lib.rs:236-246).
    pub fn handle_event(&mut self, vars: &mut dyn VarIo, event: &str, data: u32) -> bool {
        for aspect in &mut self.aspects {
            if let Some(h) = aspect.handlers.iter_mut().find(|h| h.event == event) {
                h.handle(vars, data, self.now);
                return true;
            }
        }
        false
    }

    pub fn pre_tick(&mut self, vars: &mut dyn VarIo, delta: f64) {
        self.now += delta;
        for aspect in &mut self.aspects {
            for h in &mut aspect.handlers {
                h.pre_tick(vars, delta);
            }
            for (action, on) in &mut aspect.actions {
                if *on == ExecuteOn::PreTick {
                    action.execute(vars, &mut self.outgoing);
                }
            }
        }
    }

    pub fn post_tick(&mut self, vars: &mut dyn VarIo) {
        for aspect in &mut self.aspects {
            for (action, on) in &mut aspect.actions {
                if *on == ExecuteOn::PostTick {
                    action.execute(vars, &mut self.outgoing);
                }
            }
            for h in &mut aspect.handlers {
                h.post_tick(vars, self.now);
            }
        }
    }

    pub fn take_events(&mut self) -> Vec<(String, u32)> {
        std::mem::take(&mut self.outgoing)
    }
}

/// What the MSFS key events FlyByWire's A380 aspects send do to MSFS's own
/// simulation variables, which this plugin holds. Returns whether the event
/// was known.
pub fn apply_msfs_key_event(vars: &mut dyn VarIo, event: &str, data: u32) -> bool {
    let set = |vars: &mut dyn VarIo, v: Variable, value: f64| {
        let id = vars.io_id(&v);
        vars.io_write(&id, value);
    };
    if let Some(n) = event.strip_prefix("ANTI_ICE_SET_ENG").and_then(|n| n.parse::<usize>().ok()) {
        set(vars, Variable::aircraft("ENG ANTI ICE", n), (data != 0) as i32 as f64);
        return true;
    }
    if event == "TOGGLE_STRUCTURAL_DEICE" {
        let v = Variable::aircraft("STRUCTURAL DEICE SWITCH", 0);
        let id = vars.io_id(&v);
        let now = vars.io_read(&id);
        vars.io_write(&id, if now != 0. { 0. } else { 1. });
        return true;
    }
    false
}

/// FlyByWire's A380 aspects that are neither flight controls, gear, brakes,
/// flaps, autobrake, steering nor sensor feeds (a380_systems_wasm lib.rs:
/// 559-620 and systems_wasm's builder helpers), in their registration order.
pub fn a380(vars: &mut dyn VarIo) -> Aspects {
    let mut aspects = Aspects::default();

    // with_engine_anti_ice(4) (anti_ice.rs:10-49).
    aspects.with_aspect(vars, |b| {
        for n in 1..=4usize {
            b.on_change(
                ExecuteOn::PostTick,
                vec![
                    Variable::named(&format!("BUTTON_OVHD_ANTI_ICE_ENG_{n}_POSITION")),
                    Variable::aircraft("ENG ANTI ICE", n),
                ],
                Box::new(move |prev, new, fx| {
                    let was_on = prev[0] != 0.;
                    let is_on = new[0] != 0.;
                    let anti_ice_on = new[1] != 0.;
                    if was_on != is_on && anti_ice_on != is_on {
                        fx.key_event(&format!("ANTI_ICE_SET_ENG{n}"), is_on as u32);
                    }
                }),
            );
        }
    });

    // with_wing_anti_ice() (anti_ice.rs:51-78).
    aspects.with_aspect(vars, |b| {
        b.on_change(
            ExecuteOn::PostTick,
            vec![
                Variable::named("PNEU_WING_ANTI_ICE_SYSTEM_ON"),
                Variable::named("PNEU_WING_ANTI_ICE_HAS_FAULT"),
                Variable::aircraft("STRUCTURAL DEICE SWITCH", 0),
            ],
            Box::new(|prev, new, fx| {
                let was_on = prev[0] != 0. && prev[1] == 0.;
                let is_on = new[0] != 0. && new[1] == 0.;
                let deicing = new[2] != 0.;
                if was_on != is_on && deicing != is_on {
                    fx.key_event("TOGGLE_STRUCTURAL_DEICE", 0);
                }
            }),
        );
    });

    // with_fuel_pumps(1..=21) (fuel.rs:4-16).
    aspects.with_aspect(vars, |b| {
        for n in 1..=21 {
            b.copy(Variable::aircraft("FUELSYSTEM PUMP ACTIVE", n), Variable::aspect(&format!("FUEL_PUMP_{n}_ACTIVE")));
        }
    });

    // GSX bypass pin (lib.rs:559-565).
    aspects.with_aspect(vars, |b| {
        b.copy(Variable::unprefixed("FSDT_GSX_BYPASS_PIN"), Variable::aspect("EXTERNAL_BYPASS_PIN_INSERTED"));
    });

    // Generator and external power pushbuttons (lib.rs:566-587). MSFS holds
    // the generator switches; FlyByWire's flight files leave every engine
    // generator switch on (apron.FLT:58, cruise.FLT:53, ...) and the
    // pushbuttons are built on (a380_systems electrical/mod.rs:358-360), so
    // the switches start on here too.
    aspects.with_aspect(vars, |b| {
        for i in 1..=2 {
            b.init_variable(Variable::aircraft("APU GENERATOR SWITCH", i), 1.);
            b.copy(Variable::aircraft("APU GENERATOR SWITCH", i), Variable::aspect(&format!("OVHD_ELEC_APU_GEN_{i}_PB_IS_ON")));
        }
        for i in 1..=4 {
            b.copy(Variable::named(&format!("EXT_PWR_AVAIL:{i}")), Variable::aspect(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_AVAILABLE")));
            b.init_variable(Variable::aircraft("GENERAL ENG MASTER ALTERNATOR", i), 1.);
            b.copy(
                Variable::aircraft("GENERAL ENG MASTER ALTERNATOR", i),
                Variable::aspect(&format!("OVHD_ELEC_ENG_GEN_{i}_PB_IS_ON")),
            );
        }
    });

    // cargo_doors (cargo_doors.rs:8-34).
    aspects.with_aspect(vars, |b| {
        b.map(
            ExecuteOn::PreTick,
            Variable::aircraft("INTERACTIVE POINT OPEN", 16),
            |v| if v > 0. { 1. } else { 0. },
            Variable::aspect("FWD_DOOR_CARGO_OPEN_REQ"),
        );
        b.map_many(
            ExecuteOn::PreTick,
            vec![Variable::aircraft("INTERACTIVE POINT OPEN", 16), Variable::aircraft("INTERACTIVE POINT OPEN", 17)],
            |v| if v[0] > 0. || v[1] > 0. { 1. } else { 0. },
            Variable::aspect("AFT_DOOR_CARGO_OPEN_REQ"),
        );
    });

    // fire (fire.rs:9-52): the systems' ENG_n_ON_FIRE becomes MSFS's
    // ENG ON FIRE:n every tick, which the A380's fire detectors read back
    // (a380_systems fire_and_smoke_protection.rs:459-495).
    aspects.with_aspect(vars, |b| {
        b.variables_to_object(
            (1..=4).map(|n| Variable::named(&format!("ENG_{n}_ON_FIRE"))).collect(),
            Box::new(|v| Some((1..=4).map(|n| (Variable::aircraft("ENG ON FIRE", n), v[n - 1])).collect())),
        );
    });

    // Fire intensity for the visual/damage workstream: `physics/xp_
    // effects.rs`'s `XP_ENGINE_FIRE_INTENSITY:n` is that shared interface
    // (it got there first and sources X-Plane's own continuous ramp,
    // `sim/flightmodel2/engines/is_on_fire`, rather than this plugin
    // fabricating one) -- do not duplicate it here.

    // fire pushbutton -> LP fuel valve (fire_and_smoke_protection.rs:499-
    // 523,636-661: `FirePushButton` for "ENGn" reads/writes `FIRE_BUTTON_
    // ENGn`, released=true once the pilot pulls it, which the same tick
    // already trips the IDG/generator (electrical/mod.rs:427-428) and the
    // hydraulic fire shutoff valve (hydraulic/mod.rs:3404-3429) and arms the
    // squibs (fire_and_smoke_protection.rs:644). engine_commands.rs treats
    // `GENERAL ENG STARTER:n` (its `master`) as the LP valve's own
    // fuel_valve_open (engine_commands.rs:379,394); pulling the fire
    // pushbutton forces that switch off the same way the real handle closes
    // the LP valve independent of the master lever, and only ever forces it
    // off -- releasing the pushbutton returns fuel control to the pilot's
    // own master switch, matching `bottle_stays_discharged_after_fire_pb_is_
    // reset` (the extinguishing agent used stays spent; only the valve
    // reopens).
    for n in 1..=4 {
        aspects.with_aspect(vars, |b| {
            b.map_many(
                ExecuteOn::PostTick,
                vec![Variable::named(&format!("FIRE_BUTTON_ENG{n}")), Variable::aircraft("GENERAL ENG STARTER", n)],
                |v| if v[0] != 0. { 0. } else { v[1] },
                Variable::aircraft("GENERAL ENG STARTER", n),
            );
        });
    }

    // payload (payload.rs:9-194).
    aspects.with_aspect(vars, |b| {
        for n in 1..=18 {
            b.copy(Variable::aircraft("PAYLOAD STATION WEIGHT", n), Variable::aspect(&format!("PAYLOAD_STATION_{n}_REQ")));
        }
        b.variables_to_object(
            (1..=18).map(|n| Variable::aspect(&format!("PAYLOAD_STATION_{n}_REQ"))).collect(),
            Box::new(|v| Some((1..=18).map(|n| (Variable::aircraft("PAYLOAD STATION WEIGHT", n), v[n - 1])).collect())),
        );
    });

    aspects
}

/// A plain registry for tests: simvars (with spaces) unprefixed, the rest
/// with FlyByWire's prefix, as the plugin's own `Vars`.
#[cfg(test)]
pub mod test_vars {
    use super::*;
    use std::collections::HashMap;

    #[derive(Default)]
    pub struct TestVars {
        pub index: HashMap<String, usize>,
        pub values: Vec<f64>,
    }

    impl TestVars {
        fn add(&mut self, name: String) -> VariableIdentifier {
            let i = *self.index.entry(name).or_insert_with(|| {
                self.values.push(0.);
                self.values.len() - 1
            });
            let mut id = VariableIdentifier::new::<usize>(0);
            for _ in 0..i {
                id = id.next();
            }
            id
        }

        pub fn value(&self, name: &str) -> f64 {
            self.index.get(name).map_or(0., |&i| self.values[i])
        }

        pub fn set(&mut self, name: &str, v: f64) {
            let id = self.add(name.to_owned());
            self.values[id.identifier_index()] = v;
        }
    }

    impl VariableRegistry for TestVars {
        fn get(&mut self, name: String) -> VariableIdentifier {
            if name.contains(' ') {
                self.add(name)
            } else {
                self.add(format!("A32NX_{name}"))
            }
        }
        fn get_unprefixed(&mut self, name: String) -> VariableIdentifier {
            self.add(name)
        }
    }

    impl SimulatorReaderWriter for TestVars {
        fn read(&mut self, id: &VariableIdentifier) -> f64 {
            self.values.get(id.identifier_index()).copied().unwrap_or(0.)
        }
        fn write(&mut self, id: &VariableIdentifier, v: f64) {
            if let Some(s) = self.values.get_mut(id.identifier_index()) {
                *s = v;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::test_vars::TestVars;
    use super::*;

    #[test]
    fn copy_and_map_run_before_the_tick_reduce_after() {
        let mut v = TestVars::default();
        let mut a = Aspects::default();
        a.with_aspect(&mut v, |b| {
            b.copy(Variable::aircraft("PAYLOAD STATION WEIGHT", 3), Variable::aspect("PAYLOAD_STATION_3_REQ"));
            b.reduce(ExecuteOn::PostTick, vec![Variable::named("A"), Variable::named("B")], f64::MIN, f64::max, Variable::named("MAX"));
        });
        v.set("PAYLOAD STATION WEIGHT:3", 1234.);
        v.set("A32NX_A", 2.);
        v.set("A32NX_B", 5.);
        a.pre_tick(&mut v, 0.1);
        assert_eq!(v.value("A32NX_PAYLOAD_STATION_3_REQ"), 1234.);
        assert_eq!(v.value("A32NX_MAX"), 0.);
        a.post_tick(&mut v);
        assert_eq!(v.value("A32NX_MAX"), 5.);
    }

    #[test]
    fn on_change_fires_only_on_a_change_with_previous_values() {
        let mut v = TestVars::default();
        let mut a = Aspects::default();
        a.with_aspect(&mut v, |b| {
            b.on_change_with_starting_values(
                ExecuteOn::PostTick,
                vec![Variable::named("X")],
                vec![1.],
                Box::new(|prev, new, fx| fx.key_event("CHANGED", (prev[0] * 10. + new[0]) as u32)),
            );
        });
        v.set("A32NX_X", 1.);
        a.post_tick(&mut v);
        assert!(a.take_events().is_empty());
        v.set("A32NX_X", 0.);
        a.post_tick(&mut v);
        assert_eq!(a.take_events(), vec![("CHANGED".to_owned(), 10)]);
        a.post_tick(&mut v);
        assert!(a.take_events().is_empty());
    }

    #[test]
    fn variable_to_event_writes_on_change_only() {
        let mut v = TestVars::default();
        let mut a = Aspects::default();
        a.with_aspect(&mut v, |b| {
            b.variable_to_event(Variable::named("POS"), VariableToEventMapping::EventDataRaw, VariableToEventWriteOn::Change, "EV");
        });
        v.set("A32NX_POS", 3.);
        a.post_tick(&mut v);
        a.post_tick(&mut v);
        v.set("A32NX_POS", 4.);
        a.post_tick(&mut v);
        assert_eq!(a.take_events(), vec![("EV".to_owned(), 3), ("EV".to_owned(), 4)]);
    }

    #[test]
    fn leading_debounce_ignores_repeats_and_resets_afterwards() {
        let mut v = TestVars::default();
        let mut a = Aspects::default();
        a.with_aspect(&mut v, |b| {
            b.event_to_variable("PRESS", EventToVariableMapping::CurrentValueToValue(|x| x + 1.), Variable::named("COUNT"), |o| {
                o.leading_debounce(Duration::from_millis(250)).afterwards_reset_to(0.)
            });
        });
        a.pre_tick(&mut v, 0.05);
        assert!(a.handle_event(&mut v, "PRESS", 0));
        assert!(a.handle_event(&mut v, "PRESS", 0));
        assert_eq!(v.value("A32NX_COUNT"), 1.);
        a.post_tick(&mut v);
        assert_eq!(v.value("A32NX_COUNT"), 1., "still within the debounce");
        for _ in 0..6 {
            a.pre_tick(&mut v, 0.05);
            a.post_tick(&mut v);
        }
        assert_eq!(v.value("A32NX_COUNT"), 0., "reset once the debounce passed");
        assert!(!a.handle_event(&mut v, "OTHER", 0));
    }

    #[test]
    fn no_debounce_resets_after_the_tick() {
        let mut v = TestVars::default();
        let mut a = Aspects::default();
        a.with_aspect(&mut v, |b| {
            b.event_to_variable("PUSH", EventToVariableMapping::Value(1.), Variable::named("PB"), |o| o.afterwards_reset_to(0.));
        });
        a.handle_event(&mut v, "PUSH", 0);
        a.pre_tick(&mut v, 0.05);
        assert_eq!(v.value("A32NX_PB"), 1.);
        a.post_tick(&mut v);
        assert_eq!(v.value("A32NX_PB"), 0.);
    }

    #[test]
    fn smooth_press_ramps_up_while_pressed_and_down_after() {
        let mut v = TestVars::default();
        let mut a = Aspects::default();
        a.with_aspect(&mut v, |b| {
            b.event_to_variable("BRAKE", EventToVariableMapping::SmoothPress(0.6, 0.6), Variable::named("BRK"), |o| o);
        });
        for _ in 0..10 {
            a.handle_event(&mut v, "BRAKE", 0);
            a.pre_tick(&mut v, 0.1);
            a.post_tick(&mut v);
        }
        assert!((v.value("A32NX_BRK") - 0.6).abs() < 1e-9);
        a.pre_tick(&mut v, 0.5);
        assert!((v.value("A32NX_BRK") - 0.3).abs() < 1e-9);
        for _ in 0..20 {
            a.pre_tick(&mut v, 0.1);
        }
        assert_eq!(v.value("A32NX_BRK"), 0.);
    }

    #[test]
    fn position_32k_round_trips() {
        assert_eq!(sim_connect_32k_pos_to_f64((-16384i32) as u32), 0.);
        assert_eq!(sim_connect_32k_pos_to_f64(16384), 1.);
        assert!((sim_connect_32k_pos_to_f64(f64_to_sim_connect_32k_pos(0.25)) - 0.25).abs() < 1e-4);
        assert_eq!(sim_connect_32k_pos_inv_to_f64(16384), 0.);
    }

    #[test]
    fn engine_anti_ice_button_sets_msfs_anti_ice() {
        let mut v = TestVars::default();
        let mut a = a380(&mut v);
        v.set("A32NX_BUTTON_OVHD_ANTI_ICE_ENG_2_POSITION", 1.);
        a.post_tick(&mut v);
        let events = a.take_events();
        assert_eq!(events, vec![("ANTI_ICE_SET_ENG2".to_owned(), 1)]);
        for (e, d) in events {
            assert!(apply_msfs_key_event(&mut v, &e, d));
        }
        assert_eq!(v.value("ENG ANTI ICE:2"), 1.);
    }

    #[test]
    fn wing_anti_ice_toggles_structural_deice() {
        let mut v = TestVars::default();
        let mut a = a380(&mut v);
        v.set("A32NX_PNEU_WING_ANTI_ICE_SYSTEM_ON", 1.);
        a.post_tick(&mut v);
        for (e, d) in a.take_events() {
            apply_msfs_key_event(&mut v, &e, d);
        }
        assert_eq!(v.value("STRUCTURAL DEICE SWITCH"), 1.);
        // A fault stops it again.
        v.set("A32NX_PNEU_WING_ANTI_ICE_HAS_FAULT", 1.);
        a.post_tick(&mut v);
        for (e, d) in a.take_events() {
            apply_msfs_key_event(&mut v, &e, d);
        }
        assert_eq!(v.value("STRUCTURAL DEICE SWITCH"), 0.);
    }

    #[test]
    fn fire_pushbutton_released_closes_the_lp_fuel_valve() {
        let mut v = TestVars::default();
        let mut a = a380(&mut v);
        // The pilot has the engine running (master/LP valve open).
        v.set("GENERAL ENG STARTER:1", 1.);
        a.post_tick(&mut v);
        assert_eq!(v.value("GENERAL ENG STARTER:1"), 1., "untouched while the fire pushbutton is not released");

        // FlyByWire's FirePushButton echoes FIRE_BUTTON_ENG1 back out once
        // released (fire_and_smoke_protection.rs's overhead FirePushButton,
        // via `EngineFireOverheadPanel`); the plugin doesn't own how it gets
        // set, only that once it reads true, the LP valve follows.
        v.set("A32NX_FIRE_BUTTON_ENG1", 1.);
        a.post_tick(&mut v);
        assert_eq!(v.value("GENERAL ENG STARTER:1"), 0., "the fire pushbutton cuts the LP fuel valve");

        // Other engines are untouched.
        v.set("GENERAL ENG STARTER:2", 1.);
        a.post_tick(&mut v);
        assert_eq!(v.value("GENERAL ENG STARTER:2"), 1.);

        // Releasing the guard again (pb un-pulled) returns fuel control to
        // the pilot's own switch -- it does not latch the valve shut.
        v.set("A32NX_FIRE_BUTTON_ENG1", 0.);
        v.set("GENERAL ENG STARTER:1", 1.);
        a.post_tick(&mut v);
        assert_eq!(v.value("GENERAL ENG STARTER:1"), 1.);
    }

    #[test]
    fn a380_copies_reach_the_systems_names() {
        let mut v = TestVars::default();
        let mut a = a380(&mut v);
        v.set("FUELSYSTEM PUMP ACTIVE:21", 1.);
        v.set("FSDT_GSX_BYPASS_PIN", 1.);
        v.set("A32NX_EXT_PWR_AVAIL:3", 1.);
        v.set("INTERACTIVE POINT OPEN:17", 50.);
        v.set("A32NX_ENG_4_ON_FIRE", 1.);
        a.pre_tick(&mut v, 0.05);
        assert_eq!(v.value("A32NX_FUEL_PUMP_21_ACTIVE"), 1.);
        assert_eq!(v.value("A32NX_EXTERNAL_BYPASS_PIN_INSERTED"), 1.);
        assert_eq!(v.value("A32NX_OVHD_ELEC_EXT_PWR_3_PB_IS_AVAILABLE"), 1.);
        assert_eq!(v.value("A32NX_OVHD_ELEC_APU_GEN_1_PB_IS_ON"), 1.);
        assert_eq!(v.value("A32NX_OVHD_ELEC_ENG_GEN_4_PB_IS_ON"), 1.);
        assert_eq!(v.value("A32NX_FWD_DOOR_CARGO_OPEN_REQ"), 0.);
        assert_eq!(v.value("A32NX_AFT_DOOR_CARGO_OPEN_REQ"), 1.);
        a.post_tick(&mut v);
        assert_eq!(v.value("ENG ON FIRE:4"), 1.);
        assert_eq!(v.value("ENG ON FIRE:1"), 0.);
    }
}

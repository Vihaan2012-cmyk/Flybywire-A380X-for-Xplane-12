mod breaker_failures;
mod flight_controls_authority;
mod maintenance_log;
mod mel;
mod persistence;
mod pneumatic_valves;
mod hot_section;
mod power_effects;
mod scheduled;
mod consumer_failures;
mod sim_effects;
mod start_state;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_start_state;
#[cfg(test)]
mod tests_electrical;
#[cfg(test)]
mod tests_sim_effects;
#[cfg(test)]
mod tests_hydraulics;
#[cfg(test)]
mod tests_air_conditioning;
#[cfg(test)]
mod tests_oxygen;
#[cfg(test)]
mod tests_pneumatic;
#[cfg(test)]
mod tests_ice_rain;
#[cfg(test)]
mod tests_fire_gear_apu;
#[cfg(test)]
mod tests_apu_authority;
#[cfg(test)]
mod tests_fire_authority;
#[cfg(test)]
mod tests_gear_brakes_authority;
#[cfg(test)]
mod tests_engines_fuel;
#[cfg(test)]
mod tests_engine_physics;
#[cfg(test)]
mod tests_flight_controls;
#[cfg(test)]
mod tests_avionics;
#[cfg(test)]
mod tests_autoflight;
#[cfg(test)]
mod tests_breakers;
#[cfg(test)]
mod tests_ecam;
#[cfg(test)]
mod tests_flight_emulator;
#[cfg(test)]
mod tests_ecl_mel;
#[cfg(test)]
mod tests_failures;
#[cfg(test)]
mod tests_maintenance;
#[cfg(test)]
mod tests_wiring;
#[cfg(test)]
mod tests_perf;
#[cfg(test)]
mod tests_random;
#[cfg(test)]
mod tests_persistence;
#[cfg(test)]
mod tests_suite;
#[cfg(test)]
mod tests_cpp_host;
#[cfg(test)]
mod tests_flight_state;
mod truth;
mod tyres;

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::time::Instant;

use deep_systems::random_failures::{Config as RandomConfig, RandomFailures};
use deep_systems::scripted_failures::{ArmCondition, Phase, Sample, Scripted};
use deep_systems::wear::WearStore;
use deep_systems::{all_gates, lvar_key, BreakerCommand, DeepSystems, Faults};
use persistence::Persistence;
use systems::{
    shared::ElectricalBuses,
    simulation::{
        InitContext, Read, Reader, SimulationElement, SimulatorReader, SimulatorWriter,
        UpdateContext, VariableIdentifier, Write, Writer,
    },
};

use crate::electrical::{RESET_PANEL_FAILURES, RESET_PANEL_UNITS};
use flight_controls_authority::FlightControlsAuthority;
use sim_effects::SimEffects;
use truth::TruthReader;
use tyres::Tyres;

pub const MAX_ARMED: usize = 32;

const DEEP_TICK_S: f64 = 0.05;

const RESULT_APPLIED: f64 = 1.;
const RESULT_FULL: f64 = 2.;
const RESULT_UNKNOWN: f64 = 3.;

struct Unit {
    current_id: VariableIdentifier,
    cmd_id: VariableIdentifier,
    command: Option<BreakerCommand>,
    consumed: bool,
    open: bool,
    cut: bool,
    cut_index: Option<usize>,
    current_a: f64,
}

struct Gate {
    id: VariableIdentifier,
    units: Vec<usize>,
    open: bool,
}

struct ResetButton {
    id: VariableIdentifier,
    units: Vec<usize>,
    failures: &'static [u64],
    pulled: bool,
    was_pulled: bool,
}

struct Published {
    id: VariableIdentifier,
    value: f64,
    sent: f64,
    changed: bool,
}

struct Arming {
    cmd_id_id: VariableIdentifier,
    cmd_magnitude_id: VariableIdentifier,
    result_id: VariableIdentifier,
    count_id: VariableIdentifier,
    slot_ids: [(VariableIdentifier, VariableIdentifier); MAX_ARMED],
    known: BTreeSet<u64>,
    armed: BTreeMap<u64, f64>,
    command: Option<(f64, f64)>,
    result: f64,
}

impl Arming {
    fn new(context: &mut InitContext) -> Self {
        Self {
            cmd_id_id: context.get_identifier("DEEP_FAILURE_CMD_ID".to_owned()),
            cmd_magnitude_id: context.get_identifier("DEEP_FAILURE_CMD_MAGNITUDE".to_owned()),
            result_id: context.get_identifier("DEEP_FAILURE_CMD_RESULT".to_owned()),
            count_id: context.get_identifier("DEEP_FAILURES_ARMED_COUNT".to_owned()),
            slot_ids: std::array::from_fn(|k| {
                (
                    context.get_identifier(format!("DEEP_FAILURE_ARMED_{k}_ID")),
                    context.get_identifier(format!("DEEP_FAILURE_ARMED_{k}_MAGNITUDE")),
                )
            }),
            known: deep_systems::failure_ids().into_iter().chain(Tyres::failure_ids()).collect(),
            armed: BTreeMap::new(),
            command: None,
            result: 0.,
        }
    }

    fn apply(&mut self) {
        let Some((id, magnitude)) = self.command else { return };
        if id < 0. {
            self.armed.clear();
            self.result = RESULT_APPLIED;
            return;
        }
        let id = id.round() as u64;
        if !self.known.contains(&id) {
            self.result = RESULT_UNKNOWN;
        } else if magnitude <= 0. {
            self.armed.remove(&id);
            self.result = RESULT_APPLIED;
        } else if !self.armed.contains_key(&id) && self.armed.len() >= MAX_ARMED {
            self.result = RESULT_FULL;
        } else {
            self.armed.insert(id, magnitude.min(1.));
            self.result = RESULT_APPLIED;
        }
    }

    fn faults(&self) -> Faults {
        Faults::from_pairs(self.armed.iter().map(|(&id, &m)| (id, m)))
    }
}

pub struct DeepSystemsHost {
    deep: DeepSystems,
    truth: TruthReader,
    tyres: Tyres,
    arming: Arming,
    pneumatic_valves: pneumatic_valves::PneumaticValves,
    flight_controls_authority: FlightControlsAuthority,
    mel: mel::Mel,
    maintenance_log: maintenance_log::MaintenanceLog,
    sim_effects: SimEffects,
    random: RandomFailures,
    scripted: Scripted,
    schedule: scheduled::ScheduleLink,
    hot_section: hot_section::HotSection,
    airborne_s: f64,
    wear: WearStore,
    persistence: Persistence,
    random_enabled_id: VariableIdentifier,
    random_rate_multiplier_id: VariableIdentifier,
    random_component_count_id: VariableIdentifier,
    scripted_pending_count_id: VariableIdentifier,
    airframe_hours_id: VariableIdentifier,
    wear_engine_hot_hours_ids: [VariableIdentifier; 4],
    wear_airframe_cycles_id: VariableIdentifier,
    was_airborne: bool,

    units: Vec<Unit>,
    published: Vec<Published>,
    published_index: HashMap<String, usize>,
    gates: Vec<Gate>,
    reset_buttons: Vec<ResetButton>,
    load_effect_cut_index: Vec<Option<usize>>,
    effects: power_effects::Active,
    consumer_failure_ids: Vec<VariableIdentifier>,

    tick_us_id: VariableIdentifier,
    tick_us: f64,
    pending_dt_s: f64,

    #[cfg(test)]
    last_truth: Option<deep_systems::Truth>,
}

impl DeepSystemsHost {
    pub fn new(context: &mut InitContext) -> Self {
        let deep = DeepSystems::new();
        let ids = deep.unit_ids();
        let index_of = |id: &str| {
            ids.iter()
                .position(|u| *u == id)
                .unwrap_or_else(|| panic!("{id} is not a protection unit"))
        };

        let mut units: Vec<Unit> = ids
            .iter()
            .map(|id| {
                let key = lvar_key(id);
                Unit {
                    current_id: context.get_identifier(format!("BKR_{key}_CURRENT_A")),
                    cmd_id: context.get_identifier(format!("BKR_{key}_CMD")),
                    command: None,
                    consumed: false,
                    open: false,
                    cut: false,
                    cut_index: None,
                    current_a: 0.,
                }
            })
            .collect();

        let mut published = Vec::new();
        let mut published_index = HashMap::new();
        for name in deep.published_names() {
            if published_index.contains_key(&name) {
                continue;
            }
            published_index.insert(name.clone(), published.len());
            let key = lvar_key(&name);
            let key = key.strip_prefix("A32NX_").unwrap_or(&key).to_owned();
            published.push(Published {
                id: context.get_identifier(key),
                value: 0.,
                sent: f64::NAN,
                changed: false,
            });
        }

        for (unit, id) in units.iter_mut().zip(&ids) {
            unit.cut_index = power_effects::load_of_unit(id, |n| published_index.contains_key(n))
                .and_then(|load| published_index.get(&power_effects::cut_name(load)).copied());
        }
        let load_effect_cut_index: Vec<Option<usize>> = power_effects::LOAD_EFFECTS
            .iter()
            .map(|row| published_index.get(&power_effects::cut_name(row.load)).copied())
            .collect();

        let gates = all_gates()
            .map(|g| Gate {
                id: context.get_identifier(g.variable.to_owned()),
                units: g.units.iter().map(|u| index_of(u)).collect(),
                open: false,
            })
            .collect();

        let mut reset_buttons: Vec<ResetButton> = RESET_PANEL_UNITS
            .iter()
            .map(|(name, units)| ResetButton {
                id: context.get_identifier(format!("RESET_PANEL_{name}")),
                units: units.iter().map(|u| index_of(u)).collect(),
                failures: &[],
                pulled: false,
                was_pulled: false,
            })
            .collect();
        reset_buttons.extend(RESET_PANEL_FAILURES.iter().map(|(name, failures)| ResetButton {
            id: context.get_identifier(format!("RESET_PANEL_{name}")),
            units: Vec::new(),
            failures,
            pulled: false,
            was_pulled: false,
        }));

        let mut random = RandomFailures::new(0x1234_5678_9abc_def0);
        let mut scripted = Scripted::new();
        let mut wear = WearStore::default();
        let persistence = Persistence::for_host();
        persistence.apply_to(&mut random, &mut scripted, &mut wear);

        let mut arming = Arming::new(context);
        arming.known.extend(power_effects::alias_sources());
        arming.known.extend(consumer_failures::CONSUMER_FAILURES.iter().copied());

        let mut mel = mel::Mel::new(context);
        for item in persistence.deferred_mel_items() {
            mel.restore_deferred(item);
        }

        let maintenance_log = maintenance_log::MaintenanceLog::new(context, &ids, units.len());

        Self {
            truth: TruthReader::new(context),
            tyres: Tyres::new(context),
            arming,
            pneumatic_valves: pneumatic_valves::PneumaticValves::new(context),
            flight_controls_authority: FlightControlsAuthority::new(context),
            mel,
            maintenance_log,
            sim_effects: SimEffects::new(),
            random,
            scripted,
            schedule: scheduled::ScheduleLink::new(context),
            hot_section: hot_section::HotSection::new(context),
            airborne_s: 0.,
            wear,
            persistence,
            random_enabled_id: context.get_identifier("DEEP_RANDOM_FAILURES_ENABLED".to_owned()),
            random_rate_multiplier_id: context.get_identifier("DEEP_RANDOM_FAILURES_RATE_MULTIPLIER".to_owned()),
            random_component_count_id: context.get_identifier("DEEP_RANDOM_FAILURES_COMPONENT_COUNT".to_owned()),
            scripted_pending_count_id: context.get_identifier("DEEP_SCRIPTED_FAILURES_PENDING_COUNT".to_owned()),
            airframe_hours_id: context.get_identifier("DEEP_AIRFRAME_HOURS".to_owned()),
            wear_engine_hot_hours_ids: std::array::from_fn(|k| context.get_identifier(format!("DEEP_WEAR_ENGINE_{}_HOT_HOURS", k + 1))),
            wear_airframe_cycles_id: context.get_identifier("DEEP_WEAR_AIRFRAME_CYCLES".to_owned()),
            was_airborne: false,
            deep,
            units,
            published,
            published_index,
            gates,
            reset_buttons,
            load_effect_cut_index,
            effects: power_effects::Active::default(),
            consumer_failure_ids: consumer_failures::CONSUMER_FAILURES
                .iter()
                .map(|id| context.get_identifier(format!("DEEP_FAILURE_{id}_ACTIVE")))
                .collect(),
            tick_us_id: context.get_identifier("DEEP_SYSTEMS_TICK_US".to_owned()),
            tick_us: 0.,
            pending_dt_s: 0.,
            #[cfg(test)]
            last_truth: None,
        }
    }

    pub fn update(&mut self, context: &UpdateContext) {
        for (i, unit) in self.units.iter_mut().enumerate() {
            if let Some(command) = unit.command.take() {
                self.deep.command_at(i, command);
            }
        }
        for button in &mut self.reset_buttons {
            if button.pulled {
                for &i in &button.units {
                    if !self.deep.is_open_at(i) {
                        self.deep.command_at(i, BreakerCommand::Open);
                    }
                }
            } else if button.was_pulled {
                for &i in &button.units {
                    self.deep.command_at(i, BreakerCommand::Close);
                }
            }
            button.was_pulled = button.pulled;
        }
        self.arming.apply();
        self.mel.update(context.delta_as_secs_f64() / 3600.0);
        self.tyres.update(context, &mut self.arming.armed);

        let mut truth = self.truth.truth(context);
        truth.tyre_pressure_pa = self.tyres.pressures_pa();
        truth.tyre_temp_c = self.tyres.temperatures_c();
        #[cfg(test)]
        {
            self.last_truth = Some(truth.clone());
        }
        self.sim_effects.update(
            context.indicated_airspeed().get::<uom::si::velocity::knot>(),
            truth.controls.gear_lever_down,
            context.delta_as_secs_f64(),
        );

        let delta_hours = context.delta_as_secs_f64() / 3600.0;
        self.airborne_s = if truth.on_ground { 0. } else { self.airborne_s + context.delta_as_secs_f64() };
        self.schedule.apply(&mut self.scripted, &self.arming.known, self.persistence.state.airframe_hours);
        let sample = Sample {
            elapsed_hours: self.persistence.state.airframe_hours,
            altitude_ft: truth.altitude_ft,
            speed_kt: truth.environment.tas_ms * 1.943_844,
            phase: Phase::from_fmgc(truth.fmgc_flight_phase),
            ias_kt: context.indicated_airspeed().get::<uom::si::velocity::knot>(),
            radio_height_ft: truth.radio_height_ft,
            airborne_s: self.airborne_s,
        };
        let already_active: BTreeSet<u64> = self.arming.armed.keys().copied().collect();
        let mut triggered: Vec<(u64, f64)> = self.random.update(delta_hours, &already_active).into_iter().map(|id| (id, 1.0)).collect();
        for due in self.scripted.update(&sample) {
            if due.external {
                self.schedule.fire_external(due.id);
            } else {
                triggered.push((due.id, due.magnitude));
            }
        }
        for (id, magnitude) in triggered {
            if self.arming.known.contains(&id) && (self.arming.armed.contains_key(&id) || self.arming.armed.len() < MAX_ARMED) {
                self.arming.armed.insert(id, magnitude.min(1.0));
            }
        }

        for (i, running) in truth.engine_running.iter().enumerate() {
            if *running {
                self.wear.accumulate(&format!("engine-{}", i + 1), delta_hours, 0, 0.0, 0.0);
            }
        }
        if !truth.on_ground {
            self.was_airborne = true;
        } else if self.was_airborne {
            self.wear.accumulate("airframe", 0.0, 1, 0.0, 0.0);
            self.was_airborne = false;
        }
        let tgt_untrimmed_c: [f64; 4] = std::array::from_fn(|e| self.published_value(&format!("A32NX_ENG_{}_PHYS_EGT_C", e + 1)));
        let engine_lit: [bool; 4] = std::array::from_fn(|e| self.published_value(&format!("A32NX_ENG_{}_PHYS_LIT", e + 1)) >= 0.5);
        self.hot_section.update(tgt_untrimmed_c, engine_lit, context.delta_as_secs_f64(), &mut self.wear);
        deep_systems::wear::publish(self.wear.clone());
        let mel_deferred_items: BTreeSet<usize> = self.mel.deferred_items().collect();
        self.persistence.capture_from(&self.arming.armed, &self.random, &self.scripted, &self.wear, &mel_deferred_items);
        self.persistence.tick(delta_hours, context.delta_as_secs_f64());

        let load_cut: Vec<bool> = self
            .load_effect_cut_index
            .iter()
            .map(|i| i.is_some_and(|i| self.published[i].value >= 0.5))
            .collect();
        self.effects = power_effects::active(&power_effects::Conditions::of(&truth), &load_cut, &self.arming.armed);
        let faults = Faults::from_pairs(
            self.effects
                .deep
                .iter()
                .copied()
                .chain(self.hot_section.derived(&self.wear))
                .chain(self.arming.armed.iter().map(|(&id, &m)| (id, m))),
        );
        self.flight_controls_authority.update(&faults);
        self.pending_dt_s += context.delta_as_secs_f64();
        if self.pending_dt_s < DEEP_TICK_S {
            self.tick_us = 0.;
            for p in &mut self.published {
                p.changed = false;
            }
            return;
        }
        truth.dt_s = self.pending_dt_s.clamp(0.001, 0.2);
        self.pending_dt_s = 0.;
        let maint_zulu_s = self.truth.zulu_time_s();
        let maint_phase = truth.fmgc_flight_phase;
        let started = Instant::now();
        let published = &mut self.published;
        let index = &self.published_index;
        self.deep.tick(truth, &faults, &mut |name, value| {
            if let Some(&i) = index.get(name) {
                published[i].value = value;
            }
        });
        self.tick_us = started.elapsed().as_secs_f64() * 1e6;
        self.pneumatic_valves.update(self.deep.derived_failures(), &self.arming.armed);
        for p in &mut self.published {
            p.changed = p.value != p.sent;
            if p.changed {
                p.sent = p.value;
            }
        }

        let units: Vec<(bool, f64)> = self.deep.units().map(|(_, b, a)| (!b.closed, a)).collect();
        for (unit, (open, current_a)) in self.units.iter_mut().zip(units) {
            unit.open = open;
            unit.current_a = current_a;
            unit.cut = unit.cut_index.is_some_and(|i| self.published[i].value >= 0.5);
        }
        for gate in &mut self.gates {
            gate.open = gate.units.iter().all(|&i| self.units[i].open || self.units[i].cut);
        }

        let mel_deferred: BTreeSet<usize> = self.mel.deferred_items().collect();
        let unit_open: Vec<(bool, bool)> = self.units.iter().map(|u| (u.open, u.consumed)).collect();
        self.maintenance_log.update(maint_zulu_s, maint_phase, &self.arming.armed, self.deep.derived_failures(), &unit_open, &mel_deferred);
    }

    pub fn derived_failure_ids(&self) -> Vec<u64> {
        let mut ids: Vec<u64> = self.deep.derived_failures().iter().map(|d| d.fbw_id).collect();
        ids.extend(breaker_failures::active(self.deep.unit_ids().into_iter().zip(self.units.iter().map(|u| u.open || u.cut))));
        ids.extend_from_slice(&self.effects.fbw);
        for button in self.reset_buttons.iter().filter(|b| b.pulled) {
            ids.extend_from_slice(button.failures);
        }
        ids.extend(self.sim_effects.derived_failure_ids());
        ids.extend(self.mel.derived_failure_ids());
        ids
    }

    pub fn armed_failures(&self) -> &BTreeMap<u64, f64> {
        &self.arming.armed
    }

    #[cfg(test)]
    pub(super) fn mel_is_deferred(&self, id: u64) -> bool {
        self.mel.is_deferred(id)
    }

    fn published_value(&self, name: &str) -> f64 {
        self.published_index.get(name).map_or(0., |&i| self.published[i].value)
    }

    #[cfg(test)]
    pub(super) fn schedule_scripted(&mut self, id: u64, condition: ArmCondition) {
        self.scripted.schedule(id, condition);
    }

    #[cfg(test)]
    pub(super) fn snapshot(&self) -> BTreeMap<String, f64> {
        let mut out: BTreeMap<String, f64> =
            self.published_index.iter().map(|(name, &i)| (name.clone(), self.published[i].value)).collect();
        for (gate, def) in self.gates.iter().zip(all_gates()) {
            out.insert(format!("GATE {}", def.variable), if gate.open { 1. } else { 0. });
        }
        for id in self.derived_failure_ids() {
            out.insert(format!("FBW FAILURE {id}"), 1.);
        }
        out
    }
}

impl SimulationElement for DeepSystemsHost {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.truth.read(reader);
        self.tyres.read(reader);
        self.mel.read(reader);
        for unit in &mut self.units {
            unit.command = match reader.read_f64(&unit.cmd_id).round() as i64 {
                1 => Some(BreakerCommand::Open),
                2 => Some(BreakerCommand::Close),
                _ => None,
            };
            unit.consumed = unit.command.is_some();
        }
        for button in &mut self.reset_buttons {
            button.pulled = reader.read_f64(&button.id) != 0.;
        }
        self.schedule.read(reader);
        self.hot_section.read(reader);
        let id: f64 = reader.read(&self.arming.cmd_id_id);
        self.arming.command = if id != 0. {
            Some((id, reader.read(&self.arming.cmd_magnitude_id)))
        } else {
            None
        };

        let enabled: f64 = reader.read(&self.random_enabled_id);
        let rate_multiplier: f64 = reader.read(&self.random_rate_multiplier_id);
        self.random.config = RandomConfig { enabled: enabled != 0., rate_multiplier: if rate_multiplier > 0. { rate_multiplier } else { 1.0 } };
    }

    fn receive_power(&mut self, buses: &impl ElectricalBuses) {
        self.truth.receive_power(buses);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        for p in self.published.iter().filter(|p| p.changed) {
            writer.write_f64(&p.id, p.value);
        }
        for unit in &self.units {
            writer.write_f64(&unit.current_id, unit.current_a);
            if unit.consumed {
                writer.write_f64(&unit.cmd_id, 0.);
            }
        }
        for gate in &self.gates {
            writer.write_f64(&gate.id, if gate.open { 1. } else { 0. });
        }
        for (id, var) in consumer_failures::CONSUMER_FAILURES.iter().zip(&self.consumer_failure_ids) {
            writer.write_f64(var, self.arming.armed.get(id).copied().unwrap_or(0.));
        }
        let a = &self.arming;
        if a.command.is_some() {
            writer.write(&a.cmd_id_id, 0.);
            writer.write(&a.result_id, a.result);
        }
        writer.write(&a.count_id, a.armed.len() as f64);
        let mut armed_iter = a.armed.iter();
        for (id_id, magnitude_id) in &a.slot_ids {
            let (id, magnitude) = armed_iter.next().map_or((0., 0.), |(&id, &m)| (id as f64, m));
            writer.write(id_id, id);
            writer.write(magnitude_id, magnitude);
        }
        self.tyres.write(writer);
        self.pneumatic_valves.write(writer);
        self.flight_controls_authority.write(writer);
        self.mel.write(writer);
        self.maintenance_log.write(writer);
        writer.write_f64(&self.tick_us_id, self.tick_us);

        writer.write_f64(&self.random_component_count_id, self.random.component_count() as f64);
        writer.write_f64(&self.scripted_pending_count_id, self.scripted.list().len() as f64);
        self.schedule.write(writer, &self.scripted, self.persistence.state.airframe_hours);
        self.hot_section.write(writer, &self.wear);
        writer.write_f64(&self.airframe_hours_id, self.persistence.state.airframe_hours);
        for (i, id) in self.wear_engine_hot_hours_ids.iter().enumerate() {
            writer.write_f64(id, self.wear.get(&format!("engine-{}", i + 1)).hot_hours);
        }
        writer.write_f64(&self.wear_airframe_cycles_id, self.wear.get("airframe").cycles as f64);
    }
}

impl Drop for DeepSystemsHost {
    fn drop(&mut self) {
        let mel_deferred_items: BTreeSet<usize> = self.mel.deferred_items().collect();
        self.persistence.capture_from(&self.arming.armed, &self.random, &self.scripted, &self.wear, &mel_deferred_items);
        if let Err(e) = self.persistence.save() {
            deep_systems::log(&format!("could not save a380x_deep_airframe.toml at shutdown: {e}"));
        }
    }
}

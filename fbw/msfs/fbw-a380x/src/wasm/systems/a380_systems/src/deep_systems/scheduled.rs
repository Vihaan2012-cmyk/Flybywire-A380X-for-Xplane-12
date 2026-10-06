use std::collections::{BTreeSet, VecDeque};

use deep_systems::scripted_failures::{ArmCondition, Phase, Scripted};
use systems::simulation::{InitContext, Read, SimulatorReader, SimulatorWriter, VariableIdentifier, Write};

use super::persistence::phase_to_u8;

pub(super) const MAX_SCHEDULED: usize = 32;

const KIND_CANCEL: i64 = 0;
const KIND_IAS_ABOVE_KT: i64 = 1;
const KIND_RADIO_HEIGHT_ABOVE_FT: i64 = 2;
const KIND_RADIO_HEIGHT_BELOW_FT: i64 = 3;
const KIND_SECONDS_AFTER_LIFTOFF: i64 = 4;
const KIND_SECONDS_FROM_NOW: i64 = 5;
const KIND_ALTITUDE_ABOVE_FT: i64 = 6;
const KIND_FLIGHT_PHASE: i64 = 7;
const KIND_TAS_ABOVE_KT: i64 = 8;
const KIND_ALTITUDE_BELOW_FT: i64 = 9;

const RESULT_APPLIED: f64 = 1.;
const RESULT_FULL: f64 = 2.;
const RESULT_REJECTED: f64 = 3.;

struct Command {
    id: f64,
    kind: f64,
    value: f64,
    magnitude: f64,
    external: bool,
}

pub(super) struct ScheduleLink {
    cmd_id_id: VariableIdentifier,
    cmd_kind_id: VariableIdentifier,
    cmd_value_id: VariableIdentifier,
    cmd_magnitude_id: VariableIdentifier,
    cmd_external_id: VariableIdentifier,
    result_id: VariableIdentifier,
    slot_ids: [[VariableIdentifier; 4]; MAX_SCHEDULED],
    external_id_id: VariableIdentifier,
    external_ack_id: VariableIdentifier,
    command: Option<Command>,
    result: f64,
    external: VecDeque<u64>,
    ack_consumed: bool,
}

impl ScheduleLink {
    pub(super) fn new(context: &mut InitContext) -> Self {
        Self {
            cmd_id_id: context.get_identifier("DEEP_SCRIPTED_CMD_ID".to_owned()),
            cmd_kind_id: context.get_identifier("DEEP_SCRIPTED_CMD_KIND".to_owned()),
            cmd_value_id: context.get_identifier("DEEP_SCRIPTED_CMD_VALUE".to_owned()),
            cmd_magnitude_id: context.get_identifier("DEEP_SCRIPTED_CMD_MAGNITUDE".to_owned()),
            cmd_external_id: context.get_identifier("DEEP_SCRIPTED_CMD_EXTERNAL".to_owned()),
            result_id: context.get_identifier("DEEP_SCRIPTED_CMD_RESULT".to_owned()),
            slot_ids: std::array::from_fn(|k| {
                ["ID", "KIND", "VALUE", "MAGNITUDE"].map(|field| context.get_identifier(format!("DEEP_SCRIPTED_PENDING_{k}_{field}")))
            }),
            external_id_id: context.get_identifier("DEEP_SCRIPTED_EXTERNAL_FIRE_ID".to_owned()),
            external_ack_id: context.get_identifier("DEEP_SCRIPTED_EXTERNAL_FIRE_ACK".to_owned()),
            command: None,
            result: 0.,
            external: VecDeque::new(),
            ack_consumed: false,
        }
    }

    pub(super) fn read(&mut self, reader: &mut SimulatorReader) {
        let id: f64 = reader.read(&self.cmd_id_id);
        self.command = if id != 0. {
            Some(Command {
                id,
                kind: reader.read(&self.cmd_kind_id),
                value: reader.read(&self.cmd_value_id),
                magnitude: reader.read(&self.cmd_magnitude_id),
                external: Read::<f64>::read(reader, &self.cmd_external_id) != 0.,
            })
        } else {
            None
        };

        let ack: f64 = reader.read(&self.external_ack_id);
        self.ack_consumed = false;
        if ack != 0. && self.external.front().is_some_and(|&front| front as f64 == ack) {
            self.external.pop_front();
            self.ack_consumed = true;
        }
    }

    pub(super) fn apply(&mut self, scripted: &mut Scripted, known: &BTreeSet<u64>, airframe_hours: f64) {
        let Some(command) = &self.command else { return };
        let kind = command.kind.round() as i64;
        if kind == KIND_CANCEL {
            if command.id < 0. {
                scripted.clear();
            } else {
                scripted.cancel(command.id.round() as u64);
            }
            self.result = RESULT_APPLIED;
            return;
        }

        let id = command.id.round();
        let value = command.value;
        if id <= 0. || !value.is_finite() || value < 0. || !(command.magnitude > 0.) {
            self.result = RESULT_REJECTED;
            return;
        }
        let condition = match kind {
            KIND_IAS_ABOVE_KT => ArmCondition::AboveIasKt(value),
            KIND_RADIO_HEIGHT_ABOVE_FT => ArmCondition::AboveRadioHeightFt(value),
            KIND_RADIO_HEIGHT_BELOW_FT => ArmCondition::BelowRadioHeightFt { ft: value, seen_above: false },
            KIND_SECONDS_AFTER_LIFTOFF => ArmCondition::SecondsAfterLiftoff(value),
            KIND_SECONDS_FROM_NOW => ArmCondition::ElapsedHours(airframe_hours + value / 3600.),
            KIND_ALTITUDE_ABOVE_FT => ArmCondition::AboveAltitudeFt(value),
            KIND_FLIGHT_PHASE => match Phase::from_fmgc(value) {
                Some(phase) => ArmCondition::OnFlightPhase(phase),
                None => {
                    self.result = RESULT_REJECTED;
                    return;
                }
            },
            _ => {
                self.result = RESULT_REJECTED;
                return;
            }
        };
        let id = id as u64;
        if !command.external && !known.contains(&id) {
            self.result = RESULT_REJECTED;
            return;
        }
        if !scripted.list().iter().any(|t| t.id == id) && scripted.list().len() >= MAX_SCHEDULED {
            self.result = RESULT_FULL;
            return;
        }
        if command.external {
            scripted.schedule_external(id, condition);
        } else {
            scripted.schedule_at(id, condition, command.magnitude.min(1.));
        }
        self.result = RESULT_APPLIED;
    }

    pub(super) fn fire_external(&mut self, id: u64) {
        if !self.external.contains(&id) {
            self.external.push_back(id);
        }
    }

    pub(super) fn write(&self, writer: &mut SimulatorWriter, scripted: &Scripted, airframe_hours: f64) {
        if self.command.is_some() {
            writer.write(&self.cmd_id_id, 0.);
            writer.write(&self.result_id, self.result);
        }

        let mut pending = scripted.list().iter();
        for [id_id, kind_id, value_id, magnitude_id] in &self.slot_ids {
            let (id, kind, value, magnitude) = pending.next().map_or((0., 0., 0., 0.), |t| {
                let (kind, value) = describe(t.condition, airframe_hours);
                (t.id as f64, kind as f64, value, t.magnitude)
            });
            writer.write(id_id, id);
            writer.write(kind_id, kind);
            writer.write(value_id, value);
            writer.write(magnitude_id, magnitude);
        }

        writer.write(&self.external_id_id, self.external.front().map_or(0., |&id| id as f64));
        if self.ack_consumed {
            writer.write(&self.external_ack_id, 0.);
        }
    }
}

fn describe(condition: ArmCondition, airframe_hours: f64) -> (i64, f64) {
    match condition {
        ArmCondition::AboveIasKt(kt) => (KIND_IAS_ABOVE_KT, kt),
        ArmCondition::AboveRadioHeightFt(ft) => (KIND_RADIO_HEIGHT_ABOVE_FT, ft),
        ArmCondition::BelowRadioHeightFt { ft, .. } => (KIND_RADIO_HEIGHT_BELOW_FT, ft),
        ArmCondition::SecondsAfterLiftoff(t) => (KIND_SECONDS_AFTER_LIFTOFF, t),
        ArmCondition::ElapsedHours(h) => (KIND_SECONDS_FROM_NOW, ((h - airframe_hours) * 3600.).max(0.)),
        ArmCondition::AboveAltitudeFt(ft) => (KIND_ALTITUDE_ABOVE_FT, ft),
        ArmCondition::OnFlightPhase(p) => (KIND_FLIGHT_PHASE, phase_to_u8(p) as f64),
        ArmCondition::AboveSpeedKt(kt) => (KIND_TAS_ABOVE_KT, kt),
        ArmCondition::BelowAltitudeFt(ft) => (KIND_ALTITUDE_BELOW_FT, ft),
    }
}

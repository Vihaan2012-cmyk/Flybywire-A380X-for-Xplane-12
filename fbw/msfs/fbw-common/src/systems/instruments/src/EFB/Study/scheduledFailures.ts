// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { useMemo } from 'react';
import { useSimVar, useSimVarList } from '@flybywiresim/fbw-sdk-react';

export const MAX_SCHEDULED_FAILURES = 32;

export enum ScheduleTrigger {
  Now = 0,
  IasAboveKt = 1,
  RadioHeightAboveFt = 2,
  RadioHeightBelowFt = 3,
  SecondsAfterLiftoff = 4,
  SecondsFromNow = 5,
  AltitudeAboveFt = 6,
  FlightPhase = 7,
  TasAboveKt = 8,
  AltitudeBelowFt = 9,
}

export enum ScheduleCommandResult {
  None = 0,
  Applied = 1,
  RejectedFull = 2,
  Rejected = 3,
}

export const SCHEDULE_TRIGGER_OPTIONS = [
  { value: ScheduleTrigger.Now, displayValue: 'Now' },
  { value: ScheduleTrigger.IasAboveKt, displayValue: 'When IAS reaches (kt)' },
  { value: ScheduleTrigger.SecondsAfterLiftoff, displayValue: 'Seconds after liftoff' },
  { value: ScheduleTrigger.RadioHeightAboveFt, displayValue: 'Climbing through radio altitude (ft)' },
  { value: ScheduleTrigger.RadioHeightBelowFt, displayValue: 'Descending through radio altitude (ft)' },
  { value: ScheduleTrigger.AltitudeAboveFt, displayValue: 'Climbing through altitude (ft)' },
  { value: ScheduleTrigger.SecondsFromNow, displayValue: 'In seconds' },
  { value: ScheduleTrigger.FlightPhase, displayValue: 'On flight phase' },
];

export const FLIGHT_PHASE_OPTIONS = [
  { value: 2, displayValue: 'Takeoff' },
  { value: 3, displayValue: 'Climb' },
  { value: 4, displayValue: 'Cruise' },
  { value: 5, displayValue: 'Descent' },
  { value: 6, displayValue: 'Approach' },
  { value: 7, displayValue: 'Go-around' },
];

const PHASE_NAMES = ['Preflight', 'Taxi', 'Takeoff', 'Climb', 'Cruise', 'Descent', 'Approach', 'Go-around'];

export function describeTrigger(trigger: ScheduleTrigger, value: number): string {
  const n = Math.round(value);
  switch (trigger) {
    case ScheduleTrigger.IasAboveKt:
      return `when IAS reaches ${n} kt`;
    case ScheduleTrigger.RadioHeightAboveFt:
      return `climbing through ${n} ft RA`;
    case ScheduleTrigger.RadioHeightBelowFt:
      return `descending through ${n} ft RA`;
    case ScheduleTrigger.SecondsAfterLiftoff:
      return `${n} s after liftoff`;
    case ScheduleTrigger.SecondsFromNow:
      return `in ${n} s`;
    case ScheduleTrigger.AltitudeAboveFt:
      return `climbing through ${n} ft`;
    case ScheduleTrigger.AltitudeBelowFt:
      return `below ${n} ft`;
    case ScheduleTrigger.TasAboveKt:
      return `when TAS reaches ${n} kt`;
    case ScheduleTrigger.FlightPhase:
      return `on ${PHASE_NAMES[n] ?? `phase ${n}`}`;
    default:
      return 'now';
  }
}

export function describeScheduleCommandResult(result: ScheduleCommandResult): string {
  switch (result) {
    case ScheduleCommandResult.RejectedFull:
      return `Rejected -- ${MAX_SCHEDULED_FAILURES} failures are already scheduled. Cancel one first.`;
    case ScheduleCommandResult.Rejected:
      return "Rejected -- this aircraft's systems module does not model this failure, or the trigger value is invalid.";
    default:
      return '';
  }
}

export interface ScheduledFailure {
  id: number;
  trigger: ScheduleTrigger;
  value: number;
  magnitude: number;
}

export interface ScheduledFailuresState {
  scheduled: ReadonlyMap<number, ScheduledFailure>;
  lastResult: ScheduleCommandResult;
  schedule: (id: number, trigger: ScheduleTrigger, value: number, magnitude: number, flyByWire: boolean) => void;
  cancel: (id: number) => void;
  cancelAll: () => void;
}

const REFRESH_MS = 500;

const slotNames = (field: string) =>
  Array.from({ length: MAX_SCHEDULED_FAILURES }, (_, slot) => `L:A32NX_DEEP_SCRIPTED_PENDING_${slot}_${field}`);
const ID_NAMES = slotNames('ID');
const KIND_NAMES = slotNames('KIND');
const VALUE_NAMES = slotNames('VALUE');
const MAGNITUDE_NAMES = slotNames('MAGNITUDE');
const UNITS = ID_NAMES.map(() => 'number' as const);

function sendCommand(id: number, trigger: ScheduleTrigger, value: number, magnitude: number, flyByWire: boolean): void {
  SimVar.SetSimVarValue('L:A32NX_DEEP_SCRIPTED_CMD_EXTERNAL', 'number', flyByWire ? 1 : 0);
  SimVar.SetSimVarValue('L:A32NX_DEEP_SCRIPTED_CMD_MAGNITUDE', 'number', magnitude);
  SimVar.SetSimVarValue('L:A32NX_DEEP_SCRIPTED_CMD_VALUE', 'number', value);
  SimVar.SetSimVarValue('L:A32NX_DEEP_SCRIPTED_CMD_KIND', 'number', trigger);
  SimVar.SetSimVarValue('L:A32NX_DEEP_SCRIPTED_CMD_ID', 'number', id);
}

export function useScheduledFailures(): ScheduledFailuresState {
  const [count] = useSimVar('L:A32NX_DEEP_SCRIPTED_FAILURES_PENDING_COUNT', 'number', REFRESH_MS);
  const [result] = useSimVar('L:A32NX_DEEP_SCRIPTED_CMD_RESULT', 'number', REFRESH_MS);
  const [ids] = useSimVarList(ID_NAMES, UNITS, REFRESH_MS);
  const [kinds] = useSimVarList(KIND_NAMES, UNITS, REFRESH_MS);
  const [values] = useSimVarList(VALUE_NAMES, UNITS, REFRESH_MS);
  const [magnitudes] = useSimVarList(MAGNITUDE_NAMES, UNITS, REFRESH_MS);

  const scheduled = useMemo(() => {
    const map = new Map<number, ScheduledFailure>();
    const validCount = Math.max(0, Math.min(MAX_SCHEDULED_FAILURES, Math.round(count)));
    for (let slot = 0; slot < validCount; slot += 1) {
      const id = Math.round(ids[slot]);
      if (id > 0) {
        map.set(id, { id, trigger: Math.round(kinds[slot]), value: values[slot], magnitude: magnitudes[slot] });
      }
    }
    return map;
  }, [count, ids, kinds, values, magnitudes]);

  return {
    scheduled,
    lastResult: result as ScheduleCommandResult,
    schedule: sendCommand,
    cancel: (id: number) => sendCommand(id, ScheduleTrigger.Now, 0, 0, false),
    cancelAll: () => sendCommand(-1, ScheduleTrigger.Now, 0, 0, false),
  };
}

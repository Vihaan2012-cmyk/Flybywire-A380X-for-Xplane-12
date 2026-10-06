// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { useMemo } from 'react';
import { useSimVar, useSimVarList } from '@flybywiresim/fbw-sdk-react';
import { useFailuresOrchestrator } from '../failures-orchestrator-provider';

export const MAX_ARMED_FAILURES = 32;

export enum FailureCommandResult {
  None = 0,
  Applied = 1,
  RejectedTooManyArmed = 2,
  RejectedNotModelled = 3,
}

export interface ArmedFailuresState {
  armed: ReadonlyMap<number, number>;
  isFlyByWireFailure: (id: number) => boolean;
  lastResult: FailureCommandResult;
  arm: (id: number, magnitude: number) => void;
  clear: (id: number) => void;
  clearAll: () => void;
}

const REFRESH_MS = 500;

const ARMED_ID_NAMES = Array.from(
  { length: MAX_ARMED_FAILURES },
  (_, slot) => `L:A32NX_DEEP_FAILURE_ARMED_${slot}_ID`,
);
const ARMED_MAGNITUDE_NAMES = Array.from(
  { length: MAX_ARMED_FAILURES },
  (_, slot) => `L:A32NX_DEEP_FAILURE_ARMED_${slot}_MAGNITUDE`,
);
const ARMED_UNITS = ARMED_ID_NAMES.map(() => 'number' as const);

export function useArmedFailures(): ArmedFailuresState {
  const [count] = useSimVar('L:A32NX_DEEP_FAILURES_ARMED_COUNT', 'number', REFRESH_MS);
  const [result] = useSimVar('L:A32NX_DEEP_FAILURE_CMD_RESULT', 'number', REFRESH_MS);
  const [ids] = useSimVarList(ARMED_ID_NAMES, ARMED_UNITS, REFRESH_MS);
  const [magnitudes] = useSimVarList(ARMED_MAGNITUDE_NAMES, ARMED_UNITS, REFRESH_MS);

  const { allFailures, activeFailures, activate, deactivate } = useFailuresOrchestrator();
  const flyByWireIds = useMemo(() => new Set(allFailures.map((f) => f.identifier)), [allFailures]);
  const isFlyByWireFailure = (id: number): boolean => flyByWireIds.has(id);

  const armed = useMemo(() => {
    const map = new Map<number, number>();
    const validCount = Math.max(0, Math.min(MAX_ARMED_FAILURES, Math.round(count)));
    for (let slot = 0; slot < validCount; slot += 1) {
      map.set(Math.round(ids[slot]), magnitudes[slot]);
    }
    for (const id of activeFailures) {
      map.set(id, 1);
    }
    return map;
  }, [count, ids, magnitudes, activeFailures]);

  const arm = (id: number, magnitude: number): void => {
    if (isFlyByWireFailure(id)) {
      void (magnitude > 0 ? activate(id) : deactivate(id));
      return;
    }
    SimVar.SetSimVarValue('L:A32NX_DEEP_FAILURE_CMD_MAGNITUDE', 'number', magnitude);
    SimVar.SetSimVarValue('L:A32NX_DEEP_FAILURE_CMD_ID', 'number', id);
  };

  const clear = (id: number): void => arm(id, 0);

  const clearAll = (): void => {
    for (const id of activeFailures) {
      void deactivate(id);
    }
    SimVar.SetSimVarValue('L:A32NX_DEEP_FAILURE_CMD_MAGNITUDE', 'number', 0);
    SimVar.SetSimVarValue('L:A32NX_DEEP_FAILURE_CMD_ID', 'number', -1);
  };

  return { armed, isFlyByWireFailure, lastResult: result as FailureCommandResult, arm, clear, clearAll };
}

export function describeFailureCommandResult(result: FailureCommandResult): string {
  switch (result) {
    case FailureCommandResult.Applied:
      return 'Armed.';
    case FailureCommandResult.RejectedTooManyArmed:
      return 'Rejected -- too many failures armed already (max 32). Clear one first.';
    case FailureCommandResult.RejectedNotModelled:
      return "Rejected -- this aircraft's systems module does not model this failure.";
    default:
      return '';
  }
}

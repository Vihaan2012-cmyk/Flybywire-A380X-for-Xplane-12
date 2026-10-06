// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React, { useMemo } from 'react';
import { Catalogue } from './catalogue';
import { ArmedFailuresState } from './armedFailures';
import { ScheduledFailuresState, describeTrigger } from './scheduledFailures';

interface ArmedFailuresSummaryProps {
  catalogue: Catalogue;
  armedState: ArmedFailuresState;
  scheduledState: ScheduledFailuresState;
}

export const ArmedFailuresSummary = ({ catalogue, armedState, scheduledState }: ArmedFailuresSummaryProps) => {
  const failuresById = useMemo(() => new Map(catalogue.failures.map((failure) => [failure.id, failure])), [catalogue]);
  const entries = useMemo(() => [...armedState.armed.entries()].sort(([a], [b]) => a - b), [armedState.armed]);
  const scheduled = useMemo(() => [...scheduledState.scheduled.values()], [scheduledState.scheduled]);

  if (entries.length === 0 && scheduled.length === 0) {
    return (
      <div className="rounded-md border border-theme-accent px-4 py-2 text-sm text-theme-unselected">
        No failures armed or scheduled.
      </div>
    );
  }

  return (
    <div className="rounded-md border-2 border-theme-highlight px-4 py-2">
      {entries.length > 0 && (
        <>
          <div className="flex flex-row items-center justify-between">
            <span className="font-bold">{entries.length} armed</span>
            <button
              type="button"
              onClick={armedState.clearAll}
              className="rounded-md border-2 border-utility-red px-3 py-1 text-xs text-utility-red transition duration-100 hover:bg-utility-red hover:text-theme-body"
            >
              Clear all
            </button>
          </div>

          <div className="mt-2 space-y-1">
            {entries.map(([id, magnitude]) => {
              const failure = failuresById.get(id);
              return (
                <div key={id} className="flex flex-row items-center justify-between text-sm">
                  <span className="mr-4">
                    {failure?.name ?? `Failure ${id}`}{' '}
                    <span className="font-mono text-theme-unselected">{Math.round(magnitude * 100)}%</span>
                  </span>
                  <button
                    type="button"
                    onClick={() => armedState.clear(id)}
                    className="shrink-0 rounded-md border border-utility-red px-2 py-0.5 text-xs text-utility-red transition duration-100 hover:bg-utility-red hover:text-theme-body"
                  >
                    Clear
                  </button>
                </div>
              );
            })}
          </div>
        </>
      )}

      {scheduled.length > 0 && (
        <>
          <div className={`flex flex-row items-center justify-between ${entries.length > 0 ? 'mt-3 border-t border-theme-accent pt-2' : ''}`}>
            <span className="font-bold">{scheduled.length} scheduled</span>
            <button
              type="button"
              onClick={scheduledState.cancelAll}
              className="rounded-md border-2 border-utility-red px-3 py-1 text-xs text-utility-red transition duration-100 hover:bg-utility-red hover:text-theme-body"
            >
              Cancel all
            </button>
          </div>

          <div className="mt-2 space-y-1">
            {scheduled.map((entry) => {
              const failure = failuresById.get(entry.id);
              return (
                <div key={entry.id} className="flex flex-row items-center justify-between text-sm">
                  <span className="mr-4">
                    {failure?.name ?? `Failure ${entry.id}`}{' '}
                    <span className="text-theme-unselected">{describeTrigger(entry.trigger, entry.value)}</span>{' '}
                    <span className="font-mono text-theme-unselected">{Math.round(entry.magnitude * 100)}%</span>
                  </span>
                  <button
                    type="button"
                    onClick={() => scheduledState.cancel(entry.id)}
                    className="shrink-0 rounded-md border border-utility-red px-2 py-0.5 text-xs text-utility-red transition duration-100 hover:bg-utility-red hover:text-theme-body"
                  >
                    Cancel
                  </button>
                </div>
              );
            })}
          </div>
        </>
      )}
    </div>
  );
};

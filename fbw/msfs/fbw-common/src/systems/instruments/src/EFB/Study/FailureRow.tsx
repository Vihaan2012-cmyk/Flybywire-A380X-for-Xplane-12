// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React, { useEffect, useRef, useState } from 'react';
import { toast } from 'react-toastify';
import { CatalogueFailure } from './catalogue';
import { ArmedFailuresState, FailureCommandResult, describeFailureCommandResult } from './armedFailures';
import {
  FLIGHT_PHASE_OPTIONS,
  SCHEDULE_TRIGGER_OPTIONS,
  ScheduleCommandResult,
  ScheduleTrigger,
  ScheduledFailuresState,
  describeScheduleCommandResult,
  describeTrigger,
} from './scheduledFailures';
import { ConsequenceChart } from './ConsequenceChart';
import { SelectInput } from '../UtilComponents/Form/SelectInput/SelectInput';
import { SimpleInput } from '../UtilComponents/Form/SimpleInput/SimpleInput';

const ANSWER_TIMEOUT_MS = 3000;

type Pending = { action: 'arm' | 'clear' | 'schedule' | 'unschedule'; magnitude: number };
type Outcome = { ok: boolean; text: string };

interface FailureRowProps {
  failure: CatalogueFailure;
  armedState: ArmedFailuresState;
  scheduledState: ScheduledFailuresState;
}

export const FailureRow = ({ failure, armedState, scheduledState }: FailureRowProps) => {
  const [expanded, setExpanded] = useState(false);
  const [magnitudePercent, setMagnitudePercent] = useState(100);
  const [trigger, setTrigger] = useState<ScheduleTrigger>(ScheduleTrigger.Now);
  const [triggerValue, setTriggerValue] = useState('');
  const [phase, setPhase] = useState(2);
  const [pending, setPending] = useState<Pending | null>(null);
  const [outcome, setOutcome] = useState<Outcome | null>(null);
  const timeout = useRef<ReturnType<typeof setTimeout> | null>(null);
  const lastResult = useRef(armedState.lastResult);
  lastResult.current = armedState.lastResult;
  const lastScheduleResult = useRef(scheduledState.lastResult);
  lastScheduleResult.current = scheduledState.lastResult;

  const armedMagnitude = armedState.armed.get(failure.id);
  const isArmed = armedMagnitude !== undefined;
  const scheduled = scheduledState.scheduled.get(failure.id);
  const isScheduled = scheduled !== undefined;
  const onOff = armedState.isFlyByWireFailure(failure.id);

  const triggerNumber = trigger === ScheduleTrigger.FlightPhase ? phase : Number(triggerValue);
  const triggerValid =
    trigger === ScheduleTrigger.Now ||
    trigger === ScheduleTrigger.FlightPhase ||
    (triggerValue !== '' && Number.isFinite(triggerNumber) && triggerNumber >= 0);

  useEffect(() => {
    if (expanded) {
      setMagnitudePercent(isArmed ? Math.round((armedMagnitude as number) * 100) : 100);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [expanded]);

  useEffect(() => {
    if (!pending) {
      return;
    }
    let done: boolean;
    if (pending.action === 'arm') {
      done = isArmed && Math.abs((armedMagnitude as number) - pending.magnitude) < 0.01;
    } else if (pending.action === 'clear') {
      done = !isArmed;
    } else if (pending.action === 'schedule') {
      done = isScheduled;
    } else {
      done = !isScheduled;
    }
    if (done) {
      if (timeout.current) clearTimeout(timeout.current);
      setPending(null);
      if (pending.action === 'arm') {
        const text = onOff ? 'Armed: it is active now.' : `Armed at ${Math.round(pending.magnitude * 100)}%: it is active now.`;
        setOutcome({ ok: true, text });
        toast.success(`${failure.name}: armed`);
      } else if (pending.action === 'clear') {
        setOutcome({ ok: true, text: 'Cleared.' });
        toast.info(`${failure.name}: cleared`);
      } else if (pending.action === 'schedule' && scheduled) {
        const when = describeTrigger(scheduled.trigger, scheduled.value);
        setOutcome({ ok: true, text: `Scheduled: it arms ${when}.` });
        toast.success(`${failure.name}: scheduled ${when}`);
      } else {
        setOutcome({ ok: true, text: 'Schedule cancelled.' });
        toast.info(`${failure.name}: schedule cancelled`);
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pending, isArmed, armedMagnitude, isScheduled]);

  useEffect(
    () => () => {
      if (timeout.current) clearTimeout(timeout.current);
    },
    [],
  );

  const send = (action: Pending['action'], magnitude: number): void => {
    setOutcome(null);
    setPending({ action, magnitude });
    if (action === 'arm') {
      armedState.arm(failure.id, magnitude);
    } else if (action === 'clear') {
      armedState.clear(failure.id);
    } else if (action === 'schedule') {
      scheduledState.schedule(failure.id, trigger, triggerNumber, magnitude, onOff);
    } else {
      scheduledState.cancel(failure.id);
    }
    if (timeout.current) clearTimeout(timeout.current);
    timeout.current = setTimeout(() => {
      setPending((current) => {
        if (current) {
          let text = 'No answer from the aircraft yet: is the aircraft loaded and its systems running?';
          if (current.action === 'schedule' || current.action === 'unschedule') {
            const result = lastScheduleResult.current;
            if (result === ScheduleCommandResult.RejectedFull || result === ScheduleCommandResult.Rejected) {
              text = describeScheduleCommandResult(result);
            }
          } else if (
            lastResult.current === FailureCommandResult.RejectedTooManyArmed ||
            lastResult.current === FailureCommandResult.RejectedNotModelled
          ) {
            text = describeFailureCommandResult(lastResult.current);
          }
          setOutcome({ ok: false, text });
          toast.error(`${failure.name}: ${text}`);
        }
        return null;
      });
    }, ANSWER_TIMEOUT_MS);
  };

  const handleArm = (): void => {
    const magnitude = onOff ? 1 : magnitudePercent / 100;
    send(trigger === ScheduleTrigger.Now ? 'arm' : 'schedule', magnitude);
  };

  const handleClear = (): void => send('clear', 0);

  const handleUnschedule = (): void => send('unschedule', 0);

  const armLabel = (): string => {
    if (pending?.action === 'arm') return 'Arming…';
    if (pending?.action === 'schedule') return 'Scheduling…';
    if (trigger !== ScheduleTrigger.Now) return isScheduled ? 'Reschedule' : 'Schedule';
    if (isArmed) return onOff ? 'Armed' : 'Update';
    return 'Arm';
  };

  let borderClass = 'border-theme-accent';
  if (isArmed) {
    borderClass = 'border-utility-amber';
  } else if (isScheduled) {
    borderClass = 'border-theme-highlight';
  }

  return (
    <div className={`rounded-md border px-4 py-2 ${borderClass}`}>
      <button
        type="button"
        className="flex w-full flex-row items-baseline bg-transparent text-left"
        onClick={() => setExpanded(!expanded)}
      >
        <span className="flex flex-row items-baseline">
          <span className="mr-2 font-bold">{failure.name}</span>
          {isArmed && (
            <span className="mr-2 rounded-sm bg-utility-amber px-2 py-0.5 text-xs font-bold text-theme-body">
              ARMED {Math.round((armedMagnitude as number) * 100)}%
            </span>
          )}
          {scheduled && (
            <span className="rounded-sm bg-theme-highlight px-2 py-0.5 text-xs font-bold text-theme-body">
              SCHEDULED {describeTrigger(scheduled.trigger, scheduled.value).toUpperCase()}
            </span>
          )}
        </span>
      </button>

      {expanded && (
        <div className="mt-2 border-t border-theme-accent pt-2">
          {onOff ? (
            <p className="text-sm text-theme-unselected">On or off: FlyByWire's own failure has no severity.</p>
          ) : (
            <div className="flex flex-row items-center">
              <span className="mr-3 shrink-0 text-sm text-theme-unselected">Severity</span>
              <input
                type="range"
                min={0}
                max={100}
                step={1}
                value={magnitudePercent}
                onChange={(event) => setMagnitudePercent(Number(event.target.value))}
                className="mr-4 grow"
              />
              <span className="w-12 shrink-0 text-right font-mono text-sm">{magnitudePercent}%</span>
            </div>
          )}

          <div className="mt-2 flex flex-row items-center">
            <span className="mr-3 shrink-0 text-sm text-theme-unselected">When</span>
            <SelectInput
              className="mr-2 w-96"
              value={trigger}
              options={SCHEDULE_TRIGGER_OPTIONS}
              onChange={(value) => setTrigger(value as ScheduleTrigger)}
            />
            {trigger === ScheduleTrigger.FlightPhase && (
              <SelectInput
                className="w-48"
                value={phase}
                options={FLIGHT_PHASE_OPTIONS}
                onChange={(value) => setPhase(value as number)}
              />
            )}
            {trigger !== ScheduleTrigger.Now && trigger !== ScheduleTrigger.FlightPhase && (
              <SimpleInput
                className="w-32"
                number
                min={0}
                placeholder="value"
                value={triggerValue}
                onChange={(value) => setTriggerValue(value)}
              />
            )}
          </div>

          <div className="mt-2 flex flex-row">
            <button
              type="button"
              disabled={!triggerValid}
              onClick={handleArm}
              className={`mr-2 rounded-md border-2 border-utility-green px-3 py-1 text-sm text-utility-green transition duration-100 hover:bg-utility-green hover:text-theme-body ${
                triggerValid ? '' : 'opacity-50'
              }`}
            >
              {armLabel()}
            </button>
            {isArmed && (
              <button
                type="button"
                onClick={handleClear}
                className="mr-2 rounded-md border-2 border-utility-red px-3 py-1 text-sm text-utility-red transition duration-100 hover:bg-utility-red hover:text-theme-body"
              >
                Clear
              </button>
            )}
            {isScheduled && (
              <button
                type="button"
                onClick={handleUnschedule}
                className="rounded-md border-2 border-utility-red px-3 py-1 text-sm text-utility-red transition duration-100 hover:bg-utility-red hover:text-theme-body"
              >
                {pending?.action === 'unschedule' ? 'Cancelling…' : 'Cancel schedule'}
              </button>
            )}
          </div>

          {outcome && (
            <p className={`mt-2 text-sm ${outcome.ok ? 'text-utility-green' : 'text-utility-red'}`}>{outcome.text}</p>
          )}

          <div className="mt-3 border-t border-theme-accent pt-2">
            <h3 className="mb-1 text-sm font-bold">Consequences</h3>
            {onOff ? (
              <p className="text-sm text-theme-unselected">
                One of FlyByWire's own failures: FlyByWire's systems carry out its effects directly.
              </p>
            ) : (
              <ConsequenceChart kind="failure" id={String(failure.id)} root={failure.name} />
            )}
          </div>
        </div>
      )}
    </div>
  );
};

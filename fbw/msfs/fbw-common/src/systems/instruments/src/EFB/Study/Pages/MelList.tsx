// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React from 'react';
import { useSimVar, useSimVarList } from '@flybywiresim/fbw-sdk-react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import { Catalogue, CatalogueMelEntry, byChapter, chapterNames } from '../catalogue';

interface MelListProps {
  catalogue: Catalogue;
  query: string;
}

const MEL_ITEM_COUNT = 64;

enum MelCommandResult {
  None = 0,
  Applied = 1,
  Unknown = 3,
}

function describeMelCommandResult(result: MelCommandResult): string {
  switch (result) {
    case MelCommandResult.Applied:
      return 'Applied.';
    case MelCommandResult.Unknown:
      return 'Rejected: not a known MEL item.';
    default:
      return '';
  }
}

const DEFERRED_NAMES = Array.from({ length: MEL_ITEM_COUNT }, (_, k) => `L:A32NX_DEEP_MEL_ITEM_${k}_DEFERRED`);
const REMAINING_NAMES = Array.from({ length: MEL_ITEM_COUNT }, (_, k) => `L:A32NX_DEEP_MEL_ITEM_${k}_REMAINING_HOURS`);
const DEFERRED_UNITS = DEFERRED_NAMES.map(() => 'number' as const);
const REMAINING_UNITS = REMAINING_NAMES.map(() => 'number' as const);

interface MelDeferralState {
  deferred: ReadonlySet<number>;
  remainingHours: ReadonlyMap<number, number>;
  lastResult: MelCommandResult;
  defer: (item: number) => void;
  release: (item: number) => void;
}

function useMelDeferred(): MelDeferralState {
  const [deferredFlags] = useSimVarList(DEFERRED_NAMES, DEFERRED_UNITS, 1_000);
  const [remaining] = useSimVarList(REMAINING_NAMES, REMAINING_UNITS, 1_000);
  const [result] = useSimVar('L:A32NX_DEEP_MEL_CMD_RESULT', 'number', 500);

  const deferred = React.useMemo(() => {
    const set = new Set<number>();
    deferredFlags.forEach((v, item) => {
      if (v !== 0) {
        set.add(item);
      }
    });
    return set;
  }, [deferredFlags]);

  const remainingHours = React.useMemo(() => new Map<number, number>(remaining.map((v, item): [number, number] => [item, Number(v)])), [remaining]);

  const defer = (item: number): void => {
    SimVar.SetSimVarValue('L:A32NX_DEEP_MEL_CMD_ITEM', 'number', item);
    SimVar.SetSimVarValue('L:A32NX_DEEP_MEL_CMD', 'number', 1);
  };
  const release = (item: number): void => {
    SimVar.SetSimVarValue('L:A32NX_DEEP_MEL_CMD_ITEM', 'number', item);
    SimVar.SetSimVarValue('L:A32NX_DEEP_MEL_CMD', 'number', 2);
  };

  return { deferred, remainingHours, lastResult: result as MelCommandResult, defer, release };
}

export const MelList = ({ catalogue, query }: MelListProps) => {
  const melState = useMelDeferred();
  const [onGround] = useSimVar('SIM ON GROUND', 'bool', 1_000);
  const failuresById = new Map(catalogue.failures.map((failure) => [failure.id, failure]));
  const itemIndexByRef = React.useMemo(() => new Map(catalogue.melEntries.map((e, i) => [e.melReference, i])), [catalogue]);

  const matches = (entry: CatalogueMelEntry): boolean => {
    if (query === '') {
      return true;
    }
    if (entry.melReference.toUpperCase().includes(query)) {
      return true;
    }
    return entry.failures.some((id) => failuresById.get(id)?.name.toUpperCase().includes(query));
  };

  const filtered = catalogue.melEntries.filter(matches);
  const grouped = byChapter(filtered);
  const names = chapterNames(catalogue);

  return (
    <ScrollableContainer height={43}>
      {!onGround && (
        <p className="mb-3 rounded-md border border-utility-amber px-4 py-2 text-sm text-utility-amber">
          Deferring an item under the MEL is a dispatch action, on the ground before departure. In the air, Defer
          still activates the failure (its real effect), but it is not a real MEL deferral.
        </p>
      )}
      {filtered.length === 0 && <p className="mt-4 text-theme-unselected">Nothing matches that search.</p>}

      {[...grouped.entries()].map(([chapter, entries]) => (
        <div key={chapter} className="mb-3">
          <h2 className="sticky top-0 bg-theme-body py-1 font-bold">{names.get(chapter) ?? `ATA ${chapter}`}</h2>

          <div className="space-y-1">
            {entries.map((entry) => (
              <MelEntryRow
                key={entry.melReference}
                entry={entry}
                failuresById={failuresById}
                melState={melState}
                item={itemIndexByRef.get(entry.melReference) ?? -1}
              />
            ))}
          </div>
        </div>
      ))}
    </ScrollableContainer>
  );
};

interface MelEntryRowProps {
  entry: CatalogueMelEntry;
  failuresById: Map<number, Catalogue['failures'][number]>;
  melState: MelDeferralState;
  item: number;
}

const MelEntryRow = ({ entry, failuresById, melState, item }: MelEntryRowProps) => {
  const [awaitingResult, setAwaitingResult] = React.useState(false);
  const [shownResult, setShownResult] = React.useState<MelCommandResult | null>(null);

  const failureChapters = new Set(entry.failures.map((id) => failuresById.get(id)?.ataChapterNumber).filter((n) => n !== undefined));
  const crossChapter = !failureChapters.has(entry.ataChapterNumber);

  const deferred = item >= 0 && melState.deferred.has(item);
  const remainingHours = item >= 0 ? (melState.remainingHours.get(item) ?? 0) : 0;

  React.useEffect(() => {
    if (awaitingResult) {
      setShownResult(melState.lastResult);
      setAwaitingResult(false);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [melState.lastResult]);

  const defer = (): void => {
    melState.defer(item);
    setShownResult(null);
    setAwaitingResult(true);
  };
  const repair = (): void => {
    melState.release(item);
    setShownResult(null);
    setAwaitingResult(true);
  };

  return (
    <div className={`rounded-md border px-4 py-2 ${deferred ? 'border-utility-amber' : 'border-theme-accent'}`}>
      <div className="flex flex-row items-baseline justify-between space-x-4">
        <span className="flex flex-row items-baseline">
          <span className="mr-2 font-mono font-bold">{entry.melReference}</span>
          {deferred && (
            <span className="rounded-sm bg-utility-amber px-2 py-0.5 text-xs font-bold text-theme-body">
              DEFERRED -- INOP{remainingHours > 0 ? ` (${remainingHours.toFixed(1)} h left)` : ' (expired)'}
            </span>
          )}
        </span>
        <span className="shrink-0 text-sm text-theme-unselected">
          {entry.failures.length} {entry.failures.length === 1 ? 'failure' : 'failures'}
        </span>
      </div>

      <div className="mt-1 space-y-0.5">
        {entry.failures.map((id) => {
          const failure = failuresById.get(id);
          return (
            <div key={id} className="flex flex-row items-baseline justify-between space-x-4 text-sm">
              <span className={deferred ? 'text-utility-amber' : ''}>{failure?.name ?? `Failure ${id}`}</span>
              <span className="shrink-0 font-mono text-theme-unselected">{id}</span>
            </div>
          );
        })}
      </div>

      {crossChapter && (
        <p className="mt-2 text-xs text-theme-unselected">
          Filed under ATA {entry.ataChapterNumber}, covering failures in {[...failureChapters].sort((a, b) => a - b).join(', ')}.
        </p>
      )}

      <div className="mt-2 flex flex-row border-t border-theme-accent pt-2">
        {!deferred && (
          <button
            type="button"
            onClick={defer}
            disabled={item < 0}
            className="mr-2 rounded-md border-2 border-utility-amber px-3 py-1 text-sm text-utility-amber transition duration-100 hover:bg-utility-amber hover:text-theme-body"
          >
            Defer -- placard INOP
          </button>
        )}
        {deferred && (
          <button
            type="button"
            onClick={repair}
            className="rounded-md border-2 border-utility-green px-3 py-1 text-sm text-utility-green transition duration-100 hover:bg-utility-green hover:text-theme-body"
          >
            Repair
          </button>
        )}
      </div>

      {shownResult !== null && shownResult !== MelCommandResult.None && (
        <p className={`mt-2 text-xs ${shownResult === MelCommandResult.Applied ? 'text-utility-green' : 'text-utility-red'}`}>
          {describeMelCommandResult(shownResult)}
        </p>
      )}
    </div>
  );
};

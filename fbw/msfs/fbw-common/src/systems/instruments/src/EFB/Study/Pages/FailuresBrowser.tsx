// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React, { useState } from 'react';
import { ChevronDown, ChevronRight } from 'react-bootstrap-icons';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import { Catalogue, CatalogueFailure, byChapter, chapterNames } from '../catalogue';
import { useArmedFailures } from '../armedFailures';
import { useScheduledFailures } from '../scheduledFailures';
import { FailureRow } from '../FailureRow';
import { ArmedFailuresSummary } from '../ArmedFailuresSummary';

interface FailuresBrowserProps {
  catalogue: Catalogue;
  query: string;
}

export const FailuresBrowser = ({ catalogue, query }: FailuresBrowserProps) => {
  const [openChapters, setOpenChapters] = useState<Set<number>>(new Set());
  const armedState = useArmedFailures();
  const scheduledState = useScheduledFailures();

  const matches = (failure: CatalogueFailure): boolean => {
    if (query === '') {
      return true;
    }
    return (
      failure.name.toUpperCase().includes(query) ||
      failure.component.toUpperCase().includes(query) ||
      failure.cause.toUpperCase().includes(query) ||
      failure.id.toString().includes(query)
    );
  };

  const filtered = catalogue.failures.filter(matches);
  const grouped = byChapter(filtered);
  const names = chapterNames(catalogue);

  const toggle = (chapter: number) => {
    const next = new Set(openChapters);
    if (next.has(chapter)) {
      next.delete(chapter);
    } else {
      next.add(chapter);
    }
    setOpenChapters(next);
  };

  const expandAll = query !== '' && filtered.length <= 60;

  return (
    <>
      <ArmedFailuresSummary catalogue={catalogue} armedState={armedState} scheduledState={scheduledState} />

      <ScrollableContainer height={41}>
        {filtered.length === 0 && <p className="mt-4 text-theme-unselected">Nothing matches that search.</p>}

        {[...grouped.entries()].map(([chapter, failures]) => {
          const open = expandAll || openChapters.has(chapter);

          return (
            <div key={chapter} className="mb-2">
              <button
                type="button"
                className="flex w-full flex-row items-center justify-between rounded-md bg-theme-accent px-4 py-2 text-left transition duration-100 hover:bg-theme-highlight hover:text-theme-body"
                onClick={() => toggle(chapter)}
              >
                <div className="flex flex-row items-center space-x-3">
                  {open ? <ChevronDown size={20} /> : <ChevronRight size={20} />}
                  <span className="font-bold">{names.get(chapter) ?? `ATA ${chapter}`}</span>
                </div>
                <span className="text-sm opacity-75">{failures.length}</span>
              </button>

              {open && (
                <div className="mt-1 space-y-1">
                  {failures.map((failure) => (
                    <FailureRow key={failure.id} failure={failure} armedState={armedState} scheduledState={scheduledState} />
                  ))}
                </div>
              )}
            </div>
          );
        })}
      </ScrollableContainer>
    </>
  );
};

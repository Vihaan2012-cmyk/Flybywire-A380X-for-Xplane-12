// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React, { useState } from 'react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import { Catalogue, CatalogueComponent, byChapter, chapterNames } from '../catalogue';
import { useArmedFailures } from '../armedFailures';
import { useScheduledFailures } from '../scheduledFailures';
import { FailureRow } from '../FailureRow';

interface ComponentsBrowserProps {
  catalogue: Catalogue;
  query: string;
}

export const ComponentsBrowser = ({ catalogue, query }: ComponentsBrowserProps) => {
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const armedState = useArmedFailures();
  const scheduledState = useScheduledFailures();

  const matches = (component: CatalogueComponent): boolean => {
    if (query === '') {
      return true;
    }
    return component.name.toUpperCase().includes(query) || component.id.toUpperCase().includes(query);
  };

  const filtered = catalogue.components.filter(matches);
  const grouped = byChapter(filtered);
  const names = chapterNames(catalogue);

  const selected = filtered.find((component) => component.id === selectedId) ?? null;
  const failuresById = new Map(catalogue.failures.map((failure) => [failure.id, failure]));

  return (
    <div className="flex flex-row space-x-4">
      <div className="w-1/2 shrink-0">
        <ScrollableContainer height={43}>
          {filtered.length === 0 && <p className="mt-4 text-theme-unselected">Nothing matches that search.</p>}

          {[...grouped.entries()].map(([chapter, components]) => (
            <div key={chapter} className="mb-3">
              <h2 className="sticky top-0 bg-theme-body py-1 font-bold">{names.get(chapter) ?? `ATA ${chapter}`}</h2>

              <div className="space-y-1">
                {components.map((component) => (
                  <button
                    key={component.id}
                    type="button"
                    onClick={() => setSelectedId(component.id)}
                    className={`flex w-full flex-row items-baseline justify-between space-x-4 rounded-md border px-3 py-2 text-left transition duration-100 ${
                      selectedId === component.id
                        ? 'border-theme-highlight bg-theme-highlight text-theme-body'
                        : 'border-theme-accent bg-transparent hover:border-theme-highlight'
                    }`}
                  >
                    <span>
                      {component.name}
                      {component.instance !== null && <span className="opacity-75"> {component.instance}</span>}
                    </span>
                    <span className="shrink-0 text-sm opacity-75">{component.failures.length}</span>
                  </button>
                ))}
              </div>
            </div>
          ))}
        </ScrollableContainer>
      </div>

      <div className="grow rounded-md border border-theme-accent p-4">
        {selected === null ? (
          <p className="text-theme-unselected">Select a component.</p>
        ) : (
          <ScrollableContainer height={41}>
            <h2 className="font-bold">{selected.name}</h2>

            <div className="mt-3 space-y-1">
              {selected.failures.map((id) => {
                const failure = failuresById.get(id);
                if (!failure) {
                  return null;
                }
                return <FailureRow key={id} failure={failure} armedState={armedState} scheduledState={scheduledState} />;
              })}
            </div>
          </ScrollableContainer>
        )}
      </div>
    </div>
  );
};

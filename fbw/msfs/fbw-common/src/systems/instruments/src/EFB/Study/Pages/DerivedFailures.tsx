// Copyright (c) 2023-2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React from 'react';
import { useSimVar } from '@flybywiresim/fbw-sdk-react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import { DEPTH_REFRESH_MS } from './DepthField';
import { DERIVED_FAILURE_TABLE, DerivedFailureEntry } from '../derivedFailures';

const DerivedFailureRow = ({ entry }: { entry: DerivedFailureEntry }) => {
  const [magnitude] = useSimVar(`L:A32NX_DEEP_DERIVED_FBW_FAILURE_${entry.id}`, 'number', DEPTH_REFRESH_MS);
  if (!(magnitude > 0)) {
    return null;
  }
  return (
    <div className="flex flex-row items-center justify-between space-x-4 border-b border-theme-accent/40 py-1.5 text-sm last:border-b-0">
      <div>
        <p className="font-bold">{entry.name}</p>
        <p className="text-xs text-theme-unselected" title={entry.component}>
          {entry.area} · {entry.component} · id {entry.id}
        </p>
      </div>
      <span className="font-mono text-utility-amber">{magnitude.toFixed(2)}</span>
    </div>
  );
};

export const DerivedFailures = () => {
  const active = DERIVED_FAILURE_TABLE;
  return (
    <ScrollableContainer height={43}>

      <div className="rounded-md border border-theme-accent p-3">
        {active.map((entry) => (
          <DerivedFailureRow key={entry.id} entry={entry} />
        ))}
      </div>
    </ScrollableContainer>
  );
};

// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React from 'react';
import { useSimVar } from '@flybywiresim/fbw-sdk-react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';

const REFRESH_MS = 1_000;

export const WeightBalance = () => {
  const [zfwKg] = useSimVar('L:A32NX_AIRFRAME_ZFW', 'number', REFRESH_MS);
  const [zfwCg] = useSimVar('L:A32NX_AIRFRAME_ZFW_CG_PERCENT_MAC', 'number', REFRESH_MS);
  const [gwKg] = useSimVar('L:A32NX_AIRFRAME_GW', 'number', REFRESH_MS);
  const [gwCg] = useSimVar('L:A32NX_AIRFRAME_GW_CG_PERCENT_MAC', 'number', REFRESH_MS);
  const [towKg] = useSimVar('L:A32NX_AIRFRAME_TOW', 'number', REFRESH_MS);
  const [toCg] = useSimVar('L:A32NX_AIRFRAME_TO_CG_PERCENT_MAC', 'number', REFRESH_MS);
  const [targetZfwKg] = useSimVar('L:A32NX_AIRFRAME_ZFW_DESIRED', 'number', REFRESH_MS);
  const [targetZfwCg] = useSimVar('L:A32NX_AIRFRAME_ZFW_CG_PERCENT_MAC_DESIRED', 'number', REFRESH_MS);
  const [totalWeightLb] = useSimVar('TOTAL WEIGHT', 'pounds', REFRESH_MS);

  const row = (label: string, value: string, desired?: string) => (
    <div className="flex flex-row items-center justify-between border-b border-theme-accent py-2 last:border-b-0">
      <span className="text-theme-text">{label}</span>
      <span className="font-mono text-theme-highlight">
        {value}
        {desired !== undefined && desired !== value && (
          <span className="ml-2 text-sm text-theme-unselected">/ {desired} boarding</span>
        )}
      </span>
    </div>
  );

  const kg = (v: number | undefined) => (v !== undefined ? `${Math.round(v).toLocaleString()} kg` : '---');
  const mac = (v: number | undefined) => (v !== undefined ? `${v.toFixed(1)} % MAC` : '---');

  const towEqualsGw = towKg !== undefined && gwKg !== undefined && Math.abs(towKg - gwKg) < 1;

  return (
    <ScrollableContainer height={43}>
      <div className="rounded-md border border-theme-accent p-4">
        <h2 className="mb-2 font-bold">Weight and balance</h2>
        {row('Zero fuel weight', kg(zfwKg), targetZfwKg !== undefined ? kg(targetZfwKg) : undefined)}
        {row('Zero fuel CG', mac(zfwCg), targetZfwCg !== undefined ? mac(targetZfwCg) : undefined)}
        {row('Gross weight', kg(gwKg))}
        {row('Gross weight CG', mac(gwCg))}
        {row('Take-off weight', kg(towKg))}
        {row('Take-off CG', mac(toCg))}
        {row('X-Plane-equivalent total weight (MSFS TOTAL WEIGHT)', kg(totalWeightLb !== undefined ? totalWeightLb * 0.453_592_37 : undefined))}
        {towEqualsGw && (
          <p className="mt-3 rounded-md border border-utility-amber px-4 py-2 text-sm text-utility-amber">
            Take-off weight equals gross weight because FlyByWire does not yet subtract taxi fuel
            (airframe/mod.rs&apos;s own TODO). This is not this page inventing a figure -- it is what the systems
            module currently publishes.
          </p>
        )}
      </div>
    </ScrollableContainer>
  );
};

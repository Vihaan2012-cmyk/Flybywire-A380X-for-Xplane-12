// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React from 'react';
import { useSimVar } from '@flybywiresim/fbw-sdk-react';

export const DEPTH_REFRESH_MS = 750;

interface DepthFieldDef {
  label: string;
  lvar: string;
  unit?: string;
  decimals?: number;
  lamp?: boolean;
}

export const DepthNum = ({ label, lvar, unit, decimals = 1 }: DepthFieldDef) => {
  const [value] = useSimVar(`L:A32NX_${lvar}`, 'number', DEPTH_REFRESH_MS);
  return (
    <div className="flex flex-row items-baseline justify-between space-x-4 border-b border-theme-accent/40 py-1 text-sm last:border-b-0">
      <span className="text-theme-unselected" title={lvar}>
        {label}
      </span>
      <span className="font-mono">
        {Number.isFinite(value) ? value.toFixed(decimals) : '--'}
        {unit ? ` ${unit}` : ''}
      </span>
    </div>
  );
};

export const DepthLamp = ({ label, lvar }: { label: string; lvar: string }) => {
  const [value] = useSimVar(`L:A32NX_${lvar}`, 'number', DEPTH_REFRESH_MS);
  const active = value !== 0;
  return (
    <div className="flex flex-row items-center justify-between space-x-4 border-b border-theme-accent/40 py-1 text-sm last:border-b-0">
      <span className="text-theme-unselected" title={lvar}>
        {label}
      </span>
      <span className={`h-2.5 w-2.5 shrink-0 rounded-full ${active ? 'bg-utility-amber' : 'bg-theme-accent'}`} />
    </div>
  );
};

export const DepthGroup = ({ title, children }: { title: string; children: React.ReactNode }) => (
  <div className="rounded-md border border-theme-accent p-3">
    <h3 className="mb-1 font-bold">{title}</h3>
    <div>{children}</div>
  </div>
);

export const DepthGrid = ({ children }: { children: React.ReactNode }) => (
  <div className="grid grid-cols-2 gap-3">{children}</div>
);

// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React, { useEffect, useState } from 'react';
import { CatalogueAlert, loadCatalogue } from './catalogue';
import { ConsequenceChain, loadConsequences, profilePhrase, systemName, variablePhrase } from './consequences';

interface ConsequenceChartProps {
  kind: 'failure' | 'breaker';
  id: string;
  root: string;
  compact?: boolean;
}

const MAX_STAGES = 8;

const ALERT_COLOUR: Record<string, string> = {
  warning: 'border-utility-red text-utility-red',
  caution: 'border-utility-amber text-utility-amber',
};

type Loaded = { chain: ConsequenceChain | null; alerts: Map<string, CatalogueAlert> };

export const ConsequenceChart = ({ kind, id, root, compact = false }: ConsequenceChartProps) => {
  const [loaded, setLoaded] = useState<Loaded | null>(null);
  const [error, setError] = useState(false);

  useEffect(() => {
    let live = true;
    Promise.all([loadConsequences(), loadCatalogue()])
      .then(([consequences, catalogue]) => {
        if (!live) {
          return;
        }
        const table = kind === 'failure' ? consequences.failures : consequences.breakers;
        setLoaded({ chain: table[id] ?? null, alerts: new Map(catalogue.alerts.map((a) => [a.key, a])) });
      })
      .catch(() => live && setError(true));
    return () => {
      live = false;
    };
  }, [kind, id]);

  if (error) {
    return <p className="text-sm text-theme-unselected">Consequence data is not installed.</p>;
  }
  if (loaded === null) {
    return <p className="text-sm text-theme-unselected">Tracing consequences…</p>;
  }
  const { chain, alerts } = loaded;
  if (chain === null) {
    return (
      <p className="text-sm text-theme-unselected">
        No consequence the systems model can show in any of the flight conditions it was traced in.
      </p>
    );
  }

  if (chain.d !== undefined) {
    const hours = chain.d / 3600;
    return (
      <p className="text-sm text-theme-unselected">
        Develops over about {hours >= 1.5 ? `${Math.round(hours)} hours` : `${Math.round(chain.d / 60)} minutes`} of
        flight {profilePhrase(chain.p)} -- slower than the systems model traces, so it is not charted.
      </p>
    );
  }

  const stages = chain.s.slice(0, MAX_STAGES);
  const hiddenStages = chain.s.length - stages.length;

  return (
    <div className="flex flex-col items-stretch">
      <div className="rounded-md border-2 border-utility-amber px-3 py-2 text-center text-sm font-bold text-utility-amber">
        {root}
      </div>

      {stages.map((stage, i) => (
        <React.Fragment key={i}>
          <Connector label={i === 0 ? 'immediately' : 'then'} />
          <div className={compact ? 'flex flex-col' : 'flex flex-row flex-wrap justify-center'}>
            {stage.map(([area, moved, examples]) => (
              <div key={area} className={`${compact ? 'my-0.5' : 'm-1 w-44'} rounded-md border border-theme-highlight px-2 py-1`}>
                <div className="text-sm font-bold text-theme-highlight">{systemName(area)}</div>
                <ul className="mt-0.5 text-xs">
                  {examples.map((name) => (
                    <li key={name}>{variablePhrase(name)}</li>
                  ))}
                </ul>
                {moved > examples.length && (
                  <div className="mt-0.5 text-xs text-theme-unselected">and {moved - examples.length} more</div>
                )}
              </div>
            ))}
          </div>
        </React.Fragment>
      ))}
      {hiddenStages > 0 && (
        <p className="mt-1 text-center text-xs text-theme-unselected">
          …and {hiddenStages} further wave{hiddenStages === 1 ? '' : 's'}
        </p>
      )}

      {chain.a.length > 0 && (
        <>
          <Connector label="ECAM" />
          <div className={compact ? 'flex flex-col' : 'flex flex-row flex-wrap justify-center'}>
            {chain.a.map((key) => {
              const alert = alerts.get(key);
              return (
                <div
                  key={key}
                  className={`m-1 rounded-md border-2 px-2 py-1 font-mono text-sm ${ALERT_COLOUR[alert?.level ?? ''] ?? 'border-utility-green text-utility-green'}`}
                >
                  {alert?.title ?? key.replace(/_/g, ' ')}
                </div>
              );
            })}
          </div>
        </>
      )}

      {chain.f.length > 0 && (
        <>
          <Connector label="FlyByWire systems" />
          <div className="rounded-md border border-theme-accent px-3 py-2 text-sm">
            {chain.f.map(([fbwId, component, reason]) => (
              <div key={fbwId} className="flex flex-row justify-between space-x-4">
                <span>{reason}</span>
                <span className="shrink-0 font-mono text-xs text-theme-unselected">{component}</span>
              </div>
            ))}
          </div>
        </>
      )}

      <p className="mt-2 text-center text-xs text-theme-unselected">Traced by the systems model {profilePhrase(chain.p)}.</p>
    </div>
  );
};

const Connector = ({ label }: { label: string }) => (
  <div className="flex flex-col items-center py-0.5">
    <div className="h-2 w-px bg-theme-unselected" />
    <span className="text-xs text-theme-unselected">{label}</span>
    <span className="-mt-1 text-xs leading-none text-theme-unselected">▼</span>
  </div>
);

// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React, { useState } from 'react';
import { useSimVar } from '@flybywiresim/fbw-sdk-react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import { SimpleInput } from '../../UtilComponents/Form/SimpleInput/SimpleInput';
import { ConsequenceChart } from '../ConsequenceChart';
import { Catalogue, CatalogueBreaker, humanisePanel, lvarKey, panelGrid } from '../catalogue';
import { unitIsWired, useWiring, Wiring } from '../wiring';

interface BreakerPanelsProps {
  catalogue: Catalogue;
  query: string;
}

enum BreakerStatus {
  Closed = 0,
  OpenByCommand = 1,
  TrippedThermal = 2,
  TrippedMagnetic = 3,
  TrippedArcFault = 4,
  LockedOut = 5,
}

const STATUS_LABEL: Record<number, string> = {
  [BreakerStatus.Closed]: 'Closed',
  [BreakerStatus.OpenByCommand]: 'Open',
  [BreakerStatus.TrippedThermal]: 'Tripped (thermal)',
  [BreakerStatus.TrippedMagnetic]: 'Tripped (magnetic)',
  [BreakerStatus.TrippedArcFault]: 'Tripped (arc fault)',
  [BreakerStatus.LockedOut]: 'LOCKED OUT (maintenance)',
};

const STATUS_COLOUR: Record<number, string> = {
  [BreakerStatus.Closed]: 'text-utility-green',
  [BreakerStatus.OpenByCommand]: 'text-utility-amber',
  [BreakerStatus.TrippedThermal]: 'text-utility-amber',
  [BreakerStatus.TrippedMagnetic]: 'text-utility-amber',
  [BreakerStatus.TrippedArcFault]: 'text-utility-amber',
  [BreakerStatus.LockedOut]: 'text-utility-red',
};

const statusLabel = (status: number): string => STATUS_LABEL[status] ?? `Unknown (${status})`;
const statusColour = (status: number): string => STATUS_COLOUR[status] ?? 'text-theme-unselected';

const REFRESH_MS = 500;

const writeBreakerCmd = (id: string, cmd: 1 | 2): void => {
  SimVar.SetSimVarValue(`L:A32NX_BKR_${lvarKey(id)}_CMD`, 'number', cmd);
};

const Stat = ({ label, value, className, title }: { label: string; value: string; className?: string; title?: string }) => (
  <div className={`mb-1 mr-8 flex flex-row items-baseline ${className ?? ''}`} title={title}>
    <span className="mr-2 text-theme-unselected">{label}</span>
    <span className="font-bold">{value}</span>
  </div>
);

const SummaryStrip = () => {
  const [total] = useSimVar('L:A32NX_BREAKERS_TOTAL', 'number', REFRESH_MS);
  const [open] = useSimVar('L:A32NX_BREAKERS_OPEN_COUNT', 'number', REFRESH_MS);
  const [tripped] = useSimVar('L:A32NX_BREAKERS_TRIPPED_NOT_COMMANDED_COUNT', 'number', REFRESH_MS);
  const [lockedOut] = useSimVar('L:A32NX_BREAKERS_LOCKED_OUT_COUNT', 'number', REFRESH_MS);
  const [unprotected] = useSimVar('L:A32NX_BREAKERS_PROTECTING_NO_MODELLED_LOAD', 'number', REFRESH_MS);
  const [cbMonitoringFault] = useSimVar('L:A32NX_BREAKERS_CB_MONITORING_FAULT', 'number', REFRESH_MS);
  const [emerCbMonitoringFault] = useSimVar('L:A32NX_BREAKERS_EMER_CB_MONITORING_FAULT', 'number', REFRESH_MS);
  const [remoteCtlActive] = useSimVar('L:A32NX_BREAKERS_REMOTE_CTL_ACTIVE', 'number', REFRESH_MS);
  const [arcHeat] = useSimVar('L:A32NX_WIRING_TOTAL_ARC_HEAT_W', 'number', REFRESH_MS);

  return (
    <div className="mb-3 flex flex-row flex-wrap items-baseline rounded-md border border-theme-accent px-3 pt-2 text-sm">
      <Stat label="Breakers" value={total.toFixed(0)} />
      <Stat label="Opened" value={open.toFixed(0)} className={open > 0 ? 'text-utility-amber' : ''} title="Opened by command" />
      <Stat label="Tripped" value={tripped.toFixed(0)} className={tripped > 0 ? 'text-utility-red' : ''} title="Tripped by themselves" />
      <Stat label="Locked out" value={lockedOut.toFixed(0)} className={lockedOut > 0 ? 'text-utility-red' : ''} title="Locked out after repeated trips" />
      <Stat label="Protect nothing modelled" value={unprotected.toFixed(0)} title="Opening these changes nothing in the model" />
      {(cbMonitoringFault > 0 || emerCbMonitoringFault > 0) && <div className="mb-1 mr-8 text-utility-red">C/B monitoring fault</div>}
      {remoteCtlActive > 0 && <div className="mb-1 mr-8 text-utility-amber">Remote C/B control on</div>}
      {arcHeat > 0 && <Stat label="Arc heat" value={`${arcHeat.toFixed(0)} W`} className="text-utility-red" />}
    </div>
  );
};

export const BreakerPanels = ({ catalogue, query }: BreakerPanelsProps) => {
  const wiring = useWiring();
  const [selected, setSelected] = useState<CatalogueBreaker | null>(null);
  const [search, setSearch] = useState('');

  const words = `${query} ${search}`.toUpperCase().split(/\s+/).filter((w) => w !== '');

  const matches = (breaker: CatalogueBreaker): boolean => {
    const haystack = `${breaker.label} ${breaker.name} ${breaker.consumer} ${breaker.bus} ${humanisePanel(breaker.panel)} ATA ${breaker.ataChapterNumber}`.toUpperCase();
    return words.every((w) => haystack.includes(w));
  };

  const filtered = catalogue.breakers.filter(matches);

  const panels = new Map<string, CatalogueBreaker[]>();
  filtered.forEach((breaker) => {
    const existing = panels.get(breaker.panel);
    if (existing) {
      existing.push(breaker);
    } else {
      panels.set(breaker.panel, [breaker]);
    }
  });

  return (
    <div className="flex flex-col">
      <SummaryStrip />
      <div className="mb-3 flex flex-row items-center">
        <SimpleInput placeholder="Search breakers: name, bus, ATA" className="mr-4 w-96 uppercase" value={search} onChange={setSearch} />
        <span className="text-sm text-theme-unselected">
          {filtered.length} of {catalogue.breakers.length}
        </span>
      </div>
      <div className="flex flex-row space-x-4">
        <div className="grow">
          <ScrollableContainer height={40}>
          {filtered.length === 0 && <p className="mt-4 text-theme-unselected">Nothing matches that search.</p>}

          {[...panels.entries()].sort(([a], [b]) => a.localeCompare(b)).map(([panel, breakers]) => (
            <div key={panel} className="mb-4">
              <div className="flex flex-row items-baseline justify-between">
                <h2 className="font-bold">{humanisePanel(panel)}</h2>
                <span className="text-sm text-theme-unselected">{breakers.length}</span>
              </div>

              <div className="mt-2 space-y-1 rounded-md border border-theme-accent p-2">
                {[...panelGrid(breakers).entries()].map(([row, inRow]) => (
                  <div key={row} className="flex flex-row flex-wrap">
                    {inRow.map((breaker) => (
                      <BreakerButton
                        key={breaker.id}
                        breaker={breaker}
                        isSelected={selected?.id === breaker.id}
                        onSelect={() => setSelected(breaker)}
                        wiring={wiring}
                      />
                    ))}
                  </div>
                ))}
              </div>
            </div>
          ))}
        </ScrollableContainer>
      </div>

        <div className="w-96 shrink-0 rounded-md border border-theme-accent p-4">
          <ScrollableContainer height={40} innerClassName="pr-4">
            {selected === null ? (
              <p className="text-theme-unselected">Select a breaker.</p>
            ) : (
              <SelectedBreakerDetail breaker={selected} catalogue={catalogue} wiring={wiring} />
            )}
          </ScrollableContainer>
        </div>
      </div>
    </div>
  );
};

interface BreakerButtonProps {
  breaker: CatalogueBreaker;
  isSelected: boolean;
  onSelect: () => void;
  wiring: Wiring | null;
}

const BreakerButton = ({ breaker, isSelected, onSelect, wiring }: BreakerButtonProps) => {
  const key = lvarKey(breaker.id);
  const [status] = useSimVar(`L:A32NX_BKR_${key}_STATUS`, 'number', REFRESH_MS);
  const [current] = useSimVar(`L:A32NX_BKR_${key}_CURRENT_A`, 'number', REFRESH_MS);
  const wired = unitIsWired(wiring, breaker.id);

  return (
    <button
      type="button"
      onClick={onSelect}
      title={`${breaker.consumer} — ${breaker.ratingAmperes} A on ${breaker.bus} — ${statusLabel(status)}${wired ? ' — opening it reaches the cockpit' : ''}`}
      className={`relative mb-1 mr-1 w-28 shrink-0 rounded-sm border px-1 py-1 text-center text-xs transition duration-100 ${
        isSelected
          ? 'border-theme-highlight bg-theme-highlight text-theme-body'
          : 'border-theme-accent hover:border-theme-highlight'
      } ${breaker.protectsModelledLoad ? '' : 'opacity-50'}`}
    >
      {wired && <span className="absolute right-1 top-1 h-1.5 w-1.5 rounded-full bg-utility-green" />}
      <div className="h-8 overflow-hidden whitespace-normal font-mono leading-4">{breaker.name}</div>
      <div className={isSelected ? '' : statusColour(status)}>{statusLabel(status)}</div>
      <div className="opacity-75">{current.toFixed(1)} A</div>
    </button>
  );
};

const SelectedBreakerDetail = ({ breaker, catalogue, wiring }: { breaker: CatalogueBreaker; catalogue: Catalogue; wiring: Wiring | null }) => {
  const key = lvarKey(breaker.id);
  const [status] = useSimVar(`L:A32NX_BKR_${key}_STATUS`, 'number', REFRESH_MS);
  const [current] = useSimVar(`L:A32NX_BKR_${key}_CURRENT_A`, 'number', REFRESH_MS);

  const lockedOut = status === BreakerStatus.LockedOut;
  const model = describeWiring(wiring, breaker.id, catalogue);

  return (
    <>
      <h2 className="font-bold">{breaker.label}</h2>
      <p className="mt-1 text-sm text-theme-unselected">{breaker.consumer}</p>

      <dl className="mt-4 space-y-2 text-sm">
        <Row label="Panel" value={humanisePanel(breaker.panel)} />
        <Row label="Position" value={`Row ${breaker.row}, column ${breaker.column}`} />
        <Row label="Bus" value={breaker.bus} />
        <Row label="Rating" value={`${breaker.ratingAmperes} A`} />
        <Row label="Type" value={breaker.kind.replace(/([A-Z])/g, ' $1').toLowerCase()} />
        <Row label="State" value={statusLabel(status)} className={statusColour(status)} />
        <Row label="Current" value={`${current.toFixed(1)} A`} />
        <Row label="Opening it" value={model.text} className={model.className} />
      </dl>

      <div className="mt-4 flex flex-row space-x-2">
        <button
          type="button"
          onClick={() => writeBreakerCmd(breaker.id, 1)}
          disabled={status === BreakerStatus.OpenByCommand}
          className="grow rounded-md border-2 border-utility-amber px-3 py-2 text-center text-utility-amber transition duration-100 hover:bg-utility-amber hover:text-theme-body disabled:opacity-40 disabled:hover:bg-transparent disabled:hover:text-utility-amber"
        >
          Open
        </button>
        <button
          type="button"
          onClick={() => writeBreakerCmd(breaker.id, 2)}
          className="grow rounded-md border-2 border-utility-green px-3 py-2 text-center text-utility-green transition duration-100 hover:bg-utility-green hover:text-theme-body"
        >
          Close / Reset
        </button>
      </div>

      {lockedOut && (
        <p className="mt-2 text-xs text-utility-red">
          LOCKED OUT (maintenance) after repeated trips. Close/Reset will not clear it -- the WASM refuses the
          command until the unit is serviced.
        </p>
      )}

      <p className="mt-4 border-t border-theme-accent pt-3 text-xs text-theme-unselected">{breaker.basis}</p>

      {!breaker.protectsModelledLoad && (
        <p className="mt-2 text-xs text-utility-amber">
          This breaker exists on the aircraft but protects nothing the model currently solves, so opening it would
          change nothing.
        </p>
      )}

      <div className="mt-4 border-t border-theme-accent pt-3">
        <h3 className="mb-1 text-sm font-bold">If this breaker opens</h3>
        <ConsequenceChart kind="breaker" id={breaker.id} root={`${breaker.label} open`} compact />
      </div>
    </>
  );
};

const Row = ({
  label,
  value,
  muted,
  className,
}: {
  label: string;
  value: string;
  muted?: boolean;
  className?: string;
}) => (
  <div className="flex flex-row justify-between space-x-4">
    <dt className="text-theme-unselected">{label}</dt>
    <dd className={`text-right ${muted ? 'italic text-theme-unselected' : ''} ${className ?? ''}`}>{value}</dd>
  </div>
);

function describeWiring(wiring: Wiring | null, unit: string, catalogue: Catalogue): { text: string; className: string } {
  if (!wiring) {
    return { text: 'Loading…', className: 'text-theme-unselected' };
  }
  const w = wiring.units[unit];
  if (!w) {
    return { text: 'Not in the wiring table', className: 'text-utility-red' };
  }
  const ecam = w.ecam ?? [];
  if (w.failures.length === 0 && w.gates.length === 0 && ecam.length === 0) {
    return { text: w.none ? `Nothing on the real aircraft: ${w.none}` : 'Nothing', className: 'text-theme-unselected' };
  }
  const names = w.failures.map((id) => catalogue.failures.find((f) => f.id === id)?.name ?? `failure ${id}`);
  if (w.gates.length > 0) {
    names.unshift('cuts its FlyByWire system');
  }
  for (const title of ecam) {
    names.push(`ECAM ${title}`);
  }
  const shown = names.slice(0, 4).join('; ');
  const more = names.length > 4 ? `; and ${names.length - 4} more` : '';
  return { text: `${shown}${more}`, className: 'text-utility-green' };
}

// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React, { useEffect, useRef, useState } from 'react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import {
  CutawayShape,
  DiagramField,
  DiagramPage,
  StudyPages,
  Topology,
  collectNames,
  evalGate,
  fieldLabel,
  formatValue,
  loadStudyPages,
} from '../studyDiagrams';

const POLL_MS = 500;

const C = {
  line: '#9a9a9a',
  lineDim: '#5a5a5a',
  text: '#f2f2f2',
  textDim: '#a8a8a8',
  accent: '#3fb0ff',
  ok: '#3ccf6e',
  nodeBg: 'rgba(36,36,36,0.92)',
  busDim: '#6a5a2a',
};

function useLiveVars(page: DiagramPage | undefined, simvars: Record<string, string>): Record<string, number> {
  const [vars, setVars] = useState<Record<string, number>>({});
  useEffect(() => {
    if (!page) {
      return undefined;
    }
    const names = collectNames(page);
    const read = () => {
      const next: Record<string, number> = {};
      for (const name of names) {
        const [simvar, unit] = (simvars[name] ?? `L:A32NX_${name}`).split('|');
        const value = SimVar.GetSimVarValue(simvar, unit ?? 'number');
        if (typeof value === 'number') {
          next[name] = value;
        }
      }
      setVars(next);
    };
    read();
    const timer = setInterval(read, POLL_MS);
    return () => clearInterval(timer);
  }, [page, simvars]);
  return vars;
}

export const LiveDiagram = ({ titles, labels }: { titles: string[]; labels?: string[] }) => {
  const [data, setData] = useState<StudyPages | null>(null);
  const [error, setError] = useState(false);
  const [choice, setChoice] = useState(0);
  const areaRef = useRef<HTMLDivElement>(null);
  const [width, setWidth] = useState(900);

  useEffect(() => {
    loadStudyPages()
      .then(setData)
      .catch(() => setError(true));
  }, []);
  useEffect(() => {
    if (areaRef.current && areaRef.current.clientWidth > 0) {
      setWidth(areaRef.current.clientWidth - 8);
    }
  }, [data]);

  const page = data?.pages.find((p) => p.title === titles[choice]);
  const vars = useLiveVars(page, data?.simvars ?? {});

  return (
    <div ref={areaRef} className="mb-4">
      {titles.length > 1 && (
        <div className="mb-2 flex flex-row space-x-2">
          {titles.map((t, i) => (
            <button
              type="button"
              key={t}
              onClick={() => setChoice(i)}
              className={`rounded-md border px-3 py-1 text-sm ${
                i === choice ? 'border-theme-highlight bg-theme-highlight text-theme-body' : 'border-theme-accent bg-transparent'
              }`}
            >
              {labels?.[i] ?? t}
            </button>
          ))}
        </div>
      )}
      {error && <p className="text-sm text-theme-unselected">The system diagrams are not installed.</p>}
      {page?.kind === 'topology' && page.topology && <TopologyDiagram topo={page.topology} vars={vars} width={width} />}
      {page?.kind === 'engine' && <EngineCutaway page={page} vars={vars} width={width} />}
    </div>
  );
};

export const SystemPage = ({ titles, labels }: { titles: string[]; labels?: string[] }) => (
  <ScrollableContainer height={43}>
    <LiveDiagram titles={titles} labels={labels} />
  </ScrollableContainer>
);

const linkColour = (kind: string): string => (kind === 'pipe' || kind === 'air' ? C.accent : kind === 'gas' ? '#ff9f43' : C.ok);

const DiagramBox = ({
  title,
  kind,
  x,
  y,
  w,
  fields,
  vars,
  sc,
}: {
  title: string;
  kind: string;
  x: number;
  y: number;
  w: number;
  fields: DiagramField[];
  vars: Record<string, number>;
  sc: number;
}) => {
  const live = fields.some((f) => vars[f.name] !== undefined && vars[f.name] !== 0);
  const fontPx = Math.round(11.5 * sc);
  const border = kind === 'bus' ? (live ? C.ok : C.busDim) : live ? C.line : C.lineDim;
  const narrow = w < 130;
  return (
    <div
      style={{
        position: 'absolute',
        left: x * sc,
        top: y * sc,
        width: w * sc,
        height: (22 + 16 * fields.length) * sc,
        border: `2px solid ${border}`,
        background: C.nodeBg,
        fontSize: fontPx,
        lineHeight: `${Math.round(14 * sc)}px`,
        overflow: 'hidden',
        display: 'flex',
        flexDirection: 'column',
      }}
    >
      <div
        style={{
          height: 22 * sc,
          flex: 'none',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          fontWeight: 600,
          textTransform: 'uppercase',
          color: C.text,
          whiteSpace: 'nowrap',
          overflow: 'hidden',
          textOverflow: 'ellipsis',
          padding: '0 0.35em',
          borderBottom: `1px solid ${C.lineDim}`,
        }}
      >
        {title}
      </div>
      {fields.map((f) => {
        const value = vars[f.name];
        const on = value !== undefined && value !== 0;
        const lamp = f.kind === 'lamp' || f.kind === 'flag';
        const centre = (lamp && (f.kind === 'flag' || narrow)) || (!lamp && narrow);
        const dot = (
          <span
            style={{
              display: 'inline-block',
              width: '0.61em',
              height: '0.61em',
              borderRadius: '50%',
              marginRight: '0.43em',
              background: on ? C.ok : '#666',
            }}
          />
        );
        return (
          <div
            key={f.name}
            style={{
              height: 16 * sc,
              flex: 'none',
              display: 'flex',
              alignItems: 'center',
              justifyContent: centre ? 'center' : 'space-between',
              padding: '0 6px',
              whiteSpace: 'nowrap',
              overflow: 'hidden',
              fontSize: fontPx,
            }}
          >
            {lamp && centre ? (
              <span style={{ color: on ? C.text : C.lineDim }}>
                {dot}
                {f.kind === 'flag' ? f.flagText : fieldLabel(f)}
              </span>
            ) : (
              <>
                {!narrow && <span style={{ color: C.textDim, overflow: 'hidden', textOverflow: 'ellipsis' }}>{fieldLabel(f)}</span>}
                <span style={{ color: C.text }}>
                  {lamp && dot}
                  {lamp ? (on ? 'ON' : 'OFF') : formatValue(value, f.kind, f.decimals, f.unit)}
                </span>
              </>
            )}
          </div>
        );
      })}
    </div>
  );
};

const TopologyDiagram = ({ topo, vars, width }: { topo: Topology; vars: Record<string, number>; width: number }) => {
  const sc = Math.max(0.9, Math.min(1.25, width / topo.designW));
  return (
    <div style={{ overflowX: 'auto', padding: '4px 0 12px' }}>
      <div style={{ position: 'relative', margin: '0 auto', width: topo.designW * sc, height: topo.designH * sc, fontSize: 11.5 * sc }}>
        <svg
          viewBox={`0 0 ${topo.designW} ${topo.designH}`}
          preserveAspectRatio="none"
          style={{ position: 'absolute', left: 0, top: 0, width: '100%', height: '100%', overflow: 'visible' }}
        >
          {topo.links.map((link, i) => {
            const live = evalGate(link.gate, vars);
            return (
              <polyline
                // eslint-disable-next-line react/no-array-index-key
                key={i}
                points={link.points.map((p) => p.join(',')).join(' ')}
                fill="none"
                stroke={live ? linkColour(link.kind) : C.lineDim}
                strokeWidth={live ? 3 : 2}
                strokeLinejoin="round"
                strokeDasharray={link.kind === 'shaft' && !live ? '6 4' : undefined}
              />
            );
          })}
        </svg>
        {topo.nodes.map((n, i) => (
          // eslint-disable-next-line react/no-array-index-key
          <DiagramBox key={i} title={n.title} kind={n.kind} x={n.x} y={n.y} w={n.w} fields={n.fields} vars={vars} sc={sc} />
        ))}
      </div>
    </div>
  );
};

const CUTAWAY_W = 1000;
const CUTAWAY_H = 600;

const CutawayShapeSvg = ({ s }: { s: CutawayShape }) => {
  const pts = (list: [number, number][]) => list.map((p) => p.join(',')).join(' ');
  switch (s.type) {
    case 'poly':
      return <polygon points={pts(s.points ?? [])} fill={s.fill} stroke="#1e1e30" strokeWidth={1} />;
    case 'rect':
      return <rect x={s.x} y={s.y} width={s.w} height={s.h} fill={s.fill} stroke="#1e1e30" strokeWidth={1} />;
    case 'line':
      return <line x1={s.x1} y1={s.y1} x2={s.x2} y2={s.y2} stroke={s.stroke} strokeWidth={s.width} />;
    case 'arrow': {
      const x = s.x ?? 0;
      const y = s.y ?? 0;
      const l = s.len ?? 0;
      const hh = s.half ?? 0;
      const head = hh * 1.6;
      return (
        <polygon
          points={pts([
            [x, y - hh * 0.5],
            [x + l - head, y - hh * 0.5],
            [x + l - head, y - hh],
            [x + l, y],
            [x + l - head, y + hh],
            [x + l - head, y + hh * 0.5],
            [x, y + hh * 0.5],
          ])}
          fill={s.fill}
          stroke="#1e1e30"
          strokeWidth={1}
        />
      );
    }
    case 'text':
      return (
        <text x={s.x} y={s.y} fill={C.textDim} fontSize={12}>
          {s.text}
        </text>
      );
    default:
      return null;
  }
};

const EngineCutaway = ({ page, vars, width }: { page: DiagramPage; vars: Record<string, number>; width: number }) => {
  const sc = Math.max(0.9, Math.min(1.25, width / CUTAWAY_W));
  const stations = page.stations ?? [];
  const summaries = page.summaries ?? [];
  return (
    <div style={{ overflowX: 'auto', padding: '4px 0 12px' }}>
      <div style={{ position: 'relative', margin: '0 auto', width: CUTAWAY_W * sc, height: CUTAWAY_H * sc, fontSize: 11.5 * sc }}>
        <svg
          viewBox={`0 0 ${CUTAWAY_W} ${CUTAWAY_H}`}
          preserveAspectRatio="none"
          style={{ position: 'absolute', left: 0, top: 0, width: '100%', height: '100%', overflow: 'visible' }}
        >
          <line x1={0} y1={196} x2={CUTAWAY_W} y2={196} stroke={C.lineDim} strokeWidth={1} />
          <line x1={0} y1={520} x2={CUTAWAY_W} y2={520} stroke={C.lineDim} strokeWidth={1} />
          {(page.cutaway ?? []).map((s, i) => (
            // eslint-disable-next-line react/no-array-index-key
            <CutawayShapeSvg key={i} s={s} />
          ))}
          {stations
            .filter((s) => s.leader)
            .map((s) => {
              const [tx, ty, label] = s.leader as [number, number, string];
              const bottom = s.y + 22 + 16 * s.fields.length;
              const r = Math.max(9, label.length * 3.5 + 6);
              return (
                <g key={`${s.title}-leader`}>
                  <line x1={s.x + s.w / 2} y1={bottom} x2={tx} y2={ty} stroke={C.textDim} strokeOpacity={0.55} strokeDasharray="3 3" strokeWidth={1} />
                  <ellipse cx={tx} cy={ty} rx={r} ry={r} fill="#d6d6e6" stroke="#1e1e30" strokeWidth={1} />
                  <text x={tx} y={ty + 4} textAnchor="middle" fill="#08081a" fontSize={11} fontWeight={600}>
                    {label}
                  </text>
                </g>
              );
            })}
        </svg>
        {stations.map((s) => (
          <DiagramBox key={s.title} title={s.title} kind="source" x={s.x} y={s.y} w={s.w} fields={s.fields} vars={vars} sc={sc} />
        ))}
        {summaries.map((s, i) => (
          <DiagramBox key={s.title} title={s.title} kind="source" x={6 + i * 124.5} y={527} w={118} fields={s.fields} vars={vars} sc={sc} />
        ))}
      </div>
    </div>
  );
};

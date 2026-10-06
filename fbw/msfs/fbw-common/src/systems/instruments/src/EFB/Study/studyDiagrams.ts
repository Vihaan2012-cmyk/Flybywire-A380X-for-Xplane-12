// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { fetchFirst } from './catalogue';

export interface DiagramField {
  name: string;
  kind: string;
  label?: string;
  unit?: string;
  decimals?: number;
  flagText?: string;
}

export interface DiagramNode {
  x: number;
  y: number;
  w: number;
  kind: string;
  title: string;
  fields: DiagramField[];
}

export interface Gate {
  op: string;
  name?: string;
  a?: string;
  b?: string;
  value?: number;
  gates?: Gate[];
  gate?: Gate;
}

export interface DiagramLink {
  points: [number, number][];
  kind: string;
  gate?: Gate;
}

export interface Topology {
  designW: number;
  designH: number;
  nodes: DiagramNode[];
  links: DiagramLink[];
}

export interface CutawayShape {
  type: 'poly' | 'rect' | 'line' | 'arrow' | 'text';
  points?: [number, number][];
  fill?: string;
  stroke?: string;
  width?: number;
  x?: number;
  y?: number;
  w?: number;
  h?: number;
  x1?: number;
  y1?: number;
  x2?: number;
  y2?: number;
  half?: number;
  len?: number;
  text?: string;
}

export interface Station {
  title: string;
  x: number;
  y: number;
  w: number;
  fields: DiagramField[];
  leader?: [number, number, string];
}

export interface DiagramPage {
  title: string;
  kind: 'topology' | 'engine';
  topology?: Topology;
  cutaway?: CutawayShape[];
  stations?: Station[];
  summaries?: { title: string; fields: DiagramField[] }[];
}

export interface StudyPages {
  schemaVersion: number;
  pages: DiagramPage[];
  simvars: Record<string, string>;
}

const STUDY_PAGES_URLS = ['/VFS/html_ui/Pages/VCockpit/Instruments/A380X/EFB/study-pages.json', 'study-pages.json'];

let cached: StudyPages | null = null;
let inFlight: Promise<StudyPages> | null = null;

export function loadStudyPages(): Promise<StudyPages> {
  if (cached) {
    return Promise.resolve(cached);
  }
  if (inFlight) {
    return inFlight;
  }
  inFlight = fetchFirst(STUDY_PAGES_URLS)
    .then((response) => {
      if (!response.ok) {
        throw new Error(`study-pages.json: HTTP ${response.status}`);
      }
      return response.json();
    })
    .then((data: StudyPages) => {
      cached = data;
      inFlight = null;
      return data;
    })
    .catch((error) => {
      inFlight = null;
      throw error;
    });
  return inFlight;
}

function gateNames(gate: Gate | undefined, out: Set<string>): void {
  if (!gate) {
    return;
  }
  if (gate.name) out.add(gate.name);
  if (gate.a) out.add(gate.a);
  if (gate.b) out.add(gate.b);
  gate.gates?.forEach((g) => gateNames(g, out));
  if (gate.gate) gateNames(gate.gate, out);
}

export function collectNames(page: DiagramPage): string[] {
  const out = new Set<string>();
  const addFields = (fields?: DiagramField[]) => fields?.forEach((f) => out.add(f.name));
  page.topology?.nodes.forEach((n) => addFields(n.fields));
  page.topology?.links.forEach((l) => gateNames(l.gate, out));
  page.stations?.forEach((s) => addFields(s.fields));
  page.summaries?.forEach((s) => addFields(s.fields));
  return [...out];
}

const COND_CMP: Record<string, (a: number, b: number) => boolean> = {
  lt: (a, b) => a < b,
  le: (a, b) => a <= b,
  gt: (a, b) => a > b,
  ge: (a, b) => a >= b,
  eq: (a, b) => Math.abs(a - b) < 1e-9,
  ne: (a, b) => Math.abs(a - b) >= 1e-9,
};

export function evalGate(gate: Gate | undefined, vars: Record<string, number>): boolean {
  if (!gate) {
    return false;
  }
  const v = (n?: string) => (n === undefined ? undefined : vars[n]);
  const cmp = COND_CMP[gate.op];
  if (cmp) {
    const left = gate.a !== undefined ? v(gate.a) : v(gate.name);
    const right = gate.b !== undefined ? v(gate.b) : gate.value;
    return left !== undefined && right !== undefined && cmp(left, right);
  }
  switch (gate.op) {
    case 'on':
      return (v(gate.name) ?? 0) !== 0;
    case 'off':
      return (v(gate.name) ?? 0) === 0;
    case 'absGt':
      return Math.abs(v(gate.name) ?? 0) > (gate.value ?? 0);
    case 'all':
      return (gate.gates ?? []).every((g) => evalGate(g, vars));
    case 'any':
      return (gate.gates ?? []).some((g) => evalGate(g, vars));
    case 'not':
      return !evalGate(gate.gate, vars);
    case 'always':
      return true;
    default:
      return false;
  }
}

function isPacked(v: number): boolean {
  return Number.isFinite(v) && v >= 1.0e9 && v < 17179869184 && v === Math.trunc(v);
}

function unpackArinc(v: number): [number, number] {
  if (!Number.isFinite(v) || v < 0 || v >= 17179869184 || v !== Math.trunc(v)) {
    return [v, 3];
  }
  const low = v % 4294967296;
  // eslint-disable-next-line no-bitwise
  const ssm = Math.floor(v / 4294967296) & 3;
  const buf = new ArrayBuffer(4);
  new Uint32Array(buf)[0] = low;
  return [new Float32Array(buf)[0], ssm];
}

const ARINC_STATUS: Record<number, string> = { 0: 'FW', 1: 'NCD', 2: 'FT' };

function withUnit(n: string, unit?: string): string {
  return unit ? `${n} ${unit}` : n;
}

export function formatValue(value: number | undefined, kind: string, decimals = 0, unit?: string): string {
  if (value === undefined || !Number.isFinite(value)) return 'no reading';
  if (kind === 'lamp') return value !== 0 ? 'ON' : 'OFF';
  if (kind === 'arinc' || (kind === 'num' && isPacked(value))) {
    const [v, ssm] = unpackArinc(value);
    const shown = withUnit(v.toFixed(decimals), unit);
    const status = ARINC_STATUS[ssm];
    return status ? `${shown} ${status}` : shown;
  }
  if (Math.abs(value) >= 1e7) return withUnit(value.toFixed(0), unit);
  return withUnit(value.toFixed(decimals), unit);
}

const UNIT_LABEL: Record<string, string> = {
  V: 'Voltage',
  Hz: 'Frequency',
  A: 'Current',
  '%': 'Load',
  psi: 'Pressure',
  RPM: 'Speed',
  gal: 'Quantity',
  kg: 'Quantity',
  lb: 'Quantity',
  '°C': 'Temperature',
  C: 'Temperature',
};

export function fieldLabel(f: DiagramField): string {
  if (f.label) return f.label;
  if (f.kind === 'flag') return f.flagText ?? '';
  return (f.unit && UNIT_LABEL[f.unit]) || f.name.replace(/^(A32NX|A380X)_/, '').replace(/_/g, ' ').toLowerCase();
}

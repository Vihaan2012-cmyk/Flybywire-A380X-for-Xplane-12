// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { describe, expect, it, vi } from 'vitest';
import * as fs from 'node:fs';
import * as path from 'node:path';

const DIR = process.env.DISPLAY_REPLAY_DIR ?? '';
const SHARD = (process.env.DISPLAY_SHARD ?? '0/1').split('/').map(Number);
const DT_MS = 100;
const SETTLE_TICKS = Number(process.env.DISPLAY_TICKS ?? 30);

interface DisplayDef {
  name: string;
  tag: string;
  module: () => Promise<unknown>;
  content: string;
  url: string;
}

const DISPLAYS: DisplayDef[] = [
  { name: 'PFD', tag: 'a380x-pfd', module: () => import('../PFD/instrument'), content: 'PFD_CONTENT', url: 'A380X/PFD/pfd.html?Index=1' },
  { name: 'ND', tag: 'a380x-nd', module: () => import('../ND/instrument'), content: 'ND_CONTENT', url: 'A380X/ND/nd.html?Index=1' },
  { name: 'EWD', tag: 'a380x-ewd', module: () => import('../EWD/instrument'), content: 'EWD_CONTENT', url: 'A380X/EWD/ewd.html?Index=1' },
  { name: 'SD', tag: 'a380x-sdv2', module: () => import('../SDv2/instrument'), content: 'SDv2_CONTENT', url: 'A380X/SDv2/sdv2.html?Index=1' },
  { name: 'OIT', tag: 'a380x-oit', module: () => import('../OIT/instrument'), content: 'OIT_CONTENT', url: 'A380X/OIT/oit.html?Index=1' },
];

const values = new Map<string, number>();
const registered: string[] = [];
let reads: Set<string> | null = null;
const instruments = new Map<string, new () => any>();

function key(name: string): string {
  const n = name.trim();
  return n.startsWith('A:') ? n.substring(2) : n;
}

function read(name: string): number {
  const k = key(name);
  reads?.add(k);
  return values.get(k) ?? 0;
}

function installGlobals() {
  const g = globalThis as any;
  g.SimVar.GetSimVarValue = (name: string) => read(name);
  g.SimVar.GetSimVarValueFast = (name: string) => read(name);
  g.SimVar.GetGlobalVarValue = (name: string) => read(name);
  g.SimVar.GetGameVarValue = (name: string) => read(name);
  g.SimVar.GetRegisteredId = (name: string) => {
    registered.push(key(name));
    return registered.length - 1;
  };
  g.SimVar.GetSimVarValueFastReg = (id: number) => read(registered[id]);
  g.SimVar.GetSimVarValueFastRegString = () => '';
  g.SimVar.SetSimVarValue = (name: string, _unit: string, value: number) => {
    values.set(key(name), Number(value));
    return Promise.resolve();
  };
  g.simvar = {
    getValueReg: (id: number) => read(registered[id]),
    getValueReg_String: () => '',
    getValue_LatLongAlt: () => ({ lat: 0, long: 0, alt: 0 }),
  };
  g.GameState = g.GameState ?? { mainmenu: 0, loading: 1, briefing: 2, ingame: 3 };
  g.Aircraft = g.Aircraft ?? { A320_NEO: 0, B747_8: 1, AS01B: 2, CJ4: 3, AS02A: 4 };
  g.Simplane = new Proxy(
    {
      getIsGrounded: () => read('SIM ON GROUND') > 0,
      getPressureSelectedMode: () => 'QNH',
      getPressureSelectedUnits: () => 'hectopascal',
    },
    { get: (t: any, p: string) => (p in t ? t[p] : () => 0) },
  );
  g.BaseInstrument = class {
    deltaTime = DT_MS;
    connectedCallback() {}
    Update() {}
    get isInteractive() {
      return false;
    }
    getChildById(id: string) {
      return document.getElementById(id);
    }
  };
  g.registerInstrument = (name: string, cls: new () => any) => instruments.set(name, cls);
  document.body.setAttribute('gamestate', 'ingame');
}

function visible(el: Element | null): boolean {
  for (let e = el; e; e = e.parentElement) {
    const style = (e as HTMLElement).style;
    if (style && (style.display === 'none' || style.visibility === 'hidden')) {
      return false;
    }
    const v = e.getAttribute('visibility');
    if (v === 'hidden') {
      return false;
    }
    const d = e.getAttribute('display');
    if (d === 'none') {
      return false;
    }
  }
  return true;
}

function snapshot(root: Element): string[] {
  const out: string[] = [];
  const walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT);
  for (let n = walker.nextNode(); n; n = walker.nextNode()) {
    const text = (n.textContent ?? '').replace(/\s+/g, ' ').trim();
    const el = n.parentElement;
    if (!text || !el || !visible(el)) {
      continue;
    }
    const cls = [el.getAttribute('class') ?? '', el.parentElement?.getAttribute('class') ?? ''].join('/');
    out.push(`${text}|${cls}`);
  }
  return out.sort();
}

describe('display replay', () => {
  it('renders the real displays', async () => {
    vi.useFakeTimers();
    installGlobals();
    const basePath = path.join(DIR || '/replay', 'base.json');
    const base: Record<string, number> = fs.existsSync(basePath) ? JSON.parse(fs.readFileSync(basePath, 'utf8')) : {};
    for (const [k, v] of Object.entries(base)) {
      values.set(key(k), v);
    }
    values.set('SIM ON GROUND', 1);
    document.body.innerHTML = `<vcockpit-panel>${DISPLAYS.map((d) => `<${d.tag} url="${d.url}"></${d.tag}>`).join('')}</vcockpit-panel>${DISPLAYS.map(
      (d) => `<div id="${d.content}"><h1>loading</h1></div>`,
    ).join('')}`;
    const live: { def: DisplayDef; inst: any; reads: Set<string> }[] = [];
    for (const def of DISPLAYS) {
      const t = performance.now();
      try {
        await def.module();
        const cls = instruments.get(def.tag);
        if (!cls) {
          console.log(`${def.name}: did not register`);
          continue;
        }
        const inst = new cls();
        reads = new Set();
        inst.connectedCallback();
        live.push({ def, inst, reads });
        reads = null;
        console.log(`${def.name}: connected in ${(performance.now() - t).toFixed(0)} ms`);
      } catch (e) {
        console.log(`${def.name}: failed ${(e as Error).message}\n${(e as Error).stack?.split('\n').slice(0, 6).join('\n')}`);
      }
    }
    const t = performance.now();
    for (let tick = 0; tick < SETTLE_TICKS; tick++) {
      vi.advanceTimersByTime(DT_MS);
      for (const d of live) {
        reads = d.reads;
        try {
          d.inst.Update();
        } catch (e) {
          console.log(`${d.def.name}: Update threw ${(e as Error).message}`);
        }
      }
      reads = null;
    }
    console.log(`${SETTLE_TICKS} ticks of ${live.length} displays in ${(performance.now() - t).toFixed(0)} ms`);
    for (const d of live) {
      const snap = snapshot(document.getElementById(d.def.content)!);
      console.log(`${d.def.name}: ${d.reads.size} variables read, ${snap.length} visible text items, e.g. ${JSON.stringify(snap.slice(0, 8))}`);
    }
    expect(live.length).toBeGreaterThan(0);
    vi.useRealTimers();
  }, 3_600_000);
});

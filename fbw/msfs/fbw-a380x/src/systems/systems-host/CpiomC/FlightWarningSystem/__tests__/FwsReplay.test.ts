// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { describe, expect, it, vi } from 'vitest';
import * as fs from 'node:fs';
import * as path from 'node:path';
import * as readline from 'node:readline';
import { EventBus, Subject } from '@microsoft/msfs-sdk';
import { FailuresConsumer } from '@flybywiresim/fbw-sdk';
import { FwsCore } from '../FwsCore';
import { FuelSystemPublisher } from '../../../../instruments/src/MsfsAvionicsCommon/providers/FuelSystemPublisher';
import { PseudoFwcSimvarPublisher } from '../../../../instruments/src/MsfsAvionicsCommon/providers/PseudoFwcPublisher';
import { FcdcSimvarPublisher } from '../../../../instruments/src/MsfsAvionicsCommon/providers/FcdcPublisher';
import { FqmsBusPublisher } from '@shared/publishers/FqmsBusPublisher';
import { EcamAbnormalSensedProcedures } from '../../../../instruments/src/MsfsAvionicsCommon/EcamMessages';

const DIR = process.env.FWS_REPLAY_DIR ?? '';
const BRIDGE_ON = process.env.FWS_REPLAY_BRIDGE !== '0' && process.env.FWS_REPLAY_BRIDGE !== 'false';
const BRIDGE_PATCHES_PATH =
  process.env.FWS_REPLAY_BRIDGE_PATCHES ?? 'D:/A380/fbw-build/wasm-fs2020/ecam-msfs/patches-msfs.json';

interface DeepEcamPatch {
  path: string;
  reason: string;
  find: string;
  replace: string;
}

function loadDeepEcamBridge(): { install: (fws: FwsCore) => void } | null {
  if (!BRIDGE_ON || !fs.existsSync(BRIDGE_PATCHES_PATH)) {
    return null;
  }
  const patches: DeepEcamPatch[] = JSON.parse(fs.readFileSync(BRIDGE_PATCHES_PATH, 'utf8'));
  const appended = patches
    .filter((p) => p.reason.includes('deep ECAM bridge') && p.path.endsWith('SystemsHost/SystemsHost.js'))
    .map((p) => (p.replace.startsWith(p.find) ? p.replace.slice(p.find.length) : null))
    .filter((s): s is string => !!s);
  const mergeCode = appended.find((s) => s.trimStart().startsWith('Object.assign(EcamAbnormalSensedProcedures'));
  const installCode = appended.find((s) => s.trimStart().startsWith('if (!this.__deepEcamInstalled)'));
  if (!mergeCode || !installCode) {
    return null;
  }
  // eslint-disable-next-line no-new-func
  new Function('EcamAbnormalSensedProcedures', mergeCode)(EcamAbnormalSensedProcedures);
  // eslint-disable-next-line no-new-func
  const tick = new Function('EcamAbnormalProcedures', installCode) as (
    this: FwsCore,
    EcamAbnormalProcedures: typeof EcamAbnormalSensedProcedures,
  ) => void;
  return {
    install(fws: FwsCore) {
      tick.call(fws, EcamAbnormalSensedProcedures);
    },
  };
}

const deepEcamBridge = loadDeepEcamBridge();
const SHARD = (process.env.FWS_SHARD ?? '0/1').split('/').map(Number);
const PHASE_S = Number(process.env.FWS_PHASE_S ?? 20);
const DT_MS = 100;

const values = new Map<string, number>();
const registered: string[] = [];
let simSeconds = 0;

function key(name: string): string {
  const n = name.trim();
  return n.startsWith('A:') ? n.substring(2) : n;
}

function read(name: string): number {
  return values.get(key(name)) ?? 0;
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
  g.SimVar.GetSimVarValueFastReg = (id: number) => values.get(registered[id]) ?? 0;
  g.SimVar.GetSimVarValueFastRegString = () => '';
  g.simvar = {
    getValueReg: (id: number) => values.get(registered[id]) ?? 0,
    getValueReg_String: () => '',
    getValue_LatLongAlt: () => ({ lat: 0, long: 0, alt: 0 }),
  };
  g.SimVar.SetSimVarValue = (name: string, _unit: string, value: number) => {
    values.set(key(name), Number(value));
    return Promise.resolve();
  };
  g.GameState = g.GameState ?? { mainmenu: 0, loading: 1, briefing: 2, ingame: 3 };
  g.Aircraft = g.Aircraft ?? { A320_NEO: 0, B747_8: 1, AS01B: 2, CJ4: 3, AS02A: 4 };
  g.Simplane = {
    getIsGrounded: () => read('SIM ON GROUND') > 0,
    getAltitude: () => read('INDICATED ALTITUDE'),
    getPressureValue: () => read('KOHLSMAN SETTING MB:1') || 1013,
    getPressureSelectedMode: () => 'QNH',
    getPressureSelectedUnits: () => 'hectopascal',
    getAutoPilotAirspeedHoldValue: () => read('AUTOPILOT AIRSPEED HOLD VAR'),
    getAutoPilotAirspeedSelected: () => true,
    getAutoPilotAltitudeManaged: () => false,
    getAutoPilotDisplayedAltitudeLockValue: () => read('AUTOPILOT ALTITUDE LOCK VAR:3'),
    getAutoPilotMachHoldValue: () => read('AUTOPILOT MACH HOLD VAR'),
    getAutoPilotMachModeActive: () => read('AUTOPILOT MANAGED SPEED IN MACH') > 0,
    getAutoPilotSelectedHeadingLockValue: () => read('AUTOPILOT HEADING LOCK DIR:1'),
  };
  document.body.setAttribute('gamestate', 'ingame');
}

function setState(base: Record<string, number>, overlay: Record<string, number>) {
  values.clear();
  for (const [k, v] of Object.entries(base)) {
    values.set(key(k), v);
  }
  for (const [k, v] of Object.entries(overlay)) {
    values.set(key(k), v);
  }
  values.set('E:SIMULATION TIME', simSeconds);
  for (let n = 1; n <= 11; n++) {
    const kg = values.get(`L:A32NX_FUEL_TANK_QTY_KG_${n}`);
    if (kg !== undefined) {
      values.set(`FUELSYSTEM TANK WEIGHT:${n}`, kg);
    }
  }
  if (!values.has('SIM ON GROUND')) {
    values.set('SIM ON GROUND', Number(process.env.FWS_ON_GROUND ?? 1));
  }
  if (process.env.FWS_STATIONARY !== '0' && values.get('SIM ON GROUND')) {
    values.set('AIRSPEED INDICATED', 0);
    for (const n of [1, 2, 3]) {
      for (const k of [`L:A32NX_ADIRS_ADR_${n}_COMPUTED_AIRSPEED`, `L:A32NX_ADIRS_ADR_${n}_MACH`]) {
        if (Math.floor((values.get(k) ?? 0) / 2 ** 32) === 3) {
          values.set(k, 2 ** 32);
        }
      }
    }
  }
}

interface DisplayRule {
  du: string;
  buses: string[];
  breaker: string | null;
}

function displayRules(): DisplayRule[] {
  const src = fs.readFileSync(path.join(__dirname, '../../../../instruments/src/MsfsAvionicsCommon/CdsDisplayUnit.tsx'), 'utf8');
  const busEnum = fs.readFileSync(path.join(__dirname, '../../../../shared/src/electrical.ts'), 'utf8');
  const busValue = new Map<string, string>();
  for (const m of busEnum.matchAll(/(\w+)\s*=\s*'([^']+)'/g)) {
    busValue.set(m[1], m[2]);
  }
  const block = (name: string) => src.substring(src.indexOf(name), src.indexOf('};', src.indexOf(name)));
  const buses = new Map<string, string[]>();
  for (const m of block('DisplayUnitToDCBus').matchAll(/\[DisplayUnitID\.(\w+)\]:\s*\[([^\]]*)\]/g)) {
    buses.set(
      m[1],
      m[2]
        .split(',')
        .map((s) => s.trim().replace('DcElectricalBus.', ''))
        .filter((s) => s)
        .map((s) => busValue.get(s) ?? s),
    );
  }
  const breakers = new Map<string, string | null>();
  for (const m of block('DisplayUnitToBreakerVar').matchAll(/\[DisplayUnitID\.(\w+)\]:\s*('([^']*)'|null)/g)) {
    breakers.set(m[1], m[3] ?? null);
  }
  return [...buses.entries()].map(([du, b]) => ({ du, buses: b, breaker: breakers.get(du) ?? null }));
}

function darkDisplays(rules: DisplayRule[]): string[] {
  return rules
    .filter((r) => {
      const powered = r.buses.some((b) => read(`L:A32NX_ELEC_${b}_BUS_IS_POWERED`) > 0);
      const breakerOpen = r.breaker ? read(r.breaker) > 0 : false;
      return !powered || breakerOpen;
    })
    .map((r) => r.du);
}

function makeFws() {
  const bus = new EventBus();
  const publishers = [new FuelSystemPublisher(bus), new PseudoFwcSimvarPublisher(bus), new FcdcSimvarPublisher(bus), new FqmsBusPublisher(bus)];
  publishers.forEach((p) => p.startPublish());
  const failuresConsumer = new FailuresConsumer();
  const fws = new FwsCore(1, bus, failuresConsumer, Subject.create(false), Subject.create(false));
  fws.init();
  return { publishers, fws, failuresConsumer };
}

function snapshot(fws: FwsCore) {
  const f = fws as any;
  const memos = Object.entries(f.memos.ewdMemos as Record<string, { simVarIsActive: { get(): boolean } }>)
    .filter(([, m]) => m.simVarIsActive.get())
    .map(([k]) => k);
  return {
    abnormal: Array.from(fws.presentedAbnormalProceduresList.get().keys()),
    memos,
    inopSys: [...(f.inopSysAllPhasesKeys.get() as string[]), ...(f.inopSysApprLdgKeys.get() as string[])],
    limitations: [...(f.limitationsAllPhasesKeys.get() as string[]), ...(f.limitationsApprLdgKeys.get() as string[])],
    masterWarning: !!fws.masterWarning.get(),
    masterCaution: !!fws.masterCaution.get(),
  };
}

function step(publishers: { onUpdate(): void }[], fws: FwsCore, seconds: number) {
  for (let t = 0; t < (seconds * 1000) / DT_MS; t++) {
    simSeconds += DT_MS / 1000;
    values.set('E:SIMULATION TIME', simSeconds);
    vi.advanceTimersByTime(DT_MS);
    publishers.forEach((p) => p.onUpdate());
    if (deepEcamBridge) {
      deepEcamBridge.install(fws);
    }
    fws.update(DT_MS);
  }
}

const diff = (a: string[], b: string[]) => a.filter((x) => !b.includes(x));

describe.skipIf(!DIR)('FWS replay of the headless suite', () => {
  it('replays every run', async () => {
    vi.useFakeTimers();
    installGlobals();
    const rules = displayRules();
    const base: Record<string, number> = JSON.parse(fs.readFileSync(path.join(DIR, 'base.json'), 'utf8'));
    const [shard, shards] = SHARD;
    const outPath = path.join(DIR, `fws-${shard}.jsonl`);
    fs.writeFileSync(outPath, '');
    let written = 0;
    setState(base, {});
    const reference = makeFws();
    step(reference.publishers, reference.fws, 2 * PHASE_S);
    const healthy = snapshot(reference.fws);
    const healthyDark = darkDisplays(rules);
    if (shard === 0) {
      fs.writeFileSync(path.join(DIR, 'healthy.json'), JSON.stringify({ ...healthy, dark: healthyDark }));
    }
    const reader = readline.createInterface({ input: fs.createReadStream(path.join(DIR, 'runs.jsonl')), crlfDelay: Infinity });
    let i = -1;
    for await (const line of reader) {
      i++;
      if (i % shards !== shard || !line.trim()) {
        continue;
      }
      let run: { label: string; fbw: number[]; end: Record<string, number> };
      try {
        run = JSON.parse(line);
      } catch {
        continue;
      }
      try {
        setState(base, {});
        const { publishers, fws, failuresConsumer } = makeFws();
        step(publishers, fws, PHASE_S);
        setState(base, run.end);
        (failuresConsumer as any).onActiveFailuresChanged(new Set(run.fbw));
        step(publishers, fws, PHASE_S);
        const after = snapshot(fws);
        const dark = darkDisplays(rules);
        fs.appendFileSync(
          outPath,
          JSON.stringify({
            label: run.label,
            raised: diff(after.abnormal, healthy.abnormal),
            cleared: diff(healthy.abnormal, after.abnormal),
            memos: diff(after.memos, healthy.memos),
            memosCleared: diff(healthy.memos, after.memos),
            inopSys: diff(after.inopSys, healthy.inopSys),
            limitations: diff(after.limitations, healthy.limitations),
            masterWarning: after.masterWarning && !healthy.masterWarning,
            masterCaution: after.masterCaution && !healthy.masterCaution,
            dark: diff(dark, healthyDark),
          }) + '\n',
        );
        written++;
      } catch (e) {
        fs.appendFileSync(outPath, JSON.stringify({ label: run.label, error: String(e) }) + '\n');
      }
    }
    console.log(`shard ${shard}/${shards}: ${written} runs replayed`);
    expect(written).toBeGreaterThanOrEqual(0);
    vi.useRealTimers();
  }, 86_400_000);
});

// Loads a named simulator state (states/<name>.json) and resolves it to the
// flat form both renderers consume. Format: docs/reference-harness.md.
import fs from 'node:fs';
import path from 'node:path';
import { FLT_DIR, STATES_DIR } from './config.mjs';

export const FORMAT = 'fbw-reference-state/1';

const SSM = { FailureWarning: 0, NoComputedData: 1, FunctionalTest: 2, NormalOperation: 3 };

/** FBW's Arinc429Word raw simvar encoding: float32 bits of the value + ssm * 2^32. */
export function encodeArinc429({ value = 0, bits, ssm = 'NormalOperation' }) {
  let v = value;
  if (Array.isArray(bits)) {
    // Discrete words: FBW's bitValue(bit) is ((value >> (bit - 1)) & 1) on the float value.
    v = bits.reduce((acc, b) => acc | (1 << (b - 1)), 0) >>> 0;
  }
  const s = typeof ssm === 'number' ? ssm : SSM[ssm];
  if (s === undefined) throw new Error(`unknown ARINC429 SSM ${ssm}`);
  const f = new Float32Array(1);
  const u = new Uint32Array(f.buffer);
  f[0] = v;
  return u[0] + s * 2 ** 32;
}

export function normKey(name) {
  let s = String(name).trim();
  const m = s.match(/^([A-Za-z]{1,4}):(.*)$/);
  let prefix = 'A';
  if (m && (m[1].length === 1 || m[1].toUpperCase() === 'GAME')) {
    prefix = m[1].toUpperCase();
    s = m[2];
  }
  if (prefix === 'A' || prefix === 'E') s = s.toUpperCase().replace(/\s+/g, ' ');
  return `${prefix}:${s}`;
}

export function listStates() {
  return fs
    .readdirSync(STATES_DIR)
    .filter((f) => f.endsWith('.json') && !f.startsWith('_'))
    .map((f) => f.slice(0, -5))
    .sort();
}

/**
 * The variables an MSFS flight file sets when the aircraft spawns from it:
 * [LocalVars.0] as L-vars, [Switches.0] Potentiometer.N as LIGHT POTENTIOMETER:N
 * and [Gauges.0] KollsmanSetting (inHg) as KOHLSMAN SETTING MB:1. FBW ships
 * one per starting point (apron, taxi, runway, climb, cruise, approach, final).
 */
export function readFlt(file) {
  const full = path.isAbsolute(file) ? file : path.join(FLT_DIR, file);
  const vars = {};
  let section = '';
  for (const raw of fs.readFileSync(full, 'utf8').split(/\r?\n/)) {
    const line = raw.replace(/;.*$/, '').trim();
    const head = line.match(/^\[(.+)\]$/);
    if (head) {
      section = head[1].toLowerCase();
      continue;
    }
    const eq = line.indexOf('=');
    if (eq < 0) continue;
    const key = line.slice(0, eq).trim();
    const value = Number(line.slice(eq + 1).trim());
    if (!Number.isFinite(value)) continue;
    const src = `${path.basename(full)} [${section}] ${key}`;
    if (section === 'localvars.0') vars[normKey(`L:${key}`)] = { value, unit: 'number', src };
    else if (section === 'switches.0' && /^Potentiometer\.\d+$/i.test(key)) {
      vars[normKey(`LIGHT POTENTIOMETER:${key.split('.')[1]}`)] = { value, unit: 'percent over 100', src };
    } else if (section === 'gauges.0' && key === 'KollsmanSetting') {
      vars[normKey('KOHLSMAN SETTING MB:1')] = { value, unit: 'inches of mercury', src };
    }
  }
  return vars;
}

function readRaw(name, seen = new Set()) {
  if (seen.has(name)) throw new Error(`state extends cycle at ${name}`);
  seen.add(name);
  const file = path.join(STATES_DIR, `${name}.json`);
  if (!fs.existsSync(file)) throw new Error(`state not found: ${file}`);
  const raw = JSON.parse(fs.readFileSync(file, 'utf8'));
  const parents = [].concat(raw.extends ?? []);
  // The flight file is what the aircraft spawned with; parents and own vars
  // (what the systems computed since) override it.
  const merged = { vars: raw.flt ? readFlt(raw.flt) : {}, storage: {}, time: {}, sources: [] };
  for (const p of parents) {
    const parent = readRaw(p, new Set(seen));
    Object.assign(merged.vars, parent.vars);
    Object.assign(merged.storage, parent.storage);
    Object.assign(merged.time, parent.time);
    merged.sources.push(...parent.sources);
  }
  for (const [k, v] of Object.entries(raw.vars ?? {})) merged.vars[normKey(k)] = v;
  Object.assign(merged.storage, raw.storage ?? {});
  Object.assign(merged.time, raw.time ?? {});
  merged.sources.push(file);
  merged.name = raw.name ?? name;
  merged.description = raw.description ?? '';
  merged.screens = raw.screens ?? null;
  return merged;
}

function timeVars(time) {
  const utc = new Date(time.utc ?? '2026-06-21T12:00:00Z');
  const ms = utc.getTime();
  const secOfDay = (ms / 1000) % 86400;
  const localOffset = (time.localOffsetHours ?? 0) * 3600;
  const startOfYear = Date.UTC(utc.getUTCFullYear(), 0, 1);
  return {
    epochMs: ms,
    vars: {
      'E:ZULU TIME': { value: secOfDay, unit: 'seconds' },
      'E:LOCAL TIME': { value: (((secOfDay + localOffset) % 86400) + 86400) % 86400, unit: 'seconds' },
      'E:ZULU DAY OF MONTH': { value: utc.getUTCDate(), unit: 'number' },
      'E:ZULU MONTH OF YEAR': { value: utc.getUTCMonth() + 1, unit: 'number' },
      'E:ZULU YEAR': { value: utc.getUTCFullYear(), unit: 'number' },
      'E:ZULU DAY OF WEEK': { value: utc.getUTCDay(), unit: 'number' },
      'E:ZULU DAY OF YEAR': { value: Math.floor((ms - startOfYear) / 86400000) + 1, unit: 'number' },
      'E:SIMULATION TIME': { value: time.simulationTime ?? 600, unit: 'seconds' },
      // Seconds since 0001-01-01T00:00:00.
      'E:ABSOLUTE TIME': { value: ms / 1000 + 62135596800, unit: 'seconds' },
    },
  };
}

/**
 * Resolve a state: inheritance merged, ARINC 429 words encoded, E: time
 * variables derived from time.utc (explicit E: entries win).
 * Returns { name, description, time, vars: {KEY: {value, unit}}, storage, meta }.
 */
export function loadState(name) {
  if (name === 'none') {
    const t = timeVars({});
    return { name: 'none', description: 'empty state (exploration)', time: { epochMs: t.epochMs, settleMs: 8000, frameMs: 1000 / 60 }, vars: t.vars, storage: {}, meta: {} };
  }
  const raw = readRaw(name);
  const t = timeVars(raw.time);
  const vars = { ...t.vars };
  const meta = {};
  for (const [key, entry] of Object.entries(raw.vars)) {
    const e = typeof entry === 'object' && entry !== null && !Array.isArray(entry) ? entry : { value: entry };
    let value = e.value;
    let unit = e.unit ?? '';
    if (e.arinc429) {
      value = encodeArinc429(e.arinc429);
      unit = 'number';
    }
    if (value === undefined) throw new Error(`${name}: ${key} has no value`);
    vars[key] = { value, unit };
    if (e.src || e.why) meta[key] = { src: e.src, why: e.why };
  }
  return {
    name: raw.name,
    description: raw.description,
    screens: raw.screens,
    time: {
      utc: raw.time.utc,
      epochMs: t.epochMs,
      settleMs: raw.time.settleMs ?? 8000,
      frameMs: raw.time.frameMs ?? 1000 / 60,
    },
    vars,
    storage: raw.storage,
    meta,
    sources: raw.sources,
  };
}

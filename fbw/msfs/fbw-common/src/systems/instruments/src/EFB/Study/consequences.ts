// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { fetchFirst } from './catalogue';

export type AreaEffect = [area: string, moved: number, examples: string[]];

export interface ConsequenceChain {
  p: string;
  s: AreaEffect[][];
  a: string[];
  f: [number, string, string][];
  d?: number;
}

export interface Consequences {
  schemaVersion: number;
  failures: Record<string, ConsequenceChain>;
  breakers: Record<string, ConsequenceChain>;
}

const CONSEQUENCES_URLS = ['/VFS/html_ui/Pages/VCockpit/Instruments/A380X/EFB/consequences.json', 'consequences.json'];

let cached: Consequences | null = null;
let inFlight: Promise<Consequences> | null = null;

export function loadConsequences(): Promise<Consequences> {
  if (cached) {
    return Promise.resolve(cached);
  }
  if (inFlight) {
    return inFlight;
  }
  inFlight = fetchFirst(CONSEQUENCES_URLS)
    .then((response) => {
      if (!response.ok) {
        throw new Error(`consequences.json: HTTP ${response.status}`);
      }
      return response.json();
    })
    .then((data: Consequences) => {
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

const SYSTEM_NAMES: Record<string, string> = {
  apu: 'APU',
  autoflight: 'Autoflight',
  avionics_network: 'Avionics network',
  breakers: 'Circuit breakers',
  cabin: 'Cabin',
  communications: 'Communications',
  electrical: 'Electrical',
  engine_accessories: 'Engines',
  environment: 'Environment',
  fire_ice: 'Fire and ice protection',
  flight_controls: 'Flight controls',
  fuel: 'Fuel',
  gear_structure: 'Landing gear',
  hydraulics: 'Hydraulics',
  oxygen: 'Oxygen',
  pneumatic_ducts: 'Pneumatics and air conditioning',
  sensors: 'Sensors and probes',
  thermal_zones: 'Bay temperatures and smoke',
  wiring: 'Wiring',
};

export function systemName(area: string): string {
  return SYSTEM_NAMES[area] ?? area.replace(/_/g, ' ');
}

const PROFILE_NAMES: Record<string, string> = {
  cruise: 'in cruise',
  all_commands_exercised: 'with every crew control exercised',
  ground_apu: 'on the ground on APU power',
  touchdown: 'at touchdown',
  takeoff_roll: 'on the take-off roll',
  engine_start: 'during engine start',
  icing_climb: 'climbing through icing',
  cold_dark: 'cold and dark',
  gear_cycle: 'while cycling the gear',
  cruise_soak: 'over a minute of cruise',
  apu_start_soak: 'over two minutes of APU start',
};

export function profilePhrase(profile: string): string {
  return PROFILE_NAMES[profile] ?? profile.replace(/_/g, ' ');
}

const ACRONYMS = new Set([
  'AC', 'ADR', 'AFT', 'AOA', 'APU', 'CAS', 'CB', 'DC', 'ECAM', 'EDP', 'EGT', 'ENG', 'ESS', 'FADEC', 'FCU', 'FWD', 'GPU',
  'HP', 'IDG', 'IP', 'IR', 'LGCIU', 'LP', 'MLG', 'N1', 'N2', 'N3', 'NLG', 'PR', 'PRV', 'RAT', 'SEC', 'PRIM', 'TAT', 'TGT',
  'TR', 'VFG', 'VIB', 'VSV', 'WAI', 'NAI', 'ATC', 'VHF', 'HF', 'AFDX', 'CPIOM', 'IOM', 'OIT', 'EFB', 'LDCR', 'ODLS',
]);

const UNITS: Record<string, string> = {
  C: '°C',
  K: 'K',
  PA: 'Pa',
  PSI: 'psi',
  KG: 'kg',
  M: 'm',
  MM: 'mm',
  A: 'A',
  V: 'V',
  HZ: 'Hz',
  MS: 'm/s',
  S: 's',
  W: 'W',
  RPM: 'rpm',
  PCT: '%',
  KG_S: 'kg/s',
  KG_M3: 'kg/m³',
  L_S: 'L/s',
  MM_S: 'mm/s',
};

export function variablePhrase(name: string): string {
  let base = name.replace(/^A32NX_/, '').replace(/^DEEP_/, '');
  let index = '';
  const colon = base.indexOf(':');
  if (colon >= 0) {
    index = ` ${base.slice(colon + 1)}`;
    base = base.slice(0, colon);
  }
  let unit = '';
  for (const suffix of ['KG_M3', 'KG_S', 'L_S', 'MM_S']) {
    if (base.endsWith(`_${suffix}`)) {
      unit = UNITS[suffix];
      base = base.slice(0, -(suffix.length + 1));
      break;
    }
  }
  const words = base.split('_').filter((w) => w !== '');
  if (unit === '' && words.length > 1 && UNITS[words[words.length - 1]] !== undefined) {
    unit = UNITS[words.pop() as string];
  }
  const phrase = words.map((w) => (ACRONYMS.has(w) || /\d/.test(w) ? w : w.toLowerCase())).join(' ');
  const sentence = phrase.charAt(0).toUpperCase() + phrase.slice(1);
  return `${sentence}${unit !== '' ? ` (${unit})` : ''}${index}`;
}

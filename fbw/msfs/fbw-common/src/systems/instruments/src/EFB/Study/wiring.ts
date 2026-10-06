// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { useEffect, useState } from 'react';
import { fetchFirst } from './catalogue';

export interface UnitWiring {
  failures: number[];
  gates: string[];
  ecam?: string[];
  none?: string;
}

export interface Wiring {
  schemaVersion: number;
  units: Record<string, UnitWiring>;
}

const WIRING_URLS = ['/VFS/html_ui/Pages/VCockpit/Instruments/A380X/EFB/wiring.json', 'wiring.json'];

let cached: Wiring | null = null;
let inFlight: Promise<Wiring> | null = null;

export function loadWiring(): Promise<Wiring> {
  if (cached) {
    return Promise.resolve(cached);
  }
  if (inFlight) {
    return inFlight;
  }
  inFlight = fetchFirst(WIRING_URLS)
    .then((response) => {
      if (!response.ok) {
        throw new Error(`wiring.json: HTTP ${response.status}`);
      }
      return response.json();
    })
    .then((data: Wiring) => {
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

export function useWiring(): Wiring | null {
  const [wiring, setWiring] = useState<Wiring | null>(cached);
  useEffect(() => {
    if (wiring) {
      return;
    }
    loadWiring()
      .then(setWiring)
      .catch(() => setWiring(null));
  }, [wiring]);
  return wiring;
}

export function unitIsWired(wiring: Wiring | null, unit: string): boolean {
  const w = wiring?.units[unit];
  return !!w && (w.failures.length > 0 || w.gates.length > 0 || (w.ecam?.length ?? 0) > 0);
}

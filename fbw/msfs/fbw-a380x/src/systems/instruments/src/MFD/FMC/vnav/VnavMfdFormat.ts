// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { VnavProfile, VnavProfilePoint } from './types';

export function vnavEtaBaseMs(
  pastTakeoff: boolean,
  zuluTimeS: number,
  estimatedTakeoffTimeS: number | undefined,
): number {
  if (pastTakeoff) {
    return zuluTimeS * 1000;
  }
  if (estimatedTakeoffTimeS !== undefined) {
    return estimatedTakeoffTimeS * 1000;
  }
  return 0;
}

export function vnavEtaMs(point: VnavProfilePoint | undefined, baseMs: number): number | null {
  if (!point) {
    return null;
  }
  return baseMs + point.timeFromStartS * 1000;
}

export function vnavAltitudeDisplay(point: VnavProfilePoint | undefined): number | null {
  return point ? Math.round(point.altitudeFt) : null;
}

export function vnavSpeedDisplay(point: VnavProfilePoint | undefined): number | null {
  if (!point) {
    return null;
  }
  return point.mach !== undefined && point.mach > 0 ? null : Math.round(point.speedKt);
}

export function vnavMachDisplay(point: VnavProfilePoint | undefined): number | null {
  return point?.mach !== undefined ? point.mach : null;
}

export function vnavDistanceToTopOfDescentNm(
  profile: VnavProfile | undefined,
  aircraftDistanceFromStartNm: number,
): number | null {
  if (!profile || profile.invalidReason !== undefined || profile.topOfDescentNm === undefined) {
    return null;
  }
  const distance = profile.topOfDescentNm - aircraftDistanceFromStartNm;
  return Number.isFinite(distance) && distance >= 0 ? distance : null;
}

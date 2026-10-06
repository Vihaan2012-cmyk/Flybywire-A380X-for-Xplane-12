// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import type { VnavLeg, VnavProfile, VnavProfilePoint, VnavSegment } from './types';

const ALTITUDE_TOLERANCE_FT = 50;

export interface VnavLegPrediction {
  legIndex: number;
  ident: string;
  distanceFromStartNm: number;
  altitudeFt: number;
  speedKt: number;
  mach?: number;
  timeFromStartS: number;
  fuelRemainingKg: number;
  segment: VnavSegment;
  altitudeConstraintMet?: boolean;
}

export function predict(profile: VnavProfile, legs: VnavLeg[]): VnavLegPrediction[] {
  const points = profile.points;
  const predictions: VnavLegPrediction[] = [];

  for (const leg of legs) {
    if (leg.isMissedApproach) {
      continue;
    }

    const sample = points.length > 0 ? sampleAt(points, leg.distanceFromStartNm) : undefined;

    predictions.push({
      legIndex: leg.index,
      ident: leg.ident,
      distanceFromStartNm: leg.distanceFromStartNm,
      altitudeFt: sample?.altitudeFt ?? NaN,
      speedKt: sample?.speedKt ?? NaN,
      mach: sample?.mach,
      timeFromStartS: sample?.timeFromStartS ?? NaN,
      fuelRemainingKg: sample?.fuelRemainingKg ?? NaN,
      segment: sample?.segment ?? inferSegment(leg),
      altitudeConstraintMet: sample ? checkAltitudeConstraint(leg, sample.altitudeFt) : undefined,
    });
  }

  return predictions;
}

function sampleAt(points: VnavProfilePoint[], distanceNm: number): VnavProfilePoint {
  if (distanceNm <= points[0].distanceFromStartNm) {
    return points[0];
  }
  const last = points[points.length - 1];
  if (distanceNm >= last.distanceFromStartNm) {
    return last;
  }

  let lo = 0;
  let hi = points.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (points[mid].distanceFromStartNm <= distanceNm) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  const a = points[lo];
  const b = points[hi];
  const span = b.distanceFromStartNm - a.distanceFromStartNm;
  const frac = span > 1e-6 ? (distanceNm - a.distanceFromStartNm) / span : 1;

  return {
    distanceFromStartNm: distanceNm,
    altitudeFt: lerp(a.altitudeFt, b.altitudeFt, frac),
    speedKt: lerp(a.speedKt, b.speedKt, frac),
    mach: a.mach !== undefined && b.mach !== undefined ? lerp(a.mach, b.mach, frac) : undefined,
    timeFromStartS: lerp(a.timeFromStartS, b.timeFromStartS, frac),
    fuelRemainingKg: lerp(a.fuelRemainingKg, b.fuelRemainingKg, frac),
    segment: frac < 0.5 ? a.segment : b.segment,
  };
}

function lerp(a: number, b: number, frac: number): number {
  return a + (b - a) * frac;
}

function inferSegment(leg: VnavLeg): VnavSegment {
  switch (leg.segment) {
    case 'departure':
      return 'climb';
    case 'arrival':
      return 'descent';
    case 'approach':
      return 'approach';
    default:
      return 'cruise';
  }
}

function checkAltitudeConstraint(leg: VnavLeg, predictedFt: number): boolean | undefined {
  const c = leg.altitudeConstraint;
  if (!c) {
    return undefined;
  }
  switch (c.type) {
    case 'at':
      return Math.abs(predictedFt - c.ft1) <= ALTITUDE_TOLERANCE_FT;
    case 'atOrAbove':
      return predictedFt >= c.ft1 - ALTITUDE_TOLERANCE_FT;
    case 'atOrBelow':
      return predictedFt <= c.ft1 + ALTITUDE_TOLERANCE_FT;
    case 'between':
      return predictedFt <= c.ft1 + ALTITUDE_TOLERANCE_FT && predictedFt >= (c.ft2 ?? c.ft1) - ALTITUDE_TOLERANCE_FT;
    default:
      return undefined;
  }
}

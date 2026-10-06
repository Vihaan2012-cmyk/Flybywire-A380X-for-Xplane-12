// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import type {
  VnavAircraftState,
  VnavLeg,
  VnavPerformanceModel,
  VnavProfilePoint,
  VnavPseudoWaypoint,
  VnavSpeedProfile,
} from './types';

export interface ClimbProfileResult {
  points: VnavProfilePoint[];
  pseudoWaypoints: VnavPseudoWaypoint[];
  topOfClimbNm: number | null;
}

interface ClimbCap {
  distanceFromStartNm: number;
  capFt: number;
  legIndex: number;
}

const MAX_CLIMB_STEPS = 400;
const STEP_FT = 500;

function climbCaps(legs: VnavLeg[], cruiseAltitudeFt: number): ClimbCap[] {
  const caps: ClimbCap[] = [];

  for (const leg of legs) {
    if (leg.isMissedApproach || (leg.segment !== 'departure' && leg.segment !== 'enroute')) {
      continue;
    }

    const constraint = leg.altitudeConstraint;
    if (!constraint || constraint.type === 'atOrAbove') {
      continue;
    }

    const capFt = constraint.type === 'between' ? constraint.ft1 : constraint.ft1;
    if (capFt >= cruiseAltitudeFt) {
      continue;
    }

    caps.push({ distanceFromStartNm: leg.distanceFromStartNm, capFt, legIndex: leg.index });
  }

  return caps.sort((a, b) => a.distanceFromStartNm - b.distanceFromStartNm);
}

function makePoint(
  distanceFromStartNm: number,
  altitudeFt: number,
  speedKt: number,
  mach: number,
  timeFromStartS: number,
  fuelRemainingKg: number,
  legIndex?: number,
): VnavProfilePoint {
  return {
    distanceFromStartNm,
    altitudeFt,
    speedKt,
    mach,
    timeFromStartS,
    fuelRemainingKg,
    segment: 'climb',
    legIndex,
  };
}

export function buildClimb(
  state: VnavAircraftState,
  legs: VnavLeg[],
  perf: VnavPerformanceModel,
  speed: VnavSpeedProfile,
): ClimbProfileResult {
  let distanceNm = state.distanceFromStartNm;
  let altitudeFt = state.altitudeFt;
  let timeS = 0;
  let fuelKg = state.fuelKg;

  const casAtStart = speed.climbSpeedKt(altitudeFt, distanceNm);
  const machAtStart = perf.mach(casAtStart, altitudeFt, state.isaDeviationC);
  const points: VnavProfilePoint[] = [
    {
      ...makePoint(distanceNm, altitudeFt, casAtStart, machAtStart, timeS, fuelKg),
      segment: state.onGround ? 'takeoff' : 'climb',
    },
  ];

  if (altitudeFt >= state.cruiseAltitudeFt) {
    return { points, pseudoWaypoints: [], topOfClimbNm: distanceNm };
  }

  const caps = climbCaps(legs, state.cruiseAltitudeFt);
  let capIndex = 0;
  let steps = 0;

  while (altitudeFt < state.cruiseAltitudeFt - 1 && steps++ < MAX_CLIMB_STEPS) {
    while (capIndex < caps.length && caps[capIndex].distanceFromStartNm <= distanceNm) {
      capIndex++;
    }
    const nextCap = capIndex < caps.length ? caps[capIndex] : undefined;

    const grossWeightKg = state.zeroFuelWeightKg + fuelKg;
    const casKt = speed.climbSpeedKt(altitudeFt, distanceNm);

    if (nextCap && nextCap.capFt <= altitudeFt) {
      const distanceToConstraintNm = nextCap.distanceFromStartNm - distanceNm;
      if (distanceToConstraintNm <= 0) {
        capIndex++;
        continue;
      }

      const groundSpeedKt = perf.trueAirspeedKt(casKt, altitudeFt, state.isaDeviationC) + state.windAlongTrackKt;
      const timeStepS = groundSpeedKt > 0 ? (distanceToConstraintNm / groundSpeedKt) * 3600 : 0;
      const fuelBurnKg = (perf.fuelFlowKgPerHour('climb', altitudeFt, casKt, grossWeightKg, state.isaDeviationC) / 3600) * timeStepS;

      distanceNm += distanceToConstraintNm;
      timeS += timeStepS;
      fuelKg = Math.max(0, fuelKg - fuelBurnKg);

      points.push(makePoint(distanceNm, altitudeFt, casKt, perf.mach(casKt, altitudeFt, state.isaDeviationC), timeS, fuelKg, nextCap.legIndex));
      capIndex++;
      continue;
    }

    const ceilingFt = nextCap ? Math.min(nextCap.capFt, state.cruiseAltitudeFt) : state.cruiseAltitudeFt;
    const stepTargetFt = Math.min(altitudeFt + STEP_FT, ceilingFt);

    const groundSpeedKt = perf.trueAirspeedKt(casKt, altitudeFt, state.isaDeviationC) + state.windAlongTrackKt;
    const gradientFtPerNm = perf.climbGradientFtPerNm(altitudeFt, casKt, grossWeightKg, state.isaDeviationC, groundSpeedKt);

    if (gradientFtPerNm <= 0 || groundSpeedKt <= 0) {
      break;
    }

    const deltaFt = stepTargetFt - altitudeFt;
    const deltaNm = deltaFt / gradientFtPerNm;
    const timeStepS = (deltaNm / groundSpeedKt) * 3600;
    const fuelBurnKg = (perf.fuelFlowKgPerHour('climb', altitudeFt, casKt, grossWeightKg, state.isaDeviationC) / 3600) * timeStepS;

    distanceNm += deltaNm;
    altitudeFt = stepTargetFt;
    timeS += timeStepS;
    fuelKg = Math.max(0, fuelKg - fuelBurnKg);

    const legIndex = altitudeFt >= ceilingFt && nextCap && ceilingFt === nextCap.capFt ? nextCap.legIndex : undefined;
    points.push(makePoint(distanceNm, altitudeFt, casKt, perf.mach(casKt, altitudeFt, state.isaDeviationC), timeS, fuelKg, legIndex));
  }

  const reachedCruise = altitudeFt >= state.cruiseAltitudeFt - 1;
  const topOfClimbNm = reachedCruise ? distanceNm : null;

  const pseudoWaypoints: VnavPseudoWaypoint[] = reachedCruise
    ? [
        {
          ident: '(T/C)',
          distanceFromStartNm: distanceNm,
          altitudeFt: state.cruiseAltitudeFt,
        },
      ]
    : [];

  return { points, pseudoWaypoints, topOfClimbNm };
}

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

const STEP_NM = 1;
const ALT_TOLERANCE_FT = 10;
const DECEL_RATE_KT_PER_NM = 2.5;
const DEFAULT_DECEL_HEIGHT_FT = 2000;
const DEFAULT_DECEL_SPEED_KT = 210;

export interface DescentBuildInput {
  legs: VnavLeg[];
  aircraft: VnavAircraftState;
  performanceModel: VnavPerformanceModel;
  speedProfile: VnavSpeedProfile;
  fuelAtTopOfDescentKg?: number;
  timeAtTopOfDescentS?: number;
}

export interface DescentBuildResult {
  points: VnavProfilePoint[];
  pseudoWaypoints: VnavPseudoWaypoint[];
  topOfDescentNm: number | null;
  invalidReason?: string;
}

interface ConstraintWindow {
  minFt: number;
  maxFt: number;
}

function constraintWindow(leg: VnavLeg): ConstraintWindow | null {
  const c = leg.altitudeConstraint;
  if (!c) return null;
  switch (c.type) {
    case 'at':
      return { minFt: c.ft1, maxFt: c.ft1 };
    case 'atOrAbove':
      return { minFt: c.ft1, maxFt: Infinity };
    case 'atOrBelow':
      return { minFt: -Infinity, maxFt: c.ft1 };
    case 'between':
      return { minFt: c.ft2 ?? -Infinity, maxFt: c.ft1 };
    default:
      return null;
  }
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

interface AnchorPoint {
  distanceFromStartNm: number;
  altitudeFt: number;
  speedKt: number;
  isGeometricFromHere: boolean;
  legIndex?: number;
}

function idleAltitudeGainFt(
  from: Pick<AnchorPoint, 'altitudeFt' | 'speedKt'>,
  deltaNm: number,
  performanceModel: VnavPerformanceModel,
  aircraft: VnavAircraftState,
): number {
  const groundSpeedKt = performanceModel.trueAirspeedKt(from.speedKt, from.altitudeFt, aircraft.isaDeviationC) + aircraft.windAlongTrackKt;
  const gradient = performanceModel.idleDescentGradientFtPerNm(
    from.altitudeFt,
    from.speedKt,
    aircraft.grossWeightKg,
    aircraft.isaDeviationC,
    groundSpeedKt,
  );
  return gradient * deltaNm;
}

export function buildDescent(input: DescentBuildInput): DescentBuildResult {
  const { legs, aircraft, performanceModel, speedProfile } = input;

  const descentLegs = legs
    .filter((l) => !l.isMissedApproach && (l.segment === 'arrival' || l.segment === 'approach'))
    .slice()
    .sort((a, b) => a.distanceFromStartNm - b.distanceFromStartNm);

  if (descentLegs.length === 0) {
    return { points: [], pseudoWaypoints: [], topOfDescentNm: null, invalidReason: 'no arrival or approach legs in the active plan' };
  }

  const destinationElevationFt = aircraft.destinationElevationFt ?? 0;

  const approachLegs = descentLegs.filter((l) => l.segment === 'approach');
  const decelTargetLeg =
    approachLegs.find((l) => l.speedConstraintKt !== undefined || l.altitudeConstraint !== undefined) ??
    approachLegs[0] ??
    descentLegs[descentLegs.length - 1];

  const decelWindow = constraintWindow(decelTargetLeg);
  const decelAltitudeFt = decelWindow
    ? decelWindow.maxFt === Infinity
      ? decelWindow.minFt
      : decelWindow.maxFt
    : destinationElevationFt + DEFAULT_DECEL_HEIGHT_FT;

  const cleanSpeedKt = speedProfile.descentSpeedKt(decelAltitudeFt, decelTargetLeg.distanceFromStartNm);
  const approachInitialSpeedKt = Math.min(cleanSpeedKt, decelTargetLeg.speedConstraintKt ?? DEFAULT_DECEL_SPEED_KT);
  const decelDistanceNm = Math.max(0, (cleanSpeedKt - approachInitialSpeedKt) / DECEL_RATE_KT_PER_NM);
  const decelPointNm = Math.max(0, decelTargetLeg.distanceFromStartNm - decelDistanceNm);

  const decelAnchor: AnchorPoint = {
    distanceFromStartNm: decelPointNm,
    altitudeFt: decelAltitudeFt,
    speedKt: cleanSpeedKt,
    isGeometricFromHere: false,
  };

  const constrained = descentLegs.filter((l) => l.distanceFromStartNm < decelPointNm && l.altitudeConstraint);
  const anchors: AnchorPoint[] = [decelAnchor];
  let current = decelAnchor;

  for (let i = constrained.length - 1; i >= 0; i--) {
    const leg = constrained[i];
    if (leg.distanceFromStartNm >= current.distanceFromStartNm) continue;

    const window = constraintWindow(leg)!;
    const deltaNm = current.distanceFromStartNm - leg.distanceFromStartNm;
    const idleAltitudeFt = current.altitudeFt + idleAltitudeGainFt(current, deltaNm, performanceModel, aircraft);

    let altitudeFt: number;
    let geometric: boolean;
    if (idleAltitudeFt > window.maxFt + ALT_TOLERANCE_FT) {
      altitudeFt = window.maxFt;
      geometric = true;
    } else if (idleAltitudeFt < window.minFt - ALT_TOLERANCE_FT) {
      altitudeFt = window.minFt;
      geometric = true;
    } else {
      altitudeFt = clamp(idleAltitudeFt, window.minFt, window.maxFt);
      geometric = false;
    }

    const speedKt = Math.min(speedProfile.descentSpeedKt(altitudeFt, leg.distanceFromStartNm), leg.speedConstraintKt ?? Infinity);
    current = {
      distanceFromStartNm: leg.distanceFromStartNm,
      altitudeFt,
      speedKt,
      isGeometricFromHere: geometric,
      legIndex: leg.index,
    };
    anchors.push(current);
  }

  let topOfDescentNm: number | null = null;
  {
    let distanceFromStartNm = current.distanceFromStartNm;
    let altitudeFt = current.altitudeFt;
    let speedKt = current.speedKt;
    while (altitudeFt < aircraft.cruiseAltitudeFt && distanceFromStartNm > 0) {
      const gradientFtPerNm = idleAltitudeGainFt({ altitudeFt, speedKt }, 1, performanceModel, aircraft);
      if (gradientFtPerNm <= 0) {
        break;
      }
      const stepNm = Math.min(STEP_NM, distanceFromStartNm);
      distanceFromStartNm -= stepNm;
      altitudeFt = Math.min(aircraft.cruiseAltitudeFt, altitudeFt + gradientFtPerNm * stepNm);
      speedKt = speedProfile.descentSpeedKt(altitudeFt, distanceFromStartNm);
      anchors.push({ distanceFromStartNm, altitudeFt, speedKt, isGeometricFromHere: false });
    }
    topOfDescentNm = distanceFromStartNm;
  }

  anchors.reverse();

  const points: VnavProfilePoint[] = [];
  const fuelAtTodKg = input.fuelAtTopOfDescentKg ?? 0;
  const timeAtTodS = input.timeAtTopOfDescentS ?? 0;
  let fuelBurnedSinceTodKg = 0;
  let timeSinceTodS = 0;
  let prevDistanceNm = anchors[0].distanceFromStartNm;
  let prevAltitudeFt = anchors[0].altitudeFt;
  let prevSpeedKt = anchors[0].speedKt;

  const pushPoint = (distanceFromStartNm: number, altitudeFt: number, speedKt: number, legIndex: number | undefined, geometric: boolean) => {
    const segmentNm = distanceFromStartNm - prevDistanceNm;
    if (segmentNm > 0) {
      const groundSpeedKt = Math.max(
        1,
        performanceModel.trueAirspeedKt((prevSpeedKt + speedKt) / 2, (prevAltitudeFt + altitudeFt) / 2, aircraft.isaDeviationC) +
          aircraft.windAlongTrackKt,
      );
      const hours = segmentNm / groundSpeedKt;
      const fuelFlowKgPerHour = performanceModel.fuelFlowKgPerHour(
        'descent',
        (prevAltitudeFt + altitudeFt) / 2,
        (prevSpeedKt + speedKt) / 2,
        aircraft.grossWeightKg,
        aircraft.isaDeviationC,
      );
      fuelBurnedSinceTodKg += fuelFlowKgPerHour * hours;
      timeSinceTodS += hours * 3600;
    }
    points.push({
      distanceFromStartNm,
      altitudeFt,
      speedKt,
      timeFromStartS: timeAtTodS + timeSinceTodS,
      fuelRemainingKg: fuelAtTodKg - fuelBurnedSinceTodKg,
      segment: 'descent',
      legIndex,
      geometric,
    });
    prevDistanceNm = distanceFromStartNm;
    prevAltitudeFt = altitudeFt;
    prevSpeedKt = speedKt;
  };

  pushPoint(anchors[0].distanceFromStartNm, anchors[0].altitudeFt, anchors[0].speedKt, anchors[0].legIndex, false);
  for (let i = 1; i < anchors.length; i++) {
    const from = anchors[i - 1];
    const to = anchors[i];
    const spanNm = to.distanceFromStartNm - from.distanceFromStartNm;
    if (spanNm > STEP_NM) {
      const steps = Math.ceil(spanNm / STEP_NM);
      for (let s = 1; s < steps; s++) {
        const f = s / steps;
        const distanceFromStartNm = from.distanceFromStartNm + spanNm * f;
        const altitudeFt = from.altitudeFt + (to.altitudeFt - from.altitudeFt) * f;
        const speedKt = speedProfile.descentSpeedKt(altitudeFt, distanceFromStartNm);
        pushPoint(distanceFromStartNm, altitudeFt, speedKt, undefined, from.isGeometricFromHere);
      }
    }
    pushPoint(to.distanceFromStartNm, to.altitudeFt, to.speedKt, to.legIndex, from.isGeometricFromHere);
  }

  const pseudoWaypoints: VnavPseudoWaypoint[] = [
    { ident: '(T/D)', distanceFromStartNm: topOfDescentNm ?? anchors[0].distanceFromStartNm, altitudeFt: aircraft.cruiseAltitudeFt },
    { ident: '(DECEL)', distanceFromStartNm: decelPointNm, altitudeFt: decelAltitudeFt, speedKt: approachInitialSpeedKt },
  ];

  return { points, pseudoWaypoints, topOfDescentNm };
}

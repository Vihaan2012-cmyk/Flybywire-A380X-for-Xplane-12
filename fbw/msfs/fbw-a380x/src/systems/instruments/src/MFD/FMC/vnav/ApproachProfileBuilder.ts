// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { VnavAircraftState, VnavLeg, VnavPerformanceModel, VnavProfilePoint, VnavPseudoWaypoint } from './types';

const FINAL_APPROACH_ANGLE_DEG = 3.0;

const THRESHOLD_CROSSING_HEIGHT_FT = 50;

const DEFAULT_GLIDE_INTERCEPT_HEIGHT_FT = 1500;

const LEVEL_DECELERATION_KT_PER_MIN = 8;

const NM_TO_FT = 6076.12;
const DEG_TO_RAD = Math.PI / 180;

export interface ApproachProfileResult {
  points: VnavProfilePoint[];
  pseudoWaypoints: VnavPseudoWaypoint[];
  decelDistanceNm: number;
  glideInterceptDistanceNm: number;
  invalidReason?: string;
}

export function buildApproach(
  legs: VnavLeg[],
  aircraftState: VnavAircraftState,
  performanceModel: VnavPerformanceModel,
  approachSpeedKt: number,
  entryPoint: VnavProfilePoint,
): ApproachProfileResult {
  const approachLegs = legs
    .filter((leg) => !leg.isMissedApproach && leg.segment === 'approach')
    .sort((a, b) => a.distanceFromStartNm - b.distanceFromStartNm);

  if (approachLegs.length === 0) {
    return {
      points: [],
      pseudoWaypoints: [],
      decelDistanceNm: entryPoint.distanceFromStartNm,
      glideInterceptDistanceNm: entryPoint.distanceFromStartNm,
      invalidReason: 'no approach legs in the active flight plan',
    };
  }

  const runwayLeg = approachLegs[approachLegs.length - 1];
  const thresholdElevationFt = aircraftState.destinationElevationFt ?? 0;
  const thresholdDistanceNm = runwayLeg.distanceFromStartNm;
  const thresholdCrossingAltFt = thresholdElevationFt + THRESHOLD_CROSSING_HEIGHT_FT;

  if (thresholdDistanceNm <= entryPoint.distanceFromStartNm) {
    return {
      points: [],
      pseudoWaypoints: [],
      decelDistanceNm: entryPoint.distanceFromStartNm,
      glideInterceptDistanceNm: entryPoint.distanceFromStartNm,
      invalidReason: 'threshold is not downstream of the profile entry point',
    };
  }

  let interceptAltFt = thresholdElevationFt + DEFAULT_GLIDE_INTERCEPT_HEIGHT_FT;
  for (let i = approachLegs.length - 2; i >= 0; i--) {
    const constraint = approachLegs[i].altitudeConstraint;
    if (constraint) {
      interceptAltFt = constraint.ft1;
      break;
    }
  }
  interceptAltFt = Math.max(interceptAltFt, thresholdCrossingAltFt + 1);

  const glideDistanceNm =
    (interceptAltFt - thresholdCrossingAltFt) / (NM_TO_FT * Math.tan(FINAL_APPROACH_ANGLE_DEG * DEG_TO_RAD));
  const glideInterceptDistanceNm = Math.max(entryPoint.distanceFromStartNm, thresholdDistanceNm - glideDistanceNm);

  const entrySpeedKt = Math.max(entryPoint.speedKt, approachSpeedKt);
  const decelMinutes = Math.max(0, entrySpeedKt - approachSpeedKt) / LEVEL_DECELERATION_KT_PER_MIN;
  const avgGroundSpeedKt = Math.max(1, (entrySpeedKt + approachSpeedKt) / 2 + aircraftState.windAlongTrackKt);
  const decelDistanceNm = (avgGroundSpeedKt * decelMinutes) / 60;
  const decelPointDistanceNm = Math.max(entryPoint.distanceFromStartNm, glideInterceptDistanceNm - decelDistanceNm);

  const points: VnavProfilePoint[] = [];
  const pseudoWaypoints: VnavPseudoWaypoint[] = [];

  let distanceNm = entryPoint.distanceFromStartNm;
  let altitudeFt = entryPoint.altitudeFt;
  let speedKt = entryPoint.speedKt;
  let timeS = entryPoint.timeFromStartS;
  let fuelKg = entryPoint.fuelRemainingKg;

  let geometricSegment = false;
  const pushPoint = (toDistanceNm: number, toAltitudeFt: number, toSpeedKt: number, legIndex?: number) => {
    const segmentNm = Math.max(0, toDistanceNm - distanceNm);
    const gsKt = Math.max(1, (speedKt + toSpeedKt) / 2 + aircraftState.windAlongTrackKt);
    const hours = segmentNm / gsKt;
    const fuelFlowKgPerHour = performanceModel.fuelFlowKgPerHour(
      'approach',
      (altitudeFt + toAltitudeFt) / 2,
      (speedKt + toSpeedKt) / 2,
      aircraftState.grossWeightKg,
      aircraftState.isaDeviationC,
    );

    distanceNm = toDistanceNm;
    altitudeFt = toAltitudeFt;
    speedKt = toSpeedKt;
    timeS += hours * 3600;
    fuelKg = Math.max(0, fuelKg - fuelFlowKgPerHour * hours);

    points.push({
      distanceFromStartNm: distanceNm,
      altitudeFt,
      speedKt,
      timeFromStartS: timeS,
      fuelRemainingKg: fuelKg,
      segment: 'approach',
      legIndex,
      geometric: geometricSegment,
    });
  };

  if (decelPointDistanceNm > entryPoint.distanceFromStartNm + 1e-6) {
    pushPoint(decelPointDistanceNm, interceptAltFt, entryPoint.speedKt);
  }

  pseudoWaypoints.push({
    ident: '(DECEL)',
    distanceFromStartNm: decelPointDistanceNm,
    altitudeFt: interceptAltFt,
    speedKt: approachSpeedKt,
  });

  geometricSegment = true;
  if (glideInterceptDistanceNm > distanceNm + 1e-6) {
    pushPoint(glideInterceptDistanceNm, interceptAltFt, approachSpeedKt);
  }

  for (const leg of approachLegs) {
    if (leg.distanceFromStartNm > distanceNm + 1e-6 && leg.distanceFromStartNm < thresholdDistanceNm - 1e-6) {
      const legAltFt =
        thresholdCrossingAltFt +
        (thresholdDistanceNm - leg.distanceFromStartNm) * NM_TO_FT * Math.tan(FINAL_APPROACH_ANGLE_DEG * DEG_TO_RAD);
      pushPoint(leg.distanceFromStartNm, legAltFt, approachSpeedKt, leg.index);
    }
  }
  pushPoint(thresholdDistanceNm, thresholdCrossingAltFt, approachSpeedKt, runwayLeg.index);

  return { points, pseudoWaypoints, decelDistanceNm: decelPointDistanceNm, glideInterceptDistanceNm };
}

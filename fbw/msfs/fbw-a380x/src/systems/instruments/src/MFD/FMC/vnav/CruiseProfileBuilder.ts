// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import {
  VnavAircraftState,
  VnavLeg,
  VnavPerformanceModel,
  VnavProfilePoint,
  VnavPseudoWaypoint,
  VnavSpeedProfile,
} from './types';

export function buildCruise(
  aircraftState: VnavAircraftState,
  performanceModel: VnavPerformanceModel,
  speedProfile: VnavSpeedProfile,
  legs: VnavLeg[],
  topOfClimb: VnavProfilePoint,
  topOfDescent: VnavProfilePoint,
): { points: VnavProfilePoint[]; pseudoWaypoints: VnavPseudoWaypoint[] } {
  const points: VnavProfilePoint[] = [];
  const pseudoWaypoints: VnavPseudoWaypoint[] = [];

  const cruiseStartNm = topOfClimb.distanceFromStartNm;
  const cruiseEndNm = topOfDescent.distanceFromStartNm;

  if (cruiseEndNm <= cruiseStartNm) {
    return { points, pseudoWaypoints };
  }

  const cruiseLegs = legs
    .filter((leg) => !leg.isMissedApproach && leg.segment === 'enroute')
    .filter((leg) => leg.distanceFromStartNm > cruiseStartNm && leg.distanceFromStartNm < cruiseEndNm)
    .sort((a, b) => a.distanceFromStartNm - b.distanceFromStartNm);

  let distanceNm = cruiseStartNm;
  let altitudeFt = topOfClimb.altitudeFt;
  let timeS = topOfClimb.timeFromStartS;
  let fuelKg = topOfClimb.fuelRemainingKg;

  const grossWeightKg = () => aircraftState.zeroFuelWeightKg + fuelKg;

  const cruiseMach = speedProfile.cruiseMach();

  const CRUISE_STEP_NM = 10;

  const advanceCruise = (toNm: number): void => {
    while (distanceNm < toNm - 1e-6) {
      const stepNm = Math.min(CRUISE_STEP_NM, toNm - distanceNm);
      const casKt = performanceModel.casForMach(cruiseMach, altitudeFt, aircraftState.isaDeviationC);
      const tasKt = performanceModel.trueAirspeedKt(casKt, altitudeFt, aircraftState.isaDeviationC);
      const groundSpeedKt = tasKt + aircraftState.windAlongTrackKt;

      if (groundSpeedKt <= 0) {
        break;
      }

      const fuelFlowKgPerHour = performanceModel.fuelFlowKgPerHour(
        'cruise',
        altitudeFt,
        casKt,
        grossWeightKg(),
        aircraftState.isaDeviationC,
      );
      const dtS = (stepNm / groundSpeedKt) * 3600;
      const dFuelKg = (fuelFlowKgPerHour / 3600) * dtS;

      distanceNm += stepNm;
      timeS += dtS;
      fuelKg = Math.max(0, fuelKg - dFuelKg);
    }
  };

  const pushCruisePoint = (legIndex?: number): void => {
    const casKt = performanceModel.casForMach(cruiseMach, altitudeFt, aircraftState.isaDeviationC);
    points.push({
      distanceFromStartNm: distanceNm,
      altitudeFt,
      speedKt: casKt,
      mach: cruiseMach,
      timeFromStartS: timeS,
      fuelRemainingKg: fuelKg,
      segment: 'cruise',
      legIndex,
    });
  };

  const runStepClimb = (targetFt: number): void => {
    const startNm = distanceNm;
    const startFt = altitudeFt;
    const CLIMB_STEP_NM = 2;

    while (altitudeFt < targetFt) {
      const casKt = speedProfile.climbSpeedKt(altitudeFt, distanceNm);
      const tasKt = performanceModel.trueAirspeedKt(casKt, altitudeFt, aircraftState.isaDeviationC);
      const groundSpeedKt = tasKt + aircraftState.windAlongTrackKt;
      const referenceSpeedKt = groundSpeedKt > 0 ? groundSpeedKt : tasKt;
      const gradientFtPerNm = performanceModel.climbGradientFtPerNm(
        altitudeFt,
        casKt,
        grossWeightKg(),
        aircraftState.isaDeviationC,
        referenceSpeedKt,
      );

      if (gradientFtPerNm <= 0 || groundSpeedKt <= 0) {
        break;
      }

      const remainingFt = targetFt - altitudeFt;
      const stepNm = Math.min(CLIMB_STEP_NM, remainingFt / gradientFtPerNm);
      const fuelFlowKgPerHour = performanceModel.fuelFlowKgPerHour(
        'climb',
        altitudeFt,
        casKt,
        grossWeightKg(),
        aircraftState.isaDeviationC,
      );
      const dtS = (stepNm / groundSpeedKt) * 3600;
      const dFuelKg = (fuelFlowKgPerHour / 3600) * dtS;

      distanceNm += stepNm;
      altitudeFt += stepNm * gradientFtPerNm;
      timeS += dtS;
      fuelKg = Math.max(0, fuelKg - dFuelKg);
    }

    if (distanceNm > startNm) {
      pseudoWaypoints.push({ ident: '(S/C)', distanceFromStartNm: startNm, altitudeFt: startFt });
    }
  };

  for (const leg of cruiseLegs) {
    advanceCruise(leg.distanceFromStartNm);

    const constraint = leg.altitudeConstraint;
    const stepTargetFt =
      constraint && (constraint.type === 'atOrAbove' || constraint.type === 'at') && constraint.ft1 > altitudeFt
        ? constraint.ft1
        : undefined;

    if (stepTargetFt !== undefined) {
      runStepClimb(stepTargetFt);
    }

    pushCruisePoint(leg.index);
  }

  if (cruiseLegs.length === 0) {
    const FALLBACK_SAMPLE_NM = 200;
    for (let sampleNm = cruiseStartNm + FALLBACK_SAMPLE_NM; sampleNm < cruiseEndNm; sampleNm += FALLBACK_SAMPLE_NM) {
      advanceCruise(sampleNm);
      pushCruisePoint();
    }
  }

  advanceCruise(cruiseEndNm);
  pushCruisePoint();

  return { points, pseudoWaypoints };
}

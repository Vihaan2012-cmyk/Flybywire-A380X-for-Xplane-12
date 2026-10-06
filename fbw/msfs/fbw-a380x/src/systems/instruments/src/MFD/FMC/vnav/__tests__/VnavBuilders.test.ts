// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { describe, expect, it } from 'vitest';
import { buildClimb } from '../ClimbProfileBuilder';
import { buildCruise } from '../CruiseProfileBuilder';
import { buildDescent } from '../DescentProfileBuilder';
import { buildApproach } from '../ApproachProfileBuilder';
import { predict } from '../VnavPredictions';
import { A380PerformanceModel } from '../A380PerformanceModel';
import { SpeedProfile } from '../SpeedProfile';
import { buildVnavProfile, computeVnavGuidance } from '../VnavDriver';
import type {
  VnavAircraftState,
  VnavLeg,
  VnavPerformanceModel,
  VnavProfilePoint,
  VnavSegment,
  VnavSpeedProfile,
} from '../types';

function makeStubPerformanceModel(): VnavPerformanceModel {
  return {
    climbGradientFtPerNm: (_alt, _cas, _gw, _isa, gsKt) => 1500 / (Math.max(gsKt, 1) / 60),
    idleDescentGradientFtPerNm: (_alt, _cas, _gw, _isa, gsKt) => 2000 / (Math.max(gsKt, 1) / 60),
    fuelFlowKgPerHour: (segment: VnavSegment) => {
      switch (segment) {
        case 'climb':
          return 11000;
        case 'cruise':
          return 7000;
        case 'descent':
          return 2200;
        case 'approach':
          return 3200;
        default:
          return 9000;
      }
    },
    trueAirspeedKt: (casKt, altitudeFt) => casKt * (1 + altitudeFt / 100000),
    mach: (casKt, altitudeFt) => (casKt * (1 + altitudeFt / 100000)) / (661.5 - altitudeFt / 1000),
    casForMach: (mach, altitudeFt) => (mach * (661.5 - altitudeFt / 1000)) / (1 + altitudeFt / 100000),
  };
}

function makeStubSpeedProfile(): VnavSpeedProfile {
  return {
    climbSpeedKt: (altitudeFt) => (altitudeFt < 10000 ? 250 : 300),
    cruiseMach: () => 0.84,
    descentSpeedKt: (altitudeFt) => (altitudeFt < 10000 ? 250 : 300),
  };
}

function leg(partial: Partial<VnavLeg> & { index: number; distanceFromStartNm: number }): VnavLeg {
  return {
    ident: `WPT${partial.index}`,
    isMissedApproach: false,
    segment: 'enroute',
    ...partial,
  };
}

function baseState(overrides: Partial<VnavAircraftState> = {}): VnavAircraftState {
  return {
    distanceFromStartNm: 0,
    altitudeFt: 0,
    casKt: 0,
    mach: 0,
    grossWeightKg: 350000,
    fuelKg: 90000,
    zeroFuelWeightKg: 260000,
    costIndex: 100,
    cruiseAltitudeFt: 36000,
    windAlongTrackKt: 0,
    isaDeviationC: 0,
    onGround: true,
    destinationElevationFt: 0,
    ...overrides,
  };
}

function expectMonotoneDistance(points: VnavProfilePoint[]) {
  for (let i = 1; i < points.length; i++) {
    expect(points[i].distanceFromStartNm).toBeGreaterThanOrEqual(points[i - 1].distanceFromStartNm);
  }
}

function expectContinuous(a: VnavProfilePoint, b: VnavProfilePoint, altToleranceFt = 50, distToleranceNm = 0.1) {
  expect(Math.abs(a.altitudeFt - b.altitudeFt)).toBeLessThanOrEqual(altToleranceFt);
  expect(Math.abs(a.distanceFromStartNm - b.distanceFromStartNm)).toBeLessThanOrEqual(distToleranceNm);
}

const SYNTHETIC_LEGS: VnavLeg[] = [
  leg({ index: 0, distanceFromStartNm: 15, segment: 'departure' }),
  leg({ index: 1, distanceFromStartNm: 60, segment: 'enroute' }),
  leg({
    index: 2,
    distanceFromStartNm: 150,
    segment: 'enroute',
    altitudeConstraint: { type: 'atOrAbove', ft1: 30000 },
  }),
  leg({ index: 3, distanceFromStartNm: 300, segment: 'enroute' }),
  leg({
    index: 4,
    distanceFromStartNm: 420,
    segment: 'arrival',
    altitudeConstraint: { type: 'atOrBelow', ft1: 10000 },
    speedConstraintKt: 250,
  }),
  leg({
    index: 5,
    distanceFromStartNm: 440,
    segment: 'approach',
    altitudeConstraint: { type: 'between', ft1: 4000, ft2: 3000 },
  }),
  leg({ index: 6, distanceFromStartNm: 450, segment: 'approach' }),
];

const DESTINATION_NM = 450;

const VAPP_KT = 140;

function stubs() {
  return { perf: makeStubPerformanceModel(), speeds: makeStubSpeedProfile() };
}

function descent(state: VnavAircraftState, extra: { fuelAtTopOfDescentKg?: number; timeAtTopOfDescentS?: number } = {}) {
  const { perf, speeds } = stubs();
  return buildDescent({ legs: SYNTHETIC_LEGS, aircraft: state, performanceModel: perf, speedProfile: speeds, ...extra });
}

describe('VNAV profile builders (synthetic plans)', () => {
  it('buildClimb produces a monotone climb from the origin to the cruise altitude', () => {
    const { perf, speeds } = stubs();
    const state = baseState({ cruiseAltitudeFt: 36000, onGround: true });

    const climb = buildClimb(state, SYNTHETIC_LEGS, perf, speeds);

    expect(climb.points.length).toBeGreaterThan(0);
    for (let i = 1; i < climb.points.length; i++) {
      expect(climb.points[i].altitudeFt).toBeGreaterThanOrEqual(climb.points[i - 1].altitudeFt);
    }
    expectMonotoneDistance(climb.points);
    const topOfClimb = climb.points[climb.points.length - 1];
    expect(topOfClimb.altitudeFt).toBe(36000);
    expect(climb.topOfClimbNm).not.toBeNull();
    expect(climb.topOfClimbNm!).toBeCloseTo(topOfClimb.distanceFromStartNm, 3);
    expect(climb.pseudoWaypoints.some((w) => w.ident === '(T/C)')).toBe(true);
  });

  it('buildDescent builds backwards from the destination with T/D short of it', () => {
    const state = baseState({ cruiseAltitudeFt: 36000, destinationElevationFt: 0 });

    const result = descent(state);

    expect(result.invalidReason).toBeUndefined();
    expect(result.points.length).toBeGreaterThan(0);
    for (let i = 1; i < result.points.length; i++) {
      expect(result.points[i].altitudeFt).toBeLessThanOrEqual(result.points[i - 1].altitudeFt);
    }
    expectMonotoneDistance(result.points);
    expect(result.topOfDescentNm).not.toBeNull();
    expect(result.topOfDescentNm!).toBeLessThan(DESTINATION_NM);
    expect(result.points[0].altitudeFt).toBe(36000);
    expect(result.points[result.points.length - 1].distanceFromStartNm).toBeLessThanOrEqual(DESTINATION_NM);
  });

  it('buildDescent honours an atOrBelow constraint on the arrival leg', () => {
    const result = descent(baseState({ cruiseAltitudeFt: 36000, destinationElevationFt: 0 }));
    const arrivalPoint = result.points.find((p) => p.legIndex === 4);
    expect(arrivalPoint).toBeDefined();
    expect(arrivalPoint!.altitudeFt).toBeLessThanOrEqual(10000);
  });

  it('buildDescent with the fuel and time at T/D carries them through the descent', () => {
    const state = baseState({ cruiseAltitudeFt: 36000, destinationElevationFt: 0 });
    const result = descent(state, { fuelAtTopOfDescentKg: 20000, timeAtTopOfDescentS: 3600 });
    expect(result.points[0].fuelRemainingKg).toBeCloseTo(20000, 0);
    expect(result.points[0].timeFromStartS).toBeCloseTo(3600, 0);
    for (let i = 1; i < result.points.length; i++) {
      expect(result.points[i].timeFromStartS).toBeGreaterThanOrEqual(result.points[i - 1].timeFromStartS);
      expect(result.points[i].fuelRemainingKg).toBeLessThanOrEqual(result.points[i - 1].fuelRemainingKg);
    }
  });

  it('buildCruise holds the cruise level between T/C and T/D', () => {
    const { perf, speeds } = stubs();
    const state = baseState({ cruiseAltitudeFt: 36000, destinationElevationFt: 0 });
    const climb = buildClimb(state, SYNTHETIC_LEGS, perf, speeds);
    const probe = descent(state);
    const topOfClimb = climb.points[climb.points.length - 1];
    const topOfDescent = probe.points[0];

    const cruise = buildCruise(state, perf, speeds, SYNTHETIC_LEGS, topOfClimb, topOfDescent);

    expect(cruise.points.length).toBeGreaterThan(0);
    for (const p of cruise.points) {
      expect(p.altitudeFt).toBe(36000);
    }
    expectMonotoneDistance(cruise.points);
    expect(cruise.points[0].distanceFromStartNm).toBeGreaterThanOrEqual(topOfClimb.distanceFromStartNm);
    expectContinuous(cruise.points[cruise.points.length - 1], topOfDescent);
  });

  it('buildApproach continues from the descent end to the threshold, slowing to Vapp', () => {
    const { perf } = stubs();
    const state = baseState({ cruiseAltitudeFt: 36000, destinationElevationFt: 0 });
    const end = descent(state).points;
    const entry = end[end.length - 1];

    const approach = buildApproach(SYNTHETIC_LEGS, state, perf, VAPP_KT, entry);

    expect(approach.invalidReason).toBeUndefined();
    expect(approach.points.length).toBeGreaterThan(0);
    expectMonotoneDistance(approach.points);
    for (let i = 1; i < approach.points.length; i++) {
      expect(approach.points[i].altitudeFt).toBeLessThanOrEqual(approach.points[i - 1].altitudeFt);
    }
    const threshold = approach.points[approach.points.length - 1];
    expect(threshold.distanceFromStartNm).toBeCloseTo(DESTINATION_NM, 1);
    expect(threshold.altitudeFt).toBeLessThanOrEqual(state.destinationElevationFt! + 60);
    expect(threshold.speedKt).toBeCloseTo(VAPP_KT, 0);
  });

  it('a full profile is distance-, time- and fuel-monotone throughout', () => {
    const { perf } = stubs();
    const state = baseState({ cruiseAltitudeFt: 36000, destinationElevationFt: 0, fuelKg: 90000 });

    const profile = buildVnavProfile(state, SYNTHETIC_LEGS, perf, VAPP_KT, 0);

    expect(profile.invalidReason).toBeUndefined();
    expectMonotoneDistance(profile.points);
    for (let i = 1; i < profile.points.length; i++) {
      expect(profile.points[i].timeFromStartS).toBeGreaterThanOrEqual(profile.points[i - 1].timeFromStartS);
      expect(profile.points[i].fuelRemainingKg).toBeLessThanOrEqual(profile.points[i - 1].fuelRemainingKg);
    }
    expect(profile.topOfClimbNm!).toBeLessThan(profile.topOfDescentNm!);
  });

  it('an empty plan yields an invalid profile instead of throwing', () => {
    const { perf, speeds } = stubs();
    const state = baseState();
    expect(() => buildClimb(state, [], perf, speeds)).not.toThrow();
    expect(buildVnavProfile(state, [], perf, VAPP_KT, 0).invalidReason).toBe('noLegs');
    expect(buildVnavProfile(baseState({ cruiseAltitudeFt: 0 }), SYNTHETIC_LEGS, perf, VAPP_KT, 0).invalidReason).toBe('noCruiseAltitude');
  });
});

describe('VnavPredictions.predict', () => {
  it('produces one prediction per non-missed-approach leg, monotone in time and fuel', () => {
    const { perf } = stubs();
    const state = baseState({ cruiseAltitudeFt: 36000, destinationElevationFt: 0, fuelKg: 90000 });
    const profile = buildVnavProfile(state, SYNTHETIC_LEGS, perf, VAPP_KT, 0);

    const predictions = predict(profile, SYNTHETIC_LEGS);
    const nonMissed = SYNTHETIC_LEGS.filter((l) => !l.isMissedApproach);
    expect(predictions.length).toBe(nonMissed.length);
    for (let i = 1; i < predictions.length; i++) {
      expect(predictions[i].timeFromStartS).toBeGreaterThanOrEqual(predictions[i - 1].timeFromStartS);
      expect(predictions[i].fuelRemainingKg).toBeLessThanOrEqual(predictions[i - 1].fuelRemainingKg);
    }
    const atOrAbove = predictions.find((p) => p.legIndex === 2);
    expect(atOrAbove && atOrAbove.altitudeFt).toBeGreaterThanOrEqual(30000);
    const atOrBelow = predictions.find((p) => p.legIndex === 4);
    expect(atOrBelow && atOrBelow.altitudeFt).toBeLessThanOrEqual(10000);
  });
});

describe('VNAV on the A380 performance model', () => {
  const perf = new A380PerformanceModel();
  const state = baseState({ grossWeightKg: 500_000, fuelKg: 150_000, zeroFuelWeightKg: 350_000, cruiseAltitudeFt: 36000 });

  it('builds a valid profile and a managed speed schedule inside the A380 envelope', () => {
    const profile = buildVnavProfile(state, SYNTHETIC_LEGS, perf, VAPP_KT, 0);
    expect(profile.invalidReason).toBeUndefined();
    const speeds = new SpeedProfile(state, perf, SYNTHETIC_LEGS);
    expect(speeds.climbSpeedKt(5000, 10)).toBeLessThanOrEqual(250);
    expect(speeds.cruiseMach()).toBeGreaterThan(0.78);
    expect(speeds.cruiseMach()).toBeLessThanOrEqual(0.89);
    for (const p of profile.points) {
      expect(p.speedKt).toBeGreaterThan(100);
      expect(p.speedKt).toBeLessThanOrEqual(340);
    }
  });

  it('guidance: distance to T/C in the climb, deviation and a descending target V/S past T/D', () => {
    const profile = buildVnavProfile(state, SYNTHETIC_LEGS, perf, VAPP_KT, 0);
    const inClimb = computeVnavGuidance(profile, { ...state, onGround: false, distanceFromStartNm: 5, altitudeFt: 3000, casKt: 250 }, perf);
    expect(inClimb.distanceToTopOfClimbNm!).toBeCloseTo(profile.topOfClimbNm! - 5, 3);
    expect(inClimb.verticalDeviationFt).toBeNull();

    const d = profile.topOfDescentNm! + 20;
    const pathAt = profile.points.find((p) => p.distanceFromStartNm >= d)!;
    const high = computeVnavGuidance(
      profile,
      { ...state, onGround: false, distanceFromStartNm: d, altitudeFt: pathAt.altitudeFt + 1000, casKt: 290 },
      perf,
    );
    expect(high.verticalDeviationFt!).toBeGreaterThan(500);
    expect(high.targetVerticalSpeedFpm!).toBeLessThan(0);
    expect(high.distanceToTopOfDescentNm).toBeNull();
  });
});

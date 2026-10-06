// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { VnavAircraftState, VnavLeg, VnavManagedSpeeds, VnavPerformanceModel, VnavSpeedProfile } from './types';

const SPEED_LIMIT_ALTITUDE_FT = 10_000;
const SPEED_LIMIT_KT = 250;

export function fbwManagedSpeeds(costIndex: number): VnavManagedSpeeds {
  const d = Math.max(0, Math.min(1, costIndex / 999));
  return {
    climbCasKt: 290 * (1 - d ** 2) + 330 * d ** 2,
    climbMach: 0.84,
    cruiseMach: 0.85,
    descentCasKt: Math.round(288 * (1 - d) + 300 * d),
    descentMach: 0.84,
  };
}

export class SpeedProfile implements VnavSpeedProfile {
  private readonly speeds: VnavManagedSpeeds;

  constructor(
    private readonly aircraftState: VnavAircraftState,
    private readonly performanceModel: VnavPerformanceModel,
    private readonly legs: readonly VnavLeg[],
  ) {
    this.speeds = aircraftState.managedSpeeds ?? fbwManagedSpeeds(aircraftState.costIndex);
  }

  cruiseMach(): number {
    return this.speeds.cruiseMach;
  }

  climbSpeedKt(altitudeFt: number, distanceFromStartNm: number): number {
    const casKt = this.crossoverCasKt(altitudeFt, this.speeds.climbCasKt, this.speeds.climbMach);
    const limited = altitudeFt < SPEED_LIMIT_ALTITUDE_FT ? Math.min(casKt, SPEED_LIMIT_KT) : casKt;
    return this.applyConstraints(limited, distanceFromStartNm, ['departure', 'enroute']);
  }

  descentSpeedKt(altitudeFt: number, distanceFromStartNm: number): number {
    const casKt = this.crossoverCasKt(altitudeFt, this.speeds.descentCasKt, this.speeds.descentMach);
    const limited = altitudeFt < SPEED_LIMIT_ALTITUDE_FT ? Math.min(casKt, SPEED_LIMIT_KT) : casKt;
    return this.applyConstraints(limited, distanceFromStartNm, ['enroute', 'arrival', 'approach']);
  }

  private crossoverCasKt(altitudeFt: number, constantCasKt: number, targetMach: number): number {
    const machEquivalentCasKt = this.performanceModel.casForMach(targetMach, altitudeFt, this.aircraftState.isaDeviationC);
    return Math.min(constantCasKt, machEquivalentCasKt);
  }

  private applyConstraints(speedKt: number, distanceFromStartNm: number, segments: ReadonlyArray<VnavLeg['segment']>): number {
    let limit = speedKt;
    for (const leg of this.legs) {
      if (leg.isMissedApproach || leg.speedConstraintKt === undefined) {
        continue;
      }
      if (!segments.includes(leg.segment)) {
        continue;
      }
      if (leg.distanceFromStartNm >= distanceFromStartNm) {
        limit = Math.min(limit, leg.speedConstraintKt);
      }
    }
    return limit;
  }
}

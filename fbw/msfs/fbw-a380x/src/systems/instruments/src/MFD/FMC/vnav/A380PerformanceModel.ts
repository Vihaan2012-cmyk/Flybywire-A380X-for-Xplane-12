// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { Common, FlapConf } from '@fmgc/guidance/vnav/common';
import { EngineModel } from '@fmgc/guidance/vnav/EngineModel';
import { FlightModel } from '@fmgc/guidance/vnav/FlightModel';
import { VnavSegment } from './types';
import { FbwA380PerformanceModel } from './FbwA380PerformanceModel';

export const MTOW_KG = 575_000;

const LB_PER_KG = 2.204_622_62;

export const TRENT_900_CRUISE_TSFC = 0.56;
const TSFC_REFERENCE_MACH = 0.85;
const TSFC_REFERENCE_THETA = 0.7594;

export function trent900Tsfc(mach: number, theta: number): number {
  const shape = (m: number, t: number) => (0.4 + 0.45 * m) * Math.sqrt(t);
  return (TRENT_900_CRUISE_TSFC * shape(mach, theta)) / shape(TSFC_REFERENCE_MACH, TSFC_REFERENCE_THETA);
}

export class A380PerformanceModel extends FbwA380PerformanceModel {
  fuelFlowKgPerHour(segment: VnavSegment, altitudeFt: number, casKt: number, grossWeightKg: number, isaDeviationC: number): number {
    if (segment !== 'cruise' && segment !== 'climb' && segment !== 'takeoff') {
      return super.fuelFlowKgPerHour(segment, altitudeFt, casKt, grossWeightKg, isaDeviationC);
    }
    const aboveTropopause = altitudeFt > this.tropopauseFt;
    const theta = Common.getTheta(altitudeFt, isaDeviationC, aboveTropopause);
    const delta = Common.getDelta(altitudeFt, aboveTropopause);
    const mach = Common.CAStoMach(casKt, delta);
    return (trent900Tsfc(mach, theta) * this.thrustLbf(segment, altitudeFt, mach, delta, grossWeightKg, isaDeviationC)) / LB_PER_KG;
  }

  private thrustLbf(segment: VnavSegment, altitudeFt: number, mach: number, delta: number, grossWeightKg: number, isaDeviationC: number): number {
    if (segment === 'cruise') {
      return FlightModel.getDrag(this.config.flightModelParameters, grossWeightKg * LB_PER_KG, mach, delta, false, false, FlapConf.CLEAN);
    }
    const engine = this.config.engineModelParameters;
    const correctedThrust =
      EngineModel.tableInterpolation(engine.table1506, this.climbCorrectedN1(altitudeFt, isaDeviationC), mach) *
      engine.numberOfEngines *
      engine.maxThrust;
    return EngineModel.getUncorrectedThrust(correctedThrust, Common.getDelta2(delta, mach));
  }
}

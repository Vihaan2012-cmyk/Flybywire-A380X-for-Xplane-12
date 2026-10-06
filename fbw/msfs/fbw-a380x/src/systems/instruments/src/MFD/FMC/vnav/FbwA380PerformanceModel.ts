// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { A380AircraftConfig } from '@fmgc/flightplanning/A380AircraftConfig';
import { AircraftConfig } from '@fmgc/flightplanning/AircraftConfigTypes';
import { Predictions, StepResults } from '@fmgc/guidance/vnav/Predictions';
import { EngineModel } from '@fmgc/guidance/vnav/EngineModel';
import { Common } from '@fmgc/guidance/vnav/common';
import { VnavPerformanceModel, VnavSegment } from './types';

const LB_PER_KG = 2.204_622_62;
export const DEFAULT_TROPOPAUSE_FT = 36_090;
const SAMPLE_STEP_FT = 500;
const CRUISE_SAMPLE_NM = 10;
const NO_MACH_CAP = 0.99;

export class FbwA380PerformanceModel implements VnavPerformanceModel {
  constructor(
    protected readonly config: AircraftConfig = A380AircraftConfig,
    protected readonly tropopauseFt: number = DEFAULT_TROPOPAUSE_FT,
  ) {}

  climbGradientFtPerNm(altitudeFt: number, casKt: number, grossWeightKg: number, isaDeviationC: number, groundSpeedKt: number): number {
    const step = this.climbStep(altitudeFt, casKt, grossWeightKg, isaDeviationC);
    return Math.max(0, step.verticalSpeed / (Math.max(groundSpeedKt, 1) / 60));
  }

  idleDescentGradientFtPerNm(
    altitudeFt: number,
    casKt: number,
    grossWeightKg: number,
    isaDeviationC: number,
    groundSpeedKt: number,
  ): number {
    const step = this.idleStep(altitudeFt, casKt, grossWeightKg, isaDeviationC);
    return Math.max(0, -step.verticalSpeed / (Math.max(groundSpeedKt, 1) / 60));
  }

  fuelFlowKgPerHour(segment: VnavSegment, altitudeFt: number, casKt: number, grossWeightKg: number, isaDeviationC: number): number {
    let step: StepResults;
    switch (segment) {
      case 'takeoff':
      case 'climb':
        step = this.climbStep(altitudeFt, casKt, grossWeightKg, isaDeviationC);
        break;
      case 'cruise':
        step = Predictions.levelFlightStep(
          this.config,
          altitudeFt,
          CRUISE_SAMPLE_NM,
          casKt,
          NO_MACH_CAP,
          grossWeightKg * LB_PER_KG,
          0,
          0,
          isaDeviationC,
          this.tropopauseFt,
        );
        break;
      default:
        step = this.idleStep(altitudeFt, casKt, grossWeightKg, isaDeviationC);
        break;
    }
    if (!(step.timeElapsed > 0)) {
      return 0;
    }
    return ((step.fuelBurned / step.timeElapsed) * 3600) / LB_PER_KG;
  }

  trueAirspeedKt(casKt: number, altitudeFt: number, isaDeviationC: number): number {
    const above = altitudeFt > this.tropopauseFt;
    return Common.CAStoTAS(casKt, Common.getTheta(altitudeFt, isaDeviationC, above), Common.getDelta(altitudeFt, above));
  }

  mach(casKt: number, altitudeFt: number, _isaDeviationC: number): number {
    return Common.CAStoMach(casKt, Common.getDelta(altitudeFt, altitudeFt > this.tropopauseFt));
  }

  casForMach(mach: number, altitudeFt: number, _isaDeviationC: number): number {
    return Common.machToCas(mach, Common.getDelta(altitudeFt, altitudeFt > this.tropopauseFt));
  }

  protected staticAirTemperatureC(altitudeFt: number, isaDeviationC: number): number {
    return Common.getTemp(altitudeFt, isaDeviationC, altitudeFt > this.tropopauseFt);
  }

  protected climbCorrectedN1(altitudeFt: number, isaDeviationC: number): number {
    return EngineModel.getClimbThrustCorrectedN1(
      this.config.engineModelParameters,
      altitudeFt,
      this.staticAirTemperatureC(altitudeFt, isaDeviationC),
    );
  }

  protected idleCorrectedN1(altitudeFt: number, casKt: number): number {
    return (
      EngineModel.getIdleCorrectedN1(this.config.engineModelParameters, altitudeFt, this.mach(casKt, altitudeFt, 0), this.tropopauseFt) +
      this.config.vnavConfig.IDLE_N1_MARGIN
    );
  }

  protected climbStep(altitudeFt: number, casKt: number, grossWeightKg: number, isaDeviationC: number): StepResults {
    return Predictions.altitudeStep(
      this.config,
      altitudeFt - SAMPLE_STEP_FT / 2,
      SAMPLE_STEP_FT,
      casKt,
      NO_MACH_CAP,
      this.climbCorrectedN1(altitudeFt, isaDeviationC),
      grossWeightKg * LB_PER_KG,
      0,
      0,
      isaDeviationC,
      this.tropopauseFt,
    );
  }

  protected idleStep(altitudeFt: number, casKt: number, grossWeightKg: number, isaDeviationC: number): StepResults {
    return Predictions.altitudeStep(
      this.config,
      altitudeFt + SAMPLE_STEP_FT / 2,
      -SAMPLE_STEP_FT,
      casKt,
      NO_MACH_CAP,
      this.idleCorrectedN1(altitudeFt, casKt),
      grossWeightKg * LB_PER_KG,
      0,
      0,
      isaDeviationC,
      this.tropopauseFt,
    );
  }
}

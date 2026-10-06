// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { Arinc429Word } from '@flybywiresim/fbw-sdk';
import { AircraftConfig } from '@fmgc/flightplanning/AircraftConfigTypes';
import { RequestedVerticalMode } from '@fmgc/guidance/ControlLaws';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { SpeedMargin } from '@fmgc/guidance/vnav/descent/SpeedMargin';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { VerticalMode } from '@shared/autopilot';
import { FmgcFlightPhase } from '@shared/flightphase';
import { VnavGuidanceOutput } from './types';

export const FBW_GUIDANCE_LVAR = 'L:A32NX_FM_VNAV_FBW_GUIDANCE';

export const enum PathCaptureState {
  OffPath,
  OnPath,
  InPathCapture,
}

const PATH_CAPTURE = {
  pathCaptureGain: 0.1,
  fallbackPathCaptureDeviation: 100,
  minCaptureDeviation: 50,
  maxCaptureDeviation: 500,
};

export interface DescentGuidanceInputs {
  linearDeviationFt: number;
  pressureAltitudeFt: number;
  pathVerticalSpeedFpm: number;
  pathAngleDeg: number;
  pathIsGeometric: boolean;
  pastTopOfDescent: boolean;
  verticalSpeedFpm: number | null;
  isHoldActive: boolean;
  isSpeedAuto: boolean;
  isApproachPhase: boolean;
  isAboveSpeedLimitAltitude: boolean;
  isCloseToAirfieldElevation: boolean;
  isInOverspeed: boolean;
  isInUnderspeed: boolean;
}

export interface DescentGuidanceDecision {
  mode: RequestedVerticalMode;
  targetAltitudeFt: number;
  targetVerticalSpeed: number;
  pathCaptureState: PathCaptureState;
}

function pathCaptureConditionMet(i: DescentGuidanceInputs): boolean {
  if (i.verticalSpeedFpm === null) {
    return Math.abs(i.linearDeviationFt) < PATH_CAPTURE.fallbackPathCaptureDeviation;
  }
  return (
    Math.abs(i.linearDeviationFt) <
    Math.min(
      PATH_CAPTURE.maxCaptureDeviation,
      Math.max(PATH_CAPTURE.minCaptureDeviation, PATH_CAPTURE.pathCaptureGain * Math.abs(i.verticalSpeedFpm - i.pathVerticalSpeedFpm)),
    )
  );
}

function nextPathCaptureState(i: DescentGuidanceInputs, state: PathCaptureState): PathCaptureState {
  const allowCapture = pathCaptureConditionMet(i);
  switch (state) {
    case PathCaptureState.OffPath:
      return allowCapture ? PathCaptureState.InPathCapture : state;
    case PathCaptureState.OnPath:
      return Math.abs(i.linearDeviationFt) > PATH_CAPTURE.minCaptureDeviation && !allowCapture ? PathCaptureState.OffPath : state;
    case PathCaptureState.InPathCapture:
      if (!allowCapture) {
        return PathCaptureState.OffPath;
      }
      return Math.abs(i.linearDeviationFt) < PATH_CAPTURE.minCaptureDeviation ? PathCaptureState.OnPath : state;
    default:
      return state;
  }
}

export function decideDescentGuidance(i: DescentGuidanceInputs, previous: PathCaptureState): DescentGuidanceDecision {
  const pathCaptureState = nextPathCaptureState(i, previous);
  const offPath = pathCaptureState === PathCaptureState.OffPath;
  const targetAltitudeFt = i.pressureAltitudeFt - i.linearDeviationFt;
  let mode: RequestedVerticalMode;
  let targetVerticalSpeed = i.pathVerticalSpeedFpm;

  if ((!i.isHoldActive && offPath && i.linearDeviationFt > 0) || i.isInOverspeed) {
    mode = RequestedVerticalMode.SpeedThrust;
  } else if (offPath || !i.pastTopOfDescent || i.isHoldActive) {
    if (i.isHoldActive) {
      mode = RequestedVerticalMode.VsSpeed;
      targetVerticalSpeed = -1000;
    } else if (i.pathIsGeometric && i.pastTopOfDescent) {
      mode = RequestedVerticalMode.FpaSpeed;
      targetVerticalSpeed = i.pathAngleDeg / 2;
    } else {
      mode = RequestedVerticalMode.VsSpeed;
      targetVerticalSpeed = i.isAboveSpeedLimitAltitude && !i.isCloseToAirfieldElevation ? -1000 : -500;
    }
  } else if (!i.pathIsGeometric && i.isSpeedAuto && !i.isInUnderspeed && !i.isApproachPhase) {
    mode = RequestedVerticalMode.VpathThrust;
  } else {
    mode = RequestedVerticalMode.VpathSpeed;
  }
  return { mode, targetAltitudeFt, targetVerticalSpeed, pathCaptureState };
}

export function nextSpeedConditions(
  airspeedKt: number,
  lowerLimitKt: number,
  upperLimitKt: number,
  previous: { overspeed: boolean; underspeed: boolean },
): { overspeed: boolean; underspeed: boolean } {
  let { overspeed, underspeed } = previous;
  if (overspeed && airspeedKt < upperLimitKt) {
    overspeed = false;
  } else if (!overspeed && airspeedKt > upperLimitKt + 5) {
    overspeed = true;
    underspeed = false;
  }
  if (!underspeed && airspeedKt < lowerLimitKt) {
    underspeed = true;
    overspeed = false;
  } else if (underspeed && airspeedKt > lowerLimitKt + 5) {
    underspeed = false;
  }
  return { overspeed, underspeed };
}

export interface AutoflightGuidanceHost {
  vnavGuidance(): VnavGuidanceOutput | null;
  readonly observer: VerticalProfileComputationParametersObserver;
  readonly atmosphericConditions: AtmosphericConditions;
  isManualHoldActive(): boolean;
}

export class AutoflightGuidanceLink {
  private pathCaptureState = PathCaptureState.OffPath;

  private speedConditions = { overspeed: false, underspeed: false };

  private readonly speedMargin: SpeedMargin;

  constructor(
    private readonly host: AutoflightGuidanceHost,
    config: AircraftConfig,
  ) {
    this.speedMargin = new SpeedMargin(config, host.observer);
  }

  update(): void {
    const g = this.host.vnavGuidance();
    const params = this.host.observer.get();
    if (
      SimVar.GetSimVarValue(FBW_GUIDANCE_LVAR, 'number') === 1 ||
      !g ||
      g.pathAltitudeFt === null ||
      g.pathAngleDeg === null ||
      params.fcuVerticalMode !== VerticalMode.DES
    ) {
      this.pathCaptureState = PathCaptureState.OffPath;
      this.speedConditions = { overspeed: false, underspeed: false };
      return;
    }

    const isSpeedAuto = Simplane.getAutoPilotAirspeedManaged();
    const managedSpeedTarget =
      params.flightPhase === FmgcFlightPhase.Approach
        ? SimVar.GetSimVarValue('L:A32NX_SPEEDS_MANAGED_ATHR', 'knots')
        : Math.round(
            Math.min(
              SimVar.GetSimVarValue('L:A32NX_SPEEDS_MANAGED_PFD', 'knots'),
              SimVar.GetGameVarValue('FROM MACH TO KIAS', 'number', params.managedDescentSpeedMach),
            ),
          );
    const fcuSpeedIas = params.fcuSpeed < 1 ? SimVar.GetGameVarValue('FROM MACH TO KIAS', 'number', params.fcuSpeed) : params.fcuSpeed;
    const speedTarget = isSpeedAuto ? managedSpeedTarget : fcuSpeedIas;

    const isHoldActive = this.host.isManualHoldActive();
    const showMargins = params.flightPhase === FmgcFlightPhase.Descent && !isHoldActive && isSpeedAuto;
    const [lower, upper] = showMargins ? this.speedMargin.getMargins(speedTarget) : [speedTarget - 5, speedTarget];
    this.speedConditions = nextSpeedConditions(this.host.atmosphericConditions.currentAirspeed, lower, upper, this.speedConditions);

    const presentAltitudeFt = params.presentPosition.alt;
    const decision = decideDescentGuidance(
      {
        linearDeviationFt: presentAltitudeFt - g.pathAltitudeFt,
        pressureAltitudeFt: this.host.atmosphericConditions.currentPressureAltitude,
        pathVerticalSpeedFpm: g.targetVerticalSpeedFpm ?? 0,
        pathAngleDeg: g.pathAngleDeg,
        pathIsGeometric: g.pathIsGeometric,
        pastTopOfDescent: g.pastTopOfDescent,
        verticalSpeedFpm: verticalSpeed(),
        isHoldActive,
        isSpeedAuto,
        isApproachPhase: params.flightPhase === FmgcFlightPhase.Approach,
        isAboveSpeedLimitAltitude: presentAltitudeFt > (params.descentSpeedLimit?.underAltitude ?? Infinity),
        isCloseToAirfieldElevation: presentAltitudeFt < params.destinationElevation + 5000,
        isInOverspeed: this.speedConditions.overspeed,
        isInUnderspeed: this.speedConditions.underspeed,
      },
      this.pathCaptureState,
    );
    this.pathCaptureState = decision.pathCaptureState;

    SimVar.SetSimVarValue('L:A32NX_FG_REQUESTED_VERTICAL_MODE', 'Enum', decision.mode);
    SimVar.SetSimVarValue('L:A32NX_FG_TARGET_ALTITUDE', 'Feet', decision.targetAltitudeFt);
    SimVar.SetSimVarValue('L:A32NX_FG_TARGET_VERTICAL_SPEED', 'number', decision.targetVerticalSpeed);
    SimVar.SetSimVarValue('L:A32NX_PFD_TARGET_ALTITUDE', 'Feet', g.pathAltitudeFt);

    if (showMargins) {
      let guidanceTarget = speedTarget;
      if (decision.mode === RequestedVerticalMode.SpeedThrust && !this.speedConditions.overspeed) {
        guidanceTarget = upper;
      } else if (
        decision.mode === RequestedVerticalMode.VpathThrust ||
        (decision.mode === RequestedVerticalMode.VpathSpeed && !g.pathIsGeometric)
      ) {
        guidanceTarget = lower;
      }
      SimVar.SetSimVarValue('L:A32NX_SPEEDS_MANAGED_ATHR', 'knots', guidanceTarget);
    }
  }
}

function verticalSpeed(): number | null {
  const inertialVs = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_IR_1_VERTICAL_SPEED');
  if (inertialVs.isNormalOperation()) {
    return inertialVs.value;
  }
  const barometricVs = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_1_BAROMETRIC_VERTICAL_SPEED');
  return barometricVs.isNormalOperation() ? barometricVs.value : null;
}

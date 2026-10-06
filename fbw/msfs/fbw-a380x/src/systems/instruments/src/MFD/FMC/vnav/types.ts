// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

export type VnavSegment = 'takeoff' | 'climb' | 'cruise' | 'descent' | 'approach';

export interface VnavLeg {
  index: number;
  ident: string;
  distanceFromStartNm: number;
  altitudeConstraint?: {
    type: 'at' | 'atOrAbove' | 'atOrBelow' | 'between';
    ft1: number;
    ft2?: number;
  };
  speedConstraintKt?: number;
  isMissedApproach: boolean;
  segment: 'departure' | 'enroute' | 'arrival' | 'approach';
}

export interface VnavProfilePoint {
  distanceFromStartNm: number;
  altitudeFt: number;
  speedKt: number;
  mach?: number;
  timeFromStartS: number;
  fuelRemainingKg: number;
  segment: VnavSegment;
  legIndex?: number;
  geometric?: boolean;
}

export type VnavPseudoWaypointIdent = '(T/C)' | '(T/D)' | '(S/C)' | '(S/D)' | '(DECEL)' | '(LIM)' | '(SPD)';

export interface VnavPseudoWaypoint {
  ident: VnavPseudoWaypointIdent;
  distanceFromStartNm: number;
  altitudeFt: number;
  speedKt?: number;
}

export interface VnavProfile {
  points: VnavProfilePoint[];
  pseudoWaypoints: VnavPseudoWaypoint[];
  topOfClimbNm?: number;
  topOfDescentNm?: number;
  cruiseAltitudeFt: number;
  computedAtSimTimeS: number;
  invalidReason?: string;
}

export interface VnavAircraftState {
  distanceFromStartNm: number;
  altitudeFt: number;
  casKt: number;
  mach: number;
  grossWeightKg: number;
  fuelKg: number;
  zeroFuelWeightKg: number;
  costIndex: number;
  cruiseAltitudeFt: number;
  windAlongTrackKt: number;
  isaDeviationC: number;
  onGround: boolean;
  destinationElevationFt?: number;
  managedSpeeds?: VnavManagedSpeeds;
}

export interface VnavManagedSpeeds {
  climbCasKt: number;
  climbMach: number;
  cruiseMach: number;
  descentCasKt: number;
  descentMach: number;
}

export interface VnavPerformanceModel {
  climbGradientFtPerNm(altitudeFt: number, casKt: number, grossWeightKg: number, isaDeviationC: number, groundSpeedKt: number): number;
  idleDescentGradientFtPerNm(altitudeFt: number, casKt: number, grossWeightKg: number, isaDeviationC: number, groundSpeedKt: number): number;
  fuelFlowKgPerHour(segment: VnavSegment, altitudeFt: number, casKt: number, grossWeightKg: number, isaDeviationC: number): number;
  trueAirspeedKt(casKt: number, altitudeFt: number, isaDeviationC: number): number;
  mach(casKt: number, altitudeFt: number, isaDeviationC: number): number;
  casForMach(mach: number, altitudeFt: number, isaDeviationC: number): number;
}

export interface VnavSpeedProfile {
  climbSpeedKt(altitudeFt: number, distanceFromStartNm: number): number;
  cruiseMach(): number;
  descentSpeedKt(altitudeFt: number, distanceFromStartNm: number): number;
}

export interface VnavGuidanceOutput {
  verticalDeviationFt: number | null;
  targetVerticalSpeedFpm: number | null;
  targetAltitudeFt: number | null;
  targetSpeedKt: number | null;
  distanceToTopOfDescentNm: number | null;
  distanceToTopOfClimbNm: number | null;
  pathAltitudeFt: number | null;
  pathIsGeometric: boolean;
  pathAngleDeg: number | null;
  pastTopOfDescent: boolean;
}

export const VnavLvars = {
  verticalDeviationFt: 'L:A32NX_FM_VNAV_VERTICAL_DEVIATION_FT',
  targetVerticalSpeedFpm: 'L:A32NX_FM_VNAV_TARGET_VS_FPM',
  targetAltitudeFt: 'L:A32NX_FM_VNAV_TARGET_ALTITUDE_FT',
  targetSpeedKt: 'L:A32NX_FM_VNAV_TARGET_SPEED_KT',
  distanceToTodNm: 'L:A32NX_FM_VNAV_DISTANCE_TO_TOD_NM',
  distanceToTocNm: 'L:A32NX_FM_VNAV_DISTANCE_TO_TOC_NM',
  profileValid: 'L:A32NX_FM_VNAV_PROFILE_VALID',
} as const;

export interface VnavFmcAccessor {
  getProfile(): VnavProfile | undefined;
  getLegPrediction(legIndex: number): VnavProfilePoint | undefined;
}

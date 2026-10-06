// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { SimVarValueType } from '@microsoft/msfs-sdk';
import { RegisteredSimVar } from '@flybywiresim/fbw-sdk';
import {
  VnavAircraftState,
  VnavFmcAccessor,
  VnavGuidanceOutput,
  VnavLeg,
  VnavLvars,
  VnavPerformanceModel,
  VnavProfile,
  VnavProfilePoint,
  VnavPseudoWaypoint,
} from './types';
import { A380PerformanceModel } from './A380PerformanceModel';
import { SpeedProfile } from './SpeedProfile';
import { VnavConstraintReader } from './VnavConstraintReader';
import { buildClimb } from './ClimbProfileBuilder';
import { buildCruise } from './CruiseProfileBuilder';
import { buildDescent } from './DescentProfileBuilder';
import { buildApproach } from './ApproachProfileBuilder';

const PROFILE_REBUILD_INTERVAL_MS = 4_000;
const NM_TO_FT = 6076.12;

export interface VnavDriverHost {
  getAircraftState(): Omit<VnavAircraftState, 'distanceFromStartNm'> | null;
  getActiveFlightPlan(): Parameters<typeof VnavConstraintReader.read>[0];
  getDistanceToDestinationNm(): number | null | undefined;
  getApproachSpeedKt?(): number | null | undefined;
}

function emptyGuidance(): VnavGuidanceOutput {
  return {
    verticalDeviationFt: null,
    targetVerticalSpeedFpm: null,
    targetAltitudeFt: null,
    targetSpeedKt: null,
    distanceToTopOfDescentNm: null,
    distanceToTopOfClimbNm: null,
    pathAltitudeFt: null,
    pathIsGeometric: false,
    pathAngleDeg: null,
    pastTopOfDescent: false,
  };
}

function invalidProfile(reason: string, cruiseAltitudeFt: number, simTimeS: number): VnavProfile {
  return { points: [], pseudoWaypoints: [], cruiseAltitudeFt, computedAtSimTimeS: simTimeS, invalidReason: reason };
}

function interpolate(points: VnavProfilePoint[], distanceNm: number, key: 'timeFromStartS' | 'fuelRemainingKg'): number {
  if (distanceNm <= points[0].distanceFromStartNm) {
    return points[0][key];
  }
  for (let i = 1; i < points.length; i++) {
    if (distanceNm <= points[i].distanceFromStartNm) {
      const a = points[i - 1];
      const b = points[i];
      const span = b.distanceFromStartNm - a.distanceFromStartNm;
      return span > 0 ? a[key] + ((distanceNm - a.distanceFromStartNm) / span) * (b[key] - a[key]) : b[key];
    }
  }
  return points[points.length - 1][key];
}

function appendApproach(
  points: VnavProfilePoint[],
  pseudoWaypoints: VnavPseudoWaypoint[],
  legs: VnavLeg[],
  state: VnavAircraftState,
  perf: VnavPerformanceModel,
  approachSpeedKt: number | null | undefined,
): void {
  const lastDescent = points[points.length - 1];
  if (approachSpeedKt && approachSpeedKt > 0 && lastDescent) {
    const approach = buildApproach(legs, state, perf, approachSpeedKt, lastDescent);
    if (!approach.invalidReason) {
      points.push(...approach.points);
      pseudoWaypoints.push(...approach.pseudoWaypoints);
    }
  }
}

export function buildVnavProfile(
  state: VnavAircraftState,
  legs: VnavLeg[],
  perf: VnavPerformanceModel,
  approachSpeedKt: number | null | undefined,
  simTimeS: number,
): VnavProfile {
  if (!state.cruiseAltitudeFt || state.cruiseAltitudeFt <= 0) {
    return invalidProfile('noCruiseAltitude', 0, simTimeS);
  }
  if (legs.length === 0) {
    return invalidProfile('noLegs', state.cruiseAltitudeFt, simTimeS);
  }
  const speed = new SpeedProfile(state, perf, legs);
  const descentProbe = buildDescent({ legs, aircraft: state, performanceModel: perf, speedProfile: speed });
  if (descentProbe.topOfDescentNm === null || descentProbe.points.length === 0) {
    return invalidProfile(descentProbe.invalidReason ?? 'noDescent', state.cruiseAltitudeFt, simTimeS);
  }

  const points: VnavProfilePoint[] = [];
  const pseudoWaypoints: VnavPseudoWaypoint[] = [];
  let invalidReason: string | undefined;
  let topOfClimbNm: number | undefined;
  let topOfDescentNm: number | undefined;

  if (!state.onGround && state.distanceFromStartNm >= descentProbe.topOfDescentNm) {
    const d = state.distanceFromStartNm;
    const descent = buildDescent({
      legs,
      aircraft: state,
      performanceModel: perf,
      speedProfile: speed,
      fuelAtTopOfDescentKg: state.fuelKg + (descentProbe.points[0].fuelRemainingKg - interpolate(descentProbe.points, d, 'fuelRemainingKg')),
      timeAtTopOfDescentS: -(interpolate(descentProbe.points, d, 'timeFromStartS') - descentProbe.points[0].timeFromStartS),
    });
    points.push(...descent.points);
    pseudoWaypoints.push(...descent.pseudoWaypoints);
    topOfDescentNm = descent.topOfDescentNm ?? undefined;
    invalidReason = descent.invalidReason;
    appendApproach(points, pseudoWaypoints, legs, state, perf, approachSpeedKt);
  } else {
    const climb = buildClimb(state, legs, perf, speed);
    const toc = climb.points[climb.points.length - 1];
    points.push(...climb.points);
    pseudoWaypoints.push(...climb.pseudoWaypoints);
    topOfClimbNm = climb.topOfClimbNm ?? undefined;

    if (climb.topOfClimbNm === null || !toc) {
      invalidReason = 'climbDoesNotReachCruise';
    } else if (descentProbe.topOfDescentNm <= climb.topOfClimbNm) {
      invalidReason = 'topOfDescentBeforeTopOfClimb';
    } else {
      const todAnchor: VnavProfilePoint = {
        distanceFromStartNm: descentProbe.topOfDescentNm,
        altitudeFt: state.cruiseAltitudeFt,
        speedKt: toc.speedKt,
        timeFromStartS: toc.timeFromStartS,
        fuelRemainingKg: toc.fuelRemainingKg,
        segment: 'cruise',
      };
      const cruise = buildCruise(state, perf, speed, legs, toc, todAnchor);
      points.push(...cruise.points);
      pseudoWaypoints.push(...cruise.pseudoWaypoints);
      const endOfCruise = points[points.length - 1];

      const descent = buildDescent({
        legs,
        aircraft: state,
        performanceModel: perf,
        speedProfile: speed,
        fuelAtTopOfDescentKg: endOfCruise.fuelRemainingKg,
        timeAtTopOfDescentS: endOfCruise.timeFromStartS,
      });
      points.push(...descent.points);
      pseudoWaypoints.push(...descent.pseudoWaypoints);
      topOfDescentNm = descent.topOfDescentNm ?? undefined;
      invalidReason = descent.invalidReason;
      appendApproach(points, pseudoWaypoints, legs, state, perf, approachSpeedKt);
    }
  }

  points.sort((a, b) => a.distanceFromStartNm - b.distanceFromStartNm);
  const deduped = points.filter((p, i) => i === 0 || p.distanceFromStartNm > points[i - 1].distanceFromStartNm + 1e-6);

  return {
    points: deduped,
    pseudoWaypoints,
    topOfClimbNm,
    topOfDescentNm,
    cruiseAltitudeFt: state.cruiseAltitudeFt,
    computedAtSimTimeS: simTimeS,
    invalidReason,
  };
}

export function profileAt(profile: VnavProfile, distanceNm: number): { altitudeFt: number; speedKt: number; index: number } | null {
  const p = profile.points;
  if (p.length === 0 || distanceNm < p[0].distanceFromStartNm || distanceNm > p[p.length - 1].distanceFromStartNm) {
    return null;
  }
  for (let i = 1; i < p.length; i++) {
    if (distanceNm <= p[i].distanceFromStartNm) {
      const a = p[i - 1];
      const b = p[i];
      const span = b.distanceFromStartNm - a.distanceFromStartNm;
      const f = span > 0 ? (distanceNm - a.distanceFromStartNm) / span : 0;
      return { altitudeFt: a.altitudeFt + f * (b.altitudeFt - a.altitudeFt), speedKt: a.speedKt + f * (b.speedKt - a.speedKt), index: i };
    }
  }
  const last = p[p.length - 1];
  return { altitudeFt: last.altitudeFt, speedKt: last.speedKt, index: p.length - 1 };
}

export function computeVnavGuidance(profile: VnavProfile, state: VnavAircraftState, perf: VnavPerformanceModel): VnavGuidanceOutput {
  if (profile.points.length < 2) {
    return emptyGuidance();
  }
  const d = state.distanceFromStartNm;
  const here = profileAt(profile, d);
  const out = emptyGuidance();
  if (!here) {
    return out;
  }
  out.targetSpeedKt = here.speedKt;
  out.distanceToTopOfClimbNm = profile.topOfClimbNm !== undefined && d < profile.topOfClimbNm ? profile.topOfClimbNm - d : null;
  out.distanceToTopOfDescentNm = profile.topOfDescentNm !== undefined && d < profile.topOfDescentNm ? profile.topOfDescentNm - d : null;

  const ahead = profile.points.slice(here.index).find((p) => p.legIndex !== undefined && Math.abs(p.altitudeFt - here.altitudeFt) > 50);
  out.targetAltitudeFt = ahead ? ahead.altitudeFt : profile.cruiseAltitudeFt;

  const inDescent = profile.topOfDescentNm !== undefined && d >= profile.topOfDescentNm;
  out.pastTopOfDescent = inDescent;
  const pastTopOfClimb = profile.topOfClimbNm === undefined || d >= profile.topOfClimbNm;
  if (pastTopOfClimb) {
    const a = profile.points[Math.max(0, here.index - 1)];
    const b = profile.points[here.index];
    const span = b.distanceFromStartNm - a.distanceFromStartNm;
    const gradientFtPerNm = span > 0 ? (b.altitudeFt - a.altitudeFt) / span : 0;
    out.pathAltitudeFt = here.altitudeFt;
    out.pathAngleDeg = (Math.atan2(gradientFtPerNm, NM_TO_FT) * 180) / Math.PI;
    out.pathIsGeometric = inDescent ? b.geometric === true : true;
    if (inDescent) {
      out.verticalDeviationFt = state.altitudeFt - here.altitudeFt;
      const groundSpeedKt = Math.max(0, perf.trueAirspeedKt(state.casKt, state.altitudeFt, state.isaDeviationC) + state.windAlongTrackKt);
      out.targetVerticalSpeedFpm = (gradientFtPerNm * groundSpeedKt) / 60;
    }
  }
  return out;
}

export class VnavDriver implements VnavFmcAccessor {
  private readonly performanceModel: VnavPerformanceModel = new A380PerformanceModel();

  private profile: VnavProfile = invalidProfile('noFlightPlan', 0, 0);

  private legs: VnavLeg[] = [];

  private sinceRebuildMs = PROFILE_REBUILD_INTERVAL_MS;

  private guidance: VnavGuidanceOutput | null = null;

  private simTimeS = 0;

  private readonly lvars = {
    verticalDeviationFt: RegisteredSimVar.create<number>(VnavLvars.verticalDeviationFt, SimVarValueType.Feet),
    targetVerticalSpeedFpm: RegisteredSimVar.create<number>(VnavLvars.targetVerticalSpeedFpm, SimVarValueType.FPM),
    targetAltitudeFt: RegisteredSimVar.create<number>(VnavLvars.targetAltitudeFt, SimVarValueType.Feet),
    targetSpeedKt: RegisteredSimVar.create<number>(VnavLvars.targetSpeedKt, SimVarValueType.Knots),
    distanceToTodNm: RegisteredSimVar.create<number>(VnavLvars.distanceToTodNm, SimVarValueType.NM),
    distanceToTocNm: RegisteredSimVar.create<number>(VnavLvars.distanceToTocNm, SimVarValueType.NM),
    profileValid: RegisteredSimVar.createBoolean(VnavLvars.profileValid),
  };

  constructor(private readonly host: VnavDriverHost) {}

  update(dtMs: number): void {
    this.simTimeS += dtMs / 1000;
    const state = this.aircraftState();
    if (!state) {
      this.profile = invalidProfile('noFlightPlan', 0, this.simTimeS);
      this.guidance = null;
      this.publish(emptyGuidance());
      return;
    }
    this.sinceRebuildMs += dtMs;
    if (this.sinceRebuildMs >= PROFILE_REBUILD_INTERVAL_MS) {
      this.sinceRebuildMs = 0;
      try {
        this.legs = VnavConstraintReader.read(this.host.getActiveFlightPlan());
        this.profile = buildVnavProfile(state, this.legs, this.performanceModel, this.host.getApproachSpeedKt?.(), this.simTimeS);
      } catch (err) {
        console.error('[VnavDriver] profile rebuild failed, keeping the previous profile', err);
      }
    }
    this.guidance = this.profile.invalidReason ? null : computeVnavGuidance(this.profile, state, this.performanceModel);
    this.publish(this.guidance ?? emptyGuidance());
  }

  getProfile(): VnavProfile {
    return this.profile;
  }

  getGuidance(): VnavGuidanceOutput | null {
    return this.guidance;
  }

  getLegPrediction(legIndex: number): VnavProfilePoint | undefined {
    if (this.profile.invalidReason) {
      return undefined;
    }
    return this.profile.points.find((p) => p.legIndex === legIndex);
  }

  getLegs(): readonly VnavLeg[] {
    return this.legs;
  }

  private aircraftState(): VnavAircraftState | null {
    const partial = this.host.getAircraftState();
    if (!partial) {
      return null;
    }
    const legs = this.legs.filter((l) => !l.isMissedApproach);
    const total = legs.length > 0 ? legs[legs.length - 1].distanceFromStartNm : 0;
    const toGo = this.host.getDistanceToDestinationNm();
    const distanceFromStartNm = toGo !== null && toGo !== undefined && total > 0 ? Math.max(0, total - toGo) : 0;
    return { ...partial, distanceFromStartNm };
  }

  private publish(output: VnavGuidanceOutput): void {
    this.lvars.profileValid.set(!this.profile.invalidReason);
    this.lvars.verticalDeviationFt.set(output.verticalDeviationFt ?? 0);
    this.lvars.targetVerticalSpeedFpm.set(output.targetVerticalSpeedFpm ?? 0);
    this.lvars.targetAltitudeFt.set(output.targetAltitudeFt ?? 0);
    this.lvars.targetSpeedKt.set(output.targetSpeedKt ?? 0);
    this.lvars.distanceToTodNm.set(output.distanceToTopOfDescentNm ?? -1);
    this.lvars.distanceToTocNm.set(output.distanceToTopOfClimbNm ?? -1);
  }
}

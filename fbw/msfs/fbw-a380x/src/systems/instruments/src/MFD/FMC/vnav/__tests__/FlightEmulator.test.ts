// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { describe, expect, it } from 'vitest';
import * as fs from 'node:fs';
import * as path from 'node:path';
import type { VnavAircraftState, VnavGuidanceOutput, VnavLeg, VnavProfile, VnavSegment } from '../types';
import { A380PerformanceModel } from '../A380PerformanceModel';
import { FbwA380PerformanceModel } from '../FbwA380PerformanceModel';
import { SpeedProfile } from '../SpeedProfile';
import { predict, VnavLegPrediction } from '../VnavPredictions';
import { buildVnavProfile, computeVnavGuidance } from '../VnavDriver';

const DIR = process.env.VNAV_EMULATOR_DIR ?? 'D:/A380/msfs-a380/wave5';
const OFP_PATH = path.join(DIR, process.env.VNAV_EMULATOR_OFP ?? 'ofp-725896.json');
const haveOfp = fs.existsSync(OFP_PATH);

const DT_S = 5;
const TRAJECTORY_EVERY_S = 10;
const DUMP_EVERY_S = 60;
const ROTATE_KT = 155;
const TAKEOFF_ACCEL_KT_S = 3;
const ROLLOUT_DECEL_KT_S = 2.5;
const SPEED_CHANGE_KT_S = 1;
const PATH_GAIN_FPM_PER_FT = 3;
const VAPP_KT = 140;

interface OfpFix {
  ident: string;
  stage: string;
  is_sid_star: string;
  distance: string;
  altitude_feet: string;
  wind_component: string;
  oat_isa_dev: string;
  time_total: string;
  fuel_plan_onboard: string;
}

interface Ofp {
  params: { request_id: string; user_id?: string };
  general: { costindex: string; initial_altitude: string; route_distance: string };
  origin: { icao_code: string; elevation: string; plan_rwy: string };
  destination: { icao_code: string; elevation: string; plan_rwy: string };
  weights: { est_tow: string; est_zfw: string };
  fuel: { plan_takeoff: string; est_burn?: string; enroute_burn?: string; plan_landing?: string };
  times: { est_time_enroute: string; est_off: string };
  navlog: { fix: OfpFix[] };
}

type Phase = 'takeoff' | 'climb' | 'cruise' | 'descent' | 'approach' | 'rollout' | 'stopped';

interface Aircraft {
  tS: number;
  distanceNm: number;
  altitudeFt: number;
  casKt: number;
  vsFpm: number;
  fuelKg: number;
  onGround: boolean;
  phase: Phase;
}

function legsFromOfp(ofp: Ofp): VnavLeg[] {
  let cum = 0;
  const last = ofp.navlog.fix.length - 1;
  return ofp.navlog.fix.map((fix, index) => {
    cum += Number(fix.distance) || 0;
    const sidStar = fix.is_sid_star === '1';
    let segment: VnavLeg['segment'];
    if (fix.stage === 'CLB') {
      segment = sidStar ? 'departure' : 'enroute';
    } else if (fix.stage === 'CRZ') {
      segment = 'enroute';
    } else {
      segment = index >= last - 1 ? 'approach' : 'arrival';
    }
    return { index, ident: fix.ident, distanceFromStartNm: cum, isMissedApproach: false, segment };
  });
}

function legAhead(legs: VnavLeg[], d: number): number {
  const i = legs.findIndex((l) => l.distanceFromStartNm > d);
  return i < 0 ? legs.length - 1 : i;
}

function hhmm(epochS: number): string {
  const t = new Date(epochS * 1000);
  return `${String(t.getUTCHours()).padStart(2, '0')}${String(t.getUTCMinutes()).padStart(2, '0')}`;
}

function segmentFor(phase: Phase): VnavSegment {
  switch (phase) {
    case 'takeoff':
    case 'rollout':
    case 'stopped':
      return 'takeoff';
    default:
      return phase;
  }
}

(haveOfp ? describe : describe.skip)('Full-flight emulator: FMS/MCDU half', () => {
  it.each([['fbw'], ['a380']])('flies the OFP under VNAV (%s performance) from brake release to a stop and dumps it', (model) => {
    const ofp: Ofp = JSON.parse(fs.readFileSync(OFP_PATH, 'utf-8'));
    const id = ofp.params?.user_id ?? ofp.params?.request_id ?? 'ofp';
    const legs = legsFromOfp(ofp);
    const perf = model === 'fbw' ? new FbwA380PerformanceModel() : new A380PerformanceModel();
    const destinationNm = legs[legs.length - 1].distanceFromStartNm;
    const originElevationFt = Number(ofp.origin.elevation) || 0;
    const destinationElevationFt = Number(ofp.destination.elevation) || 0;
    const zfwKg = Number(ofp.weights.est_zfw);
    const cruiseAltitudeFt = Number(ofp.general.initial_altitude);
    const costIndex = Number(ofp.general.costindex);
    const offEpochS = Number(ofp.times.est_off);

    const ac: Aircraft = {
      tS: 0,
      distanceNm: 0,
      altitudeFt: originElevationFt,
      casKt: 0,
      vsFpm: 0,
      fuelKg: Number(ofp.fuel.plan_takeoff),
      onGround: true,
      phase: 'takeoff',
    };

    const vnavState = (): VnavAircraftState => {
      const fix = ofp.navlog.fix[legAhead(legs, ac.distanceNm)];
      return {
        distanceFromStartNm: ac.distanceNm,
        altitudeFt: ac.altitudeFt,
        casKt: ac.casKt,
        mach: perf.mach(Math.max(ac.casKt, 1), ac.altitudeFt, Number(fix.oat_isa_dev) || 0),
        grossWeightKg: zfwKg + ac.fuelKg,
        fuelKg: ac.fuelKg,
        zeroFuelWeightKg: zfwKg,
        costIndex,
        cruiseAltitudeFt,
        windAlongTrackKt: Number(fix.wind_component) || 0,
        isaDeviationC: Number(fix.oat_isa_dev) || 0,
        onGround: ac.onGround,
        destinationElevationFt,
      };
    };

    const departureProfile = buildVnavProfile(vnavState(), legs, perf, VAPP_KT, 0);
    expect(departureProfile.invalidReason).toBeUndefined();
    expect(departureProfile.topOfClimbNm!).toBeLessThan(departureProfile.topOfDescentNm!);
    const departurePredictions = predict(departureProfile, legs);

    let profile: VnavProfile = departureProfile;
    let guidance: VnavGuidanceOutput = computeVnavGuidance(profile, vnavState(), perf);
    const trajectory: Array<Record<string, unknown>> = [];
    const dumps: Array<Record<string, unknown>> = [];
    const log: string[] = [];
    const events: Array<{ tS: number; event: string }> = [];
    let maxDescentDeviationFt = 0;
    let touchdown: { tS: number; fuelKg: number; distanceNm: number } | null = null;
    let lastPhase: Phase = ac.phase;
    let liftoffS = 0;

    const sample = (): Record<string, unknown> => {
      const s = vnavState();
      const tas = perf.trueAirspeedKt(Math.max(ac.casKt, 1), ac.altitudeFt, s.isaDeviationC);
      return {
        tS: ac.tS,
        phase: ac.phase,
        distanceNm: +ac.distanceNm.toFixed(2),
        altitudeFt: Math.round(ac.altitudeFt),
        casKt: +ac.casKt.toFixed(1),
        mach: +s.mach.toFixed(3),
        tasKt: Math.round(tas),
        groundSpeedKt: Math.round(ac.onGround ? ac.casKt : tas + s.windAlongTrackKt),
        vsFpm: Math.round(ac.vsFpm),
        fuelKg: Math.round(ac.fuelKg),
        grossWeightKg: Math.round(s.grossWeightKg),
        onGround: ac.onGround,
        isaDeviationC: s.isaDeviationC,
      };
    };

    const mcdu = (predictions: VnavLegPrediction[]) => {
      const ahead = legAhead(legs, ac.distanceNm);
      const row = (p: VnavLegPrediction | undefined) =>
        p && Number.isFinite(p.timeFromStartS)
          ? {
              ident: p.ident,
              distToGoNm: +(p.distanceFromStartNm - ac.distanceNm).toFixed(1),
              utc: hhmm(offEpochS + ac.tS + p.timeFromStartS),
              speed: p.mach && p.mach >= 0.6 ? `.${Math.round(p.mach * 100)}` : `${Math.round(p.speedKt)}`,
              altitude: p.altitudeFt >= 18000 ? `FL${Math.round(p.altitudeFt / 100)}` : `${Math.round(p.altitudeFt)}`,
              efobT: +(p.fuelRemainingKg / 1000).toFixed(1),
            }
          : null;
      const speeds = new SpeedProfile(vnavState(), perf, legs);
      return {
        fpln: predictions
          .filter((p) => p.legIndex >= ahead)
          .slice(0, 5)
          .map(row),
        dest: row(predictions[predictions.length - 1]),
        perf: {
          phase: ac.phase,
          costIndex,
          crzFl: Math.round(cruiseAltitudeFt / 100),
          managedClimbKt: Math.round(speeds.climbSpeedKt(ac.altitudeFt, ac.distanceNm)),
          managedCruiseMach: +speeds.cruiseMach().toFixed(3),
          managedDescentKt: Math.round(speeds.descentSpeedKt(ac.altitudeFt, ac.distanceNm)),
          tcNm: profile.topOfClimbNm !== undefined ? +profile.topOfClimbNm.toFixed(1) : null,
          tdNm: profile.topOfDescentNm !== undefined ? +profile.topOfDescentNm.toFixed(1) : null,
        },
        pseudoWaypoints: profile.pseudoWaypoints
          .filter((w) => w.distanceFromStartNm >= ac.distanceNm)
          .map((w) => ({
            ident: w.ident,
            distToGoNm: +(w.distanceFromStartNm - ac.distanceNm).toFixed(1),
            altitudeFt: Math.round(w.altitudeFt),
          })),
        profileValid: !profile.invalidReason,
        invalidReason: profile.invalidReason ?? null,
      };
    };

    const maxSteps = Math.ceil((Number(ofp.times.est_time_enroute) * 2) / DT_S);
    for (let step = 0; step < maxSteps && ac.phase !== 'stopped'; step++) {
      const s = vnavState();
      if (ac.tS % DUMP_EVERY_S === 0 && !ac.onGround) {
        profile = buildVnavProfile(s, legs, perf, VAPP_KT, ac.tS);
      }
      guidance = computeVnavGuidance(profile, s, perf);

      const tas = perf.trueAirspeedKt(Math.max(ac.casKt, 1), ac.altitudeFt, s.isaDeviationC);
      const gs = ac.onGround ? ac.casKt : Math.max(60, tas + s.windAlongTrackKt);
      const target = guidance.targetSpeedKt ?? ac.casKt;
      const pastTod = profile.topOfDescentNm !== undefined && ac.distanceNm >= profile.topOfDescentNm;

      if (ac.phase === 'takeoff' && ac.onGround) {
        ac.casKt += TAKEOFF_ACCEL_KT_S * DT_S;
        if (ac.casKt >= ROTATE_KT) {
          ac.onGround = false;
          liftoffS = ac.tS;
          events.push({ tS: ac.tS, event: `liftoff ${ROTATE_KT} kt` });
        }
        ac.vsFpm = 0;
      } else if (ac.phase === 'rollout') {
        ac.casKt = Math.max(0, ac.casKt - ROLLOUT_DECEL_KT_S * DT_S);
        ac.vsFpm = 0;
        if (ac.casKt === 0) {
          ac.phase = 'stopped';
        }
      } else if (!pastTod && ac.altitudeFt < cruiseAltitudeFt - 1) {
        const gradient = perf.climbGradientFtPerNm(ac.altitudeFt, ac.casKt, s.grossWeightKg, s.isaDeviationC, gs);
        const accelerating = target - ac.casKt > 5;
        ac.vsFpm = (gradient * gs) / 60 / (accelerating ? 2 : 1);
        ac.altitudeFt = Math.min(cruiseAltitudeFt, ac.altitudeFt + (ac.vsFpm * DT_S) / 60);
        ac.phase = ac.altitudeFt - originElevationFt < 1500 ? 'takeoff' : 'climb';
      } else if (!pastTod) {
        ac.vsFpm = 0;
        ac.altitudeFt = cruiseAltitudeFt;
        ac.phase = 'cruise';
      } else {
        const dev = guidance.verticalDeviationFt ?? 0;
        const tvs = guidance.targetVerticalSpeedFpm ?? -1500;
        ac.vsFpm = Math.max(-4500, Math.min(500, tvs - PATH_GAIN_FPM_PER_FT * dev));
        ac.altitudeFt += (ac.vsFpm * DT_S) / 60;
        ac.phase =
          ac.distanceNm >= destinationNm - 15 || ac.altitudeFt - destinationElevationFt < 3000 ? 'approach' : 'descent';
        maxDescentDeviationFt = Math.max(maxDescentDeviationFt, Math.abs(dev));
        if (ac.distanceNm >= destinationNm || ac.altitudeFt <= destinationElevationFt) {
          ac.altitudeFt = destinationElevationFt;
          ac.onGround = true;
          ac.phase = 'rollout';
          touchdown = { tS: ac.tS, fuelKg: ac.fuelKg, distanceNm: ac.distanceNm };
          events.push({
            tS: ac.tS,
            event: `touchdown ${Math.round(ac.casKt)} kt, ${(ac.fuelKg / 1000).toFixed(1)} t`,
          });
        }
      }
      if (!ac.onGround) {
        ac.casKt += Math.max(-SPEED_CHANGE_KT_S * DT_S, Math.min(SPEED_CHANGE_KT_S * DT_S, target - ac.casKt));
      }
      ac.distanceNm += (gs * DT_S) / 3600;
      if (ac.phase !== 'stopped') {
        const ff = perf.fuelFlowKgPerHour(
          segmentFor(ac.phase),
          ac.altitudeFt,
          Math.max(ac.casKt, 1),
          s.grossWeightKg,
          s.isaDeviationC,
        );
        ac.fuelKg -= (ff * DT_S) / 3600;
      }
      ac.tS += DT_S;

      if (ac.phase !== lastPhase) {
        events.push({
          tS: ac.tS,
          event: `${lastPhase} -> ${ac.phase} at ${ac.distanceNm.toFixed(1)} NM, ${Math.round(ac.altitudeFt)} ft`,
        });
        lastPhase = ac.phase;
      }
      if (ac.tS % TRAJECTORY_EVERY_S === 0) {
        trajectory.push(sample());
      }
      if (ac.tS % DUMP_EVERY_S === 0) {
        const view = mcdu(predict(profile, legs));
        dumps.push({ ...sample(), guidance, mcdu: view });
        const dev = guidance.verticalDeviationFt;
        log.push(
          `T+${String(Math.floor(ac.tS / 60)).padStart(3, ' ')}m ${ac.phase.padEnd(8)} ${ac.distanceNm.toFixed(0).padStart(5)} NM ` +
            `${String(Math.round(ac.altitudeFt)).padStart(6)} ft ${ac.casKt.toFixed(0).padStart(3)} kt ${String(Math.round(ac.vsFpm)).padStart(6)} fpm ` +
            `FOB ${(ac.fuelKg / 1000).toFixed(1)} t | dev ${dev === null ? '-' : dev.toFixed(0)} ` +
            `| T/C ${guidance.distanceToTopOfClimbNm?.toFixed(0) ?? '-'} T/D ${guidance.distanceToTopOfDescentNm?.toFixed(0) ?? '-'} ` +
            `| DEST ${view.dest?.utc ?? '----'} ${view.dest?.efobT ?? '--'} t` +
            `${view.profileValid ? '' : ` | PROFILE INVALID: ${view.invalidReason}`}`,
        );
      }
    }

    const ofpTripS = Number(ofp.times.est_time_enroute);
    const ofpBurnKg = Number(ofp.fuel.est_burn ?? ofp.fuel.enroute_burn);
    const flownS = touchdown ? touchdown.tS - liftoffS : NaN;
    const burnKg = touchdown ? Number(ofp.fuel.plan_takeoff) - touchdown.fuelKg : NaN;
    const comparison = {
      ofpTripMin: Math.round(ofpTripS / 60),
      flownTripMin: Math.round(flownS / 60),
      tripTimeErrorPct: +((100 * (flownS - ofpTripS)) / ofpTripS).toFixed(1),
      ofpBurnKg,
      flownBurnKg: Math.round(burnKg),
      burnErrorPct: +((100 * (burnKg - ofpBurnKg)) / ofpBurnKg).toFixed(1),
      vnavTopOfClimbNm: +departureProfile.topOfClimbNm!.toFixed(1),
      vnavTopOfDescentNm: +departureProfile.topOfDescentNm!.toFixed(1),
      departurePredictedDestEfobT: +(departurePredictions[departurePredictions.length - 1].fuelRemainingKg / 1000).toFixed(1),
      ofpPlanLandingT: ofp.fuel.plan_landing ? +(Number(ofp.fuel.plan_landing) / 1000).toFixed(1) : null,
      maxDescentDeviationFt: Math.round(maxDescentDeviationFt),
    };

    fs.writeFileSync(
      path.join(DIR, model === 'a380' ? `emulator-fms-${id}.json` : `emulator-fms-${id}-${model}.json`),
      JSON.stringify(
        {
          ofp: {
            id,
            origin: ofp.origin.icao_code,
            destination: ofp.destination.icao_code,
            routeDistanceNm: destinationNm,
            cruiseAltitudeFt,
            costIndex,
            originElevationFt,
            destinationElevationFt,
          },
          constants: {
            DT_S,
            TRAJECTORY_EVERY_S,
            DUMP_EVERY_S,
            ROTATE_KT,
            TAKEOFF_ACCEL_KT_S,
            ROLLOUT_DECEL_KT_S,
            SPEED_CHANGE_KT_S,
            PATH_GAIN_FPM_PER_FT,
            VAPP_KT,
          },
          comparison,
          events,
          departurePredictions,
          trajectory,
          dumps,
        },
        null,
        1,
      ),
    );
    const md = [
      `# Full-flight emulator, FMS/MCDU half: ${ofp.origin.icao_code}-${ofp.destination.icao_code} (OFP ${id}, ${model} performance)`,
      '',
      `Cruise FL${cruiseAltitudeFt / 100}, CI ${costIndex}, TOW ${ofp.weights.est_tow} kg, ${destinationNm} NM.`,
      '',
      '## OFP comparison',
      '',
      ...Object.entries(comparison).map(([k, v]) => `- ${k}: ${v}`),
      '',
      '## Events',
      '',
      ...events.map((e) => `- T+${Math.round(e.tS / 60)}m ${e.event}`),
      '',
      '## Log (every minute)',
      '',
      '```',
      ...log,
      '```',
    ].join('\n');
    fs.writeFileSync(path.join(DIR, model === 'a380' ? `emulator-fms-${id}.md` : `emulator-fms-${id}-${model}.md`), md);

    expect(touchdown).not.toBeNull();
    expect(ac.phase).toBe('stopped');
    expect(Math.abs(touchdown!.distanceNm - destinationNm)).toBeLessThan(2);
    expect(maxDescentDeviationFt).toBeLessThan(1500);
    expect(touchdown!.fuelKg).toBeGreaterThan(0);
    expect(Math.abs(comparison.tripTimeErrorPct)).toBeLessThan(20);
  });
});

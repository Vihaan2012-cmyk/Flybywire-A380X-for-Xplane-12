//  Copyright (c) 2025 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { Arinc429SignStatusMatrix, Arinc429Word } from '@flybywiresim/fbw-sdk';
import { maxGw, maxZfw } from '@shared/PerformanceConstants';

export interface OisMsRunway {
  designation: string;
  designatorCharPrimary: number;
  designatorCharSecondary: number;
  latitude: number;
  longitude: number;
  direction: number;
  length: number;
  width: number;
  surface: number;
  primaryElevation: number;
  secondaryElevation: number;
  primaryThresholdLength: number;
  secondaryThresholdLength: number;
  primaryOverrunLength: number;
  secondaryOverrunLength: number;
}

export interface OisMsAirport {
  icao: string;
  name?: string;
  lat: number;
  lon: number;
  runways: OisMsRunway[];
}

export interface OisRunwayEnd {
  ident: string;
  bearingTrue: number;
  tora: number;
  asda: number;
  lda: number;
  elevationFt: number;
  slopePercent: number;
  width: number;
  todaKnown: boolean;
}

export function runwayEnds(airport: OisMsAirport): OisRunwayEnd[] {
  const out: OisRunwayEnd[] = [];
  for (const rwy of airport.runways ?? []) {
    const designations = (rwy.designation ?? '').split('-');
    for (let i = 0; i < 2; i++) {
      const primary = i === 0;
      const number = designations[i];
      if (number === undefined || number === '') {
        continue;
      }
      const designator = designatorChar(primary ? rwy.designatorCharPrimary : rwy.designatorCharSecondary);
      const ident = `${number.padStart(2, '0')}${designator}`;
      const thisThreshold = primary ? rwy.primaryThresholdLength : rwy.secondaryThresholdLength;
      const farOverrun = primary ? rwy.secondaryOverrunLength : rwy.primaryOverrunLength;
      const thisElevation = primary ? rwy.primaryElevation : rwy.secondaryElevation;
      const farElevation = primary ? rwy.secondaryElevation : rwy.primaryElevation;
      const between = Math.max(1, rwy.length - rwy.primaryThresholdLength - rwy.secondaryThresholdLength);
      out.push({
        ident,
        bearingTrue: primary ? rwy.direction : (rwy.direction + 180) % 360,
        tora: rwy.length,
        asda: rwy.length + (farOverrun ?? 0),
        lda: Math.max(0, rwy.length - (thisThreshold ?? 0)),
        elevationFt: (thisElevation ?? 0) / 0.3048,
        slopePercent: ((farElevation - thisElevation) / between) * 100,
        width: rwy.width,
        todaKnown: false,
      });
    }
  }
  out.sort((a, b) => a.ident.localeCompare(b.ident));
  return out;
}

function designatorChar(designator: number): string {
  switch (designator) {
    case 1:
      return 'L';
    case 2:
      return 'R';
    case 3:
      return 'C';
    case 4:
      return 'W';
    case 5:
      return 'A';
    case 6:
      return 'B';
    default:
      return '';
  }
}

export function loadAirport(ident: string, timeoutMs = 10_000): Promise<OisMsAirport | null> {
  const wanted = ident.trim().toUpperCase();
  if (wanted.length < 3 || wanted.length > 4) {
    return Promise.resolve(null);
  }
  const icao = `A      ${wanted}`;

  return new Promise((resolve) => {
    let done = false;
    const finish = (airport: OisMsAirport | null) => {
      if (done) {
        return;
      }
      done = true;
      Coherent.off('SendAirport', receive as unknown as () => void, null);
      clearTimeout(timer);
      resolve(airport);
    };
    const receive = (airport: OisMsAirport) => {
      if (airport && typeof airport.icao === 'string' && airport.icao.trim().slice(1).trim() === wanted) {
        finish(airport);
      }
    };
    const timer = setTimeout(() => finish(null), timeoutMs);

    Coherent.on('SendAirport', receive);
    Coherent.call('LOAD_AIRPORT', icao)
      .then((found: boolean) => {
        if (found === false) {
          finish(null);
        }
      })
      .catch(() => finish(null));
  });
}

export interface OisWeather {
  oat: number;
  qnh: number;
  windDirectionTrue: number;
  windKt: number;
}

export function readWeather(): OisWeather {
  return {
    oat: SimVar.GetSimVarValue('AMBIENT TEMPERATURE', 'celsius'),
    qnh: SimVar.GetSimVarValue('SEA LEVEL PRESSURE', 'millibars'),
    windDirectionTrue: SimVar.GetSimVarValue('AMBIENT WIND DIRECTION', 'degrees'),
    windKt: SimVar.GetSimVarValue('AMBIENT WIND VELOCITY', 'knots'),
  };
}

export function windComponents(
  runwayBearingTrue: number,
  weather: OisWeather,
): { headwind: number; crosswind: number } {
  const off = ((weather.windDirectionTrue - runwayBearingTrue) * Math.PI) / 180;
  return {
    headwind: weather.windKt * Math.cos(off),
    crosswind: weather.windKt * Math.sin(off),
  };
}

export function pressureAltitude(elevationFt: number, qnh: number): number {
  return elevationFt + (1013.25 - qnh) * 27;
}

export function isaTemperature(pressureAltFt: number): number {
  return 15 - 0.0019812 * pressureAltFt;
}

export function arincValue(name: string): number | null {
  const word = Arinc429Word.fromSimVarValue(name);
  return word.ssm === Arinc429SignStatusMatrix.NormalOperation ? word.value : null;
}

export interface OisWeights {
  grossWeightKg: number | null;
  cgPercentMac: number | null;
  fobKg: number | null;
  zeroFuelWeightKg: number;
  zeroFuelCgPercentMac: number;
}

export function readWeights(): OisWeights {
  return {
    grossWeightKg: arincValue('L:A32NX_FQMS_GROSS_WEIGHT'),
    cgPercentMac: arincValue('L:A32NX_FQMS_CENTER_OF_GRAVITY_MAC'),
    fobKg: arincValue('L:A32NX_FQMS_TOTAL_FUEL_ON_BOARD'),
    zeroFuelWeightKg: SimVar.GetSimVarValue('L:A32NX_AIRFRAME_ZFW', 'number'),
    zeroFuelCgPercentMac: SimVar.GetSimVarValue('L:A32NX_AIRFRAME_ZFW_CG_PERCENT_MAC', 'number'),
  };
}

export const A380_MAX_ZFW_KG = maxZfw;
export const A380_MAX_GW_KG = maxGw;

export interface OisUnits {
  weight: 't' | 'klb';
  altimeter: 'hpa' | 'inhg';
  distance: 'm' | 'ft';
}

export const DEFAULT_UNITS: OisUnits = { weight: 't', altimeter: 'hpa', distance: 'm' };

export const M_TO_FT = 1 / 0.3048;
export const KG_TO_LB = 2.20462262;

export function formatWeight(kg: number | null, units: OisUnits): string {
  if (kg === null || !Number.isFinite(kg)) {
    return '---.-';
  }
  return (units.weight === 't' ? kg / 1000 : (kg * KG_TO_LB) / 1000).toFixed(1);
}

export function weightUnitLabel(units: OisUnits): string {
  return units.weight === 't' ? 'T' : 'KLB';
}

export function formatDistance(metres: number | null, units: OisUnits): string {
  if (metres === null || !Number.isFinite(metres)) {
    return '----';
  }
  return Math.round(units.distance === 'm' ? metres : metres * M_TO_FT).toFixed(0);
}

export function distanceUnitLabel(units: OisUnits): string {
  return units.distance === 'm' ? 'M' : 'FT';
}

export function formatPressure(hpa: number, units: OisUnits): string {
  if (!Number.isFinite(hpa)) {
    return '----';
  }
  return units.altimeter === 'hpa' ? Math.round(hpa).toFixed(0) : (hpa * 0.0295299830714).toFixed(2);
}

export function pressureUnitLabel(units: OisUnits): string {
  return units.altimeter === 'hpa' ? 'HPA' : 'IN HG';
}

export const UNAVAILABLE = 'NOT AVAIL';

export const UNAVAILABLE_REASON =
  'NO CERTIFIED A380-842 TAKE-OFF/LANDING PERFORMANCE DATA SET IS INSTALLED. ' +
  'THE COMPUTED RESULTS BELOW ARE NOT AVAILABLE AND MUST BE TAKEN FROM THE QRH OR THE AIRLINE PERFORMANCE TOOL.';

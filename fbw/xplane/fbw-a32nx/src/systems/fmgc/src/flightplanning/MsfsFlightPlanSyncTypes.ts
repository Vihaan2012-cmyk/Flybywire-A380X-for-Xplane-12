//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

export interface JS_ICAO {
  __Type: 'JS_ICAO';
  type: string;
  region: string;
  airport: string;
  ident: string;
}

export interface JS_RunwayIdentifier {
  __Type: 'JS_RunwayIdentifier';
  number: string;
  designator: string;
}

export interface JS_ApproachIdentifier {
  __Type: 'JS_ApproachIdentifier';
  type: string;
  runway: JS_RunwayIdentifier;
  suffix: string;
}

export interface JS_FlightAltitude {
  __Type: 'JS_FlightAltitude';
  altitude: number;
  isFlightLevel: boolean;
}

export interface JS_FlightPlanRouteVfrPatternProcedure {
  __Type: 'JS_VfrPatternProcedure';
  type: string;
  altitude: number;
  distance: number;
  isLeftTraffic: boolean;
}

export interface JS_EnrouteLeg {
  __Type: 'JS_EnrouteLeg';
  isPpos: boolean;
  fixIcao: JS_ICAO;
  hasLatLon: boolean;
  lat: number;
  lon: number;
  hasPointBearingDistance: boolean;
  referenceIcao: JS_ICAO;
  bearing: number;
  distance: number;
  altitude: JS_FlightAltitude | null;
  name: string;
  via: string;
}

export interface JS_FlightPlanRoute {
  __Type: 'JS_FlightPlanRoute';
  departureAirport: JS_ICAO;
  destinationAirport: JS_ICAO;
  departureRunway: JS_RunwayIdentifier;
  destinationRunway: JS_RunwayIdentifier;
  departure: string;
  departureTransition: string;
  departureVfrPattern: JS_FlightPlanRouteVfrPatternProcedure;
  arrival: string;
  arrivalTransition: string;
  approach: JS_ApproachIdentifier;
  approachTransition: string;
  approachVfrPattern: JS_FlightPlanRouteVfrPatternProcedure;
  enroute: JS_EnrouteLeg[];
  isVfr: boolean;
  cruiseAltitude: JS_FlightAltitude;
}

import type { AiracCycle } from './airac';

export interface IcaoValue {
  readonly __Type: 'JS_ICAO';
  readonly type: string;
  readonly region: string;
  readonly airport: string;
  readonly ident: string;
}

export enum IcaoType {
  None = '',
  Airport = 'A',
  Vor = 'V',
  Ndb = 'N',
  Waypoint = 'W',
  Runway = 'R',
}

export enum FacilityType {
  Airport = 'Airport',
  Intersection = 'Intersection',
  VOR = 'VOR',
  NDB = 'NDB',
}

export enum FacilitySearchType {
  All = 0,
  Airport = 1,
  Intersection = 2,
  Vor = 3,
  Ndb = 4,
  AllExceptVisual = 8,
}

export enum AirportClassMask {
  None = 0,
  HardSurface = 2,
  SoftSurface = 4,
  AllWater = 8,
  HeliportOnly = 16,
  Private = 32,
}

export interface RunwayIdentifier {
  __Type: 'JS_RunwayIdentifier';
  number: string;
  designator: string;
}

export interface Facility {
  readonly icao: string;
  readonly icaoStruct: IcaoValue;
  readonly name: string;
  readonly lat: number;
  readonly lon: number;
  readonly region: string;
  readonly city: string;
  readonly [field: string]: unknown;
}

export type FacilityTypeMap = {
  [FacilityType.Airport]: Facility;
  [FacilityType.Intersection]: Facility;
  [FacilityType.VOR]: Facility;
  [FacilityType.NDB]: Facility;
};

export interface Metar {
  readonly icao: string;
  readonly metarString: string;
  readonly [field: string]: unknown;
}

export interface Taf {
  readonly icao: string;
  readonly tafString: string;
  readonly [field: string]: unknown;
}

export interface FacilityDatabaseCycles {
  previous: AiracCycle;
  current: AiracCycle;
  next: AiracCycle;
}

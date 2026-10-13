import type { Cycle } from './airac';

export interface IcaoValueX {
  readonly __Type: 'JS_ICAO';
  readonly type: string;
  readonly region: string;
  readonly airport: string;
  readonly ident: string;
}

export enum IcaoTypeX {
  None = '',
  Airport = 'A',
  Vor = 'V',
  Ndb = 'N',
  Waypoint = 'W',
  Runway = 'R',
}

export enum SiteKind {
  Airport = 'Airport',
  Intersection = 'Intersection',
  VOR = 'VOR',
  NDB = 'NDB',
}

export enum SiteSearch {
  All = 0,
  Airport = 1,
  Intersection = 2,
  Vor = 3,
  Ndb = 4,
  AllExceptVisual = 8,
}

export enum AirportClassMaskX {
  None = 0,
  HardSurface = 2,
  SoftSurface = 4,
  AllWater = 8,
  HeliportOnly = 16,
  Private = 32,
}

export interface RunwayIds {
  __Type: 'JS_RunwayIdentifier';
  number: string;
  designator: string;
}

export interface Site {
  readonly icao: string;
  readonly icaoStruct: IcaoValueX;
  readonly name: string;
  readonly lat: number;
  readonly lon: number;
  readonly region: string;
  readonly city: string;
  readonly [field: string]: unknown;
}

export type SiteKindMap = {
  [SiteKind.Airport]: Site;
  [SiteKind.Intersection]: Site;
  [SiteKind.VOR]: Site;
  [SiteKind.NDB]: Site;
};

export interface MetarData {
  readonly icao: string;
  readonly metarString: string;
  readonly [field: string]: unknown;
}

export interface TafData {
  readonly icao: string;
  readonly tafString: string;
  readonly [field: string]: unknown;
}

export interface SiteCycles {
  previous: Cycle;
  current: Cycle;
  next: Cycle;
}

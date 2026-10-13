//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

export enum AirportClass {
  Unknown = 0,
  HardSurfaceWithRunways = 1,
  SoftSurfaceWithRunways = 2,
  Seaplane = 3,
  Heliport = 4,
  Private = 5,
}

export enum AirportPrivateType {
  Unknown = 0,
  Public = 1,
  Military = 2,
  Private = 3,
}

export enum AirspaceType {
  None = 0,
  Center = 1,
  ClassA = 2,
  ClassB = 3,
  ClassC = 4,
  ClassD = 5,
  ClassE = 6,
  ClassF = 7,
  ClassG = 8,
  Tower = 9,
  Clearance = 10,
  Ground = 11,
  Departure = 12,
  Approach = 13,
  MOA = 14,
  Restricted = 15,
  Prohibited = 16,
  Warning = 17,
  Alert = 18,
  Danger = 19,
  NationalPark = 20,
  ModeC = 21,
  Radar = 22,
  Training = 23,
}

export enum AltitudeDescriptor {
  Empty = 0,
  At = 1,
  AtOrAbove = 2,
  AtOrBelow = 3,
  Between = 4,
  C = 5,
  G = 6,
  H = 7,
  I = 8,
  J = 9,
  V = 10,
}

export enum ApproachType {
  Unknown = 0,
  Gps = 1,
  Vor = 2,
  Ndb = 3,
  Ils = 4,
  Loc = 5,
  Sdf = 6,
  Lda = 7,
  VorDme = 8,
  NdbDme = 9,
  Rnav = 10,
  Backcourse = 11,
}

export enum FixTypeFlags {
  None = 0,
  IAF = 1,
  IF = 2,
  MAP = 4,
  FAF = 8,
  MAHP = 16,
}

export enum FrequencyType {
  None = 0,
  ATIS = 1,
  Multicom = 2,
  Unicom = 3,
  CTAF = 4,
  Ground = 5,
  Tower = 6,
  Clearance = 7,
  Approach = 8,
  Departure = 9,
  Center = 10,
  FSS = 11,
  AWOS = 12,
  ASOS = 13,
  ClearancePreTaxi = 14,
  RemoteDeliveryClearance = 15,
}

export enum IcaoSearchFilter {
  None = 0,
  Airports = 1,
  Intersections = 2,
  Vors = 3,
  Ndbs = 4,
}

export enum JSAirportRequestFlags {
  Minimal = 0,
  Approaches = 1,
  Departues = 2,
  Arrivals = 4,
  Frequencies = 8,
  Gates = 16,
  HoldingPatterns = 32,
  Runways = 64,
  All = 127,
}

export enum LandingSystemCategory {
  None = 0,
  Cat1 = 1,
  Cat2 = 2,
  Cat3 = 3,
  Localizer = 4,
  Igs = 5,
  LdaNoGs = 6,
  LdaWithGs = 7,
  SdfNoGs = 8,
  SdfWithGs = 9,
}

export enum LegType {
  Unknown = 0,
  AF = 1,
  CA = 2,
  CD = 3,
  CF = 4,
  CI = 5,
  CR = 6,
  DF = 7,
  FA = 8,
  FC = 9,
  FD = 10,
  FM = 11,
  HA = 12,
  HF = 13,
  HM = 14,
  IF = 15,
  PI = 16,
  RF = 17,
  TF = 18,
  VA = 19,
  VD = 20,
  VI = 21,
  VM = 22,
  VR = 23,
}

export enum MsfsFacilityType {
  None = 0,
  Airport = 1,
  Intersection = 2,
  Vor = 3,
  Ndb = 4,
  Boundary = 5,
}

export enum MsfsIntersectionType {
  None = 0,
  Named = 1,
  Unnamed = 2,
  Vor = 3,
  Ndb = 4,
  Offroute = 5,
  Iaf = 6,
  Faf = 7,
  Rnav = 8,
  Vfr = 9,
}

export enum NdbType {
  CompassLocator = 0,
  MH = 1,
  H = 2,
  HH = 3,
}

export enum NearestSearchType {
  None = 0,
  Airport = 1,
  Intersection = 2,
  Vor = 3,
  Ndb = 4,
  Boundary = 5,
}

export enum RnavTypeFlags {
  None = 0,
  LNAV = 1,
  LNAVVNAV = 2,
  LP = 4,
  LPV = 8,
}

export enum RouteType {
  None = 0,
  LowLevel = 1,
  HighLevel = 2,
  All = 3,
}

export enum RunwayDesignatorChar {
  None = 0,
  L = 1,
  R = 2,
  C = 3,
  W = 4,
  A = 5,
  B = 6,
}

export enum RunwayLighting {
  Unknown = 0,
  None = 1,
  PartTime = 2,
  FullTime = 3,
  Frequency = 4,
}

export enum RunwaySurface {
  Concrete = 0,
  Grass = 1,
  WaterFsx = 2,
  GrassBumpy = 3,
  Asphalt = 4,
  ShortGrass = 5,
  LongGrass = 6,
  HardTurf = 7,
  Snow = 8,
  Ice = 9,
  Urban = 10,
  Forest = 11,
  Dirt = 12,
  Coral = 13,
  Gravel = 14,
  OilTreated = 15,
  SteelMats = 16,
  Bituminous = 17,
  Brick = 18,
  Macadam = 19,
  Planks = 20,
  Sand = 21,
  Shale = 22,
  Tarmac = 23,
  WrightFlyerTrack = 24,
  Ocean = 26,
  Water = 27,
  Pond = 28,
  Lake = 29,
  River = 30,
  WasterWater = 31,
  Paint = 32,
}

export enum SpeedRestrictionDescriptor {
  Unused = 0,
  At = 1,
  AtOrAbove = 2,
  AtOrBelow = 3,
  Between = 4,
}

export enum TacanMode {
  None = 0,
  X = 88,
  Y = 89,
}

export enum TurnDirection {
  Unknown = 0,
  Left = 1,
  Right = 2,
  Either = 3,
}

export enum VorClass {
  Unknown = 0,
  Terminal = 1,
  LowAltitude = 2,
  HighAltitude = 3,
  ILS = 4,
  VOT = 5,
}

export enum VorType {
  Unknown = 0,
  VOR = 1,
  VORDME = 2,
  DME = 3,
  TACAN = 4,
  VORTAC = 5,
  ILS = 6,
  VOT = 7,
}

export interface JS_FacilityAirport {
  __Type: 'JS_FacilityAirport';
  airportClass: AirportClass;
  airportPrivateType: AirportPrivateType;
  airspaceType: AirspaceType;
  altitude: number;
  approaches: JS_Approach[];
  arrivals: JS_Arrival[];
  bestApproach: string;
  city: string;
  departures: JS_Departure[];
  frequencies: JS_Frequency[];
  fuel1: string;
  fuel2: string;
  gates: JS_Gate[];
  holdingPatterns: JS_HoldingPattern[];
  iata: string;
  icao: string;
  icaoStruct: JS_ICAO;
  lat: number;
  loadedDataFlags: number;
  lon: number;
  magvar: number;
  name: string;
  radarCoverage: number;
  region: string;
  runways: JS_Runway[];
  towered: boolean;
  transitionAlt: number;
  transitionLevel: number;
}

export interface JS_Approach {
  __Type: 'JS_Approach';
  approachSuffix: string;
  approachType: ApproachType;
  finalLegs: JS_Leg[];
  icaos: string[];
  missedApproachRnpAr: boolean;
  missedLegs: JS_Leg[];
  name: string;
  rnavTypeFlags: RnavTypeFlags;
  rnpAr: boolean;
  runway: string;
  runwayDesignator: RunwayDesignatorChar;
  runwayNumber: number;
  transitions: JS_ApproachTransition[];
  rnpArMissed?: boolean;
}

export interface JS_Leg {
  __Type: 'JS_Leg';
  altDesc: AltitudeDescriptor;
  altitude1: number;
  altitude2: number;
  arcCenterFixIcao: string;
  arcCenterFixIcaoStruct: JS_ICAO;
  course: number;
  distance: number;
  distanceMinutes: boolean;
  fixIcao: string;
  fixIcaoStruct: JS_ICAO;
  fixTypeFlags: FixTypeFlags;
  flyOver: boolean;
  originIcao: string;
  originIcaoStruct: JS_ICAO;
  rho: number;
  rnp: number;
  speedRestriction: number;
  speedRestrictionDesc: SpeedRestrictionDescriptor;
  theta: number;
  trueDegrees: boolean;
  turnDirection: TurnDirection;
  type: LegType;
  verticalAngle: number;
  speedRestrictionType?: SpeedRestrictionDescriptor;
}

export interface JS_ICAO {
  __Type: 'JS_ICAO';
  airport: string;
  ident: string;
  region: string;
  type: string;
}

export interface JS_ApproachTransition {
  __Type: 'JS_ApproachTransition';
  legs: JS_Leg[];
  name: string;
}

export interface JS_Arrival {
  __Type: 'JS_Arrival';
  commonLegs: JS_Leg[];
  enRouteTransitions: JS_EnRouteTransition[];
  name: string;
  rnpAr: boolean;
  runwayTransitions: JS_RunwayTransition[];
}

export interface JS_Departure {
  __Type: 'JS_Departure';
  commonLegs: JS_Leg[];
  enRouteTransitions: JS_EnRouteTransition[];
  name: string;
  rnpAr: boolean;
  runwayTransitions: JS_RunwayTransition[];
}

export interface JS_RunwayTransition {
  __Type: 'JS_RunwayTransition';
  legs: JS_Leg[];
  runwayDesignation: RunwayDesignatorChar;
  runwayNumber: number;
}

export interface JS_Frequency {
  __Type: 'JS_Frequency';
  freqBCD16: number;
  freqMHz: number;
  icao: string;
  icaoStruct: JS_ICAO;
  name: string;
  type: FrequencyType;
}

export interface JS_Gate {
  __Type: 'JS_Gate';
  latitude: number;
  longitude: number;
  name: number;
  number: number;
  suffix: number;
}

export interface JS_Runway {
  __Type: 'JS_Runway';
  designation: string;
  designatorCharPrimary: RunwayDesignatorChar;
  designatorCharSecondary: RunwayDesignatorChar;
  direction: number;
  elevation: number;
  latitude: number;
  length: number;
  lighting: RunwayLighting;
  longitude: number;
  primaryBlastpadLength: number;
  primaryElevation: number;
  primaryILSFrequency: JS_ILSFrequency;
  primaryOverrunLength: number;
  primaryThresholdLength: number;
  secondaryBlastpadLength: number;
  secondaryElevation: number;
  secondaryILSFrequency: JS_ILSFrequency;
  secondaryOverrunLength: number;
  secondaryThresholdLength: number;
  surface: RunwaySurface;
  width: number;
}

export interface JS_ILSFrequency {
  __Type: 'JS_ILSFrequency';
  freqBCD16: number;
  freqMHz: number;
  glideslopeAlt: number;
  glideslopeAngle: number;
  glideslopeLat: number;
  glideslopeLon: number;
  hasBackcourse: boolean;
  hasGlideslope: boolean;
  icao: string;
  icaoStruct: JS_ICAO;
  localizerCourse: number;
  localizerWidth: number;
  lsCategory: LandingSystemCategory;
  magvar: number;
  name: string;
  type: number;
}

export interface JS_EnRouteTransition {
  __Type: 'JS_EnRouteTransition';
  legs: JS_Leg[];
  name: string;
}

export interface JS_HoldingPattern {
  __Type: 'JS_HoldingPattern';
  icaoStruct: JS_ICAO;
  inboundCourse: number;
  legLength: number;
  legTime: number;
  maxAltitude: number;
  minAltitude: number;
  name: string;
  radius: number;
  rnp: number;
  speed: number;
  turnRight: boolean;
}

export interface JS_FacilityIntersection {
  __Type: 'JS_FacilityIntersection';
  city: string;
  icao: string;
  icaoStruct: JS_ICAO;
  lat: number;
  lon: number;
  name: string;
  nearestVorDistance: number;
  nearestVorFrequencyBCD16: number;
  nearestVorFrequencyMHz: number;
  nearestVorICAO: string;
  nearestVorICAOStruct: JS_ICAO;
  nearestVorMagneticRadial: number;
  nearestVorTrueRadial: number;
  nearestVorType: VorType;
  region: string;
  routes: JS_Route[];
  type: MsfsIntersectionType;
}

export interface JS_Route {
  __Type: 'JS_Route';
  name: string;
  nextIcao: string;
  nextIcaoStruct: JS_ICAO;
  nextMinAlt: number;
  prevIcao: string;
  prevIcaoStruct: JS_ICAO;
  prevMinAlt: number;
  type: RouteType;
}

export interface JS_FacilityNDB {
  __Type: 'JS_FacilityNDB';
  alt: number;
  bfoRequired: boolean;
  city: string;
  freqBCD16: number;
  freqMHz: number;
  icao: string;
  icaoStruct: JS_ICAO;
  lat: number;
  lon: number;
  magvar: number;
  name: string;
  range: number;
  region: string;
  type: NdbType;
  weatherBroadcast: number;
}

export interface JS_FacilityVOR {
  __Type: 'JS_FacilityVOR';
  alt: number;
  city: string;
  dme: JS_DME | null;
  freqBCD16: number;
  freqMHz: number;
  icao: string;
  icaoStruct: JS_ICAO;
  ils: JS_ILSFrequency | null;
  lat: number;
  lon: number;
  magneticVariation: number;
  magvar: number;
  name: string;
  navRange: number;
  region: string;
  tacan: JS_TACAN | null;
  trueReferenced: boolean;
  type: VorType;
  vorClass: VorClass;
  weatherBroadcast: number;
}

export interface JS_DME {
  __Type: 'JS_DME';
  alt: number;
  atGlideslope: boolean;
  atNav: boolean;
  dmeBias: number;
  lat: number;
  lon: number;
}

export interface JS_TACAN {
  __Type: 'JS_TACAN';
  alt: number;
  channel: number;
  lat: number;
  lon: number;
  mode: TacanMode;
}

export type JS_Facility = JS_FacilityAirport | JS_FacilityIntersection | JS_FacilityNDB | JS_FacilityVOR;

export type JS_Procedure = JS_Approach | JS_Departure | JS_Arrival;

export interface MsfsNearestSearchStructResult {
  sessionId: number;
  searchId: number;
  added: JS_ICAO[];
  removed: JS_ICAO[];
}

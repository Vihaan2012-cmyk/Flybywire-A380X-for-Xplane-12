import { FacilityType, IcaoType, IcaoValue } from './facilityTypes';

const typeByFacility: Record<FacilityType, string> = {
  [FacilityType.Airport]: IcaoType.Airport,
  [FacilityType.VOR]: IcaoType.Vor,
  [FacilityType.NDB]: IcaoType.Ndb,
  [FacilityType.Intersection]: IcaoType.Waypoint,
};

const facilityByType = new Map<string, FacilityType>(
  (Object.keys(typeByFacility) as FacilityType[]).map((facility) => [typeByFacility[facility], facility]),
);

const EMPTY_V1 = ' '.repeat(12);
const EMPTY_V2 = ' '.repeat(19);

const V1_LAYOUT = { region: 2, airport: 4, ident: 5 };
const V2_LAYOUT = { region: 2, airport: 4, ident: 12 };

export class ICAO {
  public static readonly EMPTY_V1 = EMPTY_V1;
  public static readonly EMPTY_V2 = EMPTY_V2;
  public static readonly emptyIcao = EMPTY_V1;

  private static readonly EMPTY_ICAO_VALUE: IcaoValue = Object.freeze({
    __Type: 'JS_ICAO',
    type: '',
    region: '',
    airport: '',
    ident: '',
  });

  public static isValue(query: unknown): query is IcaoValue {
    return typeof query === 'object' && query !== null && (query as IcaoValue).__Type === 'JS_ICAO';
  }

  public static value(type: string, region: string, airport: string, ident: string): IcaoValue {
    return { __Type: 'JS_ICAO', type, region, airport, ident };
  }

  public static emptyValue(): IcaoValue {
    return ICAO.copyValue(ICAO.EMPTY_ICAO_VALUE);
  }

  public static isValueEmpty(value: IcaoValue): boolean {
    return value.type === '' && value.region === '' && value.airport === '' && value.ident === '';
  }

  public static copyValue(value: IcaoValue): IcaoValue {
    return ICAO.value(value.type, value.region, value.airport, value.ident);
  }

  public static stringV1ToValue(icao: string): IcaoValue {
    return ICAO.parse(icao, EMPTY_V1.length, V1_LAYOUT);
  }

  public static stringV2ToValue(icao: string): IcaoValue {
    return ICAO.parse(icao, EMPTY_V2.length, V2_LAYOUT);
  }

  public static valueToStringV1(icao: IcaoValue): string {
    return ICAO.format(icao, V1_LAYOUT, 'V1');
  }

  public static tryValueToStringV1(icao: IcaoValue): string {
    try {
      return ICAO.valueToStringV1(icao);
    } catch {
      return EMPTY_V1;
    }
  }

  public static valueToStringV2(icao: IcaoValue): string {
    return ICAO.format(icao, V2_LAYOUT, 'V2');
  }

  public static tryValueToStringV2(icao: IcaoValue): string {
    try {
      return ICAO.valueToStringV2(icao);
    } catch {
      return EMPTY_V2;
    }
  }

  public static valueEquals(a: IcaoValue, b: IcaoValue): boolean {
    return a.type === b.type && a.region === b.region && a.airport === b.airport && a.ident === b.ident;
  }

  public static getRegionCodeFromStringV1(icao: string): string {
    return icao.substring(1, 3).trim();
  }

  public static getRegionCodeFromStringV2(icao: string): string {
    return icao.substring(1, 3).trim();
  }

  public static getRegionCode = ICAO.getRegionCodeFromStringV1;

  public static getAirportIdentFromStringV1(icao: string): string {
    return icao.substring(3, 7).trim();
  }

  public static getAirportIdentFromStringV2(icao: string): string {
    return icao.substring(3, 7).trim();
  }

  public static getAssociatedAirportIdent = ICAO.getAirportIdentFromStringV1;

  public static getIdentFromStringV1(icao: string): string {
    return icao.substring(7, 12).trim();
  }

  public static getIdentFromStringV2(icao: string): string {
    return icao.substring(7, 19).trim();
  }

  public static getIdent = ICAO.getIdentFromStringV1;

  public static isIcaoTypeFacility(icaoType: string, facilityType?: FacilityType): boolean {
    const found = facilityByType.get(icaoType);
    return found !== undefined && (facilityType === undefined || found === facilityType);
  }

  public static isValueFacility(icao: IcaoValue, facilityType?: FacilityType): boolean {
    return ICAO.isIcaoTypeFacility(icao.type, facilityType);
  }

  public static isStringV1Facility(icao: string, facilityType?: FacilityType): boolean {
    return ICAO.isIcaoTypeFacility(icao.charAt(0), facilityType);
  }

  public static isStringV2Facility(icao: string, facilityType?: FacilityType): boolean {
    return ICAO.isIcaoTypeFacility(icao.charAt(0), facilityType);
  }

  public static isFacility = ICAO.isStringV1Facility;

  public static getFacilityTypeFromIcaoType(icaoType: string): FacilityType {
    const found = facilityByType.get(icaoType);
    if (found === undefined) {
      throw new Error(`ICAO: '${icaoType}' is not a facility type`);
    }
    return found;
  }

  public static getFacilityTypeFromValue(icao: IcaoValue): FacilityType {
    return ICAO.getFacilityTypeFromIcaoType(icao.type);
  }

  public static getFacilityTypeFromStringV1(icao: string): FacilityType {
    return ICAO.getFacilityTypeFromIcaoType(icao.charAt(0));
  }

  public static getFacilityTypeFromStringV2(icao: string): FacilityType {
    return ICAO.getFacilityTypeFromIcaoType(icao.charAt(0));
  }

  public static getFacilityType = ICAO.getFacilityTypeFromStringV1;

  public static getUid(icao: IcaoValue): string {
    return `${icao.type}|${icao.region}|${icao.airport}|${icao.ident}`;
  }

  private static parse(text: string, length: number, layout: { region: number; airport: number; ident: number }): IcaoValue {
    const padded = text.padEnd(length, ' ');
    const regionEnd = 1 + layout.region;
    const airportEnd = regionEnd + layout.airport;
    return ICAO.value(
      padded.substring(0, 1).trim(),
      padded.substring(1, regionEnd).trim(),
      padded.substring(regionEnd, airportEnd).trim(),
      padded.substring(airportEnd, airportEnd + layout.ident).trim(),
    );
  }

  private static format(icao: IcaoValue, layout: { region: number; airport: number; ident: number }, name: string): string {
    if (
      icao.type.length > 1 ||
      icao.region.length > layout.region ||
      icao.airport.length > layout.airport ||
      icao.ident.length > layout.ident
    ) {
      throw new Error(`ICAO: value cannot be converted to a ${name} string`);
    }
    return `${icao.type.padEnd(1, ' ')}${icao.region.padEnd(layout.region, ' ')}${icao.airport.padEnd(layout.airport, ' ')}${icao.ident.padEnd(layout.ident, ' ')}`;
  }
}

import { SiteKind, IcaoTypeX, IcaoValueX } from './facilityTypes';

const typeByFacility: Record<SiteKind, string> = {
  [SiteKind.Airport]: IcaoTypeX.Airport,
  [SiteKind.VOR]: IcaoTypeX.Vor,
  [SiteKind.NDB]: IcaoTypeX.Ndb,
  [SiteKind.Intersection]: IcaoTypeX.Waypoint,
};

const facilityByType = new Map<string, SiteKind>(
  (Object.keys(typeByFacility) as SiteKind[]).map((facility) => [typeByFacility[facility], facility]),
);

const EMPTY_V1 = ' '.repeat(12);
const EMPTY_V2 = ' '.repeat(19);

const V1_LAYOUT = { region: 2, airport: 4, ident: 5 };
const V2_LAYOUT = { region: 2, airport: 4, ident: 12 };

export class IcaoKit {
  public static readonly EMPTY_V1 = EMPTY_V1;
  public static readonly EMPTY_V2 = EMPTY_V2;
  public static readonly emptyIcao = EMPTY_V1;

  private static readonly EMPTY_ICAO_VALUE: IcaoValueX = Object.freeze({
    __Type: 'JS_ICAO',
    type: '',
    region: '',
    airport: '',
    ident: '',
  });

  public static isValue(query: unknown): query is IcaoValueX {
    return typeof query === 'object' && query !== null && (query as IcaoValueX).__Type === 'JS_ICAO';
  }

  public static value(type: string, region: string, airport: string, ident: string): IcaoValueX {
    return { __Type: 'JS_ICAO', type, region, airport, ident };
  }

  public static emptyValue(): IcaoValueX {
    return IcaoKit.copyValue(IcaoKit.EMPTY_ICAO_VALUE);
  }

  public static isValueEmpty(value: IcaoValueX): boolean {
    return value.type === '' && value.region === '' && value.airport === '' && value.ident === '';
  }

  public static copyValue(value: IcaoValueX): IcaoValueX {
    return IcaoKit.value(value.type, value.region, value.airport, value.ident);
  }

  public static stringV1ToValue(icao: string): IcaoValueX {
    return IcaoKit.parse(icao, EMPTY_V1.length, V1_LAYOUT);
  }

  public static stringV2ToValue(icao: string): IcaoValueX {
    return IcaoKit.parse(icao, EMPTY_V2.length, V2_LAYOUT);
  }

  public static valueToStringV1(icao: IcaoValueX): string {
    return IcaoKit.format(icao, V1_LAYOUT, 'V1');
  }

  public static tryValueToStringV1(icao: IcaoValueX): string {
    try {
      return IcaoKit.valueToStringV1(icao);
    } catch {
      return EMPTY_V1;
    }
  }

  public static valueToStringV2(icao: IcaoValueX): string {
    return IcaoKit.format(icao, V2_LAYOUT, 'V2');
  }

  public static tryValueToStringV2(icao: IcaoValueX): string {
    try {
      return IcaoKit.valueToStringV2(icao);
    } catch {
      return EMPTY_V2;
    }
  }

  public static valueEquals(a: IcaoValueX, b: IcaoValueX): boolean {
    return a.type === b.type && a.region === b.region && a.airport === b.airport && a.ident === b.ident;
  }

  public static getRegionCodeFromStringV1(icao: string): string {
    return icao.substring(1, 3).trim();
  }

  public static getRegionCodeFromStringV2(icao: string): string {
    return icao.substring(1, 3).trim();
  }

  public static getRegionCode = IcaoKit.getRegionCodeFromStringV1;

  public static getAirportIdentFromStringV1(icao: string): string {
    return icao.substring(3, 7).trim();
  }

  public static getAirportIdentFromStringV2(icao: string): string {
    return icao.substring(3, 7).trim();
  }

  public static getAssociatedAirportIdent = IcaoKit.getAirportIdentFromStringV1;

  public static getIdentFromStringV1(icao: string): string {
    return icao.substring(7, 12).trim();
  }

  public static getIdentFromStringV2(icao: string): string {
    return icao.substring(7, 19).trim();
  }

  public static getIdent = IcaoKit.getIdentFromStringV1;

  public static isIcaoTypeFacility(icaoType: string, facilityType?: SiteKind): boolean {
    const found = facilityByType.get(icaoType);
    return found !== undefined && (facilityType === undefined || found === facilityType);
  }

  public static isValueFacility(icao: IcaoValueX, facilityType?: SiteKind): boolean {
    return IcaoKit.isIcaoTypeFacility(icao.type, facilityType);
  }

  public static isStringV1Facility(icao: string, facilityType?: SiteKind): boolean {
    return IcaoKit.isIcaoTypeFacility(icao.charAt(0), facilityType);
  }

  public static isStringV2Facility(icao: string, facilityType?: SiteKind): boolean {
    return IcaoKit.isIcaoTypeFacility(icao.charAt(0), facilityType);
  }

  public static isFacility = IcaoKit.isStringV1Facility;

  public static getFacilityTypeFromIcaoType(icaoType: string): SiteKind {
    const found = facilityByType.get(icaoType);
    if (found === undefined) {
      throw new Error(`IcaoKit: '${icaoType}' is not a facility type`);
    }
    return found;
  }

  public static getFacilityTypeFromValue(icao: IcaoValueX): SiteKind {
    return IcaoKit.getFacilityTypeFromIcaoType(icao.type);
  }

  public static getFacilityTypeFromStringV1(icao: string): SiteKind {
    return IcaoKit.getFacilityTypeFromIcaoType(icao.charAt(0));
  }

  public static getFacilityTypeFromStringV2(icao: string): SiteKind {
    return IcaoKit.getFacilityTypeFromIcaoType(icao.charAt(0));
  }

  public static getFacilityType = IcaoKit.getFacilityTypeFromStringV1;

  public static getUid(icao: IcaoValueX): string {
    return `${icao.type}|${icao.region}|${icao.airport}|${icao.ident}`;
  }

  private static parse(text: string, length: number, layout: { region: number; airport: number; ident: number }): IcaoValueX {
    const padded = text.padEnd(length, ' ');
    const regionEnd = 1 + layout.region;
    const airportEnd = regionEnd + layout.airport;
    return IcaoKit.value(
      padded.substring(0, 1).trim(),
      padded.substring(1, regionEnd).trim(),
      padded.substring(regionEnd, airportEnd).trim(),
      padded.substring(airportEnd, airportEnd + layout.ident).trim(),
    );
  }

  private static format(icao: IcaoValueX, layout: { region: number; airport: number; ident: number }, name: string): string {
    if (
      icao.type.length > 1 ||
      icao.region.length > layout.region ||
      icao.airport.length > layout.airport ||
      icao.ident.length > layout.ident
    ) {
      throw new Error(`IcaoKit: value cannot be converted to a ${name} string`);
    }
    return `${icao.type.padEnd(1, ' ')}${icao.region.padEnd(layout.region, ' ')}${icao.airport.padEnd(layout.airport, ' ')}${icao.ident.padEnd(layout.ident, ' ')}`;
  }
}

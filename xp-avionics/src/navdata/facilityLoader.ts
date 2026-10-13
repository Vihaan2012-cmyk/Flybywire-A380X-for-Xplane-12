import { AiracCycle, AiracUtils } from './airac';
import {
  Facility,
  FacilityDatabaseCycles,
  FacilitySearchType,
  FacilityType,
  FacilityTypeMap,
  IcaoValue,
  Metar,
  Taf,
} from './facilityTypes';
import { ICAO } from './icao';

declare const Coherent: {
  call(name: string, ...args: any[]): Promise<any>;
  on(name: string, handler: (...args: any[]) => void): { clear(): void };
};
declare function RegisterViewListener(name: string, callback?: () => void, requiresSingleton?: boolean): unknown;
declare const SimVar: { GetGameVarValue(name: string, unit: string): any };

export interface FacilityLoaderOptions {
  sharedFacilityCacheId?: string;
}

export interface NearestSearchResultsLike<TAdded, TRemoved> {
  readonly sessionId: number;
  readonly searchId: number;
  readonly added: readonly TAdded[];
  readonly removed: readonly TRemoved[];
}

export interface NearestSearchSession<TAdded, TRemoved> {
  searchNearest(
    lat: number,
    lon: number,
    radius: number,
    maxItems: number,
  ): Promise<NearestSearchResultsLike<TAdded, TRemoved>>;
}

interface PendingFacility {
  resolve(facility: Facility | null): void;
}

interface SearchWaiter {
  resolve(results: NearestSearchResultsLike<any, any>): void;
}

const loadCallByType: Record<FacilityType, string> = {
  [FacilityType.Airport]: 'LOAD_AIRPORT_FROM_STRUCT',
  [FacilityType.VOR]: 'LOAD_VOR_FROM_STRUCT',
  [FacilityType.NDB]: 'LOAD_NDB_FROM_STRUCT',
  [FacilityType.Intersection]: 'LOAD_INTERSECTION_FROM_STRUCT',
};

const eventByType: Record<FacilityType, string> = {
  [FacilityType.Airport]: 'SendAirport',
  [FacilityType.VOR]: 'SendVor',
  [FacilityType.NDB]: 'SendNdb',
  [FacilityType.Intersection]: 'SendIntersection',
};

const ALL_AIRPORT_DATA = 127;
const LOAD_TIMEOUT_MS = 10_000;

export class FacilityLoader {
  private static listenerPromise?: Promise<void>;
  private static readonly pending = new Map<string, PendingFacility[]>();
  private static readonly searches = new Map<number, Map<number, SearchWaiter[]>>();
  private static readonly sessionKinds = new Map<number, boolean>();
  private static cycleCache?: FacilityDatabaseCycles;

  private readonly initPromise: Promise<void>;
  private readonly cache = new Map<string, Facility>();

  constructor(
    private readonly facilityRepo?: unknown,
    public readonly onInitialized: () => void = () => {},
    private readonly options?: Readonly<FacilityLoaderOptions>,
  ) {
    this.initPromise = FacilityLoader.initListener().then(() => this.onInitialized());
  }

  private static initListener(): Promise<void> {
    if (FacilityLoader.listenerPromise === undefined) {
      FacilityLoader.listenerPromise = new Promise<void>((resolve) => {
        RegisterViewListener('JS_LISTENER_FACILITY', () => resolve(), true);
        for (const type of Object.keys(eventByType) as FacilityType[]) {
          Coherent.on(eventByType[type], (facility: Facility) => FacilityLoader.onFacilityReceived(type, facility));
        }
        Coherent.on('NearestSearchCompletedWithStruct', (results) => FacilityLoader.onSearchCompleted(results));
        Coherent.on('NearestSearchCompleted', (results) => FacilityLoader.onSearchCompleted(results));
      });
    }
    return FacilityLoader.listenerPromise;
  }

  public awaitInitialization(): Promise<void> {
    return this.initPromise;
  }

  public async tryGetFacility<T extends FacilityType>(
    type: T,
    icao: IcaoValue,
    airportDataFlags = ALL_AIRPORT_DATA,
  ): Promise<FacilityTypeMap[T] | null> {
    if (ICAO.isValueEmpty(icao)) {
      return null;
    }
    await this.initPromise;
    const key = `${type}:${ICAO.getUid(icao)}:${type === FacilityType.Airport ? airportDataFlags : 0}`;
    const cached = this.cache.get(key);
    if (cached !== undefined) {
      return cached as FacilityTypeMap[T];
    }
    const facility = await FacilityLoader.request(type, icao, airportDataFlags);
    if (facility !== null) {
      this.cache.set(key, facility);
    }
    return facility as FacilityTypeMap[T] | null;
  }

  public async getFacility<T extends FacilityType>(
    type: T,
    icao: IcaoValue | string,
    airportDataFlags?: number,
  ): Promise<FacilityTypeMap[T]> {
    const value = typeof icao === 'string' ? ICAO.stringV1ToValue(icao) : icao;
    const facility = await this.tryGetFacility(type, value, airportDataFlags);
    if (facility === null) {
      throw new Error(`FacilityLoader: facility ${ICAO.tryValueToStringV1(value)} was not found`);
    }
    return facility;
  }

  public getFacilities(icaos: readonly IcaoValue[], airportDataFlags?: number): Promise<(Facility | null)[]> {
    return Promise.all(
      icaos.map((icao) => {
        const type = ICAO.isValueFacility(icao) ? ICAO.getFacilityTypeFromValue(icao) : undefined;
        return type === undefined ? Promise.resolve(null) : this.tryGetFacility(type, icao, airportDataFlags);
      }),
    );
  }

  public getFacilitiesOfType<T extends FacilityType>(
    type: T,
    icaos: readonly IcaoValue[],
    airportDataFlags?: number,
  ): Promise<(FacilityTypeMap[T] | null)[]> {
    return Promise.all(icaos.map((icao) => this.tryGetFacility(type, icao, airportDataFlags)));
  }

  public async searchByIdentWithIcaoStructs(
    filter: FacilitySearchType,
    ident: string,
    maxItems = 40,
  ): Promise<IcaoValue[]> {
    await this.initPromise;
    const results: IcaoValue[] | undefined = await Coherent.call('SEARCH_BY_IDENT_WITH_STRUCT', ident, filter, maxItems);
    return results ?? [];
  }

  public async searchByIdent(filter: FacilitySearchType, ident: string, maxItems = 40): Promise<string[]> {
    await this.initPromise;
    const results: string[] | undefined = await Coherent.call('SEARCH_BY_IDENT', ident, filter, maxItems);
    return results ?? [];
  }

  public async startNearestSearchSessionWithIcaoStructs(
    type: FacilitySearchType,
  ): Promise<NearestSearchSession<IcaoValue, IcaoValue>> {
    return this.startSession(type, true);
  }

  public async startNearestSearchSession(type: FacilitySearchType): Promise<NearestSearchSession<string, string>> {
    return this.startSession(type, false);
  }

  public async getMetar(airport: Facility | string): Promise<Metar | undefined> {
    const ident = typeof airport === 'string' ? airport : ICAO.getIdentFromStringV1(airport.icao);
    return FacilityLoader.cleanReport(await Coherent.call('GET_METAR_BY_IDENT', ident));
  }

  public async searchMetar(lat: number, lon: number): Promise<Metar | undefined> {
    return FacilityLoader.cleanReport(await Coherent.call('GET_METAR_BY_LATLON', lat, lon));
  }

  public async getTaf(airport: Facility | string): Promise<Taf | undefined> {
    const ident = typeof airport === 'string' ? airport : ICAO.getIdentFromStringV1(airport.icao);
    return FacilityLoader.cleanReport(await Coherent.call('GET_TAF_BY_IDENT', ident));
  }

  public async searchTaf(lat: number, lon: number): Promise<Taf | undefined> {
    return FacilityLoader.cleanReport(await Coherent.call('GET_TAF_BY_LATLON', lat, lon));
  }

  public static getDatabaseCycles(): FacilityDatabaseCycles {
    if (FacilityLoader.cycleCache === undefined) {
      const range = SimVar.GetGameVarValue('FLIGHT NAVDATA DATE RANGE', 'string');
      const current: AiracCycle = AiracUtils.parseFacilitiesCycle(range) ?? AiracUtils.getCurrentCycle(new Date());
      FacilityLoader.cycleCache = {
        previous: AiracUtils.getOffsetCycle(current, -1),
        current,
        next: AiracUtils.getOffsetCycle(current, 1),
      };
    }
    return FacilityLoader.cycleCache;
  }

  private static pendingKey(icao: IcaoValue): string {
    return `${icao.type}|${icao.airport}|${icao.ident}`;
  }

  private static cleanReport<T extends { icao: string }>(report: T | undefined | null): T | undefined {
    return report === undefined || report === null || report.icao === '' ? undefined : report;
  }

  private static request(type: FacilityType, icao: IcaoValue, airportDataFlags: number): Promise<Facility | null> {
    return new Promise<Facility | null>((resolve) => {
      const key = FacilityLoader.pendingKey(icao);
      const waiters = FacilityLoader.pending.get(key) ?? [];
      const waiter: PendingFacility = { resolve: (facility) => finish(facility) };
      const timer = setTimeout(() => finish(null), LOAD_TIMEOUT_MS);
      const finish = (facility: Facility | null): void => {
        clearTimeout(timer);
        const remaining = (FacilityLoader.pending.get(key) ?? []).filter((w) => w !== waiter);
        if (remaining.length > 0) {
          FacilityLoader.pending.set(key, remaining);
        } else {
          FacilityLoader.pending.delete(key);
        }
        resolve(facility);
      };
      waiters.push(waiter);
      FacilityLoader.pending.set(key, waiters);
      const args = type === FacilityType.Airport ? [icao, airportDataFlags] : [icao];
      Coherent.call(loadCallByType[type], ...args).then(
        (found: boolean) => {
          if (!found) {
            finish(null);
          }
        },
        () => finish(null),
      );
    });
  }

  private static onFacilityReceived(type: FacilityType, facility: Facility): void {
    const key = FacilityLoader.pendingKey(ICAO.stringV1ToValue(facility.icao));
    const waiters = FacilityLoader.pending.get(key);
    if (waiters === undefined) {
      return;
    }
    for (const waiter of waiters.slice()) {
      waiter.resolve(facility);
    }
  }

  private static onSearchCompleted(results: NearestSearchResultsLike<any, any>): void {
    const bySearch = FacilityLoader.searches.get(results.sessionId);
    const waiters = bySearch?.get(results.searchId);
    if (waiters === undefined) {
      return;
    }
    bySearch?.delete(results.searchId);
    for (const waiter of waiters) {
      waiter.resolve(results);
    }
  }

  private async startSession(type: FacilitySearchType, structs: boolean): Promise<NearestSearchSession<any, any>> {
    await this.initPromise;
    const call = structs ? 'START_NEAREST_SEARCH_SESSION_WITH_STRUCT' : 'START_NEAREST_SEARCH_SESSION';
    const sessionId: number = await Coherent.call(call, type);
    FacilityLoader.sessionKinds.set(sessionId, structs);
    FacilityLoader.searches.set(sessionId, new Map());
    return {
      searchNearest: async (lat, lon, radius, maxItems) => {
        const searchId: number = await Coherent.call('SEARCH_NEAREST', sessionId, lat, lon, radius, maxItems);
        const bySearch = FacilityLoader.searches.get(sessionId) as Map<number, SearchWaiter[]>;
        return new Promise((resolve) => {
          bySearch.set(searchId, [...(bySearch.get(searchId) ?? []), { resolve }]);
        });
      },
    };
  }
}

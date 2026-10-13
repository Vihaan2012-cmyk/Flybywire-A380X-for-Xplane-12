import { Cycle, CycleKit } from './airac';
import {
  Site,
  SiteCycles,
  SiteSearch,
  SiteKind,
  SiteKindMap,
  IcaoValueX,
  MetarData,
  TafData,
} from './facilityTypes';
import { IcaoKit } from './icao';

declare const Coherent: {
  call(name: string, ...args: any[]): Promise<any>;
  on(name: string, handler: (...args: any[]) => void): { clear(): void };
};
declare function RegisterViewListener(name: string, callback?: () => void, requiresSingleton?: boolean): unknown;
declare const SimVar: { GetGameVarValue(name: string, unit: string): any };

export interface SiteLoaderOptions {
  sharedFacilityCacheId?: string;
}

export interface NearResults<TAdded, TRemoved> {
  readonly sessionId: number;
  readonly searchId: number;
  readonly added: readonly TAdded[];
  readonly removed: readonly TRemoved[];
}

export interface NearSearch<TAdded, TRemoved> {
  searchNearest(
    lat: number,
    lon: number,
    radius: number,
    maxItems: number,
  ): Promise<NearResults<TAdded, TRemoved>>;
}

interface PendingFacility {
  resolve(facility: Site | null): void;
}

interface SearchWaiter {
  resolve(results: NearResults<any, any>): void;
}

const loadCallByType: Record<SiteKind, string> = {
  [SiteKind.Airport]: 'LOAD_AIRPORT_FROM_STRUCT',
  [SiteKind.VOR]: 'LOAD_VOR_FROM_STRUCT',
  [SiteKind.NDB]: 'LOAD_NDB_FROM_STRUCT',
  [SiteKind.Intersection]: 'LOAD_INTERSECTION_FROM_STRUCT',
};

const eventByType: Record<SiteKind, string> = {
  [SiteKind.Airport]: 'SendAirport',
  [SiteKind.VOR]: 'SendVor',
  [SiteKind.NDB]: 'SendNdb',
  [SiteKind.Intersection]: 'SendIntersection',
};

const ALL_AIRPORT_DATA = 127;
const LOAD_TIMEOUT_MS = 10_000;

export class SiteLoader {
  private static listenerPromise?: Promise<void>;
  private static readonly pending = new Map<string, PendingFacility[]>();
  private static readonly searches = new Map<number, Map<number, SearchWaiter[]>>();
  private static readonly sessionKinds = new Map<number, boolean>();
  private static cycleCache?: SiteCycles;

  private readonly initPromise: Promise<void>;
  private readonly cache = new Map<string, Site>();

  constructor(
    private readonly facilityRepo?: unknown,
    public readonly onInitialized: () => void = () => {},
    private readonly options?: Readonly<SiteLoaderOptions>,
  ) {
    this.initPromise = SiteLoader.initListener().then(() => this.onInitialized());
  }

  private static initListener(): Promise<void> {
    if (SiteLoader.listenerPromise === undefined) {
      SiteLoader.listenerPromise = new Promise<void>((resolve) => {
        RegisterViewListener('JS_LISTENER_FACILITY', () => resolve(), true);
        for (const type of Object.keys(eventByType) as SiteKind[]) {
          Coherent.on(eventByType[type], (facility: Site) => SiteLoader.onFacilityReceived(type, facility));
        }
        Coherent.on('NearestSearchCompletedWithStruct', (results) => SiteLoader.onSearchCompleted(results));
        Coherent.on('NearestSearchCompleted', (results) => SiteLoader.onSearchCompleted(results));
      });
    }
    return SiteLoader.listenerPromise;
  }

  public awaitInitialization(): Promise<void> {
    return this.initPromise;
  }

  public async tryGetFacility<T extends SiteKind>(
    type: T,
    icao: IcaoValueX,
    airportDataFlags = ALL_AIRPORT_DATA,
  ): Promise<SiteKindMap[T] | null> {
    if (IcaoKit.isValueEmpty(icao)) {
      return null;
    }
    await this.initPromise;
    const key = `${type}:${IcaoKit.getUid(icao)}:${type === SiteKind.Airport ? airportDataFlags : 0}`;
    const cached = this.cache.get(key);
    if (cached !== undefined) {
      return cached as SiteKindMap[T];
    }
    const facility = await SiteLoader.request(type, icao, airportDataFlags);
    if (facility !== null) {
      this.cache.set(key, facility);
    }
    return facility as SiteKindMap[T] | null;
  }

  public async getFacility<T extends SiteKind>(
    type: T,
    icao: IcaoValueX | string,
    airportDataFlags?: number,
  ): Promise<SiteKindMap[T]> {
    const value = typeof icao === 'string' ? IcaoKit.stringV1ToValue(icao) : icao;
    const facility = await this.tryGetFacility(type, value, airportDataFlags);
    if (facility === null) {
      throw new Error(`SiteLoader: facility ${IcaoKit.tryValueToStringV1(value)} was not found`);
    }
    return facility;
  }

  public getFacilities(icaos: readonly IcaoValueX[], airportDataFlags?: number): Promise<(Site | null)[]> {
    return Promise.all(
      icaos.map((icao) => {
        const type = IcaoKit.isValueFacility(icao) ? IcaoKit.getFacilityTypeFromValue(icao) : undefined;
        return type === undefined ? Promise.resolve(null) : this.tryGetFacility(type, icao, airportDataFlags);
      }),
    );
  }

  public getFacilitiesOfType<T extends SiteKind>(
    type: T,
    icaos: readonly IcaoValueX[],
    airportDataFlags?: number,
  ): Promise<(SiteKindMap[T] | null)[]> {
    return Promise.all(icaos.map((icao) => this.tryGetFacility(type, icao, airportDataFlags)));
  }

  public async searchByIdentWithIcaoStructs(
    filter: SiteSearch,
    ident: string,
    maxItems = 40,
  ): Promise<IcaoValueX[]> {
    await this.initPromise;
    const results: IcaoValueX[] | undefined = await Coherent.call('SEARCH_BY_IDENT_WITH_STRUCT', ident, filter, maxItems);
    return results ?? [];
  }

  public async searchByIdent(filter: SiteSearch, ident: string, maxItems = 40): Promise<string[]> {
    await this.initPromise;
    const results: string[] | undefined = await Coherent.call('SEARCH_BY_IDENT', ident, filter, maxItems);
    return results ?? [];
  }

  public async startNearestSearchSessionWithIcaoStructs(
    type: SiteSearch,
  ): Promise<NearSearch<IcaoValueX, IcaoValueX>> {
    return this.startSession(type, true);
  }

  public async startNearestSearchSession(type: SiteSearch): Promise<NearSearch<string, string>> {
    return this.startSession(type, false);
  }

  public async getMetar(airport: Site | string): Promise<MetarData | undefined> {
    const ident = typeof airport === 'string' ? airport : IcaoKit.getIdentFromStringV1(airport.icao);
    return SiteLoader.cleanReport(await Coherent.call('GET_METAR_BY_IDENT', ident));
  }

  public async searchMetar(lat: number, lon: number): Promise<MetarData | undefined> {
    return SiteLoader.cleanReport(await Coherent.call('GET_METAR_BY_LATLON', lat, lon));
  }

  public async getTaf(airport: Site | string): Promise<TafData | undefined> {
    const ident = typeof airport === 'string' ? airport : IcaoKit.getIdentFromStringV1(airport.icao);
    return SiteLoader.cleanReport(await Coherent.call('GET_TAF_BY_IDENT', ident));
  }

  public async searchTaf(lat: number, lon: number): Promise<TafData | undefined> {
    return SiteLoader.cleanReport(await Coherent.call('GET_TAF_BY_LATLON', lat, lon));
  }

  public static getDatabaseCycles(): SiteCycles {
    if (SiteLoader.cycleCache === undefined) {
      const range = SimVar.GetGameVarValue('FLIGHT NAVDATA DATE RANGE', 'string');
      const current: Cycle = CycleKit.parseFacilitiesCycle(range) ?? CycleKit.getCurrentCycle(new Date());
      SiteLoader.cycleCache = {
        previous: CycleKit.getOffsetCycle(current, -1),
        current,
        next: CycleKit.getOffsetCycle(current, 1),
      };
    }
    return SiteLoader.cycleCache;
  }

  private static pendingKey(icao: IcaoValueX): string {
    return `${icao.type}|${icao.airport}|${icao.ident}`;
  }

  private static cleanReport<T extends { icao: string }>(report: T | undefined | null): T | undefined {
    return report === undefined || report === null || report.icao === '' ? undefined : report;
  }

  private static request(type: SiteKind, icao: IcaoValueX, airportDataFlags: number): Promise<Site | null> {
    return new Promise<Site | null>((resolve) => {
      const key = SiteLoader.pendingKey(icao);
      const waiters = SiteLoader.pending.get(key) ?? [];
      const waiter: PendingFacility = { resolve: (facility) => finish(facility) };
      const timer = setTimeout(() => finish(null), LOAD_TIMEOUT_MS);
      const finish = (facility: Site | null): void => {
        clearTimeout(timer);
        const remaining = (SiteLoader.pending.get(key) ?? []).filter((w) => w !== waiter);
        if (remaining.length > 0) {
          SiteLoader.pending.set(key, remaining);
        } else {
          SiteLoader.pending.delete(key);
        }
        resolve(facility);
      };
      waiters.push(waiter);
      SiteLoader.pending.set(key, waiters);
      const args = type === SiteKind.Airport ? [icao, airportDataFlags] : [icao];
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

  private static onFacilityReceived(type: SiteKind, facility: Site): void {
    const key = SiteLoader.pendingKey(IcaoKit.stringV1ToValue(facility.icao));
    const waiters = SiteLoader.pending.get(key);
    if (waiters === undefined) {
      return;
    }
    for (const waiter of waiters.slice()) {
      waiter.resolve(facility);
    }
  }

  private static onSearchCompleted(results: NearResults<any, any>): void {
    const bySearch = SiteLoader.searches.get(results.sessionId);
    const waiters = bySearch?.get(results.searchId);
    if (waiters === undefined) {
      return;
    }
    bySearch?.delete(results.searchId);
    for (const waiter of waiters) {
      waiter.resolve(results);
    }
  }

  private async startSession(type: SiteSearch, structs: boolean): Promise<NearSearch<any, any>> {
    await this.initPromise;
    const call = structs ? 'START_NEAREST_SEARCH_SESSION_WITH_STRUCT' : 'START_NEAREST_SEARCH_SESSION';
    const sessionId: number = await Coherent.call(call, type);
    SiteLoader.sessionKinds.set(sessionId, structs);
    SiteLoader.searches.set(sessionId, new Map());
    return {
      searchNearest: async (lat, lon, radius, maxItems) => {
        const searchId: number = await Coherent.call('SEARCH_NEAREST', sessionId, lat, lon, radius, maxItems);
        const bySearch = SiteLoader.searches.get(sessionId) as Map<number, SearchWaiter[]>;
        return new Promise((resolve) => {
          bySearch.set(searchId, [...(bySearch.get(searchId) ?? []), { resolve }]);
        });
      },
    };
  }
}

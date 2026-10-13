import { Cell } from '../sub';
import type { Watchable } from '../sub';
import type { RunwayIds } from './facilityTypes';

export enum ChartSource {
  Lido = 'Lido',
  Faa = 'Faa',
}

export enum ChartImageErrorCodeX {
  None,
  Timeout,
  UnspecifiedRetry,
  UnspecifiedNoRetry,
  Unknown,
}

export enum FaaChartTypeX {
  Unknown = 'Unknown',
  Iap = 'Iap',
  Min = 'Min',
  Dp = 'Dp',
  Star = 'Star',
  Apd = 'Apd',
  Hot = 'Hot',
  Odp = 'Odp',
  Lah = 'Lah',
  Dau = 'Dau',
}

export enum LidoChartTypeX {
  Unknown = 'Unknown',
  Sid = 'Sid',
  SidPt = 'SidPt',
  SidInitialClimb = 'SidInitialClimb',
  ObstDep = 'ObstDep',
  Afc = 'Afc',
  Iac = 'Iac',
  Vac = 'Vac',
  Star = 'Star',
  Mrc = 'Mrc',
  Aoi = 'Aoi',
  Lvc = 'Lvc',
  Agc = 'Agc',
  Apc = 'Apc',
}

export interface ChartImageX {
  readonly imageUrl: string;
  readonly errorCode: ChartImageErrorCodeX;
}

export interface SimChartX<T extends string = string> {
  readonly guid: string;
  readonly name: string;
  readonly type: T;
  readonly runways: readonly RunwayIds[];
}

export interface SimChartPageX {
  readonly urls: readonly { readonly name: string; readonly url: string }[];
}

export interface SimChartIndexX<T extends string = string> {
  readonly charts: readonly { readonly name: T; readonly charts: readonly SimChartX<T>[] }[];
}

export interface SimChartPagesX {
  readonly pages: readonly SimChartPageX[];
}

export class ChartViewX {
  private readonly imageSubject = Cell.create<ChartImageX>({ imageUrl: '', errorCode: ChartImageErrorCodeX.None });

  public readonly image: Watchable<ChartImageX> = this.imageSubject;

  public showChartImage(_pageUrl: string): void {}

  public destroy(): void {}
}

export class ChartPort {
  public static getIndexForAirport<T extends string = string>(_airportIcao: unknown, _provider: string): Promise<SimChartIndexX<T>> {
    return Promise.reject(new Error('Charts are not available'));
  }

  public static getChartPages(_chartGuid: string): Promise<SimChartPagesX> {
    return Promise.reject(new Error('Charts are not available'));
  }

  public static async initializeChartView(_view: ChartViewX): Promise<void> {}
}

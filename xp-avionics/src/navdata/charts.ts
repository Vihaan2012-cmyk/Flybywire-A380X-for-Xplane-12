import { Subject } from '../sub';
import type { Subscribable } from '../sub';
import type { RunwayIdentifier } from './facilityTypes';

export enum BuiltInChartProvider {
  Lido = 'Lido',
  Faa = 'Faa',
}

export enum ChartImageErrorCode {
  None,
  Timeout,
  UnspecifiedRetry,
  UnspecifiedNoRetry,
  Unknown,
}

export enum FaaChartType {
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

export enum LidoChartType {
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

export interface ChartImage {
  readonly imageUrl: string;
  readonly errorCode: ChartImageErrorCode;
}

export interface SimChart<T extends string = string> {
  readonly guid: string;
  readonly name: string;
  readonly type: T;
  readonly runways: readonly RunwayIdentifier[];
}

export interface SimChartPage {
  readonly urls: readonly { readonly name: string; readonly url: string }[];
}

export interface SimChartIndex<T extends string = string> {
  readonly charts: readonly { readonly name: T; readonly charts: readonly SimChart<T>[] }[];
}

export interface SimChartPages {
  readonly pages: readonly SimChartPage[];
}

export class ChartView {
  private readonly imageSubject = Subject.create<ChartImage>({ imageUrl: '', errorCode: ChartImageErrorCode.None });

  public readonly image: Subscribable<ChartImage> = this.imageSubject;

  public showChartImage(_pageUrl: string): void {}

  public destroy(): void {}
}

export class ChartsClient {
  public static getIndexForAirport<T extends string = string>(_airportIcao: unknown, _provider: string): Promise<SimChartIndex<T>> {
    return Promise.reject(new Error('Charts are not available'));
  }

  public static getChartPages(_chartGuid: string): Promise<SimChartPages> {
    return Promise.reject(new Error('Charts are not available'));
  }

  public static async initializeChartView(_view: ChartView): Promise<void> {}
}

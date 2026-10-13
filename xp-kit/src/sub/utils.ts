import { AbstractWatchable } from './core';
import { WritableWatchable, LifeView, Watchable, WatchableSet, Watch } from './types';

class ConstSubscribable<T> extends AbstractWatchable<T> {
  constructor(private readonly value: T) {
    super();
  }

  public get(): T {
    return this.value;
  }
}

class RelaySubscription implements Watch {
  public readonly canNotifyNow = true;
  private _isAlive = true;
  private _isPaused: boolean;
  private pipeSub?: Watch;
  private sourceSub!: Watch;

  constructor(paused: boolean) {
    this._isPaused = paused;
  }

  public attach(sourceSub: Watch): void {
    this.sourceSub = sourceSub;
  }

  public setPipe(sub: Watch | undefined): void {
    this.pipeSub?.destroy();
    this.pipeSub = sub;
    if (sub && this._isPaused) {
      sub.pause();
    }
  }

  public get isAlive(): boolean {
    return this._isAlive;
  }

  public get isPaused(): boolean {
    return this._isPaused;
  }

  public pause(): this {
    if (!this._isAlive) {
      throw new Error('Watch is not alive.');
    }
    this._isPaused = true;
    this.sourceSub.pause();
    this.pipeSub?.pause();
    return this;
  }

  public resume(initialNotify = false): this {
    if (!this._isAlive) {
      throw new Error('Watch is not alive.');
    }
    if (!this._isPaused) {
      return this;
    }
    this._isPaused = false;
    this.sourceSub.resume(true);
    this.pipeSub?.resume(initialNotify);
    return this;
  }

  public destroy(): void {
    if (!this._isAlive) {
      return;
    }
    this._isAlive = false;
    this._isPaused = true;
    this.sourceSub.destroy();
    this.pipeSub?.destroy();
    this.pipeSub = undefined;
  }

  public scoped(lifecycle: LifeView): this {
    lifecycle.register(this);
    return this;
  }
}

export class Watchables {
  public static readonly NUMERIC_NAN_EQUALITY = (a: number, b: number): boolean => a === b || (a !== a && b !== b);

  public static readonly NEVER_EQUALITY = (): false => false;

  public static isWatchable<T = any>(query: unknown): query is Watchable<T> {
    return typeof query === 'object' && query !== null && (query as any).isWatchable === true;
  }

  public static isWritable<T = any, I = T>(query: unknown): query is WritableWatchable<T, I> {
    return Watchables.isWatchable(query) && (query as any).isWritable === true;
  }

  public static isSubscribableSet<T = any>(query: unknown): query is WatchableSet<T> {
    return typeof query === 'object' && query !== null && (query as any).isSubscribableSet === true;
  }

  public static isMutableSubscribableSet<T = any>(query: unknown): query is WatchableSet<T> {
    return Watchables.isSubscribableSet(query) && (query as any).isMutableSubscribableSet === true;
  }

  public static asWatchable<V, Exclude extends boolean>(
    value: V,
    excludeSubscribables: Exclude,
  ): Exclude extends true ? (V extends Watchable<any> ? V : Watchable<V>) : Watchable<V> {
    if (excludeSubscribables && Watchables.isWatchable(value)) {
      return value as any;
    }
    return new ConstSubscribable(value) as any;
  }

  public static pipeMappedSource<S, T, U extends WritableWatchable<any, T>>(
    source: Watchable<S>,
    to: U,
    sourceMap: (source: S) => Watchable<T>,
    paused?: boolean,
  ): Watch;
  public static pipeMappedSource<S, M, T, U extends WritableWatchable<any, T>>(
    source: Watchable<S>,
    to: U,
    sourceMap: (source: S) => Watchable<M>,
    pipeMap: (fromVal: M, toVal: T) => T,
    paused?: boolean,
  ): Watch;
  public static pipeMappedSource(
    source: Watchable<any>,
    to: WritableWatchable<any, any>,
    sourceMap: (source: any) => Watchable<any>,
    arg4?: any,
    arg5?: boolean,
  ): Watch {
    const pipeMap = typeof arg4 === 'function' ? arg4 : undefined;
    const paused = (pipeMap ? arg5 : arg4) === true;
    return Watchables.relay(source, paused, (relay, v) => {
      const mapped = sourceMap(v);
      relay.setPipe(pipeMap ? mapped.pipe(to, pipeMap) : mapped.pipe(to));
    });
  }

  public static pipeOptionalMappedSource<S, T, U extends WritableWatchable<any, T>>(
    source: Watchable<S>,
    to: U,
    onOrphaned: ((to: U) => void) | undefined,
    sourceMap: (source: S) => Watchable<T> | undefined,
    paused?: boolean,
  ): Watch;
  public static pipeOptionalMappedSource<S, M, T, U extends WritableWatchable<any, T>>(
    source: Watchable<S>,
    to: U,
    onOrphaned: ((to: U) => void) | undefined,
    sourceMap: (source: S) => Watchable<M> | undefined,
    pipeMap: (fromVal: M, toVal: T) => T,
    paused?: boolean,
  ): Watch;
  public static pipeOptionalMappedSource(
    source: Watchable<any>,
    to: WritableWatchable<any, any>,
    onOrphaned: ((to: any) => void) | undefined,
    sourceMap: (source: any) => Watchable<any> | undefined,
    arg5?: any,
    arg6?: boolean,
  ): Watch {
    const pipeMap = typeof arg5 === 'function' ? arg5 : undefined;
    const paused = (pipeMap ? arg6 : arg5) === true;
    return Watchables.relay(source, paused, (relay, v) => {
      const mapped = sourceMap(v);
      if (mapped) {
        relay.setPipe(pipeMap ? mapped.pipe(to, pipeMap) : mapped.pipe(to));
      } else {
        relay.setPipe(undefined);
        onOrphaned?.(to);
      }
    });
  }

  private static relay(
    source: Watchable<any>,
    paused: boolean,
    onSource: (relay: RelaySubscription, v: any) => void,
  ): Watch {
    const relay = new RelaySubscription(paused);
    relay.attach(source.sub((v) => onSource(relay, v), !paused, paused));
    return relay;
  }
}

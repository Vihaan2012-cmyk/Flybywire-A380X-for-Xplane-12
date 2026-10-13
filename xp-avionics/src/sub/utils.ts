import { AbstractSubscribable } from './core';
import { MutableSubscribable, ReadonlyLifecycle, Subscribable, SubscribableSet, Subscription } from './types';

class ConstSubscribable<T> extends AbstractSubscribable<T> {
  constructor(private readonly value: T) {
    super();
  }

  public get(): T {
    return this.value;
  }
}

class RelaySubscription implements Subscription {
  public readonly canInitialNotify = true;
  private _isAlive = true;
  private _isPaused: boolean;
  private pipeSub?: Subscription;
  private sourceSub!: Subscription;

  constructor(paused: boolean) {
    this._isPaused = paused;
  }

  public attach(sourceSub: Subscription): void {
    this.sourceSub = sourceSub;
  }

  public setPipe(sub: Subscription | undefined): void {
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
      throw new Error('Subscription is not alive.');
    }
    this._isPaused = true;
    this.sourceSub.pause();
    this.pipeSub?.pause();
    return this;
  }

  public resume(initialNotify = false): this {
    if (!this._isAlive) {
      throw new Error('Subscription is not alive.');
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

  public withLifecycle(lifecycle: ReadonlyLifecycle): this {
    lifecycle.register(this);
    return this;
  }
}

export class SubscribableUtils {
  public static readonly NUMERIC_NAN_EQUALITY = (a: number, b: number): boolean => a === b || (a !== a && b !== b);

  public static readonly NEVER_EQUALITY = (): false => false;

  public static isSubscribable<T = any>(query: unknown): query is Subscribable<T> {
    return typeof query === 'object' && query !== null && (query as any).isSubscribable === true;
  }

  public static isMutableSubscribable<T = any, I = T>(query: unknown): query is MutableSubscribable<T, I> {
    return SubscribableUtils.isSubscribable(query) && (query as any).isMutableSubscribable === true;
  }

  public static isSubscribableSet<T = any>(query: unknown): query is SubscribableSet<T> {
    return typeof query === 'object' && query !== null && (query as any).isSubscribableSet === true;
  }

  public static isMutableSubscribableSet<T = any>(query: unknown): query is SubscribableSet<T> {
    return SubscribableUtils.isSubscribableSet(query) && (query as any).isMutableSubscribableSet === true;
  }

  public static toSubscribable<V, Exclude extends boolean>(
    value: V,
    excludeSubscribables: Exclude,
  ): Exclude extends true ? (V extends Subscribable<any> ? V : Subscribable<V>) : Subscribable<V> {
    if (excludeSubscribables && SubscribableUtils.isSubscribable(value)) {
      return value as any;
    }
    return new ConstSubscribable(value) as any;
  }

  public static pipeMappedSource<S, T, U extends MutableSubscribable<any, T>>(
    source: Subscribable<S>,
    to: U,
    sourceMap: (source: S) => Subscribable<T>,
    paused?: boolean,
  ): Subscription;
  public static pipeMappedSource<S, M, T, U extends MutableSubscribable<any, T>>(
    source: Subscribable<S>,
    to: U,
    sourceMap: (source: S) => Subscribable<M>,
    pipeMap: (fromVal: M, toVal: T) => T,
    paused?: boolean,
  ): Subscription;
  public static pipeMappedSource(
    source: Subscribable<any>,
    to: MutableSubscribable<any, any>,
    sourceMap: (source: any) => Subscribable<any>,
    arg4?: any,
    arg5?: boolean,
  ): Subscription {
    const pipeMap = typeof arg4 === 'function' ? arg4 : undefined;
    const paused = (pipeMap ? arg5 : arg4) === true;
    return SubscribableUtils.relay(source, paused, (relay, v) => {
      const mapped = sourceMap(v);
      relay.setPipe(pipeMap ? mapped.pipe(to, pipeMap) : mapped.pipe(to));
    });
  }

  public static pipeOptionalMappedSource<S, T, U extends MutableSubscribable<any, T>>(
    source: Subscribable<S>,
    to: U,
    onOrphaned: ((to: U) => void) | undefined,
    sourceMap: (source: S) => Subscribable<T> | undefined,
    paused?: boolean,
  ): Subscription;
  public static pipeOptionalMappedSource<S, M, T, U extends MutableSubscribable<any, T>>(
    source: Subscribable<S>,
    to: U,
    onOrphaned: ((to: U) => void) | undefined,
    sourceMap: (source: S) => Subscribable<M> | undefined,
    pipeMap: (fromVal: M, toVal: T) => T,
    paused?: boolean,
  ): Subscription;
  public static pipeOptionalMappedSource(
    source: Subscribable<any>,
    to: MutableSubscribable<any, any>,
    onOrphaned: ((to: any) => void) | undefined,
    sourceMap: (source: any) => Subscribable<any> | undefined,
    arg5?: any,
    arg6?: boolean,
  ): Subscription {
    const pipeMap = typeof arg5 === 'function' ? arg5 : undefined;
    const paused = (pipeMap ? arg6 : arg5) === true;
    return SubscribableUtils.relay(source, paused, (relay, v) => {
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
    source: Subscribable<any>,
    paused: boolean,
    onSource: (relay: RelaySubscription, v: any) => void,
  ): Subscription {
    const relay = new RelaySubscription(paused);
    relay.attach(source.sub((v) => onSource(relay, v), !paused, paused));
    return relay;
  }
}

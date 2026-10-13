export interface Readable<T> {
  get(): T;
}

export interface WritableReadable<T, I = T> extends Readable<T> {
  set(value: I): void;
}

export type ReadableType<A> = A extends Readable<infer T> ? T : never;
export type WritableReadableInput<A> = A extends WritableReadable<any, infer I> ? I : never;

export type LifeView = Pick<Life, 'register' | 'unregister'>;

export interface Life {
  register(sub: Watch): void;
  unregister(sub: Watch): void;
  pause(): void;
  resume(): void;
  destroy(): void;
}

export interface Watch {
  readonly isAlive: boolean;
  readonly isPaused: boolean;
  readonly canNotifyNow: boolean;
  pause(): this;
  resume(initialNotify?: boolean): this;
  destroy(): void;
  scoped(lifecycle: LifeView): this;
}

export interface Watchable<T> extends Readable<T> {
  readonly isWatchable: true;
  sub(handler: (value: T) => void, initialNotify?: boolean, paused?: boolean): Watch;
  map<M>(fn: (input: T, previousVal?: M) => M, equalityFunc?: (a: M, b: M) => boolean): DerivedWatchable<M>;
  map<M>(
    fn: (input: T, previousVal?: M) => M,
    equalityFunc: (a: M, b: M) => boolean,
    mutateFunc: (oldVal: M, newVal: M) => void,
    initialVal: M,
  ): DerivedWatchable<M>;
  pipe(to: WritableWatchable<any, T>, paused?: boolean): Watch;
  pipe<OI, OV = unknown>(to: WritableWatchable<OV, OI>, map: (fromVal: T, toVal: OV) => OI, paused?: boolean): Watch;
}

export interface DerivedWatchable<T> extends Watchable<T>, Watch {
  readonly isAlive: boolean;
  readonly isPaused: boolean;
  pause(): this;
  resume(): this;
  destroy(): void;
}

export interface WritableWatchable<T, I = T> extends Watchable<T>, WritableReadable<T, I> {
  readonly isWritable: true;
}

export type WatchedType<S> = S extends Watchable<infer T> ? T : never;
export type ToWatchable<S> = S extends Watchable<infer T> ? Watchable<T> : never;
export type MutableWatchableInputType<S> = S extends WritableWatchable<any, infer I> ? I : never;

export enum WatchableArraySignalType {
  Added = 'Added',
  Removed = 'Removed',
  Cleared = 'Cleared',
}

export type WatchableArrayCallback<T> = (
  index: number,
  type: WatchableArraySignalType,
  item: T | readonly T[] | undefined,
  array: readonly T[],
) => void;

export interface WatchableArray<T> {
  readonly length: number;
  get(index: number): T;
  tryGet(index: number): T | undefined;
  items(): readonly T[];
  sub(handler: WatchableArrayCallback<T>, initialNotify?: boolean, paused?: boolean): Watch;
}

export enum WatchableSetSignalType {
  Added = 'Added',
  Deleted = 'Deleted',
}

export type WatchableSetCallback<T> = (set: ReadonlySet<T>, type: WatchableSetSignalType, key: T) => void;

export interface WatchableSet<T> {
  readonly isSubscribableSet: true;
  readonly size: number;
  get(): ReadonlySet<T>;
  has(key: T): boolean;
  sub(handler: WatchableSetCallback<T>, initialNotify?: boolean, paused?: boolean): Watch;
  pipe(to: MutableWatchableSet<T>, paused?: boolean): Watch;
  pipe<OT>(to: MutableWatchableSet<OT>, map: (input: T) => OT, paused?: boolean): Watch;
}

export interface MutableWatchableSet<T> extends WatchableSet<T> {
  readonly isMutableSubscribableSet: true;
  add(key: T): this;
  delete(key: T): boolean;
  toggle(key: T): boolean;
  toggle(key: T, force: boolean): boolean;
  clear(): void;
}

export enum WatchableMapSignalType {
  Added = 'Added',
  Changed = 'Changed',
  Deleted = 'Deleted',
}

export type WatchableMapCallback<K, V> = (
  map: ReadonlyMap<K, V>,
  type: WatchableMapSignalType,
  key: K,
  value: V,
) => void;

export interface WatchableMap<K, V> {
  readonly isSubscribableMap: true;
  readonly size: number;
  get(): ReadonlyMap<K, V>;
  has(key: K): boolean;
  getValue(key: K): V | undefined;
  sub(handler: WatchableMapCallback<K, V>, initialNotify?: boolean, paused?: boolean): Watch;
  pipe(to: MutableWatchableMap<K, V>, paused?: boolean): Watch;
}

export interface MutableWatchableMap<K, V> extends WatchableMap<K, V> {
  readonly isMutableSubscribableMap: true;
  setValue(key: K, value: V): this;
  delete(key: K): boolean;
  clear(): void;
}

export type TupleLengthX<T extends readonly any[]> = { length: T['length'] };
export type MappedWatchableInputs<Types extends readonly any[]> = {
  [Index in keyof Types]: Watchable<Types[Index]>;
} & TupleLengthX<Types>;

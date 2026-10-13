export interface Accessible<T> {
  get(): T;
}

export interface MutableAccessible<T, I = T> extends Accessible<T> {
  set(value: I): void;
}

export type AccessibleType<A> = A extends Accessible<infer T> ? T : never;
export type MutableAccessibleInputType<A> = A extends MutableAccessible<any, infer I> ? I : never;

export type ReadonlyLifecycle = Pick<Lifecycle, 'register' | 'unregister'>;

export interface Lifecycle {
  register(sub: Subscription): void;
  unregister(sub: Subscription): void;
  pause(): void;
  resume(): void;
  destroy(): void;
}

export interface Subscription {
  readonly isAlive: boolean;
  readonly isPaused: boolean;
  readonly canInitialNotify: boolean;
  pause(): this;
  resume(initialNotify?: boolean): this;
  destroy(): void;
  withLifecycle(lifecycle: ReadonlyLifecycle): this;
}

export interface Subscribable<T> extends Accessible<T> {
  readonly isSubscribable: true;
  sub(handler: (value: T) => void, initialNotify?: boolean, paused?: boolean): Subscription;
  map<M>(fn: (input: T, previousVal?: M) => M, equalityFunc?: (a: M, b: M) => boolean): MappedSubscribable<M>;
  map<M>(
    fn: (input: T, previousVal?: M) => M,
    equalityFunc: (a: M, b: M) => boolean,
    mutateFunc: (oldVal: M, newVal: M) => void,
    initialVal: M,
  ): MappedSubscribable<M>;
  pipe(to: MutableSubscribable<any, T>, paused?: boolean): Subscription;
  pipe<OI, OV = unknown>(to: MutableSubscribable<OV, OI>, map: (fromVal: T, toVal: OV) => OI, paused?: boolean): Subscription;
}

export interface MappedSubscribable<T> extends Subscribable<T>, Subscription {
  readonly isAlive: boolean;
  readonly isPaused: boolean;
  pause(): this;
  resume(): this;
  destroy(): void;
}

export interface MutableSubscribable<T, I = T> extends Subscribable<T>, MutableAccessible<T, I> {
  readonly isMutableSubscribable: true;
}

export type SubscribableType<S> = S extends Subscribable<infer T> ? T : never;
export type ToSubscribable<S> = S extends Subscribable<infer T> ? Subscribable<T> : never;
export type MutableSubscribableInputType<S> = S extends MutableSubscribable<any, infer I> ? I : never;

export enum SubscribableArrayEventType {
  Added = 'Added',
  Removed = 'Removed',
  Cleared = 'Cleared',
}

export type SubscribableArrayHandler<T> = (
  index: number,
  type: SubscribableArrayEventType,
  item: T | readonly T[] | undefined,
  array: readonly T[],
) => void;

export interface SubscribableArray<T> {
  readonly length: number;
  get(index: number): T;
  tryGet(index: number): T | undefined;
  getArray(): readonly T[];
  sub(handler: SubscribableArrayHandler<T>, initialNotify?: boolean, paused?: boolean): Subscription;
}

export enum SubscribableSetEventType {
  Added = 'Added',
  Deleted = 'Deleted',
}

export type SubscribableSetHandler<T> = (set: ReadonlySet<T>, type: SubscribableSetEventType, key: T) => void;

export interface SubscribableSet<T> {
  readonly isSubscribableSet: true;
  readonly size: number;
  get(): ReadonlySet<T>;
  has(key: T): boolean;
  sub(handler: SubscribableSetHandler<T>, initialNotify?: boolean, paused?: boolean): Subscription;
  pipe(to: MutableSubscribableSet<T>, paused?: boolean): Subscription;
  pipe<OT>(to: MutableSubscribableSet<OT>, map: (input: T) => OT, paused?: boolean): Subscription;
}

export interface MutableSubscribableSet<T> extends SubscribableSet<T> {
  readonly isMutableSubscribableSet: true;
  add(key: T): this;
  delete(key: T): boolean;
  toggle(key: T): boolean;
  toggle(key: T, force: boolean): boolean;
  clear(): void;
}

export enum SubscribableMapEventType {
  Added = 'Added',
  Changed = 'Changed',
  Deleted = 'Deleted',
}

export type SubscribableMapHandler<K, V> = (
  map: ReadonlyMap<K, V>,
  type: SubscribableMapEventType,
  key: K,
  value: V,
) => void;

export interface SubscribableMap<K, V> {
  readonly isSubscribableMap: true;
  readonly size: number;
  get(): ReadonlyMap<K, V>;
  has(key: K): boolean;
  getValue(key: K): V | undefined;
  sub(handler: SubscribableMapHandler<K, V>, initialNotify?: boolean, paused?: boolean): Subscription;
  pipe(to: MutableSubscribableMap<K, V>, paused?: boolean): Subscription;
}

export interface MutableSubscribableMap<K, V> extends SubscribableMap<K, V> {
  readonly isMutableSubscribableMap: true;
  setValue(key: K, value: V): this;
  delete(key: K): boolean;
  clear(): void;
}

export type TupleLength<T extends readonly any[]> = { length: T['length'] };
export type MappedSubscribableInputs<Types extends readonly any[]> = {
  [Index in keyof Types]: Subscribable<Types[Index]>;
} & TupleLength<Types>;

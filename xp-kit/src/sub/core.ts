import {
  DerivedWatchable,
  MappedWatchableInputs,
  WritableWatchable,
  MutableWatchableMap,
  MutableWatchableSet,
  LifeView,
  Watchable,
  WatchableArray,
  WatchableArraySignalType,
  WatchableArrayCallback,
  WatchableMap,
  WatchableMapSignalType,
  WatchableMapCallback,
  WatchableSet,
  WatchableSetSignalType,
  WatchableSetCallback,
  Watch,
} from './types';

export class CallbackWatch<HandlerType extends (...args: any[]) => void> implements Watch {
  private _isAlive = true;
  private _isPaused = false;

  public get isAlive(): boolean {
    return this._isAlive;
  }

  public get isPaused(): boolean {
    return this._isPaused;
  }

  public readonly canNotifyNow: boolean;

  constructor(
    public readonly handler: HandlerType,
    private readonly initialNotifyFunc?: (sub: CallbackWatch<HandlerType>) => void,
    private readonly onDestroy?: (sub: CallbackWatch<HandlerType>) => void,
  ) {
    this.canNotifyNow = initialNotifyFunc !== undefined;
  }

  public initialNotify(): void {
    if (!this._isAlive) {
      throw new Error('CallbackWatch: cannot notify a dead Watch.');
    }
    this.initialNotifyFunc?.(this);
  }

  public pause(): this {
    if (!this._isAlive) {
      throw new Error('CallbackWatch: cannot pause a dead Watch.');
    }
    this._isPaused = true;
    return this;
  }

  public resume(initialNotify = false): this {
    if (!this._isAlive) {
      throw new Error('CallbackWatch: cannot resume a dead Watch.');
    }
    if (!this._isPaused) {
      return this;
    }
    this._isPaused = false;
    if (initialNotify) {
      this.initialNotify();
    }
    return this;
  }

  public destroy(): void {
    if (!this._isAlive) {
      return;
    }
    this._isAlive = false;
    this._isPaused = true;
    this.onDestroy?.(this);
  }

  public scoped(lifecycle: LifeView): this {
    lifecycle.register(this);
    return this;
  }
}

function safeCall(fn: () => void): void {
  try {
    fn();
  } catch (e) {
    console.error(`Watchable handler error: ${e}`, e instanceof Error ? e.stack : undefined);
  }
}

class SubList<H extends (...args: any[]) => void> {
  public single?: CallbackWatch<H>;
  public list?: CallbackWatch<H>[];
  public depth = 0;
  private dirty = false;

  public add(sub: CallbackWatch<H>): void {
    if (this.list) {
      this.list.push(sub);
    } else if (this.single) {
      this.list = [this.single, sub];
      this.single = undefined;
    } else {
      this.single = sub;
    }
  }

  public remove(sub: CallbackWatch<H>): void {
    if (this.single === sub) {
      this.single = undefined;
    } else if (this.list) {
      if (this.depth === 0) {
        const i = this.list.indexOf(sub);
        if (i >= 0) {
          this.list.splice(i, 1);
        }
      } else {
        this.dirty = true;
      }
    }
  }

  public each(fn: (sub: CallbackWatch<H>) => void): void {
    const single = this.single;
    if (single) {
      if (single.isAlive && !single.isPaused) {
        safeCall(() => fn(single));
      }
      return;
    }
    const list = this.list;
    if (!list) {
      return;
    }
    this.depth++;
    const n = list.length;
    for (let i = 0; i < n; i++) {
      const s = list[i];
      if (s.isAlive && !s.isPaused) {
        safeCall(() => fn(s));
      }
    }
    this.depth--;
    if (this.depth === 0 && this.dirty) {
      this.dirty = false;
      this.list = list.filter((s) => s.isAlive);
    }
  }

  public destroyAll(): void {
    const all = this.single ? [this.single] : this.list ? [...this.list] : [];
    for (const s of all) {
      s.destroy();
    }
  }
}

export abstract class AbstractWatchable<T> implements Watchable<T> {
  public readonly isWatchable = true;

  public static readonly DEFAULT_EQUALITY_FUNC = (a: any, b: any): boolean => a === b;

  protected readonly subList = new SubList<(v: T) => void>();

  protected get notifyDepth(): number {
    return this.subList.depth;
  }

  protected readonly initialNotifyFunc = (sub: CallbackWatch<(v: T) => void>): void => {
    safeCall(() => this.notifySubscription(sub));
  };

  protected readonly onSubDestroyedFunc = (sub: CallbackWatch<(v: T) => void>): void => {
    this.onSubDestroyed(sub);
  };

  protected addSubscription(sub: CallbackWatch<(v: T) => void>): void {
    this.subList.add(sub);
  }

  public abstract get(): T;

  public sub(handler: (v: T) => void, initialNotify = false, paused = false): Watch {
    const sub = new CallbackWatch(handler, this.initialNotifyFunc, this.onSubDestroyedFunc);
    this.addSubscription(sub);
    if (paused) {
      sub.pause();
    } else if (initialNotify) {
      sub.initialNotify();
    }
    return sub;
  }

  protected notify(): void {
    this.subList.each((sub) => this.notifySubscription(sub));
  }

  protected notifySubscription(sub: CallbackWatch<(v: T) => void>): void {
    sub.handler(this.get());
  }

  protected onSubDestroyed(sub: CallbackWatch<(v: T) => void>): void {
    this.subList.remove(sub);
  }

  public map<M>(fn: (input: T, previousVal?: M) => M, equalityFunc?: (a: M, b: M) => boolean): DerivedWatchable<M>;
  public map<M>(
    fn: (input: T, previousVal?: M) => M,
    equalityFunc: (a: M, b: M) => boolean,
    mutateFunc: (oldVal: M, newVal: M) => void,
    initialVal: M,
  ): DerivedWatchable<M>;
  public map<M>(
    fn: (input: T, previousVal?: M) => M,
    equalityFunc?: (a: M, b: M) => boolean,
    mutateFunc?: (oldVal: M, newVal: M) => void,
    initialVal?: M,
  ): DerivedWatchable<M> {
    const mapFn = (inputs: readonly [T], prev?: M): M => fn(inputs[0], prev);
    const eq = equalityFunc ?? AbstractWatchable.DEFAULT_EQUALITY_FUNC;
    if (mutateFunc) {
      return DerivedCell.fromMap(mapFn, eq, mutateFunc, initialVal as M, this as Watchable<T>);
    }
    return DerivedCell.fromMap(mapFn, eq, undefined, undefined, this as Watchable<T>);
  }

  public pipe(to: WritableWatchable<any, T>, paused?: boolean): Watch;
  public pipe<OI, OV = unknown>(
    to: WritableWatchable<OV, OI>,
    map: (fromVal: T, toVal: OV) => OI,
    paused?: boolean,
  ): Watch;
  public pipe(to: WritableWatchable<any, any>, arg2?: any, arg3?: boolean): Watch {
    let map: ((f: T, t: any) => any) | undefined;
    let paused = false;
    if (typeof arg2 === 'function') {
      map = arg2;
      paused = arg3 === true;
    } else {
      paused = arg2 === true;
    }
    const handler = map ? (v: T) => to.set(map!(v, to.get())) : (v: T) => to.set(v);
    const sub = new CallbackWatch(handler, this.initialNotifyFunc, this.onSubDestroyedFunc);
    this.addSubscription(sub);
    if (paused) {
      sub.pause();
    } else {
      sub.initialNotify();
    }
    return sub;
  }
}

export class DerivedCell<I extends any[], T> extends AbstractWatchable<T> implements DerivedWatchable<T> {
  private static readonly IDENTITY_MAP = (inputs: any): any => inputs;
  private static readonly NEVER_EQUALS = (): boolean => false;

  public readonly canNotifyNow = true;

  private readonly inputValues: any[];
  private readonly inputSubs: Watch[] = [];
  private value: T;
  private _isAlive = true;
  private _isPaused = false;

  public get isAlive(): boolean {
    return this._isAlive;
  }

  public get isPaused(): boolean {
    return this._isPaused;
  }

  private constructor(
    private readonly mapFunc: (inputs: Readonly<I>, previousVal?: T) => T,
    private readonly equalityFunc: (a: T, b: T) => boolean,
    private readonly mutateFunc: ((oldVal: T, newVal: T) => void) | undefined,
    initialVal: T | undefined,
    private readonly inputs: readonly Watchable<any>[],
    notifyInputs = false,
  ) {
    super();
    this.inputValues = inputs.map((x) => x.get());
    if (mutateFunc) {
      this.value = initialVal as T;
      mutateFunc(this.value, mapFunc(this.inputValues as unknown as Readonly<I>, this.value));
    } else {
      this.value = mapFunc(this.inputValues as unknown as Readonly<I>, undefined);
    }
    for (let i = 0; i < inputs.length; i++) {
      this.inputSubs.push(inputs[i].sub(this.makeInputHandler(i), notifyInputs));
    }
  }

  private makeInputHandler(index: number): (v: any) => void {
    return (v) => {
      this.inputValues[index] = v;
      this.updateValue();
    };
  }

  public static create<I extends any[]>(...inputs: MappedWatchableInputs<I>): DerivedCell<I, Readonly<I>>;
  public static create<I extends any[], T>(
    mapFunc: (inputs: Readonly<I>, previousVal?: T) => T,
    ...inputs: MappedWatchableInputs<I>
  ): DerivedCell<I, T>;
  public static create<I extends any[], T>(
    mapFunc: (inputs: Readonly<I>, previousVal?: T) => T,
    equalityFunc: (a: T, b: T) => boolean,
    ...inputs: MappedWatchableInputs<I>
  ): DerivedCell<I, T>;
  public static create<I extends any[], T>(
    mapFunc: (inputs: Readonly<I>, previousVal?: T) => T,
    equalityFunc: (a: T, b: T) => boolean,
    mutateFunc: (oldVal: T, newVal: T) => void,
    initialVal: T,
    ...inputs: MappedWatchableInputs<I>
  ): DerivedCell<I, T>;
  public static create(...args: any[]): DerivedCell<any, any> {
    let mapFunc: any = DerivedCell.IDENTITY_MAP;
    let eq: any = DerivedCell.NEVER_EQUALS;
    let mutate: any;
    let initial: any;
    let i = 0;
    if (typeof args[0] === 'function') {
      mapFunc = args[0];
      eq = AbstractWatchable.DEFAULT_EQUALITY_FUNC;
      i = 1;
      if (typeof args[1] === 'function') {
        eq = args[1];
        i = 2;
        if (typeof args[2] === 'function') {
          mutate = args[2];
          initial = args[3];
          i = 4;
        }
      }
    }
    return new DerivedCell(mapFunc, eq, mutate, initial, args.slice(i));
  }

  public static fromMap<I, T>(
    mapFunc: (inputs: Readonly<[I]>, previousVal?: T) => T,
    equalityFunc: (a: T, b: T) => boolean,
    mutateFunc: ((oldVal: T, newVal: T) => void) | undefined,
    initialVal: T | undefined,
    input: Watchable<I>,
  ): DerivedCell<[I], T> {
    return new DerivedCell<[I], T>(mapFunc, equalityFunc, mutateFunc, initialVal, [input], true);
  }

  private updateValue(): void {
    const v = this.mapFunc(this.inputValues as unknown as Readonly<I>, this.value);
    if (!this.equalityFunc(this.value, v)) {
      if (this.mutateFunc) {
        this.mutateFunc(this.value, v);
      } else {
        this.value = v;
      }
      this.notify();
    }
  }

  public get(): T {
    return this.value;
  }

  public pause(): this {
    if (!this._isAlive) {
      throw new Error('DerivedCell: cannot pause a dead subject.');
    }
    if (this._isPaused) {
      return this;
    }
    this._isPaused = true;
    for (const s of this.inputSubs) {
      s.pause();
    }
    return this;
  }

  public resume(): this {
    if (!this._isAlive) {
      throw new Error('DerivedCell: cannot resume a dead subject.');
    }
    if (!this._isPaused) {
      return this;
    }
    this._isPaused = false;
    for (let i = 0; i < this.inputSubs.length; i++) {
      this.inputSubs[i].resume();
      this.inputValues[i] = this.inputs[i].get();
    }
    this.updateValue();
    return this;
  }

  public destroy(): void {
    if (!this._isAlive) {
      return;
    }
    this._isAlive = false;
    this._isPaused = true;
    for (const s of this.inputSubs) {
      s.destroy();
    }
  }

  public scoped(lifecycle: LifeView): this {
    lifecycle.register(this);
    return this;
  }
}

export class Cell<T> extends AbstractWatchable<T> implements WritableWatchable<T> {
  public readonly isWritable = true;

  protected constructor(
    protected value: T,
    protected readonly equalityFunc: (a: T, b: T) => boolean,
    protected readonly mutateFunc?: (oldVal: T, newVal: T) => void,
  ) {
    super();
  }

  public static create<IT>(
    v: IT,
    equalityFunc?: (a: IT, b: IT) => boolean,
    mutateFunc?: (oldVal: IT, newVal: IT) => void,
  ): Cell<IT> {
    return new Cell(v, equalityFunc ?? AbstractWatchable.DEFAULT_EQUALITY_FUNC, mutateFunc);
  }

  protected notifySub(sub: (v: T) => void): void {
    sub(this.value);
  }

  public set(value: T): void {
    if (!this.equalityFunc(value, this.value)) {
      if (this.mutateFunc) {
        this.mutateFunc(this.value, value);
      } else {
        this.value = value;
      }
      this.notify();
    }
  }

  public apply(value: Partial<T>): void {
    let changed = false;
    for (const key in value) {
      if (value[key] !== this.value[key]) {
        changed = true;
        break;
      }
    }
    if (changed) {
      Object.assign(this.value as any, value);
      this.notify();
    }
  }

  public notify(): void {
    super.notify();
  }

  public get(): T {
    return this.value;
  }
}

export type ObjectCellCallback<T extends Record<string, any>> = (
  v: Readonly<T>,
  key: keyof T,
  newValue: T[keyof T],
  oldValue: T[keyof T],
) => void;

export class RecordCell<T extends Record<string, any>>
  implements WritableWatchable<Readonly<T>, Partial<Readonly<T>>>
{
  public readonly isWatchable = true;
  public readonly isWritable = true;

  private readonly subList = new SubList<ObjectCellCallback<T>>();

  private readonly initialNotifyFunc = (sub: CallbackWatch<ObjectCellCallback<T>>): void => {
    safeCall(() => {
      for (const key of Object.keys(this.obj)) {
        sub.handler(this.obj, key as keyof T, this.obj[key as keyof T], this.obj[key as keyof T]);
      }
    });
  };

  private readonly onSubDestroyedFunc = (sub: CallbackWatch<ObjectCellCallback<T>>): void => {
    this.subList.remove(sub);
  };

  private constructor(private readonly obj: T) {}

  public static create<T extends Record<string, any>>(v: T): RecordCell<T> {
    return new RecordCell<T>(v);
  }

  public get(): Readonly<T> {
    return this.obj;
  }

  public sub(handler: ObjectCellCallback<T>, initialNotify = false, paused = false): Watch {
    const sub = new CallbackWatch(handler, this.initialNotifyFunc, this.onSubDestroyedFunc);
    this.subList.add(sub);
    if (paused) {
      sub.pause();
    } else if (initialNotify) {
      sub.initialNotify();
    }
    return sub;
  }

  public set(value: Partial<Readonly<T>>): void;
  public set<K extends keyof T>(key: K, value: T[K]): void;
  public set(arg1: any, arg2?: any): void {
    if (typeof arg1 === 'object' && arg1 !== null) {
      for (const key of Object.keys(arg1)) {
        this.setKey(key as keyof T, arg1[key]);
      }
    } else {
      this.setKey(arg1, arg2);
    }
  }

  private setKey<K extends keyof T>(key: K, value: T[K]): void {
    const old = this.obj[key];
    if (old !== value) {
      (this.obj as any)[key] = value;
      this.subList.each((sub) => sub.handler(this.obj, key, value, old));
    }
  }

  public map<M>(fn: (input: T, previousVal?: M) => M, equalityFunc?: (a: M, b: M) => boolean): DerivedCell<[T], M>;
  public map<M>(
    fn: (input: Readonly<T>, previousVal?: M) => M,
    equalityFunc: (a: M, b: M) => boolean,
    mutateFunc: (oldVal: M, newVal: M) => void,
    initialVal: M,
  ): DerivedCell<[Readonly<T>], M>;
  public map<M>(
    fn: (input: T, previousVal?: M) => M,
    equalityFunc?: (a: M, b: M) => boolean,
    mutateFunc?: (oldVal: M, newVal: M) => void,
    initialVal?: M,
  ): DerivedCell<[T], M> {
    const mapFn = (inputs: readonly [T], prev?: M): M => fn(inputs[0], prev);
    const eq = equalityFunc ?? AbstractWatchable.DEFAULT_EQUALITY_FUNC;
    const self = this as unknown as Watchable<T>;
    if (mutateFunc) {
      return DerivedCell.fromMap(mapFn, eq, mutateFunc, initialVal as M, self);
    }
    return DerivedCell.fromMap(mapFn, eq, undefined, undefined, self);
  }

  public pipe(to: WritableWatchable<any, T>, paused?: boolean): Watch;
  public pipe<OI, OV = unknown>(
    to: WritableWatchable<OV, OI>,
    map: (fromVal: T, toVal: OV) => OI,
    paused?: boolean,
  ): Watch;
  public pipe(to: WritableWatchable<any, any>, arg2?: any, arg3?: boolean): Watch {
    let map: ((f: T, t: any) => any) | undefined;
    let paused: boolean;
    if (typeof arg2 === 'function') {
      map = arg2;
      paused = arg3 === true;
    } else {
      paused = arg2 === true;
    }
    const handler: any = map ? (v: T) => to.set(map!(v, to.get())) : (v: T) => to.set(v);
    const sub = new CallbackWatch<ObjectCellCallback<T>>(
      handler,
      (s) => safeCall(() => s.handler(this.obj, '' as keyof T, undefined as any, undefined as any)),
      this.onSubDestroyedFunc,
    );
    this.subList.add(sub);
    if (paused) {
      sub.pause();
    } else {
      sub.initialNotify();
    }
    return sub;
  }
}

export class Notice<SenderType, DataType> {
  private readonly subList = new SubList<(sender: SenderType, data: DataType) => void>();

  private readonly onSubDestroyedFunc = (sub: CallbackWatch<(sender: SenderType, data: DataType) => void>) => {
    this.subList.remove(sub);
  };

  public on(handler: (sender: SenderType, data: DataType) => void, paused = false): Watch {
    const sub = new CallbackWatch(handler, undefined, this.onSubDestroyedFunc);
    this.subList.add(sub);
    if (paused) {
      sub.pause();
    }
    return sub;
  }

  public clear(): void {
    this.subList.destroyAll();
  }

  public notify(sender: SenderType, data: DataType): void {
    this.subList.each((sub) => sub.handler(sender, data));
  }
}

export abstract class AbstractWatchableArray<T> implements WatchableArray<T> {
  public abstract readonly length: number;

  protected readonly subList = new SubList<WatchableArrayCallback<T>>();

  protected readonly initialNotifyFunc = (sub: CallbackWatch<WatchableArrayCallback<T>>): void => {
    this.initialNotify(sub);
  };

  protected readonly onSubDestroyedFunc = (sub: CallbackWatch<WatchableArrayCallback<T>>): void => {
    this.onSubDestroyed(sub);
  };

  protected addSubscription(sub: CallbackWatch<WatchableArrayCallback<T>>): void {
    this.subList.add(sub);
  }

  public sub(handler: WatchableArrayCallback<T>, initialNotify = false, paused = false): Watch {
    const sub = new CallbackWatch(handler, this.initialNotifyFunc, this.onSubDestroyedFunc);
    this.addSubscription(sub);
    if (paused) {
      sub.pause();
    } else if (initialNotify) {
      sub.initialNotify();
    }
    return sub;
  }

  public abstract items(): readonly T[];

  public get(index: number): T {
    const arr = this.items();
    if (index < 0 || index >= arr.length) {
      throw new Error('Index out of range');
    }
    return arr[index];
  }

  public tryGet(index: number): T | undefined {
    return this.items()[index];
  }

  protected notify(index: number, type: WatchableArraySignalType, modifiedItem?: T | readonly T[]): void {
    const arr = this.items();
    this.subList.each((sub) => sub.handler(index, type, modifiedItem, arr));
  }

  protected initialNotify(sub: CallbackWatch<WatchableArrayCallback<T>>): void {
    const arr = this.items();
    safeCall(() => sub.handler(0, WatchableArraySignalType.Added, arr, arr));
  }

  protected onSubDestroyed(sub: CallbackWatch<WatchableArrayCallback<T>>): void {
    this.subList.remove(sub);
  }
}

export class ListCell<T> extends AbstractWatchableArray<T> {
  private array: T[];

  public get length(): number {
    return this.array.length;
  }

  private constructor(arr: T[]) {
    super();
    this.array = arr;
  }

  public static create<AT>(arr: AT[] = []): ListCell<AT> {
    return new ListCell<AT>(arr);
  }

  public insert(item: T, index?: number): void {
    if (index === undefined || index >= this.array.length) {
      this.array.push(item);
      this.notify(this.array.length - 1, WatchableArraySignalType.Added, item);
    } else {
      this.array.splice(index, 0, item);
      this.notify(index, WatchableArraySignalType.Added, item);
    }
  }

  public insertRange(index: number | undefined, arr: readonly T[]): void {
    if (index === undefined || index > this.array.length) {
      index = this.array.length;
    }
    this.array.splice(index, 0, ...arr);
    this.notify(index, WatchableArraySignalType.Added, arr);
  }

  public removeAt(index: number): void {
    if (index < 0 || index >= this.array.length) {
      return;
    }
    const [item] = this.array.splice(index, 1);
    this.notify(index, WatchableArraySignalType.Removed, item);
  }

  public removeItem(item: T): boolean {
    const index = this.array.indexOf(item);
    if (index < 0) {
      return false;
    }
    this.removeAt(index);
    return true;
  }

  public set(arr: readonly T[]): void {
    this.clear();
    this.insertRange(0, arr);
  }

  public clear(): void {
    this.array.length = 0;
    this.notify(0, WatchableArraySignalType.Cleared);
  }

  public items(): readonly T[] {
    return this.array;
  }
}

export abstract class AbstractWatchableMap<K, V> implements WatchableMap<K, V>, Watchable<ReadonlyMap<K, V>> {
  public readonly isWatchable = true;
  public readonly isSubscribableMap = true;

  public get size(): number {
    return this.get().size;
  }

  protected readonly subList = new SubList<WatchableMapCallback<K, V>>();

  protected readonly initialNotifyFunc = (sub: CallbackWatch<WatchableMapCallback<K, V>>): void => {
    this.initialNotify(sub);
  };

  protected readonly onSubDestroyedFunc = (sub: CallbackWatch<WatchableMapCallback<K, V>>): void => {
    this.onSubDestroyed(sub);
  };

  protected addSubscription(sub: CallbackWatch<WatchableMapCallback<K, V>>): void {
    this.subList.add(sub);
  }

  public abstract get(): ReadonlyMap<K, V>;

  public has(key: K): boolean {
    return this.get().has(key);
  }

  public getValue(key: K): V | undefined {
    return this.get().get(key);
  }

  public sub(handler: WatchableMapCallback<K, V>, initialNotify = false, paused = false): Watch {
    const sub = new CallbackWatch(handler, this.initialNotifyFunc, this.onSubDestroyedFunc);
    this.addSubscription(sub);
    if (paused) {
      sub.pause();
    } else if (initialNotify) {
      sub.initialNotify();
    }
    return sub;
  }

  protected notify(type: WatchableMapSignalType, key: K, value: V): void {
    const map = this.get();
    this.subList.each((sub) => sub.handler(map, type, key, value));
  }

  protected initialNotify(sub: CallbackWatch<WatchableMapCallback<K, V>>): void {
    const map = this.get();
    for (const [k, v] of Array.from(map)) {
      safeCall(() => sub.handler(map, WatchableMapSignalType.Added, k, v));
    }
  }

  protected onSubDestroyed(sub: CallbackWatch<WatchableMapCallback<K, V>>): void {
    this.subList.remove(sub);
  }

  public map<M>(
    fn: (input: ReadonlyMap<K, V>, previousVal?: M) => M,
    equalityFunc?: (a: M, b: M) => boolean,
  ): DerivedCell<[ReadonlyMap<K, V>], M>;
  public map<M>(
    fn: (input: ReadonlyMap<K, V>, previousVal?: M) => M,
    equalityFunc: (a: M, b: M) => boolean,
    mutateFunc: (oldVal: M, newVal: M) => void,
    initialVal: M,
  ): DerivedCell<[ReadonlyMap<K, V>], M>;
  public map<M>(
    fn: (input: ReadonlyMap<K, V>, previousVal?: M) => M,
    equalityFunc?: (a: M, b: M) => boolean,
    mutateFunc?: (oldVal: M, newVal: M) => void,
    initialVal?: M,
  ): DerivedCell<[ReadonlyMap<K, V>], M> {
    const mapFn = (inputs: readonly [ReadonlyMap<K, V>], prev?: M): M => fn(inputs[0], prev);
    const eq = equalityFunc ?? AbstractWatchable.DEFAULT_EQUALITY_FUNC;
    const self = this as unknown as Watchable<ReadonlyMap<K, V>>;
    if (mutateFunc) {
      return DerivedCell.fromMap(mapFn, eq, mutateFunc, initialVal as M, self);
    }
    return DerivedCell.fromMap(mapFn, eq, undefined, undefined, self);
  }

  public pipe(to: MutableWatchableMap<K, V>, paused?: boolean): Watch;
  public pipe(to: WritableWatchable<any, ReadonlyMap<K, V>>, paused?: boolean): Watch;
  public pipe<OI, OV = unknown>(
    to: WritableWatchable<any, OI>,
    map: (fromVal: ReadonlyMap<K, OV>, toVal: OV) => OI,
    paused?: boolean,
  ): Watch;
  public pipe(to: any, arg2?: any, arg3?: boolean): Watch {
    let handler: any;
    let initial: (sub: CallbackWatch<WatchableMapCallback<K, V>>) => void = this.initialNotifyFunc;
    let paused: boolean;
    const whole = (s: CallbackWatch<WatchableMapCallback<K, V>>): void =>
      safeCall(() => s.handler(this.get(), WatchableMapSignalType.Added, undefined as any, undefined as any));
    if (typeof arg2 === 'function') {
      paused = arg3 === true;
      handler = (m: ReadonlyMap<K, V>) => to.set(arg2(m, to.get()));
      initial = whole;
    } else if (to.isMutableSubscribableMap) {
      paused = arg2 === true;
      handler = (_m: ReadonlyMap<K, V>, type: WatchableMapSignalType, key: K, value: V) => {
        if (type === WatchableMapSignalType.Deleted) {
          to.delete(key);
        } else {
          to.setValue(key, value);
        }
      };
    } else {
      paused = arg2 === true;
      handler = (m: ReadonlyMap<K, V>) => to.set(m);
      initial = whole;
    }
    const sub = new CallbackWatch<WatchableMapCallback<K, V>>(handler, initial, this.onSubDestroyedFunc);
    this.addSubscription(sub);
    if (paused) {
      sub.pause();
    } else {
      sub.initialNotify();
    }
    return sub;
  }
}

export class DictCell<K, V>
  extends AbstractWatchableMap<K, V>
  implements WritableWatchable<ReadonlyMap<K, V>, Iterable<readonly [K, V]>>, MutableWatchableMap<K, V>
{
  public readonly isWritable = true;
  public readonly isMutableSubscribableMap = true;

  private readonly backingMap: Map<K, V>;

  private constructor(initialEntries?: Iterable<[K, V]>) {
    super();
    this.backingMap = new Map(initialEntries);
  }

  public static create<K, V>(initialEntries?: Iterable<[K, V]>): DictCell<K, V> {
    return new DictCell<K, V>(initialEntries);
  }

  public get(): ReadonlyMap<K, V> {
    return this.backingMap;
  }

  public set(entries: Iterable<readonly [K, V]>): void {
    const incoming = new Map<K, V>(entries as Iterable<[K, V]>);
    for (const key of Array.from(this.backingMap.keys())) {
      if (!incoming.has(key)) {
        this.delete(key);
      }
    }
    for (const [k, v] of incoming) {
      this.setValue(k, v);
    }
  }

  public setValue(key: K, value: V): this {
    const has = this.backingMap.has(key);
    const old = this.backingMap.get(key);
    if (has && old === value) {
      return this;
    }
    this.backingMap.set(key, value);
    this.notify(has ? WatchableMapSignalType.Changed : WatchableMapSignalType.Added, key, value);
    return this;
  }

  public delete(key: K): boolean {
    if (!this.backingMap.has(key)) {
      return false;
    }
    const value = this.backingMap.get(key) as V;
    this.backingMap.delete(key);
    this.notify(WatchableMapSignalType.Deleted, key, value);
    return true;
  }

  public clear(): void {
    for (const key of Array.from(this.backingMap.keys())) {
      this.delete(key);
    }
  }
}

export abstract class AbstractWatchableSet<T> implements WatchableSet<T>, Watchable<ReadonlySet<T>> {
  public readonly isWatchable = true;
  public readonly isSubscribableSet = true;

  public get size(): number {
    return this.get().size;
  }

  protected readonly subList = new SubList<WatchableSetCallback<T>>();

  protected readonly initialNotifyFunc = (sub: CallbackWatch<WatchableSetCallback<T>>): void => {
    this.initialNotify(sub);
  };

  protected readonly onSubDestroyedFunc = (sub: CallbackWatch<WatchableSetCallback<T>>): void => {
    this.onSubDestroyed(sub);
  };

  protected addSubscription(sub: CallbackWatch<WatchableSetCallback<T>>): void {
    this.subList.add(sub);
  }

  public abstract get(): ReadonlySet<T>;

  public has(key: T): boolean {
    return this.get().has(key);
  }

  public sub(handler: WatchableSetCallback<T>, initialNotify = false, paused = false): Watch {
    const sub = new CallbackWatch(handler, this.initialNotifyFunc, this.onSubDestroyedFunc);
    this.addSubscription(sub);
    if (paused) {
      sub.pause();
    } else if (initialNotify) {
      sub.initialNotify();
    }
    return sub;
  }

  protected notify(type: WatchableSetSignalType, key: T): void {
    const set = this.get();
    this.subList.each((sub) => sub.handler(set, type, key));
  }

  protected initialNotify(sub: CallbackWatch<WatchableSetCallback<T>>): void {
    const set = this.get();
    for (const k of Array.from(set)) {
      safeCall(() => sub.handler(set, WatchableSetSignalType.Added, k));
    }
  }

  protected onSubDestroyed(sub: CallbackWatch<WatchableSetCallback<T>>): void {
    this.subList.remove(sub);
  }

  public map<M>(
    fn: (input: ReadonlySet<T>, previousVal?: M) => M,
    equalityFunc?: (a: M, b: M) => boolean,
  ): DerivedCell<[ReadonlySet<T>], M>;
  public map<M>(
    fn: (input: ReadonlySet<T>, previousVal?: M) => M,
    equalityFunc: (a: M, b: M) => boolean,
    mutateFunc: (oldVal: M, newVal: M) => void,
    initialVal: M,
  ): DerivedCell<[ReadonlySet<T>], M>;
  public map<M>(
    fn: (input: ReadonlySet<T>, previousVal?: M) => M,
    equalityFunc?: (a: M, b: M) => boolean,
    mutateFunc?: (oldVal: M, newVal: M) => void,
    initialVal?: M,
  ): DerivedCell<[ReadonlySet<T>], M> {
    const mapFn = (inputs: readonly [ReadonlySet<T>], prev?: M): M => fn(inputs[0], prev);
    const eq = equalityFunc ?? AbstractWatchable.DEFAULT_EQUALITY_FUNC;
    const self = this as unknown as Watchable<ReadonlySet<T>>;
    if (mutateFunc) {
      return DerivedCell.fromMap(mapFn, eq, mutateFunc, initialVal as M, self);
    }
    return DerivedCell.fromMap(mapFn, eq, undefined, undefined, self);
  }

  public pipe(to: MutableWatchableSet<T>, paused?: boolean): Watch;
  public pipe<OT>(to: MutableWatchableSet<OT>, map: (fromKey: T) => OT, paused?: boolean): Watch;
  public pipe(to: WritableWatchable<any, ReadonlySet<T>>, paused?: boolean): Watch;
  public pipe<OI, OV = unknown>(
    to: WritableWatchable<any, OI>,
    map: (fromVal: ReadonlySet<T>, toVal: OV) => OI,
    paused?: boolean,
  ): Watch;
  public pipe(to: any, arg2?: any, arg3?: boolean): Watch {
    let handler: any;
    let initial: (sub: CallbackWatch<WatchableSetCallback<T>>) => void = this.initialNotifyFunc;
    let paused: boolean;
    const whole = (s: CallbackWatch<WatchableSetCallback<T>>): void =>
      safeCall(() => s.handler(this.get(), WatchableSetSignalType.Added, undefined as any));
    if (to.isMutableSubscribableSet) {
      const map: ((k: T) => any) | undefined = typeof arg2 === 'function' ? arg2 : undefined;
      paused = (map ? arg3 : arg2) === true;
      handler = (set: ReadonlySet<T>, type: WatchableSetSignalType, key: T) => {
        const mapped = map ? map(key) : key;
        if (type === WatchableSetSignalType.Added) {
          to.add(mapped);
        } else if (!map) {
          to.delete(mapped);
        } else {
          for (const k of set) {
            if (map(k) === mapped) {
              return;
            }
          }
          to.delete(mapped);
        }
      };
    } else if (typeof arg2 === 'function') {
      paused = arg3 === true;
      handler = (s: ReadonlySet<T>) => to.set(arg2(s, to.get()));
      initial = whole;
    } else {
      paused = arg2 === true;
      handler = (s: ReadonlySet<T>) => to.set(s);
      initial = whole;
    }
    const sub = new CallbackWatch<WatchableSetCallback<T>>(handler, initial, this.onSubDestroyedFunc);
    this.addSubscription(sub);
    if (paused) {
      sub.pause();
    } else {
      sub.initialNotify();
    }
    return sub;
  }
}

export class BagCell<T>
  extends AbstractWatchableSet<T>
  implements WritableWatchable<ReadonlySet<T>, Iterable<T>>, MutableWatchableSet<T>
{
  public readonly isWritable = true;
  public readonly isMutableSubscribableSet = true;

  private readonly backingSet: Set<T>;

  private constructor(initialKeys?: Iterable<T>) {
    super();
    this.backingSet = new Set(initialKeys);
  }

  public static create<T>(initialKeys?: Iterable<T>): BagCell<T> {
    return new BagCell<T>(initialKeys);
  }

  public get(): ReadonlySet<T> {
    return this.backingSet;
  }

  public set(keys: Iterable<T>): void {
    const incoming = new Set(keys);
    for (const k of Array.from(this.backingSet)) {
      if (!incoming.has(k)) {
        this.delete(k);
      }
    }
    for (const k of incoming) {
      this.add(k);
    }
  }

  public add(key: T): this {
    if (!this.backingSet.has(key)) {
      this.backingSet.add(key);
      this.notify(WatchableSetSignalType.Added, key);
    }
    return this;
  }

  public delete(key: T): boolean {
    if (!this.backingSet.delete(key)) {
      return false;
    }
    this.notify(WatchableSetSignalType.Deleted, key);
    return true;
  }

  public toggle(key: T): boolean;
  public toggle(key: T, force: boolean): boolean;
  public toggle(key: T, force?: boolean): boolean {
    const want = force === undefined ? !this.backingSet.has(key) : force;
    if (want) {
      this.add(key);
    } else {
      this.delete(key);
    }
    return want;
  }

  public clear(): void {
    for (const k of Array.from(this.backingSet)) {
      this.delete(k);
    }
  }
}

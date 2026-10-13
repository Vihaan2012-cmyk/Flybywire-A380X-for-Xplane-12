import type { Tap } from '../bus';
import { AbstractWatchable } from './core';
import { Readable, LifeView, Watch } from './types';

export class FeedCell<T> extends AbstractWatchable<T> implements Watch {
  public readonly canNotifyNow = true;

  private readonly consumerHandler = (v: T): void => this.onEventConsumed(v);
  private value: T;
  private isValueConsumed = false;
  private consumerSub?: Watch;
  private needSetDefaultValue = false;
  private defaultValue?: T;
  private suppress = false;
  private pending = false;
  private _isAlive = true;
  private _isPaused = false;

  public get isAlive(): boolean {
    return this._isAlive;
  }

  public get isPaused(): boolean {
    return this._isPaused;
  }

  private constructor(
    consumer: Tap<T> | null,
    initialVal: T,
    private readonly equalityFunc: (a: T, b: T) => boolean,
    private readonly mutateFunc?: (oldVal: T, newVal: T) => void,
  ) {
    super();
    this.value = initialVal;
    this.consumerSub = consumer?.handle(this.consumerHandler);
  }

  public static create<T>(
    consumer: Tap<T> | null,
    initialVal: T,
    equalityFunc?: (a: T, b: T) => boolean,
  ): FeedCell<T>;
  public static create<T>(
    consumer: Tap<T> | null,
    initialVal: T,
    equalityFunc: (a: T, b: T) => boolean,
    mutateFunc: (oldVal: T, newVal: T) => void,
  ): FeedCell<T>;
  public static create<T>(
    consumer: Tap<T> | null,
    initialVal: T,
    equalityFunc?: (a: T, b: T) => boolean,
    mutateFunc?: (oldVal: T, newVal: T) => void,
  ): FeedCell<T> {
    return new FeedCell(
      consumer,
      initialVal,
      equalityFunc ?? AbstractWatchable.DEFAULT_EQUALITY_FUNC,
      mutateFunc,
    );
  }

  private setValue(value: T): void {
    if (!this.equalityFunc(this.value, value)) {
      if (this.mutateFunc) {
        this.mutateFunc(this.value, value);
      } else {
        this.value = value;
      }
      if (this.suppress) {
        this.pending = true;
      } else {
        this.notify();
      }
    }
  }

  private onEventConsumed(value: T): void {
    this.isValueConsumed = true;
    this.setValue(value);
  }

  public get(): T {
    return this.value;
  }

  private detach(): void {
    this.consumerSub?.destroy();
    this.consumerSub = undefined;
    this.needSetDefaultValue = false;
    this.defaultValue = undefined;
  }

  public follow(consumer: Tap<T> | null): this {
    this.detach();
    this.consumerSub = consumer?.handle(this.consumerHandler, this._isPaused);
    return this;
  }

  public setConsumerWithDefault(consumer: Tap<T> | null, defaultVal: T): this {
    this.detach();
    if (!consumer) {
      this.setValue(defaultVal);
      return this;
    }
    this.needSetDefaultValue = true;
    this.defaultValue = defaultVal;
    this.isValueConsumed = false;
    this.consumerSub = consumer.handle(this.consumerHandler, this._isPaused);
    if (!this._isPaused && !this.isValueConsumed) {
      this.setValue(defaultVal);
    }
    return this;
  }

  public reset(initialVal: T, consumer: Tap<T> | null = null): this {
    this.detach();
    const before = this.value;
    this.suppress = true;
    this.pending = false;
    try {
      this.setValue(initialVal);
      this.consumerSub = consumer?.handle(this.consumerHandler, this._isPaused);
    } finally {
      this.suppress = false;
    }
    const changed = this.mutateFunc ? this.pending : !this.equalityFunc(before, this.value);
    this.pending = false;
    if (changed) {
      this.notify();
    }
    return this;
  }

  public pause(): this {
    if (!this._isAlive) {
      throw new Error('FeedCell: cannot pause a dead subject.');
    }
    if (this._isPaused) {
      return this;
    }
    this._isPaused = true;
    this.consumerSub?.pause();
    return this;
  }

  public resume(): this {
    if (!this._isAlive) {
      throw new Error('FeedCell: cannot resume a dead subject.');
    }
    if (!this._isPaused) {
      return this;
    }
    this._isPaused = false;
    if (this.consumerSub) {
      this.isValueConsumed = false;
      this.consumerSub.resume(true);
      if (this.needSetDefaultValue && !this.isValueConsumed) {
        this.setValue(this.defaultValue as T);
      }
    }
    return this;
  }

  public destroy(): void {
    if (!this._isAlive) {
      return;
    }
    this._isAlive = false;
    this._isPaused = true;
    this.consumerSub?.destroy();
    this.consumerSub = undefined;
  }

  public scoped(lifecycle: LifeView): this {
    lifecycle.register(this);
    return this;
  }
}

export type TapValueView<T> = Pick<TapValue<T>, 'get' | 'isAlive' | 'isPaused'>;

export class TapValue<T> implements Readable<T>, Watch {
  public readonly canNotifyNow = true;

  private readonly consumerHandler = (v: T): void => {
    this.isValueConsumed = true;
    this.value = v;
  };
  private value: T;
  private isValueConsumed = false;
  private sub?: Watch;
  private needSetDefaultValue = false;
  private defaultValue?: T;
  private _isAlive = true;
  private _isPaused = false;

  public get isAlive(): boolean {
    return this._isAlive;
  }

  public get isPaused(): boolean {
    return this._isPaused;
  }

  private constructor(consumer: Tap<T> | null, initialValue: T) {
    this.value = initialValue;
    this.sub = consumer?.handle(this.consumerHandler);
  }

  public static create<T>(consumer: Tap<T> | null, initialValue: T): TapValue<T> {
    return new TapValue(consumer, initialValue);
  }

  public get(): T {
    return this.value;
  }

  private detach(): void {
    this.sub?.destroy();
    this.sub = undefined;
    this.needSetDefaultValue = false;
    this.defaultValue = undefined;
  }

  public follow(consumer: Tap<T> | null): this {
    this.detach();
    this.sub = consumer?.handle(this.consumerHandler, this._isPaused);
    return this;
  }

  public setConsumerWithDefault(consumer: Tap<T> | null, defaultVal: T): this {
    this.detach();
    if (!consumer) {
      this.value = defaultVal;
      return this;
    }
    this.needSetDefaultValue = true;
    this.defaultValue = defaultVal;
    this.isValueConsumed = false;
    this.sub = consumer.handle(this.consumerHandler, this._isPaused);
    if (!this._isPaused && !this.isValueConsumed) {
      this.value = defaultVal;
    }
    return this;
  }

  public reset(initialVal: T, consumer: Tap<T> | null = null): this {
    this.detach();
    this.value = initialVal;
    this.sub = consumer?.handle(this.consumerHandler, this._isPaused);
    return this;
  }

  public pause(): this {
    if (!this._isAlive) {
      throw new Error('TapValue: cannot pause a dead object.');
    }
    if (this._isPaused) {
      return this;
    }
    this._isPaused = true;
    this.sub?.pause();
    return this;
  }

  public resume(): this {
    if (!this._isAlive) {
      throw new Error('TapValue: cannot resume a dead object.');
    }
    if (!this._isPaused) {
      return this;
    }
    this._isPaused = false;
    if (this.sub) {
      this.isValueConsumed = false;
      this.sub.resume(true);
      if (this.needSetDefaultValue && !this.isValueConsumed) {
        this.value = this.defaultValue as T;
      }
    }
    return this;
  }

  public destroy(): void {
    if (!this._isAlive) {
      return;
    }
    this._isAlive = false;
    this._isPaused = true;
    this.sub?.destroy();
    this.sub = undefined;
  }

  public scoped(lifecycle: LifeView): this {
    lifecycle.register(this);
    return this;
  }
}

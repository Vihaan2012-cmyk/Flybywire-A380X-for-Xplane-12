import type { Consumer } from '../bus';
import { AbstractSubscribable } from './core';
import { Accessible, ReadonlyLifecycle, Subscription } from './types';

export class ConsumerSubject<T> extends AbstractSubscribable<T> implements Subscription {
  public readonly canInitialNotify = true;

  private readonly consumerHandler = (v: T): void => this.onEventConsumed(v);
  private value: T;
  private isValueConsumed = false;
  private consumerSub?: Subscription;
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
    consumer: Consumer<T> | null,
    initialVal: T,
    private readonly equalityFunc: (a: T, b: T) => boolean,
    private readonly mutateFunc?: (oldVal: T, newVal: T) => void,
  ) {
    super();
    this.value = initialVal;
    this.consumerSub = consumer?.handle(this.consumerHandler);
  }

  public static create<T>(
    consumer: Consumer<T> | null,
    initialVal: T,
    equalityFunc?: (a: T, b: T) => boolean,
  ): ConsumerSubject<T>;
  public static create<T>(
    consumer: Consumer<T> | null,
    initialVal: T,
    equalityFunc: (a: T, b: T) => boolean,
    mutateFunc: (oldVal: T, newVal: T) => void,
  ): ConsumerSubject<T>;
  public static create<T>(
    consumer: Consumer<T> | null,
    initialVal: T,
    equalityFunc?: (a: T, b: T) => boolean,
    mutateFunc?: (oldVal: T, newVal: T) => void,
  ): ConsumerSubject<T> {
    return new ConsumerSubject(
      consumer,
      initialVal,
      equalityFunc ?? AbstractSubscribable.DEFAULT_EQUALITY_FUNC,
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

  public setConsumer(consumer: Consumer<T> | null): this {
    this.detach();
    this.consumerSub = consumer?.handle(this.consumerHandler, this._isPaused);
    return this;
  }

  public setConsumerWithDefault(consumer: Consumer<T> | null, defaultVal: T): this {
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

  public reset(initialVal: T, consumer: Consumer<T> | null = null): this {
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
      throw new Error('ConsumerSubject: cannot pause a dead subject.');
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
      throw new Error('ConsumerSubject: cannot resume a dead subject.');
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

  public withLifecycle(lifecycle: ReadonlyLifecycle): this {
    lifecycle.register(this);
    return this;
  }
}

export type ReadonlyConsumerValue<T> = Pick<ConsumerValue<T>, 'get' | 'isAlive' | 'isPaused'>;

export class ConsumerValue<T> implements Accessible<T>, Subscription {
  public readonly canInitialNotify = true;

  private readonly consumerHandler = (v: T): void => {
    this.isValueConsumed = true;
    this.value = v;
  };
  private value: T;
  private isValueConsumed = false;
  private sub?: Subscription;
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

  private constructor(consumer: Consumer<T> | null, initialValue: T) {
    this.value = initialValue;
    this.sub = consumer?.handle(this.consumerHandler);
  }

  public static create<T>(consumer: Consumer<T> | null, initialValue: T): ConsumerValue<T> {
    return new ConsumerValue(consumer, initialValue);
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

  public setConsumer(consumer: Consumer<T> | null): this {
    this.detach();
    this.sub = consumer?.handle(this.consumerHandler, this._isPaused);
    return this;
  }

  public setConsumerWithDefault(consumer: Consumer<T> | null, defaultVal: T): this {
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

  public reset(initialVal: T, consumer: Consumer<T> | null = null): this {
    this.detach();
    this.value = initialVal;
    this.sub = consumer?.handle(this.consumerHandler, this._isPaused);
    return this;
  }

  public pause(): this {
    if (!this._isAlive) {
      throw new Error('ConsumerValue: cannot pause a dead object.');
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
      throw new Error('ConsumerValue: cannot resume a dead object.');
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

  public withLifecycle(lifecycle: ReadonlyLifecycle): this {
    lifecycle.register(this);
    return this;
  }
}

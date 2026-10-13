import type { Subscription } from '../sub';

export type Handler<T> = (data: T) => void;

export interface Consumer<T> {
  readonly isConsumer: true;
  handle(handler: Handler<T>, paused?: boolean): Subscription;
  atFrequency(frequency: number, immediateFirstPublish?: boolean): Consumer<T>;
  withPrecision(precision: number): Consumer<T>;
  whenChangedBy(amount: number): Consumer<T>;
  whenChanged(): Consumer<T>;
  onlyAfter(deltaTime: number): Consumer<T>;
}

export class BasicConsumer<T> implements Consumer<T> {
  public readonly isConsumer = true;

  public constructor(
    private readonly subscribe: (handler: Handler<T>, paused: boolean) => Subscription,
    private state: any = {},
    private readonly currentHandler?: (data: T, state: any, next: Handler<T>) => void,
  ) {}

  public handle(handler: Handler<T>, paused = false): Subscription {
    if (this.currentHandler !== undefined) {
      return this.subscribe((data: T) => {
        this.with(data, handler);
      }, paused);
    }
    return this.subscribe(handler, paused);
  }

  public atFrequency(frequency: number, immediateFirstPublish = true): Consumer<T> {
    return new BasicConsumer<T>(
      this.subscribe,
      { previousTime: Date.now(), firstPublish: immediateFirstPublish },
      this.getAtFrequencyHandler(frequency),
    );
  }

  private getAtFrequencyHandler(frequency: number): (data: T, state: any, next: Handler<T>) => void {
    const period = 1000 / frequency;
    return (data, state, next): void => {
      const now = Date.now();
      if (state.firstPublish) {
        state.firstPublish = false;
        state.previousTime = now;
        this.with(data, next);
      } else if (now - state.previousTime >= period) {
        state.previousTime = Math.max(state.previousTime + period, now - period);
        this.with(data, next);
      }
    };
  }

  public withPrecision(precision: number): Consumer<T> {
    return new BasicConsumer<T>(this.subscribe, { lastValue: undefined }, this.getWithPrecisionHandler(precision));
  }

  private getWithPrecisionHandler(precision: number): (data: any, state: any, next: Handler<T>) => void {
    const multiplier = 10 ** precision;
    return (data, state, next): void => {
      const valueAtPrecision = Math.round(data * multiplier) / multiplier;
      if (valueAtPrecision !== state.lastValue) {
        state.lastValue = valueAtPrecision;
        this.with(valueAtPrecision as T, next);
      }
    };
  }

  public whenChangedBy(amount: number): Consumer<T> {
    return new BasicConsumer<T>(this.subscribe, { lastValue: 0 }, this.getWhenChangedByHandler(amount));
  }

  private getWhenChangedByHandler(amount: number): (data: any, state: any, next: Handler<T>) => void {
    return (data, state, next): void => {
      if (Math.abs(data - state.lastValue) >= amount) {
        state.lastValue = data;
        this.with(data, next);
      }
    };
  }

  public whenChanged(): Consumer<T> {
    return new BasicConsumer<T>(this.subscribe, { lastValue: '' }, this.getWhenChangedHandler());
  }

  private getWhenChangedHandler(): (data: any, state: any, next: Handler<T>) => void {
    return (data, state, next): void => {
      if (state.lastValue !== data) {
        state.lastValue = data;
        this.with(data, next);
      }
    };
  }

  public onlyAfter(deltaTime: number): Consumer<T> {
    return new BasicConsumer<T>(this.subscribe, { previousTime: Date.now() }, this.getOnlyAfterHandler(deltaTime));
  }

  private getOnlyAfterHandler(deltaTime: number): (data: T, state: any, next: Handler<T>) => void {
    return (data, state, next): void => {
      const now = Date.now();
      if (Math.abs(now - state.previousTime) >= deltaTime) {
        this.with(data, next);
        state.previousTime = now;
      }
    };
  }

  private with(data: T, handler: Handler<T>): void {
    if (this.currentHandler !== undefined) {
      this.currentHandler(data, this.state, handler);
    } else {
      handler(data);
    }
  }
}

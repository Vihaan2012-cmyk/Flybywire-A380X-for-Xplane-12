import type { Watch } from '../sub';

export type Callback<T> = (data: T) => void;

export interface Tap<T> {
  readonly isConsumer: true;
  handle(handler: Callback<T>, paused?: boolean): Watch;
  every(frequency: number, immediateFirstPublish?: boolean): Tap<T>;
  rounded(precision: number): Tap<T>;
  changesBy(amount: number): Tap<T>;
  changes(): Tap<T>;
  afterGap(deltaTime: number): Tap<T>;
}

export class BasicTap<T> implements Tap<T> {
  public readonly isConsumer = true;

  public constructor(
    private readonly subscribe: (handler: Callback<T>, paused: boolean) => Watch,
    private state: any = {},
    private readonly currentHandler?: (data: T, state: any, next: Callback<T>) => void,
  ) {}

  public handle(handler: Callback<T>, paused = false): Watch {
    if (this.currentHandler !== undefined) {
      return this.subscribe((data: T) => {
        this.with(data, handler);
      }, paused);
    }
    return this.subscribe(handler, paused);
  }

  public every(frequency: number, immediateFirstPublish = true): Tap<T> {
    return new BasicTap<T>(
      this.subscribe,
      { previousTime: Date.now(), firstPublish: immediateFirstPublish },
      this.getAtFrequencyHandler(frequency),
    );
  }

  private getAtFrequencyHandler(frequency: number): (data: T, state: any, next: Callback<T>) => void {
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

  public rounded(precision: number): Tap<T> {
    return new BasicTap<T>(this.subscribe, { lastValue: undefined }, this.getWithPrecisionHandler(precision));
  }

  private getWithPrecisionHandler(precision: number): (data: any, state: any, next: Callback<T>) => void {
    const multiplier = 10 ** precision;
    return (data, state, next): void => {
      const valueAtPrecision = Math.round(data * multiplier) / multiplier;
      if (valueAtPrecision !== state.lastValue) {
        state.lastValue = valueAtPrecision;
        this.with(valueAtPrecision as T, next);
      }
    };
  }

  public changesBy(amount: number): Tap<T> {
    return new BasicTap<T>(this.subscribe, { lastValue: 0 }, this.getWhenChangedByHandler(amount));
  }

  private getWhenChangedByHandler(amount: number): (data: any, state: any, next: Callback<T>) => void {
    return (data, state, next): void => {
      if (Math.abs(data - state.lastValue) >= amount) {
        state.lastValue = data;
        this.with(data, next);
      }
    };
  }

  public changes(): Tap<T> {
    return new BasicTap<T>(this.subscribe, { lastValue: '' }, this.getWhenChangedHandler());
  }

  private getWhenChangedHandler(): (data: any, state: any, next: Callback<T>) => void {
    return (data, state, next): void => {
      if (state.lastValue !== data) {
        state.lastValue = data;
        this.with(data, next);
      }
    };
  }

  public afterGap(deltaTime: number): Tap<T> {
    return new BasicTap<T>(this.subscribe, { previousTime: Date.now() }, this.getOnlyAfterHandler(deltaTime));
  }

  private getOnlyAfterHandler(deltaTime: number): (data: T, state: any, next: Callback<T>) => void {
    return (data, state, next): void => {
      const now = Date.now();
      if (Math.abs(now - state.previousTime) >= deltaTime) {
        this.with(data, next);
        state.previousTime = now;
      }
    };
  }

  private with(data: T, handler: Callback<T>): void {
    if (this.currentHandler !== undefined) {
      this.currentHandler(data, this.state, handler);
    } else {
      handler(data);
    }
  }
}

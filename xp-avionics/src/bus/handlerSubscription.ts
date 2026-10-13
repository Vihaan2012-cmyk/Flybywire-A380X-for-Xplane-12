import type { Subscription } from '../sub';

interface LifecycleLike {
  register(sub: Subscription): void;
}

export class HandlerSubscription<HandlerType extends (...args: any[]) => void> {
  public isAlive = true;
  public isPaused = false;
  public readonly canInitialNotify: boolean;

  public constructor(
    public readonly handler: HandlerType,
    private readonly initialNotifyFunc?: (sub: HandlerSubscription<HandlerType>) => void,
    private readonly onDestroy?: (sub: HandlerSubscription<HandlerType>) => void,
  ) {
    this.canInitialNotify = initialNotifyFunc !== undefined;
  }

  public initialNotify(): void {
    if (!this.isAlive) {
      throw new Error('HandlerSubscription: cannot notify a dead Subscription.');
    }
    this.initialNotifyFunc?.(this);
  }

  public pause(): this {
    if (!this.isAlive) {
      throw new Error('Subscription: cannot pause a dead Subscription.');
    }
    this.isPaused = true;
    return this;
  }

  public resume(initialNotify = false): this {
    if (!this.isAlive) {
      throw new Error('Subscription: cannot resume a dead Subscription.');
    }
    if (!this.isPaused) {
      return this;
    }
    this.isPaused = false;
    if (initialNotify) {
      this.initialNotify();
    }
    return this;
  }

  public destroy(): void {
    if (!this.isAlive) {
      return;
    }
    this.isAlive = false;
    this.isPaused = true;
    this.onDestroy?.(this);
  }

  public withLifecycle(lifecycle: LifecycleLike): this {
    lifecycle.register(this as unknown as Subscription);
    return this;
  }
}

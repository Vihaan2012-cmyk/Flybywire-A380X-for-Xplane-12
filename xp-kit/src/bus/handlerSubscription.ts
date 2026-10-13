import type { Watch } from '../sub';

interface LifecycleLike {
  register(sub: Watch): void;
}

export class CallbackWatch<HandlerType extends (...args: any[]) => void> {
  public isAlive = true;
  public isPaused = false;
  public readonly canNotifyNow: boolean;

  public constructor(
    public readonly handler: HandlerType,
    private readonly initialNotifyFunc?: (sub: CallbackWatch<HandlerType>) => void,
    private readonly onDestroy?: (sub: CallbackWatch<HandlerType>) => void,
  ) {
    this.canNotifyNow = initialNotifyFunc !== undefined;
  }

  public initialNotify(): void {
    if (!this.isAlive) {
      throw new Error('CallbackWatch: cannot notify a dead Watch.');
    }
    this.initialNotifyFunc?.(this);
  }

  public pause(): this {
    if (!this.isAlive) {
      throw new Error('Watch: cannot pause a dead Watch.');
    }
    this.isPaused = true;
    return this;
  }

  public resume(initialNotify = false): this {
    if (!this.isAlive) {
      throw new Error('Watch: cannot resume a dead Watch.');
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

  public scoped(lifecycle: LifecycleLike): this {
    lifecycle.register(this as unknown as Watch);
    return this;
  }
}

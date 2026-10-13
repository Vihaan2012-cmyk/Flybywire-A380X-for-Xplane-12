import type { Hub, Writer } from './eventBus';

export interface Rate<E> {
  canPublish<K extends keyof E>(topic: K, data: E[K]): boolean;
}

export class IntervalRate<E> implements Rate<E> {
  private lastPublished = new Map<keyof E, number>();

  public constructor(private interval: number) {}

  public canPublish<K extends keyof E>(topic: K, _data: E[K]): boolean {
    const now = Date.now();
    const last = this.lastPublished.get(topic);
    if (last === undefined || now - last >= this.interval) {
      this.lastPublished.set(topic, now);
      return true;
    }
    return false;
  }
}

export class DeltaRate<E> implements Rate<E> {
  private lastPublished = new Map<keyof E, number>();

  public constructor(private delta: number) {}

  public canPublish<K extends keyof E>(topic: K, data: E[K]): boolean {
    const last = this.lastPublished.get(topic);
    const value = data as unknown as number;
    if (last === undefined || Math.abs(value - last) >= this.delta) {
      this.lastPublished.set(topic, value);
      return true;
    }
    return false;
  }
}

export class SourceBase<E extends Record<string, any>> {
  protected readonly publisher: Writer<E>;
  protected publishActive = false;

  public constructor(
    protected readonly bus: Hub,
    protected readonly pacer: Rate<E> | undefined = undefined,
  ) {
    this.publisher = bus.writer<E>();
  }

  public startFeed(): void {
    this.publishActive = true;
  }

  public stopPublish(): void {
    this.publishActive = false;
  }

  public isPublishing(): boolean {
    return this.publishActive;
  }

  public onUpdate(): void {}

  protected publish<K extends keyof E>(topic: K, data: E[K], sync = false, isCached = true): void {
    if (this.publishActive && (!this.pacer || this.pacer.canPublish(topic, data))) {
      this.publisher.send(topic, data, sync, isCached);
    }
  }
}

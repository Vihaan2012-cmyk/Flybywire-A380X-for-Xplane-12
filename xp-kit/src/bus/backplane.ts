export interface GaugeCore {
  init(): void;
  onUpdate(): void;
}

export interface RackSource {
  startFeed(): void;
  onUpdate(): void;
}

export class GaugeRack {
  private publishers = new Map<string, RackSource>();
  private instruments = new Map<string, GaugeCore>();

  public init(): void {
    for (const publisher of this.publishers.values()) {
      publisher.startFeed();
    }
    for (const instrument of this.instruments.values()) {
      instrument.init();
    }
  }

  public onUpdate(): void {
    for (const publisher of this.publishers.values()) {
      publisher.onUpdate();
    }
    for (const instrument of this.instruments.values()) {
      instrument.onUpdate();
    }
  }

  public attach(name: string, publisher: RackSource, override = false): void {
    GaugeRack.register(this.publishers, name, publisher, override);
  }

  public plug(name: string, instrument: GaugeCore, override = false): void {
    GaugeRack.register(this.instruments, name, instrument, override);
  }

  public writer<T extends RackSource = RackSource>(name: string): T | undefined {
    return this.publishers.get(name) as T | undefined;
  }

  public getInstrument<T extends GaugeCore = GaugeCore>(name: string): T | undefined {
    return this.instruments.get(name) as T | undefined;
  }

  private static register<T extends object>(map: Map<string, T>, name: string, item: T, override: boolean): void {
    if (override) {
      map.delete(name);
    } else if (GaugeRack.alreadyExists(name, item, map)) {
      return;
    }
    map.set(name, item);
  }

  private static alreadyExists<T extends object>(name: string, item: T, map: Map<string, T>): boolean {
    if (map.has(name)) {
      return true;
    }
    for (const existing of map.values()) {
      if (existing.constructor === item.constructor) {
        return true;
      }
    }
    return false;
  }
}

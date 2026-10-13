export interface Instrument {
  init(): void;
  onUpdate(): void;
}

export interface BackplanePublisher {
  startPublish(): void;
  onUpdate(): void;
}

export class InstrumentBackplane {
  private publishers = new Map<string, BackplanePublisher>();
  private instruments = new Map<string, Instrument>();

  public init(): void {
    for (const publisher of this.publishers.values()) {
      publisher.startPublish();
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

  public addPublisher(name: string, publisher: BackplanePublisher, override = false): void {
    InstrumentBackplane.register(this.publishers, name, publisher, override);
  }

  public addInstrument(name: string, instrument: Instrument, override = false): void {
    InstrumentBackplane.register(this.instruments, name, instrument, override);
  }

  public getPublisher<T extends BackplanePublisher = BackplanePublisher>(name: string): T | undefined {
    return this.publishers.get(name) as T | undefined;
  }

  public getInstrument<T extends Instrument = Instrument>(name: string): T | undefined {
    return this.instruments.get(name) as T | undefined;
  }

  private static register<T extends object>(map: Map<string, T>, name: string, item: T, override: boolean): void {
    if (override) {
      map.delete(name);
    } else if (InstrumentBackplane.alreadyExists(name, item, map)) {
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

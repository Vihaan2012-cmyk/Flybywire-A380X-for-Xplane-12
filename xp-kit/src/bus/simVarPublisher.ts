import type { Hub, HubMeta } from './eventBus';
import { SourceBase, Rate } from './publisher';
import { installVarSetter } from './simVarOverride';

installVarSetter();

export enum VarKind {
  Amps = 'Amperes',
  Bool = 'bool',
  Celsius = 'celsius',
  CubicInches = 'cubic inches',
  Degree = 'degrees',
  DegreesPerSecond = 'degrees per second',
  DegreesPerSecondSquared = 'degrees per second squared',
  Enum = 'enum',
  Farenheit = 'farenheit',
  Feet = 'feet',
  FPM = 'feet per minute',
  FtLb = 'Foot pounds',
  GAL = 'gallons',
  GPH = 'gph',
  Hertz = 'hertz',
  Hours = 'Hours',
  HPA = 'hectopascals',
  Inches = 'inches',
  InHG = 'inches of mercury',
  KHz = 'KHz',
  Kilograms = 'kilograms',
  Knots = 'knots',
  KgPerCubicMeter = 'kilogram per cubic meter',
  LBS = 'pounds',
  Liters = 'liters',
  LLA = 'latlonalt',
  LLA_PBH = 'latlonaltpbh',
  Mach = 'mach',
  MB = 'Millibars',
  Meters = 'meters',
  MetersPerSecond = 'meters per second',
  MetersPerSecondSquared = 'meters per second squared',
  MillimetersWater = 'millimeters of water',
  MHz = 'MHz',
  NM = 'nautical mile',
  Number = 'number',
  PBH = 'pbh',
  Percent = 'percent',
  PercentOver100 = 'percent over 100',
  PerSecond = 'per second',
  PIDStruct = 'pid_struct',
  Position = 'position',
  Position16k = 'position 16k',
  Position32k = 'position 32k',
  Pounds = 'pounds',
  PoundsPerSquareFoot = 'pounds per square foot',
  PPH = 'Pounds per hour',
  PSI = 'psi',
  Radians = 'radians',
  RadiansPerSecond = 'radians per second',
  RadiansPerSecondSquared = 'radians per second squared',
  Rankine = 'rankine',
  RPM = 'Rpm',
  Seconds = 'seconds',
  SlugsPerCubicFoot = 'slug per cubic foot',
  String = 'string',
  Volts = 'Volts',
  XYZ = 'xyz',
}

export type VarDef = {
  name: string;
  type: VarKind;
};

export type VarSourceEntry<T> = VarDef & {
  map?: (value: any) => T;
  indexed?: boolean | Iterable<number>;
  defaultIndex?: number | null;
};

type IndexedSimVarPublisherEntry<T> = Omit<VarSourceEntry<T>, 'indexed'> & {
  indexes: Set<number> | undefined;
};

type ResolvedSimVarPublisherEntry<T> = Omit<VarSourceEntry<T>, 'indexed' | 'defaultIndex'> & {
  unsuffixedTopic?: string;
  registeredId?: number;
};

const STRUCT_TYPES = new Set<string>([
  VarKind.LLA,
  VarKind.LLA_PBH,
  VarKind.PBH,
  VarKind.PIDStruct,
  VarKind.XYZ,
]);

export class VarSource<
  Events extends Record<string, any>,
  IndexedEventRoots extends Record<string, any> = Events,
> extends SourceBase<Events> {
  protected static readonly INDEXED_REGEX = /^(.*)_(\d+)$/;

  protected readonly resolvedSimVars = new Map<keyof Events & string, ResolvedSimVarPublisherEntry<any>>();
  protected readonly indexedSimVars = new Map<keyof IndexedEventRoots & string, IndexedSimVarPublisherEntry<any>>();
  protected readonly subscribed = new Map<keyof Events & string, ResolvedSimVarPublisherEntry<any>>();

  public constructor(
    entries: Iterable<readonly [keyof (Events & IndexedEventRoots) & string, VarSourceEntry<any>]>,
    bus: Hub,
    pacer?: Rate<Events>,
  ) {
    super(bus, pacer);

    for (const [topic, entry] of entries) {
      if (entry.indexed) {
        const { indexed, ...rest } = entry;
        this.indexedSimVars.set(topic as keyof IndexedEventRoots & string, {
          ...rest,
          indexes: indexed === true ? undefined : new Set(indexed as Iterable<number>),
        });
      } else {
        this.resolvedSimVars.set(topic as keyof Events & string, entry);
      }
    }

    bus
      .reader<HubMeta>()
      .on('event_bus_topic_first_sub')
      .handle((topic) => this.handleSubscribedTopic(topic));
    bus.forEachSubscribedTopic((topic) => this.handleSubscribedTopic(topic));
  }

  protected handleSubscribedTopic(topic: string): void {
    if (this.resolvedSimVars.has(topic as keyof Events & string)) {
      this.onTopicTaken(topic as keyof Events & string);
    } else {
      this.tryMatchIndexedSubscribedTopic(topic);
    }
  }

  protected tryMatchIndexedSubscribedTopic(topic: string): void {
    const rootEntry = this.indexedSimVars.get(topic as keyof IndexedEventRoots & string);
    if (rootEntry !== undefined) {
      const defaultIndex = rootEntry.defaultIndex === undefined ? 1 : rootEntry.defaultIndex;
      if (defaultIndex === null) {
        return;
      }
      const resolved = this.resolveIndexedSimVar(topic as keyof IndexedEventRoots & string, rootEntry, defaultIndex);
      if (resolved !== undefined) {
        this.resolvedSimVars.get(resolved as keyof Events & string)!.unsuffixedTopic = topic;
        this.onTopicTaken(resolved as keyof Events & string);
      }
      return;
    }

    const match = VarSource.INDEXED_REGEX.exec(topic);
    if (match === null) {
      return;
    }
    const root = match[1] as keyof IndexedEventRoots & string;
    const entry = this.indexedSimVars.get(root);
    if (entry === undefined) {
      return;
    }
    const resolved = this.resolveIndexedSimVar(root, entry, parseInt(match[2]));
    if (resolved !== undefined) {
      this.onTopicTaken(resolved as keyof Events & string);
    }
  }

  protected resolveIndexedSimVar(
    topic: keyof IndexedEventRoots & string,
    entry: IndexedSimVarPublisherEntry<any>,
    index: number,
  ): string | undefined {
    if (entry.indexes !== undefined && !entry.indexes.has(index)) {
      return undefined;
    }
    const resolvedTopic = `${topic}_${index}`;
    if (!this.resolvedSimVars.has(resolvedTopic as keyof Events & string)) {
      const { indexes: _indexes, defaultIndex: _defaultIndex, ...rest } = entry;
      this.resolvedSimVars.set(resolvedTopic as keyof Events & string, {
        ...rest,
        name: entry.name.replace('#index#', String(index)),
      });
    }
    return resolvedTopic;
  }

  protected onTopicTaken(topic: keyof Events & string): void {
    const entry = this.resolvedSimVars.get(topic);
    if (entry === undefined || this.subscribed.has(topic)) {
      return;
    }
    if (!STRUCT_TYPES.has(entry.type)) {
      entry.registeredId = SimVar.GetRegisteredId(entry.name, entry.type, '');
    }
    this.subscribed.set(topic, entry);
    if (this.publishActive) {
      this.publishTopic(topic, entry);
    }
  }

  public onUpdate(): void {
    if (!this.publishActive) {
      return;
    }
    for (const [topic, entry] of this.subscribed) {
      this.publishTopic(topic, entry);
    }
  }

  protected publishTopic(topic: keyof Events & string, entry: ResolvedSimVarPublisherEntry<any> | undefined): void {
    if (entry === undefined) {
      return;
    }
    const value = this.getValueFromEntry(entry);
    this.publish(topic, value);
    if (entry.unsuffixedTopic !== undefined) {
      this.publish(entry.unsuffixedTopic as keyof Events & string, value);
    }
  }

  protected getValue<K extends keyof Events & string>(topic: K): Events[K] | undefined {
    const entry = this.resolvedSimVars.get(topic);
    return entry === undefined ? undefined : this.getValueFromEntry(entry);
  }

  protected getValueFromEntry<T>(entry: ResolvedSimVarPublisherEntry<T>): T {
    let value: any;
    if (entry.registeredId === undefined || entry.registeredId < 0) {
      value = SimVar.GetSimVarValue(entry.name, entry.type);
    } else if (entry.type === VarKind.String) {
      value = SimVar.GetSimVarValueFastRegString(entry.registeredId);
    } else {
      value = SimVar.GetSimVarValueFastReg(entry.registeredId);
      if (entry.type === VarKind.Bool) {
        value = !!value;
      }
    }
    return entry.map === undefined ? value : entry.map(value);
  }
}

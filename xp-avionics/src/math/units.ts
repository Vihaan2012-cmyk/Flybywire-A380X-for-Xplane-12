export type FamilyOfUnit<U extends Unit<string>> = U extends Unit<infer T> ? T : never;

export type UnitOfNumber<N extends NumberUnitInterface<string>> = N extends NumberUnitInterface<string, infer U>
  ? U
  : never;

export interface Unit<F extends string> {
  readonly family: F;
  readonly name: string;
  canConvert(otherUnit: Unit<string>): boolean;
  convertTo(value: number, toUnit: Unit<F>): number;
  convertFrom(value: number, fromUnit: Unit<F>): number;
  createNumber(value: number): NumberUnit<F, this>;
  equals(other: Unit<string>): boolean;
}

export interface CompoundableUnit<F extends string> extends Unit<F> {
  readonly scaleFactor: number;
}

export interface NumberUnitInterface<F extends string, U extends Unit<F> = Unit<F>> {
  number: number;
  unit: U;
  add<OU extends Unit<F>>(value: NumberUnitInterface<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  add<OU extends Unit<F>>(value: number, unit: Unit<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  subtract<OU extends Unit<F>>(value: NumberUnitInterface<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  subtract<OU extends Unit<F>>(value: number, unit: Unit<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  scale<OU extends Unit<F>>(factor: number, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  ratio(value: NumberUnitInterface<F>): number;
  ratio(value: number, unit: Unit<F>): number;
  abs<OU extends Unit<F>>(out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  asUnit(unit: Unit<F>): number;
  compare(value: NumberUnitInterface<F>): number;
  compare(value: number, unit?: Unit<F>): number;
  equals(value: NumberUnitInterface<string>): boolean;
  equals(value: number, unit?: Unit<string>): boolean;
  isNaN(): boolean;
  copy(): NumberUnit<F, U>;
}

export abstract class AbstractUnit<F extends string> implements Unit<F> {
  public abstract readonly family: F;

  public constructor(public readonly name: string) {}

  public canConvert(otherUnit: Unit<string>): boolean {
    return this.family === otherUnit.family;
  }

  public abstract convertTo(value: number, toUnit: Unit<F>): number;

  public abstract convertFrom(value: number, fromUnit: Unit<F>): number;

  public createNumber(value: number): NumberUnit<F, this> {
    return new NumberUnit<F, this>(value, this);
  }

  public equals(other: Unit<string>): boolean {
    return this.family === other.family && this.name === other.name;
  }
}

export class SimpleUnit<F extends string> extends AbstractUnit<F> implements CompoundableUnit<F> {
  public constructor(
    public readonly family: F,
    name: string,
    public readonly scaleFactor: number,
    public readonly zeroOffset: number = 0,
  ) {
    super(name);
  }

  public canConvert(otherUnit: Unit<string>): boolean {
    return this.family === otherUnit.family;
  }

  public convertTo(value: number, toUnit: Unit<F>): number {
    if (!this.canConvert(toUnit)) {
      throw new Error(`Invalid conversion from ${this.name} to ${toUnit.name}.`);
    }
    const target = toUnit as unknown as { scaleFactor: number; zeroOffset?: number };
    return (value * this.scaleFactor + this.zeroOffset - (target.zeroOffset ?? 0)) / target.scaleFactor;
  }

  public convertFrom(value: number, fromUnit: Unit<F>): number {
    if (!this.canConvert(fromUnit)) {
      throw new Error(`Invalid conversion from ${fromUnit.name} to ${this.name}.`);
    }
    const source = fromUnit as unknown as { scaleFactor: number; zeroOffset?: number };
    return (value * source.scaleFactor + (source.zeroOffset ?? 0) - this.zeroOffset) / this.scaleFactor;
  }
}

export class CompoundUnit<F extends string> extends AbstractUnit<F> implements CompoundableUnit<F> {
  public readonly scaleFactor: number;

  public constructor(
    public readonly family: F,
    private readonly numerator: CompoundableUnit<string>[],
    private readonly denominator: CompoundableUnit<string>[],
    name?: string,
  ) {
    super(name ?? CompoundUnit.generateName(numerator, denominator));
    this.scaleFactor = this.computeScaleFactor();
  }

  private static generateName(numerator: CompoundableUnit<string>[], denominator: CompoundableUnit<string>[]): string {
    const top = numerator.map((unit) => unit.name).join('-');
    return denominator.length > 0 ? `${top}-per-${denominator.map((unit) => unit.name).join('-')}` : top;
  }

  private computeScaleFactor(): number {
    let numeratorProduct = 1;
    for (const unit of this.numerator) {
      numeratorProduct *= unit.scaleFactor;
    }
    let denominatorProduct = 1;
    for (const unit of this.denominator) {
      denominatorProduct *= unit.scaleFactor;
    }
    return numeratorProduct / denominatorProduct;
  }

  public canConvert(otherUnit: Unit<string>): boolean {
    return this.family === otherUnit.family;
  }

  public convertTo(value: number, toUnit: Unit<F>): number {
    if (!this.canConvert(toUnit)) {
      throw new Error(`Invalid conversion from ${this.name} to ${toUnit.name}.`);
    }
    return (value * this.scaleFactor) / (toUnit as unknown as { scaleFactor: number }).scaleFactor;
  }

  public convertFrom(value: number, fromUnit: Unit<F>): number {
    if (!this.canConvert(fromUnit)) {
      throw new Error(`Invalid conversion from ${fromUnit.name} to ${this.name}.`);
    }
    return (value * (fromUnit as unknown as { scaleFactor: number }).scaleFactor) / this.scaleFactor;
  }
}

export enum UnitFamily {
  Distance = 'distance',
  Angle = 'angle',
  Duration = 'duration',
  Weight = 'weight',
  Mass = 'weight',
  Volume = 'volume',
  Pressure = 'pressure',
  Temperature = 'temperature',
  TemperatureDelta = 'temperature_delta',
  Speed = 'speed',
  Acceleration = 'acceleration',
  WeightFlux = 'weight_flux',
  MassFlux = 'weight_flux',
  VolumeFlux = 'volume_flux',
  Density = 'density',
  Force = 'force',
  DistancePerWeight = 'distance_per_weight',
  DistanceRatio = 'distance_ratio',
  WeightPerDistance = 'weight_per_distance',
  Area = 'area',
}

export class NumberUnit<F extends string, U extends Unit<F> = Unit<F>> implements NumberUnitInterface<F, U> {
  public readonly readonly: NumberUnitReadOnly<F, U>;

  public constructor(
    private _number: number,
    private readonly _unit: U,
  ) {
    this.readonly = new NumberUnitReadOnly(this);
  }

  public get number(): number {
    return this._number;
  }

  public get unit(): U {
    return this._unit;
  }

  private toNumberOfThisUnit(value: NumberUnitInterface<F> | number, unit?: Unit<F>): number {
    if (typeof value === 'number') {
      return unit === undefined ? value : this._unit.convertFrom(value, unit);
    }
    return this._unit.convertFrom(value.number, value.unit);
  }

  public set(value: NumberUnitInterface<F>): this;
  public set(value: number, unit?: Unit<F>): this;
  public set(value: NumberUnitInterface<F> | number, unit?: Unit<F>): this {
    if (typeof value !== 'number') {
      if (!this._unit.canConvert(value.unit)) {
        return undefined as unknown as this;
      }
    }
    this._number = this.toNumberOfThisUnit(value, unit);
    return this;
  }

  public add<OU extends Unit<F>>(value: NumberUnitInterface<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public add(value: NumberUnitInterface<F>): this;
  public add<OU extends Unit<F>>(value: number, unit: Unit<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public add(value: number, unit: Unit<F>): this;
  public add(
    value: NumberUnitInterface<F> | number,
    arg2?: NumberUnit<F, Unit<F>> | Unit<F>,
    arg3?: NumberUnit<F, Unit<F>>,
  ): unknown {
    const { unit, out } = NumberUnit.resolveArguments(value, arg2, arg3);
    const sum = this._number + this.toNumberOfThisUnit(value, unit);
    return out ? out.set(sum, this._unit) : ((this._number = sum), this);
  }

  public subtract<OU extends Unit<F>>(value: NumberUnitInterface<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public subtract(value: NumberUnitInterface<F>): this;
  public subtract<OU extends Unit<F>>(value: number, unit: Unit<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public subtract(value: number, unit: Unit<F>): this;
  public subtract(
    value: NumberUnitInterface<F> | number,
    arg2?: NumberUnit<F, Unit<F>> | Unit<F>,
    arg3?: NumberUnit<F, Unit<F>>,
  ): unknown {
    const { unit, out } = NumberUnit.resolveArguments(value, arg2, arg3);
    const difference = this._number - this.toNumberOfThisUnit(value, unit);
    return out ? out.set(difference, this._unit) : ((this._number = difference), this);
  }

  private static resolveArguments<F extends string>(
    value: NumberUnitInterface<F> | number,
    arg2?: NumberUnit<F, Unit<F>> | Unit<F>,
    arg3?: NumberUnit<F, Unit<F>>,
  ): { unit: Unit<F> | undefined; out: NumberUnit<F, Unit<F>> | undefined } {
    if (typeof value === 'number') {
      return { unit: arg2 as Unit<F> | undefined, out: arg3 };
    }
    return { unit: undefined, out: arg2 as NumberUnit<F, Unit<F>> | undefined };
  }

  public scale<OU extends Unit<F>>(factor: number, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public scale(factor: number): this;
  public scale(factor: number, out?: NumberUnit<F, Unit<F>>): unknown {
    const scaled = this._number * factor;
    return out ? out.set(scaled, this._unit) : ((this._number = scaled), this);
  }

  public ratio(value: NumberUnitInterface<F>): number;
  public ratio(value: number, unit: Unit<F>): number;
  public ratio(value: NumberUnitInterface<F> | number, unit?: Unit<F>): number {
    return this._number / this.toNumberOfThisUnit(value, unit);
  }

  public abs<OU extends Unit<F>>(out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public abs(): this;
  public abs(out?: NumberUnit<F, Unit<F>>): unknown {
    const absolute = Math.abs(this._number);
    return out ? out.set(absolute, this._unit) : ((this._number = absolute), this);
  }

  public asUnit(unit: Unit<F>): number {
    return this._unit.convertTo(this._number, unit);
  }

  public compare(value: NumberUnitInterface<F>): number;
  public compare(value: number, unit?: Unit<F>): number;
  public compare(value: NumberUnitInterface<F> | number, unit?: Unit<F>): number {
    const other = this.toNumberOfThisUnit(value, unit);
    if (this._number < other) {
      return -1;
    }
    if (this._number > other) {
      return 1;
    }
    return 0;
  }

  public equals(value: NumberUnitInterface<string>): boolean;
  public equals(value: number, unit?: Unit<string>): boolean;
  public equals(value: NumberUnitInterface<string> | number, unit?: Unit<string>): boolean {
    const otherUnit = typeof value === 'number' ? (unit ?? this._unit) : value.unit;
    const otherNumber = typeof value === 'number' ? value : value.number;
    if (!this._unit.canConvert(otherUnit)) {
      return false;
    }
    const converted = this._unit.convertFrom(otherNumber, otherUnit as Unit<F>);
    return this._number === converted || (Number.isNaN(this._number) && Number.isNaN(converted));
  }

  public isNaN(): boolean {
    return Number.isNaN(this._number);
  }

  public copy(): NumberUnit<F, U> {
    return new NumberUnit<F, U>(this._number, this._unit);
  }
}

export class NumberUnitReadOnly<F extends string, U extends Unit<F> = Unit<F>> implements NumberUnitInterface<F, U> {
  public constructor(private readonly source: NumberUnit<F, U>) {}

  public get number(): number {
    return this.source.number;
  }

  public get unit(): U {
    return this.source.unit;
  }

  public add<OU extends Unit<F>>(value: NumberUnitInterface<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public add<OU extends Unit<F>>(value: number, unit: Unit<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public add(value: NumberUnitInterface<F> | number, arg2: unknown, arg3?: unknown): NumberUnit<F, Unit<F>> {
    return (this.source as any).add(value, arg2, arg3);
  }

  public subtract<OU extends Unit<F>>(value: NumberUnitInterface<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public subtract<OU extends Unit<F>>(value: number, unit: Unit<F>, out: NumberUnit<F, OU>): NumberUnit<F, OU>;
  public subtract(value: NumberUnitInterface<F> | number, arg2: unknown, arg3?: unknown): NumberUnit<F, Unit<F>> {
    return (this.source as any).subtract(value, arg2, arg3);
  }

  public scale<OU extends Unit<F>>(factor: number, out: NumberUnit<F, OU>): NumberUnit<F, OU> {
    return this.source.scale(factor, out);
  }

  public ratio(value: NumberUnitInterface<F>): number;
  public ratio(value: number, unit: Unit<F>): number;
  public ratio(value: NumberUnitInterface<F> | number, unit?: Unit<F>): number {
    return (this.source as any).ratio(value, unit);
  }

  public abs<OU extends Unit<F>>(out: NumberUnit<F, OU>): NumberUnit<F, OU> {
    return this.source.abs(out);
  }

  public asUnit(unit: Unit<F>): number {
    return this.source.asUnit(unit);
  }

  public compare(value: NumberUnitInterface<F>): number;
  public compare(value: number, unit?: Unit<F>): number;
  public compare(value: NumberUnitInterface<F> | number, unit?: Unit<F>): number {
    return (this.source as any).compare(value, unit);
  }

  public equals(value: NumberUnitInterface<string>): boolean;
  public equals(value: number, unit?: Unit<string>): boolean;
  public equals(value: NumberUnitInterface<string> | number, unit?: Unit<string>): boolean {
    return (this.source as any).equals(value, unit);
  }

  public isNaN(): boolean {
    return this.source.isNaN();
  }

  public copy(): NumberUnit<F, U> {
    return this.source.copy();
  }
}

const GALLON_LITERS = 3.785411784;
const POUND_KG = 0.45359237;
const STANDARD_GRAVITY = 9.80665;

export class UnitType {
  public static readonly METER = new SimpleUnit(UnitFamily.Distance, 'meter', 1);
  public static readonly CENTIMETER = new SimpleUnit(UnitFamily.Distance, 'centimeter', 0.01);
  public static readonly KILOMETER = new SimpleUnit(UnitFamily.Distance, 'kilometer', 1000);
  public static readonly INCH = new SimpleUnit(UnitFamily.Distance, 'inch', 0.0254);
  public static readonly FOOT = new SimpleUnit(UnitFamily.Distance, 'foot', 0.3048);
  public static readonly MILE = new SimpleUnit(UnitFamily.Distance, 'mile', 1609.344);
  public static readonly NMILE = new SimpleUnit(UnitFamily.Distance, 'nautical mile', 1852);
  public static readonly GA_RADIAN = new SimpleUnit(UnitFamily.Distance, 'great arc radian', 6378100);
  private static readonly G_METER = new SimpleUnit(UnitFamily.Distance, 'g-meter', STANDARD_GRAVITY);

  public static readonly RADIAN = new SimpleUnit(UnitFamily.Angle, 'radian', 1);
  public static readonly DEGREE = new SimpleUnit(UnitFamily.Angle, 'degree', Math.PI / 180);
  public static readonly ARC_MIN = new SimpleUnit(UnitFamily.Angle, 'arc minute', Math.PI / 10800);
  public static readonly ARC_SEC = new SimpleUnit(UnitFamily.Angle, 'arc second', Math.PI / 648000);

  public static readonly MILLISECOND = new SimpleUnit(UnitFamily.Duration, 'millisecond', 0.001);
  public static readonly SECOND = new SimpleUnit(UnitFamily.Duration, 'second', 1);
  public static readonly MINUTE = new SimpleUnit(UnitFamily.Duration, 'minute', 60);
  public static readonly HOUR = new SimpleUnit(UnitFamily.Duration, 'hour', 3600);

  public static readonly KILOGRAM = new SimpleUnit(UnitFamily.Weight, 'kilogram', 1);
  public static readonly POUND = new SimpleUnit(UnitFamily.Weight, 'pound', POUND_KG);
  public static readonly SLUG = new SimpleUnit(UnitFamily.Weight, 'slug', 14.5939029372);
  public static readonly TON = new SimpleUnit(UnitFamily.Weight, 'ton', 907.18474);
  public static readonly TONNE = new SimpleUnit(UnitFamily.Weight, 'tonne', 1000);

  public static readonly LITER = new SimpleUnit(UnitFamily.Volume, 'liter', 1);
  public static readonly GALLON = new SimpleUnit(UnitFamily.Volume, 'gallon', GALLON_LITERS);
  public static readonly IMP_GALLON = new SimpleUnit(UnitFamily.Volume, 'imperial gallon', 4.54609);

  public static readonly HPA = new SimpleUnit(UnitFamily.Pressure, 'hectopascal', 1);
  public static readonly MB = new SimpleUnit(UnitFamily.Pressure, 'millibar', 1);
  public static readonly ATM = new SimpleUnit(UnitFamily.Pressure, 'atmosphere', 1013.25);
  public static readonly IN_HG = new SimpleUnit(UnitFamily.Pressure, 'inch of mercury', 33.8638866667);
  public static readonly MM_HG = new SimpleUnit(UnitFamily.Pressure, 'millimeter of mercury', 1.33322387415);
  public static readonly PSI = new SimpleUnit(UnitFamily.Pressure, 'pound per square inch', 68.9475729318);

  public static readonly KELVIN = new SimpleUnit(UnitFamily.Temperature, 'kelvin', 1, 0);
  public static readonly CELSIUS = new SimpleUnit(UnitFamily.Temperature, 'celsius', 1, 273.15);
  public static readonly FAHRENHEIT = new SimpleUnit(UnitFamily.Temperature, 'fahrenheit', 5 / 9, 459.67 * (5 / 9));
  public static readonly RANKINE = new SimpleUnit(UnitFamily.Temperature, 'rankine', 5 / 9, 0);

  public static readonly DELTA_CELSIUS = new SimpleUnit(UnitFamily.TemperatureDelta, 'delta celsius', 1);
  public static readonly DELTA_FAHRENHEIT = new SimpleUnit(UnitFamily.TemperatureDelta, 'delta fahrenheit', 5 / 9);

  public static readonly KNOT = new CompoundUnit(UnitFamily.Speed, [UnitType.NMILE], [UnitType.HOUR], 'knot');
  public static readonly KPH = new CompoundUnit(UnitFamily.Speed, [UnitType.KILOMETER], [UnitType.HOUR], 'kilometer per hour');
  public static readonly MPH = new CompoundUnit(UnitFamily.Speed, [UnitType.MILE], [UnitType.HOUR], 'mile per hour');
  public static readonly MPM = new CompoundUnit(UnitFamily.Speed, [UnitType.METER], [UnitType.MINUTE], 'meter per minute');
  public static readonly MPS = new CompoundUnit(UnitFamily.Speed, [UnitType.METER], [UnitType.SECOND], 'meter per second');
  public static readonly FPM = new CompoundUnit(UnitFamily.Speed, [UnitType.FOOT], [UnitType.MINUTE], 'foot per minute');
  public static readonly FPS = new CompoundUnit(UnitFamily.Speed, [UnitType.FOOT], [UnitType.SECOND], 'foot per second');

  public static readonly MPM_PER_SEC = new CompoundUnit(
    UnitFamily.Acceleration,
    [UnitType.METER],
    [UnitType.MINUTE, UnitType.SECOND],
    'meter per minute per second',
  );
  public static readonly MPS_PER_SEC = new CompoundUnit(
    UnitFamily.Acceleration,
    [UnitType.METER],
    [UnitType.SECOND, UnitType.SECOND],
    'meter per second per second',
  );
  public static readonly FPM_PER_SEC = new CompoundUnit(
    UnitFamily.Acceleration,
    [UnitType.FOOT],
    [UnitType.MINUTE, UnitType.SECOND],
    'foot per minute per second',
  );
  public static readonly FPS_PER_SEC = new CompoundUnit(
    UnitFamily.Acceleration,
    [UnitType.FOOT],
    [UnitType.SECOND, UnitType.SECOND],
    'foot per second per second',
  );
  public static readonly KNOT_PER_SEC = new CompoundUnit(
    UnitFamily.Acceleration,
    [UnitType.NMILE],
    [UnitType.HOUR, UnitType.SECOND],
    'knot per second',
  );
  public static readonly G_ACCEL = new CompoundUnit(
    UnitFamily.Acceleration,
    [UnitType.G_METER],
    [UnitType.SECOND, UnitType.SECOND],
    'g-force',
  );

  public static readonly KGH = new CompoundUnit(UnitFamily.WeightFlux, [UnitType.KILOGRAM], [UnitType.HOUR], 'kilogram per hour');
  public static readonly PPH = new CompoundUnit(UnitFamily.WeightFlux, [UnitType.POUND], [UnitType.HOUR], 'pound per hour');

  public static readonly SLUG_PER_FT3 = new CompoundUnit(
    UnitFamily.Density,
    [UnitType.SLUG],
    [UnitType.FOOT, UnitType.FOOT, UnitType.FOOT],
    'slug per cubic foot',
  );
  public static readonly KG_PER_M3 = new CompoundUnit(
    UnitFamily.Density,
    [UnitType.KILOGRAM],
    [UnitType.METER, UnitType.METER, UnitType.METER],
    'kilogram per cubic meter',
  );

  public static readonly NEWTON = new CompoundUnit(
    UnitFamily.Force,
    [UnitType.KILOGRAM, UnitType.METER],
    [UnitType.SECOND, UnitType.SECOND],
    'newton',
  );
  public static readonly POUND_FORCE = new CompoundUnit(
    UnitFamily.Force,
    [UnitType.SLUG, UnitType.FOOT],
    [UnitType.SECOND, UnitType.SECOND],
    'pound force',
  );

  public static readonly FOOT_PER_NMILE = new CompoundUnit(
    UnitFamily.DistanceRatio,
    [UnitType.FOOT],
    [UnitType.NMILE],
    'foot per nautical mile',
  );

  public static readonly METER_SQ = new CompoundUnit(UnitFamily.Area, [UnitType.METER, UnitType.METER], [], 'square meter');
  public static readonly CENTIMETER_SQ = new CompoundUnit(
    UnitFamily.Area,
    [UnitType.CENTIMETER, UnitType.CENTIMETER],
    [],
    'square centimeter',
  );
  public static readonly KILOMETER_SQ = new CompoundUnit(
    UnitFamily.Area,
    [UnitType.KILOMETER, UnitType.KILOMETER],
    [],
    'square kilometer',
  );
  public static readonly INCH_SQ = new CompoundUnit(UnitFamily.Area, [UnitType.INCH, UnitType.INCH], [], 'square inch');
  public static readonly FOOT_SQ = new CompoundUnit(UnitFamily.Area, [UnitType.FOOT, UnitType.FOOT], [], 'square foot');
  public static readonly MILE_SQ = new CompoundUnit(UnitFamily.Area, [UnitType.MILE, UnitType.MILE], [], 'square mile');
  public static readonly NMILE_SQ = new CompoundUnit(
    UnitFamily.Area,
    [UnitType.NMILE, UnitType.NMILE],
    [],
    'square nautical mile',
  );

  public static createFuelWeightUnit(
    density: number,
    ...defs: { volumeUnit: Unit<UnitFamily.Volume>; name?: string }[]
  ): CompoundableUnit<UnitFamily.Weight>[] {
    return defs.map((def) => {
      const volumeScale = (def.volumeUnit as unknown as { scaleFactor: number }).scaleFactor;
      const gallonsPerUnit = volumeScale / UnitType.GALLON.scaleFactor;
      return new SimpleUnit(
        UnitFamily.Weight,
        def.name ?? `${def.volumeUnit.name} fuel (${density} pounds per gallon)`,
        density * UnitType.POUND.scaleFactor * gallonsPerUnit,
      );
    });
  }

  private static readonly GENERIC_FUEL = UnitType.createFuelWeightUnit(
    6.7,
    { volumeUnit: UnitType.LITER, name: 'liter fuel' },
    { volumeUnit: UnitType.GALLON, name: 'gallon fuel' },
    { volumeUnit: UnitType.IMP_GALLON, name: 'imperial gallon fuel' },
  );
  public static readonly LITER_FUEL = UnitType.GENERIC_FUEL[0] as SimpleUnit<UnitFamily.Weight>;
  public static readonly GALLON_FUEL = UnitType.GENERIC_FUEL[1] as SimpleUnit<UnitFamily.Weight>;
  public static readonly IMP_GALLON_FUEL = UnitType.GENERIC_FUEL[2] as SimpleUnit<UnitFamily.Weight>;

  private static readonly JET_A_FUEL = UnitType.createFuelWeightUnit(
    6.7,
    { volumeUnit: UnitType.LITER, name: 'liter Jet-A fuel' },
    { volumeUnit: UnitType.GALLON, name: 'gallon Jet-A fuel' },
    { volumeUnit: UnitType.IMP_GALLON, name: 'imperial gallon Jet-A fuel' },
  );
  public static readonly LITER_JET_A_FUEL = UnitType.JET_A_FUEL[0] as SimpleUnit<UnitFamily.Weight>;
  public static readonly GALLON_JET_A_FUEL = UnitType.JET_A_FUEL[1] as SimpleUnit<UnitFamily.Weight>;
  public static readonly IMP_GALLON_JET_A_FUEL = UnitType.JET_A_FUEL[2] as SimpleUnit<UnitFamily.Weight>;

  private static readonly LL100_FUEL = UnitType.createFuelWeightUnit(
    6.0,
    { volumeUnit: UnitType.LITER, name: 'liter 100LL fuel' },
    { volumeUnit: UnitType.GALLON, name: 'gallon 100LL fuel' },
    { volumeUnit: UnitType.IMP_GALLON, name: 'imperial gallon 100LL fuel' },
  );
  public static readonly LITER_100LL_FUEL = UnitType.LL100_FUEL[0] as SimpleUnit<UnitFamily.Weight>;
  public static readonly GALLON_100LL_FUEL = UnitType.LL100_FUEL[1] as SimpleUnit<UnitFamily.Weight>;
  public static readonly IMP_GALLON_100LL_FUEL = UnitType.LL100_FUEL[2] as SimpleUnit<UnitFamily.Weight>;

  private static readonly AUTOGAS_FUEL = UnitType.createFuelWeightUnit(
    6.0,
    { volumeUnit: UnitType.LITER, name: 'liter autogas fuel' },
    { volumeUnit: UnitType.GALLON, name: 'gallon autogas fuel' },
    { volumeUnit: UnitType.IMP_GALLON, name: 'imperial gallon autogas fuel' },
  );
  public static readonly LITER_AUTOGAS_FUEL = UnitType.AUTOGAS_FUEL[0] as SimpleUnit<UnitFamily.Weight>;
  public static readonly GALLON_AUTOGAS_FUEL = UnitType.AUTOGAS_FUEL[1] as SimpleUnit<UnitFamily.Weight>;
  public static readonly IMP_GALLON_AUTOGAS_FUEL = UnitType.AUTOGAS_FUEL[2] as SimpleUnit<UnitFamily.Weight>;

  public static readonly LPH_FUEL = new CompoundUnit(UnitFamily.WeightFlux, [UnitType.LITER_FUEL], [UnitType.HOUR], 'liter fuel per hour');
  public static readonly GPH_FUEL = new CompoundUnit(UnitFamily.WeightFlux, [UnitType.GALLON_FUEL], [UnitType.HOUR], 'gallon fuel per hour');
  public static readonly IGPH_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.IMP_GALLON_FUEL],
    [UnitType.HOUR],
    'imperial gallon fuel per hour',
  );
  public static readonly LPH_JET_A_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.LITER_JET_A_FUEL],
    [UnitType.HOUR],
    'liter Jet-A fuel per hour',
  );
  public static readonly GPH_JET_A_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.GALLON_JET_A_FUEL],
    [UnitType.HOUR],
    'gallon Jet-A fuel per hour',
  );
  public static readonly IGPH_JET_A_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.IMP_GALLON_JET_A_FUEL],
    [UnitType.HOUR],
    'imperial gallon Jet-A fuel per hour',
  );
  public static readonly LPH_100LL_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.LITER_100LL_FUEL],
    [UnitType.HOUR],
    'liter 100LL fuel per hour',
  );
  public static readonly GPH_100LL_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.GALLON_100LL_FUEL],
    [UnitType.HOUR],
    'gallon 100LL fuel per hour',
  );
  public static readonly IGPH_100LL_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.IMP_GALLON_100LL_FUEL],
    [UnitType.HOUR],
    'imperial gallon 100LL fuel per hour',
  );
  public static readonly LPH_AUTOGAS_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.LITER_AUTOGAS_FUEL],
    [UnitType.HOUR],
    'liter autogas fuel per hour',
  );
  public static readonly GPH_AUTOGAS_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.GALLON_AUTOGAS_FUEL],
    [UnitType.HOUR],
    'gallon autogas fuel per hour',
  );
  public static readonly IGPH_AUTOGAS_FUEL = new CompoundUnit(
    UnitFamily.WeightFlux,
    [UnitType.IMP_GALLON_AUTOGAS_FUEL],
    [UnitType.HOUR],
    'imperial gallon autogas fuel per hour',
  );

  public static readonly MILE_PER_GALLON_FUEL = new CompoundUnit(
    UnitFamily.DistancePerWeight,
    [UnitType.MILE],
    [UnitType.GALLON_FUEL],
    'mile per gallon fuel',
  );
  public static readonly NMILE_PER_GALLON_FUEL = new CompoundUnit(
    UnitFamily.DistancePerWeight,
    [UnitType.NMILE],
    [UnitType.GALLON_FUEL],
    'nautical mile per gallon fuel',
  );
  public static readonly MILE_PER_GALLON_JET_A_FUEL = new CompoundUnit(
    UnitFamily.DistancePerWeight,
    [UnitType.MILE],
    [UnitType.GALLON_JET_A_FUEL],
    'mile per gallon Jet-A fuel',
  );
  public static readonly NMILE_PER_GALLON_JET_A_FUEL = new CompoundUnit(
    UnitFamily.DistancePerWeight,
    [UnitType.NMILE],
    [UnitType.GALLON_JET_A_FUEL],
    'nautical mile per gallon Jet-A fuel',
  );
  public static readonly MILE_PER_GALLON_100LL_FUEL = new CompoundUnit(
    UnitFamily.DistancePerWeight,
    [UnitType.MILE],
    [UnitType.GALLON_100LL_FUEL],
    'mile per gallon 100LL fuel',
  );
  public static readonly NMILE_PER_GALLON_100LL_FUEL = new CompoundUnit(
    UnitFamily.DistancePerWeight,
    [UnitType.NMILE],
    [UnitType.GALLON_100LL_FUEL],
    'nautical mile per gallon 100LL fuel',
  );
  public static readonly MILE_PER_GALLON_AUTOGAS_FUEL = new CompoundUnit(
    UnitFamily.DistancePerWeight,
    [UnitType.MILE],
    [UnitType.GALLON_AUTOGAS_FUEL],
    'mile per gallon autogas fuel',
  );
  public static readonly NMILE_PER_GALLON_AUTOGAS_FUEL = new CompoundUnit(
    UnitFamily.DistancePerWeight,
    [UnitType.NMILE],
    [UnitType.GALLON_AUTOGAS_FUEL],
    'nautical mile per gallon autogas fuel',
  );
}

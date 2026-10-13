export type KindOfMeasure<U extends Measure<string>> = U extends Measure<infer T> ? T : never;

export type MeasureOfQuantity<N extends QuantityLike<string>> = N extends QuantityLike<string, infer U>
  ? U
  : never;

export interface Measure<F extends string> {
  readonly family: F;
  readonly name: string;
  canConvert(otherUnit: Measure<string>): boolean;
  convertTo(value: number, toUnit: Measure<F>): number;
  convertFrom(value: number, fromUnit: Measure<F>): number;
  createNumber(value: number): Quantity<F, this>;
  equals(other: Measure<string>): boolean;
}

export interface CompoundableMeasure<F extends string> extends Measure<F> {
  readonly scaleFactor: number;
}

export interface QuantityLike<F extends string, U extends Measure<F> = Measure<F>> {
  number: number;
  unit: U;
  add<OU extends Measure<F>>(value: QuantityLike<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  add<OU extends Measure<F>>(value: number, unit: Measure<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  subtract<OU extends Measure<F>>(value: QuantityLike<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  subtract<OU extends Measure<F>>(value: number, unit: Measure<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  scale<OU extends Measure<F>>(factor: number, out: Quantity<F, OU>): Quantity<F, OU>;
  ratio(value: QuantityLike<F>): number;
  ratio(value: number, unit: Measure<F>): number;
  abs<OU extends Measure<F>>(out: Quantity<F, OU>): Quantity<F, OU>;
  asUnit(unit: Measure<F>): number;
  compare(value: QuantityLike<F>): number;
  compare(value: number, unit?: Measure<F>): number;
  equals(value: QuantityLike<string>): boolean;
  equals(value: number, unit?: Measure<string>): boolean;
  isNaN(): boolean;
  copy(): Quantity<F, U>;
}

export abstract class AbstractMeasure<F extends string> implements Measure<F> {
  public abstract readonly family: F;

  public constructor(public readonly name: string) {}

  public canConvert(otherUnit: Measure<string>): boolean {
    return this.family === otherUnit.family;
  }

  public abstract convertTo(value: number, toUnit: Measure<F>): number;

  public abstract convertFrom(value: number, fromUnit: Measure<F>): number;

  public createNumber(value: number): Quantity<F, this> {
    return new Quantity<F, this>(value, this);
  }

  public equals(other: Measure<string>): boolean {
    return this.family === other.family && this.name === other.name;
  }
}

export class SimpleMeasure<F extends string> extends AbstractMeasure<F> implements CompoundableMeasure<F> {
  public constructor(
    public readonly family: F,
    name: string,
    public readonly scaleFactor: number,
    public readonly zeroOffset: number = 0,
  ) {
    super(name);
  }

  public canConvert(otherUnit: Measure<string>): boolean {
    return this.family === otherUnit.family;
  }

  public convertTo(value: number, toUnit: Measure<F>): number {
    if (!this.canConvert(toUnit)) {
      throw new Error(`Invalid conversion from ${this.name} to ${toUnit.name}.`);
    }
    const target = toUnit as unknown as { scaleFactor: number; zeroOffset?: number };
    return (value * this.scaleFactor + this.zeroOffset - (target.zeroOffset ?? 0)) / target.scaleFactor;
  }

  public convertFrom(value: number, fromUnit: Measure<F>): number {
    if (!this.canConvert(fromUnit)) {
      throw new Error(`Invalid conversion from ${fromUnit.name} to ${this.name}.`);
    }
    const source = fromUnit as unknown as { scaleFactor: number; zeroOffset?: number };
    return (value * source.scaleFactor + (source.zeroOffset ?? 0) - this.zeroOffset) / this.scaleFactor;
  }
}

export class CompoundMeasure<F extends string> extends AbstractMeasure<F> implements CompoundableMeasure<F> {
  public readonly scaleFactor: number;

  public constructor(
    public readonly family: F,
    private readonly numerator: CompoundableMeasure<string>[],
    private readonly denominator: CompoundableMeasure<string>[],
    name?: string,
  ) {
    super(name ?? CompoundMeasure.generateName(numerator, denominator));
    this.scaleFactor = this.computeScaleFactor();
  }

  private static generateName(numerator: CompoundableMeasure<string>[], denominator: CompoundableMeasure<string>[]): string {
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

  public canConvert(otherUnit: Measure<string>): boolean {
    return this.family === otherUnit.family;
  }

  public convertTo(value: number, toUnit: Measure<F>): number {
    if (!this.canConvert(toUnit)) {
      throw new Error(`Invalid conversion from ${this.name} to ${toUnit.name}.`);
    }
    return (value * this.scaleFactor) / (toUnit as unknown as { scaleFactor: number }).scaleFactor;
  }

  public convertFrom(value: number, fromUnit: Measure<F>): number {
    if (!this.canConvert(fromUnit)) {
      throw new Error(`Invalid conversion from ${fromUnit.name} to ${this.name}.`);
    }
    return (value * (fromUnit as unknown as { scaleFactor: number }).scaleFactor) / this.scaleFactor;
  }
}

export enum MeasureKind {
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

export class Quantity<F extends string, U extends Measure<F> = Measure<F>> implements QuantityLike<F, U> {
  public readonly readonly: QuantityView<F, U>;

  public constructor(
    private _number: number,
    private readonly _unit: U,
  ) {
    this.readonly = new QuantityView(this);
  }

  public get number(): number {
    return this._number;
  }

  public get unit(): U {
    return this._unit;
  }

  private toNumberOfThisUnit(value: QuantityLike<F> | number, unit?: Measure<F>): number {
    if (typeof value === 'number') {
      return unit === undefined ? value : this._unit.convertFrom(value, unit);
    }
    return this._unit.convertFrom(value.number, value.unit);
  }

  public set(value: QuantityLike<F>): this;
  public set(value: number, unit?: Measure<F>): this;
  public set(value: QuantityLike<F> | number, unit?: Measure<F>): this {
    if (typeof value !== 'number') {
      if (!this._unit.canConvert(value.unit)) {
        return undefined as unknown as this;
      }
    }
    this._number = this.toNumberOfThisUnit(value, unit);
    return this;
  }

  public add<OU extends Measure<F>>(value: QuantityLike<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  public add(value: QuantityLike<F>): this;
  public add<OU extends Measure<F>>(value: number, unit: Measure<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  public add(value: number, unit: Measure<F>): this;
  public add(
    value: QuantityLike<F> | number,
    arg2?: Quantity<F, Measure<F>> | Measure<F>,
    arg3?: Quantity<F, Measure<F>>,
  ): unknown {
    const { unit, out } = Quantity.resolveArguments(value, arg2, arg3);
    const sum = this._number + this.toNumberOfThisUnit(value, unit);
    return out ? out.set(sum, this._unit) : ((this._number = sum), this);
  }

  public subtract<OU extends Measure<F>>(value: QuantityLike<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  public subtract(value: QuantityLike<F>): this;
  public subtract<OU extends Measure<F>>(value: number, unit: Measure<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  public subtract(value: number, unit: Measure<F>): this;
  public subtract(
    value: QuantityLike<F> | number,
    arg2?: Quantity<F, Measure<F>> | Measure<F>,
    arg3?: Quantity<F, Measure<F>>,
  ): unknown {
    const { unit, out } = Quantity.resolveArguments(value, arg2, arg3);
    const difference = this._number - this.toNumberOfThisUnit(value, unit);
    return out ? out.set(difference, this._unit) : ((this._number = difference), this);
  }

  private static resolveArguments<F extends string>(
    value: QuantityLike<F> | number,
    arg2?: Quantity<F, Measure<F>> | Measure<F>,
    arg3?: Quantity<F, Measure<F>>,
  ): { unit: Measure<F> | undefined; out: Quantity<F, Measure<F>> | undefined } {
    if (typeof value === 'number') {
      return { unit: arg2 as Measure<F> | undefined, out: arg3 };
    }
    return { unit: undefined, out: arg2 as Quantity<F, Measure<F>> | undefined };
  }

  public scale<OU extends Measure<F>>(factor: number, out: Quantity<F, OU>): Quantity<F, OU>;
  public scale(factor: number): this;
  public scale(factor: number, out?: Quantity<F, Measure<F>>): unknown {
    const scaled = this._number * factor;
    return out ? out.set(scaled, this._unit) : ((this._number = scaled), this);
  }

  public ratio(value: QuantityLike<F>): number;
  public ratio(value: number, unit: Measure<F>): number;
  public ratio(value: QuantityLike<F> | number, unit?: Measure<F>): number {
    return this._number / this.toNumberOfThisUnit(value, unit);
  }

  public abs<OU extends Measure<F>>(out: Quantity<F, OU>): Quantity<F, OU>;
  public abs(): this;
  public abs(out?: Quantity<F, Measure<F>>): unknown {
    const absolute = Math.abs(this._number);
    return out ? out.set(absolute, this._unit) : ((this._number = absolute), this);
  }

  public asUnit(unit: Measure<F>): number {
    return this._unit.convertTo(this._number, unit);
  }

  public compare(value: QuantityLike<F>): number;
  public compare(value: number, unit?: Measure<F>): number;
  public compare(value: QuantityLike<F> | number, unit?: Measure<F>): number {
    const other = this.toNumberOfThisUnit(value, unit);
    if (this._number < other) {
      return -1;
    }
    if (this._number > other) {
      return 1;
    }
    return 0;
  }

  public equals(value: QuantityLike<string>): boolean;
  public equals(value: number, unit?: Measure<string>): boolean;
  public equals(value: QuantityLike<string> | number, unit?: Measure<string>): boolean {
    const otherUnit = typeof value === 'number' ? (unit ?? this._unit) : value.unit;
    const otherNumber = typeof value === 'number' ? value : value.number;
    if (!this._unit.canConvert(otherUnit)) {
      return false;
    }
    const converted = this._unit.convertFrom(otherNumber, otherUnit as Measure<F>);
    return this._number === converted || (Number.isNaN(this._number) && Number.isNaN(converted));
  }

  public isNaN(): boolean {
    return Number.isNaN(this._number);
  }

  public copy(): Quantity<F, U> {
    return new Quantity<F, U>(this._number, this._unit);
  }
}

export class QuantityView<F extends string, U extends Measure<F> = Measure<F>> implements QuantityLike<F, U> {
  public constructor(private readonly source: Quantity<F, U>) {}

  public get number(): number {
    return this.source.number;
  }

  public get unit(): U {
    return this.source.unit;
  }

  public add<OU extends Measure<F>>(value: QuantityLike<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  public add<OU extends Measure<F>>(value: number, unit: Measure<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  public add(value: QuantityLike<F> | number, arg2: unknown, arg3?: unknown): Quantity<F, Measure<F>> {
    return (this.source as any).add(value, arg2, arg3);
  }

  public subtract<OU extends Measure<F>>(value: QuantityLike<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  public subtract<OU extends Measure<F>>(value: number, unit: Measure<F>, out: Quantity<F, OU>): Quantity<F, OU>;
  public subtract(value: QuantityLike<F> | number, arg2: unknown, arg3?: unknown): Quantity<F, Measure<F>> {
    return (this.source as any).subtract(value, arg2, arg3);
  }

  public scale<OU extends Measure<F>>(factor: number, out: Quantity<F, OU>): Quantity<F, OU> {
    return this.source.scale(factor, out);
  }

  public ratio(value: QuantityLike<F>): number;
  public ratio(value: number, unit: Measure<F>): number;
  public ratio(value: QuantityLike<F> | number, unit?: Measure<F>): number {
    return (this.source as any).ratio(value, unit);
  }

  public abs<OU extends Measure<F>>(out: Quantity<F, OU>): Quantity<F, OU> {
    return this.source.abs(out);
  }

  public asUnit(unit: Measure<F>): number {
    return this.source.asUnit(unit);
  }

  public compare(value: QuantityLike<F>): number;
  public compare(value: number, unit?: Measure<F>): number;
  public compare(value: QuantityLike<F> | number, unit?: Measure<F>): number {
    return (this.source as any).compare(value, unit);
  }

  public equals(value: QuantityLike<string>): boolean;
  public equals(value: number, unit?: Measure<string>): boolean;
  public equals(value: QuantityLike<string> | number, unit?: Measure<string>): boolean {
    return (this.source as any).equals(value, unit);
  }

  public isNaN(): boolean {
    return this.source.isNaN();
  }

  public copy(): Quantity<F, U> {
    return this.source.copy();
  }
}

const GALLON_LITERS = 3.785411784;
const POUND_KG = 0.45359237;
const STANDARD_GRAVITY = 9.80665;

export class Units {
  public static readonly METER = new SimpleMeasure(MeasureKind.Distance, 'meter', 1);
  public static readonly CENTIMETER = new SimpleMeasure(MeasureKind.Distance, 'centimeter', 0.01);
  public static readonly KILOMETER = new SimpleMeasure(MeasureKind.Distance, 'kilometer', 1000);
  public static readonly INCH = new SimpleMeasure(MeasureKind.Distance, 'inch', 0.0254);
  public static readonly FOOT = new SimpleMeasure(MeasureKind.Distance, 'foot', 0.3048);
  public static readonly MILE = new SimpleMeasure(MeasureKind.Distance, 'mile', 1609.344);
  public static readonly NMILE = new SimpleMeasure(MeasureKind.Distance, 'nautical mile', 1852);
  public static readonly GA_RADIAN = new SimpleMeasure(MeasureKind.Distance, 'great arc radian', 6378100);
  private static readonly G_METER = new SimpleMeasure(MeasureKind.Distance, 'g-meter', STANDARD_GRAVITY);

  public static readonly RADIAN = new SimpleMeasure(MeasureKind.Angle, 'radian', 1);
  public static readonly DEGREE = new SimpleMeasure(MeasureKind.Angle, 'degree', Math.PI / 180);
  public static readonly ARC_MIN = new SimpleMeasure(MeasureKind.Angle, 'arc minute', Math.PI / 10800);
  public static readonly ARC_SEC = new SimpleMeasure(MeasureKind.Angle, 'arc second', Math.PI / 648000);

  public static readonly MILLISECOND = new SimpleMeasure(MeasureKind.Duration, 'millisecond', 0.001);
  public static readonly SECOND = new SimpleMeasure(MeasureKind.Duration, 'second', 1);
  public static readonly MINUTE = new SimpleMeasure(MeasureKind.Duration, 'minute', 60);
  public static readonly HOUR = new SimpleMeasure(MeasureKind.Duration, 'hour', 3600);

  public static readonly KILOGRAM = new SimpleMeasure(MeasureKind.Weight, 'kilogram', 1);
  public static readonly POUND = new SimpleMeasure(MeasureKind.Weight, 'pound', POUND_KG);
  public static readonly SLUG = new SimpleMeasure(MeasureKind.Weight, 'slug', 14.5939029372);
  public static readonly TON = new SimpleMeasure(MeasureKind.Weight, 'ton', 907.18474);
  public static readonly TONNE = new SimpleMeasure(MeasureKind.Weight, 'tonne', 1000);

  public static readonly LITER = new SimpleMeasure(MeasureKind.Volume, 'liter', 1);
  public static readonly GALLON = new SimpleMeasure(MeasureKind.Volume, 'gallon', GALLON_LITERS);
  public static readonly IMP_GALLON = new SimpleMeasure(MeasureKind.Volume, 'imperial gallon', 4.54609);

  public static readonly HPA = new SimpleMeasure(MeasureKind.Pressure, 'hectopascal', 1);
  public static readonly MB = new SimpleMeasure(MeasureKind.Pressure, 'millibar', 1);
  public static readonly ATM = new SimpleMeasure(MeasureKind.Pressure, 'atmosphere', 1013.25);
  public static readonly IN_HG = new SimpleMeasure(MeasureKind.Pressure, 'inch of mercury', 33.8638866667);
  public static readonly MM_HG = new SimpleMeasure(MeasureKind.Pressure, 'millimeter of mercury', 1.33322387415);
  public static readonly PSI = new SimpleMeasure(MeasureKind.Pressure, 'pound per square inch', 68.9475729318);

  public static readonly KELVIN = new SimpleMeasure(MeasureKind.Temperature, 'kelvin', 1, 0);
  public static readonly CELSIUS = new SimpleMeasure(MeasureKind.Temperature, 'celsius', 1, 273.15);
  public static readonly FAHRENHEIT = new SimpleMeasure(MeasureKind.Temperature, 'fahrenheit', 5 / 9, 459.67 * (5 / 9));
  public static readonly RANKINE = new SimpleMeasure(MeasureKind.Temperature, 'rankine', 5 / 9, 0);

  public static readonly DELTA_CELSIUS = new SimpleMeasure(MeasureKind.TemperatureDelta, 'delta celsius', 1);
  public static readonly DELTA_FAHRENHEIT = new SimpleMeasure(MeasureKind.TemperatureDelta, 'delta fahrenheit', 5 / 9);

  public static readonly KNOT = new CompoundMeasure(MeasureKind.Speed, [Units.NMILE], [Units.HOUR], 'knot');
  public static readonly KPH = new CompoundMeasure(MeasureKind.Speed, [Units.KILOMETER], [Units.HOUR], 'kilometer per hour');
  public static readonly MPH = new CompoundMeasure(MeasureKind.Speed, [Units.MILE], [Units.HOUR], 'mile per hour');
  public static readonly MPM = new CompoundMeasure(MeasureKind.Speed, [Units.METER], [Units.MINUTE], 'meter per minute');
  public static readonly MPS = new CompoundMeasure(MeasureKind.Speed, [Units.METER], [Units.SECOND], 'meter per second');
  public static readonly FPM = new CompoundMeasure(MeasureKind.Speed, [Units.FOOT], [Units.MINUTE], 'foot per minute');
  public static readonly FPS = new CompoundMeasure(MeasureKind.Speed, [Units.FOOT], [Units.SECOND], 'foot per second');

  public static readonly MPM_PER_SEC = new CompoundMeasure(
    MeasureKind.Acceleration,
    [Units.METER],
    [Units.MINUTE, Units.SECOND],
    'meter per minute per second',
  );
  public static readonly MPS_PER_SEC = new CompoundMeasure(
    MeasureKind.Acceleration,
    [Units.METER],
    [Units.SECOND, Units.SECOND],
    'meter per second per second',
  );
  public static readonly FPM_PER_SEC = new CompoundMeasure(
    MeasureKind.Acceleration,
    [Units.FOOT],
    [Units.MINUTE, Units.SECOND],
    'foot per minute per second',
  );
  public static readonly FPS_PER_SEC = new CompoundMeasure(
    MeasureKind.Acceleration,
    [Units.FOOT],
    [Units.SECOND, Units.SECOND],
    'foot per second per second',
  );
  public static readonly KNOT_PER_SEC = new CompoundMeasure(
    MeasureKind.Acceleration,
    [Units.NMILE],
    [Units.HOUR, Units.SECOND],
    'knot per second',
  );
  public static readonly G_ACCEL = new CompoundMeasure(
    MeasureKind.Acceleration,
    [Units.G_METER],
    [Units.SECOND, Units.SECOND],
    'g-force',
  );

  public static readonly KGH = new CompoundMeasure(MeasureKind.WeightFlux, [Units.KILOGRAM], [Units.HOUR], 'kilogram per hour');
  public static readonly PPH = new CompoundMeasure(MeasureKind.WeightFlux, [Units.POUND], [Units.HOUR], 'pound per hour');

  public static readonly SLUG_PER_FT3 = new CompoundMeasure(
    MeasureKind.Density,
    [Units.SLUG],
    [Units.FOOT, Units.FOOT, Units.FOOT],
    'slug per cubic foot',
  );
  public static readonly KG_PER_M3 = new CompoundMeasure(
    MeasureKind.Density,
    [Units.KILOGRAM],
    [Units.METER, Units.METER, Units.METER],
    'kilogram per cubic meter',
  );

  public static readonly NEWTON = new CompoundMeasure(
    MeasureKind.Force,
    [Units.KILOGRAM, Units.METER],
    [Units.SECOND, Units.SECOND],
    'newton',
  );
  public static readonly POUND_FORCE = new CompoundMeasure(
    MeasureKind.Force,
    [Units.SLUG, Units.FOOT],
    [Units.SECOND, Units.SECOND],
    'pound force',
  );

  public static readonly FOOT_PER_NMILE = new CompoundMeasure(
    MeasureKind.DistanceRatio,
    [Units.FOOT],
    [Units.NMILE],
    'foot per nautical mile',
  );

  public static readonly METER_SQ = new CompoundMeasure(MeasureKind.Area, [Units.METER, Units.METER], [], 'square meter');
  public static readonly CENTIMETER_SQ = new CompoundMeasure(
    MeasureKind.Area,
    [Units.CENTIMETER, Units.CENTIMETER],
    [],
    'square centimeter',
  );
  public static readonly KILOMETER_SQ = new CompoundMeasure(
    MeasureKind.Area,
    [Units.KILOMETER, Units.KILOMETER],
    [],
    'square kilometer',
  );
  public static readonly INCH_SQ = new CompoundMeasure(MeasureKind.Area, [Units.INCH, Units.INCH], [], 'square inch');
  public static readonly FOOT_SQ = new CompoundMeasure(MeasureKind.Area, [Units.FOOT, Units.FOOT], [], 'square foot');
  public static readonly MILE_SQ = new CompoundMeasure(MeasureKind.Area, [Units.MILE, Units.MILE], [], 'square mile');
  public static readonly NMILE_SQ = new CompoundMeasure(
    MeasureKind.Area,
    [Units.NMILE, Units.NMILE],
    [],
    'square nautical mile',
  );

  public static createFuelWeightUnit(
    density: number,
    ...defs: { volumeUnit: Measure<MeasureKind.Volume>; name?: string }[]
  ): CompoundableMeasure<MeasureKind.Weight>[] {
    return defs.map((def) => {
      const volumeScale = (def.volumeUnit as unknown as { scaleFactor: number }).scaleFactor;
      const gallonsPerUnit = volumeScale / Units.GALLON.scaleFactor;
      return new SimpleMeasure(
        MeasureKind.Weight,
        def.name ?? `${def.volumeUnit.name} fuel (${density} pounds per gallon)`,
        density * Units.POUND.scaleFactor * gallonsPerUnit,
      );
    });
  }

  private static readonly GENERIC_FUEL = Units.createFuelWeightUnit(
    6.7,
    { volumeUnit: Units.LITER, name: 'liter fuel' },
    { volumeUnit: Units.GALLON, name: 'gallon fuel' },
    { volumeUnit: Units.IMP_GALLON, name: 'imperial gallon fuel' },
  );
  public static readonly LITER_FUEL = Units.GENERIC_FUEL[0] as SimpleMeasure<MeasureKind.Weight>;
  public static readonly GALLON_FUEL = Units.GENERIC_FUEL[1] as SimpleMeasure<MeasureKind.Weight>;
  public static readonly IMP_GALLON_FUEL = Units.GENERIC_FUEL[2] as SimpleMeasure<MeasureKind.Weight>;

  private static readonly JET_A_FUEL = Units.createFuelWeightUnit(
    6.7,
    { volumeUnit: Units.LITER, name: 'liter Jet-A fuel' },
    { volumeUnit: Units.GALLON, name: 'gallon Jet-A fuel' },
    { volumeUnit: Units.IMP_GALLON, name: 'imperial gallon Jet-A fuel' },
  );
  public static readonly LITER_JET_A_FUEL = Units.JET_A_FUEL[0] as SimpleMeasure<MeasureKind.Weight>;
  public static readonly GALLON_JET_A_FUEL = Units.JET_A_FUEL[1] as SimpleMeasure<MeasureKind.Weight>;
  public static readonly IMP_GALLON_JET_A_FUEL = Units.JET_A_FUEL[2] as SimpleMeasure<MeasureKind.Weight>;

  private static readonly LL100_FUEL = Units.createFuelWeightUnit(
    6.0,
    { volumeUnit: Units.LITER, name: 'liter 100LL fuel' },
    { volumeUnit: Units.GALLON, name: 'gallon 100LL fuel' },
    { volumeUnit: Units.IMP_GALLON, name: 'imperial gallon 100LL fuel' },
  );
  public static readonly LITER_100LL_FUEL = Units.LL100_FUEL[0] as SimpleMeasure<MeasureKind.Weight>;
  public static readonly GALLON_100LL_FUEL = Units.LL100_FUEL[1] as SimpleMeasure<MeasureKind.Weight>;
  public static readonly IMP_GALLON_100LL_FUEL = Units.LL100_FUEL[2] as SimpleMeasure<MeasureKind.Weight>;

  private static readonly AUTOGAS_FUEL = Units.createFuelWeightUnit(
    6.0,
    { volumeUnit: Units.LITER, name: 'liter autogas fuel' },
    { volumeUnit: Units.GALLON, name: 'gallon autogas fuel' },
    { volumeUnit: Units.IMP_GALLON, name: 'imperial gallon autogas fuel' },
  );
  public static readonly LITER_AUTOGAS_FUEL = Units.AUTOGAS_FUEL[0] as SimpleMeasure<MeasureKind.Weight>;
  public static readonly GALLON_AUTOGAS_FUEL = Units.AUTOGAS_FUEL[1] as SimpleMeasure<MeasureKind.Weight>;
  public static readonly IMP_GALLON_AUTOGAS_FUEL = Units.AUTOGAS_FUEL[2] as SimpleMeasure<MeasureKind.Weight>;

  public static readonly LPH_FUEL = new CompoundMeasure(MeasureKind.WeightFlux, [Units.LITER_FUEL], [Units.HOUR], 'liter fuel per hour');
  public static readonly GPH_FUEL = new CompoundMeasure(MeasureKind.WeightFlux, [Units.GALLON_FUEL], [Units.HOUR], 'gallon fuel per hour');
  public static readonly IGPH_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.IMP_GALLON_FUEL],
    [Units.HOUR],
    'imperial gallon fuel per hour',
  );
  public static readonly LPH_JET_A_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.LITER_JET_A_FUEL],
    [Units.HOUR],
    'liter Jet-A fuel per hour',
  );
  public static readonly GPH_JET_A_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.GALLON_JET_A_FUEL],
    [Units.HOUR],
    'gallon Jet-A fuel per hour',
  );
  public static readonly IGPH_JET_A_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.IMP_GALLON_JET_A_FUEL],
    [Units.HOUR],
    'imperial gallon Jet-A fuel per hour',
  );
  public static readonly LPH_100LL_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.LITER_100LL_FUEL],
    [Units.HOUR],
    'liter 100LL fuel per hour',
  );
  public static readonly GPH_100LL_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.GALLON_100LL_FUEL],
    [Units.HOUR],
    'gallon 100LL fuel per hour',
  );
  public static readonly IGPH_100LL_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.IMP_GALLON_100LL_FUEL],
    [Units.HOUR],
    'imperial gallon 100LL fuel per hour',
  );
  public static readonly LPH_AUTOGAS_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.LITER_AUTOGAS_FUEL],
    [Units.HOUR],
    'liter autogas fuel per hour',
  );
  public static readonly GPH_AUTOGAS_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.GALLON_AUTOGAS_FUEL],
    [Units.HOUR],
    'gallon autogas fuel per hour',
  );
  public static readonly IGPH_AUTOGAS_FUEL = new CompoundMeasure(
    MeasureKind.WeightFlux,
    [Units.IMP_GALLON_AUTOGAS_FUEL],
    [Units.HOUR],
    'imperial gallon autogas fuel per hour',
  );

  public static readonly MILE_PER_GALLON_FUEL = new CompoundMeasure(
    MeasureKind.DistancePerWeight,
    [Units.MILE],
    [Units.GALLON_FUEL],
    'mile per gallon fuel',
  );
  public static readonly NMILE_PER_GALLON_FUEL = new CompoundMeasure(
    MeasureKind.DistancePerWeight,
    [Units.NMILE],
    [Units.GALLON_FUEL],
    'nautical mile per gallon fuel',
  );
  public static readonly MILE_PER_GALLON_JET_A_FUEL = new CompoundMeasure(
    MeasureKind.DistancePerWeight,
    [Units.MILE],
    [Units.GALLON_JET_A_FUEL],
    'mile per gallon Jet-A fuel',
  );
  public static readonly NMILE_PER_GALLON_JET_A_FUEL = new CompoundMeasure(
    MeasureKind.DistancePerWeight,
    [Units.NMILE],
    [Units.GALLON_JET_A_FUEL],
    'nautical mile per gallon Jet-A fuel',
  );
  public static readonly MILE_PER_GALLON_100LL_FUEL = new CompoundMeasure(
    MeasureKind.DistancePerWeight,
    [Units.MILE],
    [Units.GALLON_100LL_FUEL],
    'mile per gallon 100LL fuel',
  );
  public static readonly NMILE_PER_GALLON_100LL_FUEL = new CompoundMeasure(
    MeasureKind.DistancePerWeight,
    [Units.NMILE],
    [Units.GALLON_100LL_FUEL],
    'nautical mile per gallon 100LL fuel',
  );
  public static readonly MILE_PER_GALLON_AUTOGAS_FUEL = new CompoundMeasure(
    MeasureKind.DistancePerWeight,
    [Units.MILE],
    [Units.GALLON_AUTOGAS_FUEL],
    'mile per gallon autogas fuel',
  );
  public static readonly NMILE_PER_GALLON_AUTOGAS_FUEL = new CompoundMeasure(
    MeasureKind.DistancePerWeight,
    [Units.NMILE],
    [Units.GALLON_AUTOGAS_FUEL],
    'nautical mile per gallon autogas fuel',
  );
}

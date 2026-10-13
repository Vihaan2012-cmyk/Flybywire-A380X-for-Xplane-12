import { RoundMode } from './utils';
import { Measure, MeasureKind, QuantityLike, Units } from './units';

export interface NumFormatOptions {
  precision?: number;
  round?: RoundMode;
  hysteresis?: number | readonly [number, number];
  maxDigits?: number;
  forceDecimalZeroes?: boolean;
  pad?: number;
  showCommas?: boolean;
  useMinusSign?: boolean;
  forceSign?: boolean;
  hideSign?: boolean;
  nanString?: string;
  posInfinityString?: string;
  negInfinityString?: string;
  cache?: boolean;
}

interface ResolvedNumberOptions {
  precision: number;
  round: RoundMode;
  hysteresisLower: number;
  hysteresisUpper: number;
  maxDigits: number;
  forceDecimalZeroes: boolean;
  pad: number;
  showCommas: boolean;
  useMinusSign: boolean;
  forceSign: boolean;
  hideSign: boolean;
  nanString: string;
  posInfinityString: string;
  negInfinityString: string;
  cache: boolean;
}

function decimalPlacesOf(value: number): number {
  if (!Number.isFinite(value) || value === 0) {
    return 0;
  }
  let text = value.toString();
  const exponentIndex = text.indexOf('e');
  let exponent = 0;
  if (exponentIndex >= 0) {
    exponent = parseInt(text.slice(exponentIndex + 1), 10);
    text = text.slice(0, exponentIndex);
  }
  const point = text.indexOf('.');
  const fraction = point >= 0 ? text.length - point - 1 : 0;
  return Math.max(0, fraction - exponent);
}

function plainDecimalString(value: number): string {
  const text = value.toString();
  if (!text.includes('e')) {
    return text;
  }
  const places = Math.min(100, Math.max(decimalPlacesOf(value), 0));
  const fixed = value.toFixed(Math.min(places, 100));
  return fixed.includes('.') ? fixed.replace(/\.?0+$/, '') : fixed;
}

function roundWithMode(value: number, precision: number, mode: RoundMode): number {
  if (precision <= 0) {
    return value;
  }
  const scaled = value / precision;
  let rounded: number;
  switch (mode) {
    case RoundMode.Down:
      rounded = Math.floor(scaled);
      break;
    case RoundMode.Up:
      rounded = Math.ceil(scaled);
      break;
    default:
      rounded = Math.round(scaled);
  }
  if (precision < 1) {
    const inverse = Math.round(1 / precision);
    if (inverse > 1 && Math.abs(inverse * precision - 1) < 1e-9) {
      return rounded / inverse;
    }
  }
  return rounded * precision;
}

export class NumFormat {
  public static readonly DEFAULT_OPTIONS: Readonly<Required<NumFormatOptions>> = {
    precision: 0,
    round: RoundMode.Nearest,
    hysteresis: 0,
    maxDigits: Infinity,
    forceDecimalZeroes: true,
    pad: 1,
    showCommas: false,
    useMinusSign: false,
    forceSign: false,
    hideSign: false,
    nanString: 'NaN',
    posInfinityString: 'Infinity',
    negInfinityString: '-Infinity',
    cache: false,
  };

  private static resolveOptions(options?: NumFormatOptions): ResolvedNumberOptions {
    const merged = { ...NumFormat.DEFAULT_OPTIONS };
    if (options) {
      for (const key of Object.keys(options) as (keyof NumFormatOptions)[]) {
        if (options[key] !== undefined) {
          (merged as any)[key] = options[key];
        }
      }
    }
    const hysteresis = merged.hysteresis;
    const lower = Array.isArray(hysteresis) ? hysteresis[0] : (hysteresis as number);
    const upper = Array.isArray(hysteresis) ? hysteresis[1] : (hysteresis as number);
    return {
      precision: Math.max(0, merged.precision),
      round: merged.round,
      hysteresisLower: Math.max(0, lower),
      hysteresisUpper: Math.max(0, upper),
      maxDigits: merged.maxDigits,
      forceDecimalZeroes: merged.forceDecimalZeroes,
      pad: merged.pad,
      showCommas: merged.showCommas,
      useMinusSign: merged.useMinusSign,
      forceSign: merged.forceSign,
      hideSign: merged.hideSign,
      nanString: merged.nanString,
      posInfinityString: merged.posInfinityString,
      negInfinityString: merged.negInfinityString,
      cache: merged.cache,
    };
  }

  private static formatNumber(number: number, options: ResolvedNumberOptions, rounded: number): string {
    const negative = rounded < 0;
    const magnitude = Math.abs(rounded);
    let decimals = options.precision > 0 ? decimalPlacesOf(options.precision) : -1;
    let value = magnitude;

    if (Number.isFinite(options.maxDigits)) {
      let intDigits = Math.max(options.pad, Math.floor(value).toString().length);
      if (Math.floor(value) === 0) {
        intDigits = Math.max(options.pad, 1);
      }
      let allowed = Math.max(0, options.maxDigits - intDigits);
      const currentDecimals = decimals >= 0 ? decimals : decimalPlacesOf(value);
      if (currentDecimals > allowed) {
        for (let guard = 0; guard < 4; guard++) {
          value = roundWithMode(magnitude, Math.pow(10, -allowed), options.round);
          const newIntDigits = Math.max(options.pad, value < 1 ? 1 : Math.floor(value).toString().length);
          const newAllowed = Math.max(0, options.maxDigits - newIntDigits);
          if (newAllowed === allowed) {
            break;
          }
          allowed = newAllowed;
        }
        decimals = allowed;
      }
    }

    let text: string;
    if (decimals >= 0) {
      text = value.toFixed(Math.min(decimals, 100));
    } else {
      text = plainDecimalString(value);
    }

    if (text.includes('.') && !options.forceDecimalZeroes) {
      text = text.replace(/\.?0+$/, '');
    }

    let [integerPart, fractionPart] = text.split('.');
    if (options.pad > integerPart.length) {
      integerPart = integerPart.padStart(options.pad, '0');
    } else if (options.pad < 1 && fractionPart !== undefined) {
      integerPart = integerPart.replace(/^0+/, '');
    }
    if (options.showCommas) {
      integerPart = integerPart.replace(/\B(?=(\d{3})+(?!\d))/g, ',');
    }

    let sign = '';
    if (!options.hideSign) {
      if (negative) {
        sign = options.useMinusSign ? '−' : '-';
      } else if (options.forceSign) {
        sign = '+';
      }
    }
    void number;
    return sign + integerPart + (fractionPart !== undefined ? '.' + fractionPart : '');
  }

  public static create(options?: NumFormatOptions): (number: number) => string {
    const resolved = NumFormat.resolveOptions(options);
    const useHysteresis = resolved.hysteresisLower > 0 || resolved.hysteresisUpper > 0;
    const useCache = resolved.cache || useHysteresis;

    let lastInput = NaN;
    let lastRounded = NaN;
    let lastString: string | undefined;

    return (number: number): string => {
      if (Number.isNaN(number)) {
        return resolved.nanString;
      }
      if (number === Infinity) {
        return resolved.posInfinityString;
      }
      if (number === -Infinity) {
        return resolved.negInfinityString;
      }

      if (useCache && lastString !== undefined) {
        if (number === lastInput) {
          return lastString;
        }
        if (useHysteresis && resolved.precision > 0) {
          let lowest: number;
          let highest: number;
          if (resolved.round === RoundMode.Down) {
            lowest = lastRounded;
            highest = lastRounded + resolved.precision;
          } else if (resolved.round === RoundMode.Up) {
            lowest = lastRounded - resolved.precision;
            highest = lastRounded;
          } else {
            lowest = lastRounded - resolved.precision / 2;
            highest = lastRounded + resolved.precision / 2;
          }
          if (number >= lowest - resolved.hysteresisLower && number <= highest + resolved.hysteresisUpper) {
            return lastString;
          }
        }
      }

      const rounded = roundWithMode(number, resolved.precision, resolved.round);
      const text = NumFormat.formatNumber(number, resolved, rounded);
      if (useCache) {
        lastInput = number;
        lastRounded = rounded;
        lastString = text;
      }
      return text;
    };
  }
}

type Builder<T> = (value: T) => string;

interface Segment {
  literal?: string;
  fragment?: string;
}

function splitTemplate(format: string): Segment[] {
  const segments: Segment[] = [];
  let literal = '';
  let i = 0;
  while (i < format.length) {
    const char = format[i];
    if (char === '{') {
      let depth = 1;
      let j = i + 1;
      while (j < format.length && depth > 0) {
        if (format[j] === '{') {
          depth++;
        } else if (format[j] === '}') {
          depth--;
        }
        j++;
      }
      if (depth !== 0) {
        literal += format.slice(i);
        break;
      }
      if (literal) {
        segments.push({ literal });
        literal = '';
      }
      segments.push({ fragment: format.slice(i + 1, j - 1) });
      i = j;
    } else {
      literal += char;
      i++;
    }
  }
  if (literal) {
    segments.push({ literal });
  }
  return segments;
}

function buildFromTemplate<T>(format: string, parse: (fragment: string) => Builder<T>): (value: T) => string {
  const parts: Builder<T>[] = splitTemplate(format).map((segment) => {
    if (segment.literal !== undefined) {
      const text = segment.literal;
      return () => text;
    }
    return parse(segment.fragment as string);
  });
  return (value: T) => {
    let out = '';
    for (const part of parts) {
      out += part(value);
    }
    return out;
  };
}

function parseSignFragment(fragment: string): ((negative: boolean) => string) | undefined {
  const match = /^(?:(\+)(?:\[([^\]]*)\])?)?-(?:\[([^\]]*)\])?$/.exec(fragment);
  if (!match) {
    return undefined;
  }
  const positive = match[1] ? (match[2] ?? '+') : '';
  const negative = match[3] ?? '-';
  return (isNegative: boolean) => (isNegative ? negative : positive);
}

interface SexagesimalUnitInfo {
  size: number;
  modulo: number;
}

function formatSexagesimalFragment(
  fragment: string,
  letters: string,
  info: Record<string, SexagesimalUnitInfo>,
): ((micro: number) => string) | undefined {
  const match = new RegExp(`^([${letters}]+)(\\?)?(?:\\.([${letters}]*)(?:\\(([${letters}]*)\\))?)?$`).exec(fragment);
  if (!match) {
    return undefined;
  }
  const integerLetters = match[1];
  const letter = integerLetters[0];
  if (integerLetters.split('').some((c) => c !== letter)) {
    return undefined;
  }
  if (match[3] !== undefined && match[3].split('').some((c) => c !== letter)) {
    return undefined;
  }
  if (match[4] !== undefined && match[4].split('').some((c) => c !== letter)) {
    return undefined;
  }
  const unit = info[letter];
  const padding = integerLetters.length;
  const dropZero = match[2] !== undefined;
  const hasPoint = fragment.includes('.');
  const requiredDigits = match[3]?.length ?? 0;
  const optionalDigits = match[4]?.length ?? 0;
  const unlimited = hasPoint && requiredDigits === 0 && optionalDigits === 0;
  const totalDigits = Math.min(6, unlimited ? 6 : requiredDigits + optionalDigits);

  return (micro: number) => {
    const wholeUnits = Math.floor(micro / unit.size);
    const remainder = micro - wholeUnits * unit.size;
    const integerValue = unit.modulo > 0 ? wholeUnits % unit.modulo : wholeUnits;
    let integerText = integerValue.toString().padStart(padding, '0');
    if (dropZero && integerValue === 0) {
      integerText = '';
    }
    if (!hasPoint) {
      return integerText;
    }
    let digits = '';
    if (totalDigits > 0) {
      digits = Math.floor((remainder * Math.pow(10, totalDigits)) / unit.size)
        .toString()
        .padStart(totalDigits, '0');
    }
    const optionalFloor = unlimited ? 0 : requiredDigits;
    let end = digits.length;
    while (end > optionalFloor && digits[end - 1] === '0') {
      end--;
    }
    digits = digits.slice(0, end);
    return digits.length > 0 ? `${integerText}.${digits}` : integerText;
  };
}

export interface DmsFormatOptions {
  nanString: string;
  cache: boolean;
}

export class DmsFormat {
  public static readonly DEFAULT_OPTIONS: Readonly<DmsFormatOptions> = {
    nanString: 'NaN',
    cache: false,
  };

  private static readonly UNIT_INFO: Record<string, SexagesimalUnitInfo> = {
    D: { size: 3.6e9, modulo: 0 },
    d: { size: 3.6e9, modulo: 0 },
    M: { size: 6e7, modulo: 0 },
    m: { size: 6e7, modulo: 60 },
    S: { size: 1e6, modulo: 0 },
    s: { size: 1e6, modulo: 60 },
  };

  public static create(
    format: string,
    unit: Measure<MeasureKind.Angle>,
    precision: number,
    nanString?: string,
  ): (angle: number) => string;
  public static create(
    format: string,
    unit: Measure<MeasureKind.Angle>,
    precision: number,
    options?: Readonly<Partial<DmsFormatOptions>>,
  ): (angle: number) => string;
  public static create(
    format: string,
    unit: Measure<MeasureKind.Angle>,
    precision: number,
    arg?: string | Readonly<Partial<DmsFormatOptions>>,
  ): (angle: number) => string {
    const options = DmsFormat.resolveOptions(arg);
    const builder = DmsFormat.createBuilder(format);
    let lastInput = NaN;
    let lastOutput: string | undefined;
    return (angle: number): string => {
      if (Number.isNaN(angle)) {
        return options.nanString;
      }
      if (options.cache && lastOutput !== undefined && angle === lastInput) {
        return lastOutput;
      }
      const rounded = precision > 0 ? Math.round(angle / precision) * precision : angle;
      const arcSeconds = Units.ARC_SEC.convertFrom(rounded, unit);
      const output = builder(arcSeconds);
      lastInput = angle;
      lastOutput = output;
      return output;
    };
  }

  public static createForNumberUnit(
    format: string,
    precision: QuantityLike<MeasureKind.Angle>,
    nanString?: string,
  ): (angle: QuantityLike<MeasureKind.Angle>) => string;
  public static createForNumberUnit(
    format: string,
    precision: QuantityLike<MeasureKind.Angle>,
    options?: Readonly<Partial<DmsFormatOptions>>,
  ): (angle: QuantityLike<MeasureKind.Angle>) => string;
  public static createForNumberUnit(
    format: string,
    precision: QuantityLike<MeasureKind.Angle>,
    arg?: string | Readonly<Partial<DmsFormatOptions>>,
  ): (angle: QuantityLike<MeasureKind.Angle>) => string {
    const inner = DmsFormat.create(format, precision.unit, precision.number, arg as Readonly<Partial<DmsFormatOptions>>);
    return (angle) => inner(angle.asUnit(precision.unit));
  }

  private static resolveOptions(arg?: string | Readonly<Partial<DmsFormatOptions>>): DmsFormatOptions {
    if (typeof arg === 'string') {
      return { ...DmsFormat.DEFAULT_OPTIONS, nanString: arg };
    }
    return { ...DmsFormat.DEFAULT_OPTIONS, ...(arg ?? {}) };
  }

  private static createBuilder(format: string): (arcSeconds: number) => string {
    return buildFromTemplate<number>(format, (fragment) => DmsFormat.parseFragment(fragment));
  }

  private static parseFragment(fragment: string): Builder<number> {
    const sign = parseSignFragment(fragment);
    if (sign) {
      return (arcSeconds) => sign(arcSeconds < 0);
    }
    const numeric = formatSexagesimalFragment(fragment, 'DMSdms', DmsFormat.UNIT_INFO);
    if (numeric) {
      return (arcSeconds) => numeric(Math.round(Math.abs(arcSeconds) * 1e6));
    }
    return () => '';
  }
}

export interface SpanFormatOptions {
  nanString: string;
  cache: boolean;
}

export class SpanFormat {
  public static readonly DEFAULT_OPTIONS: Readonly<SpanFormatOptions> = {
    nanString: 'NaN',
    cache: false,
  };

  private static readonly UNIT_INFO: Record<string, SexagesimalUnitInfo> = {
    H: { size: 3.6e9, modulo: 0 },
    h: { size: 3.6e9, modulo: 0 },
    M: { size: 6e7, modulo: 0 },
    m: { size: 6e7, modulo: 60 },
    S: { size: 1e6, modulo: 0 },
    s: { size: 1e6, modulo: 60 },
  };

  public static create(
    format: string,
    unit: Measure<MeasureKind.Duration>,
    precision: number,
    arg?: string | Readonly<Partial<SpanFormatOptions>>,
  ): (duration: number) => string {
    const options =
      typeof arg === 'string'
        ? { ...SpanFormat.DEFAULT_OPTIONS, nanString: arg }
        : { ...SpanFormat.DEFAULT_OPTIONS, ...(arg ?? {}) };
    const builder = buildFromTemplate<number>(format, (fragment) => {
      const sign = parseSignFragment(fragment);
      if (sign) {
        return (seconds) => sign(seconds < 0);
      }
      const numeric = formatSexagesimalFragment(fragment, 'HMShms', SpanFormat.UNIT_INFO);
      if (numeric) {
        return (seconds) => numeric(Math.round(Math.abs(seconds) * 1e6));
      }
      return () => '';
    });
    let lastInput = NaN;
    let lastOutput: string | undefined;
    return (duration: number): string => {
      if (Number.isNaN(duration)) {
        return options.nanString;
      }
      if (options.cache && lastOutput !== undefined && duration === lastInput) {
        return lastOutput;
      }
      const rounded = precision > 0 ? Math.round(duration / precision) * precision : duration;
      const output = builder(Units.SECOND.convertFrom(rounded, unit));
      lastInput = duration;
      lastOutput = output;
      return output;
    };
  }

  public static createForNumberUnit(
    format: string,
    precision: QuantityLike<MeasureKind.Duration>,
    arg?: string | Readonly<Partial<SpanFormatOptions>>,
  ): (duration: QuantityLike<MeasureKind.Duration>) => string {
    const inner = SpanFormat.create(format, precision.unit, precision.number, arg);
    return (duration) => inner(duration.asUnit(precision.unit));
  }
}

export interface TimeFormatOptions {
  monthNames: [string, string, string, string, string, string, string, string, string, string, string, string];
  monthNamesShort: [string, string, string, string, string, string, string, string, string, string, string, string];
  dayNames: [string, string, string, string, string, string, string];
  dayNamesShort: [string, string, string, string, string, string, string];
  nanString: string;
  cache: boolean;
}

const MS_PER_HOUR = 3_600_000;
const MS_PER_MINUTE = 60_000;

export class TimeFormat {
  public static readonly DEFAULT_OPTIONS: Readonly<TimeFormatOptions> = {
    monthNames: [
      'January',
      'February',
      'March',
      'April',
      'May',
      'June',
      'July',
      'August',
      'September',
      'October',
      'November',
      'December',
    ],
    monthNamesShort: ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'],
    dayNames: ['Sunday', 'Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday'],
    dayNamesShort: ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'],
    nanString: 'NaN',
    cache: false,
  };

  private static readonly INT_GETTERS: Record<string, (date: Date) => number> = {
    M: (date) => date.getUTCMonth() + 1,
    d: (date) => date.getUTCDate(),
    w: (date) => date.getUTCDay(),
  };

  public static create(
    format: string,
    options?: Readonly<Partial<TimeFormatOptions>>,
  ): (time: number) => string {
    const resolved = TimeFormat.resolveOptions(options);
    const builder = buildFromTemplate<{ date: Date; time: number }>(format, (fragment) =>
      TimeFormat.parseFragment(fragment, resolved),
    );
    let lastInput = NaN;
    let lastOutput: string | undefined;
    return (time: number): string => {
      if (Number.isNaN(time)) {
        return resolved.nanString;
      }
      if (resolved.cache && lastOutput !== undefined && time === lastInput) {
        return lastOutput;
      }
      const output = builder({ date: new Date(time), time });
      lastInput = time;
      lastOutput = output;
      return output;
    };
  }

  private static resolveOptions(options?: Readonly<Partial<TimeFormatOptions>>): TimeFormatOptions {
    return { ...TimeFormat.DEFAULT_OPTIONS, ...(options ?? {}) };
  }

  private static parseFragment(
    fragment: string,
    options: TimeFormatOptions,
  ): Builder<{ date: Date; time: number }> {
    let match = /^(mon|MON)(\.)?$|^(month|MONTH)$/.exec(fragment);
    if (match) {
      const upper = fragment.startsWith('MON');
      const full = fragment.toLowerCase() === 'month';
      const period = match[2] ? '.' : '';
      return ({ date }) => {
        const index = date.getUTCMonth();
        const name = full ? options.monthNames[index] : options.monthNamesShort[index];
        return (upper ? name.toUpperCase() : name.toLowerCase()) + (full ? '' : period);
      };
    }
    match = /^(dy|DY)(\.)?$|^(day|DAY)$/.exec(fragment);
    if (match) {
      const upper = fragment.startsWith('DY') || fragment === 'DAY';
      const full = fragment.toLowerCase() === 'day';
      const period = match[2] ? '.' : '';
      return ({ date }) => {
        const index = date.getUTCDay();
        const name = full ? options.dayNames[index] : options.dayNamesShort[index];
        return (upper ? name.toUpperCase() : name.toLowerCase()) + (full ? '' : period);
      };
    }
    match = /^(YY|YYYY)$/.exec(fragment);
    if (match) {
      const short = fragment === 'YY';
      return ({ date }) => {
        const year = date.getUTCFullYear();
        return short ? (Math.abs(year) % 100).toString().padStart(2, '0') : year.toString();
      };
    }
    match = /^(am|AM|a\.m\.|A\.M\.)$/.exec(fragment);
    if (match) {
      const upper = fragment === fragment.toUpperCase();
      const periods = fragment.includes('.');
      return ({ date }) => {
        const pm = date.getUTCHours() >= 12;
        let text = periods ? (pm ? 'p.m.' : 'a.m.') : pm ? 'pm' : 'am';
        if (upper) {
          text = text.toUpperCase();
        }
        return text;
      };
    }
    match = /^(M+|d+|w+)$/.exec(fragment);
    if (match) {
      const letter = fragment[0];
      if (fragment.split('').every((c) => c === letter)) {
        const getter = TimeFormat.INT_GETTERS[letter];
        const width = fragment.length;
        return ({ date }) => getter(date).toString().padStart(width, '0');
      }
    }
    match = /^([Hhms]+)(\?)?(?:\.([Hhms]*)(?:\(([Hhms]*)\))?)?$/.exec(fragment);
    if (match) {
      return TimeFormat.parseNumericFragment(fragment, match);
    }
    return () => '';
  }

  private static parseNumericFragment(
    fragment: string,
    match: RegExpExecArray,
  ): Builder<{ date: Date; time: number }> {
    const letter = match[1][0];
    const same = (text: string | undefined) => text === undefined || text.split('').every((c) => c === letter);
    if (!same(match[1]) || !same(match[3]) || !same(match[4])) {
      return () => '';
    }
    const padding = match[1].length;
    const dropZero = match[2] !== undefined;
    const hasPoint = fragment.includes('.');
    const requiredDigits = match[3]?.length ?? 0;
    const optionalDigits = match[4]?.length ?? 0;
    const unlimited = hasPoint && requiredDigits === 0 && optionalDigits === 0;
    const totalDigits = Math.min(6, unlimited ? 6 : requiredDigits + optionalDigits);
    const unitMs = letter === 'H' || letter === 'h' ? MS_PER_HOUR : letter === 'm' ? MS_PER_MINUTE : 1000;

    return ({ date, time }) => {
      const dayMs = ((time % 86_400_000) + 86_400_000) % 86_400_000;
      const intoUnit = letter === 'H' || letter === 'h' ? dayMs : letter === 'm' ? dayMs % MS_PER_HOUR : dayMs % MS_PER_MINUTE;
      const whole = Math.floor(intoUnit / unitMs);
      const remainder = intoUnit - whole * unitMs;
      let integerValue = whole;
      if (letter === 'h') {
        integerValue = date.getUTCHours() % 12;
        if (integerValue === 0) {
          integerValue = 12;
        }
      }
      let integerText = integerValue.toString().padStart(padding, '0');
      if (dropZero && integerValue === 0) {
        integerText = '';
      }
      if (!hasPoint) {
        return integerText;
      }
      const fractionMs = remainder + (time - Math.floor(time));
      let digits = '';
      if (totalDigits > 0) {
        digits = Math.floor((fractionMs * Math.pow(10, totalDigits)) / unitMs)
          .toString()
          .padStart(totalDigits, '0');
      }
      const optionalFloor = unlimited ? 0 : requiredDigits;
      let end = digits.length;
      while (end > optionalFloor && digits[end - 1] === '0') {
        end--;
      }
      digits = digits.slice(0, end);
      return digits.length > 0 ? `${integerText}.${digits}` : integerText;
    };
  }
}

export interface CycleLike {
  ident: string;
  cycle: number;
  cycleString: string;
  effectiveTimestamp: number;
  expirationTimestamp: number;
}

export type CycleFormatFunction = (cycle: CycleLike) => string;

export class CycleFormat {
  private static dateFormatterCache = new Map<string, (time: number) => string>();

  private constructor() {}

  public static create(format: string): CycleFormatFunction {
    const parts = splitTemplate(format).map((segment): ((cycle: CycleLike) => string) => {
      if (segment.literal !== undefined) {
        const text = segment.literal;
        return () => text;
      }
      return CycleFormat.parseFragment(segment.fragment as string);
    });
    return (cycle) => {
      let out = '';
      for (const part of parts) {
        out += part(cycle);
      }
      return out;
    };
  }

  private static dateFormatter(format: string): (time: number) => string {
    let formatter = CycleFormat.dateFormatterCache.get(format);
    if (!formatter) {
      formatter = TimeFormat.create(format);
      CycleFormat.dateFormatterCache.set(format, formatter);
    }
    return formatter;
  }

  private static parseFragment(fragment: string): (cycle: CycleLike) => string {
    if (fragment === 'CC') {
      return (cycle) => cycle.cycleString;
    }
    if (fragment === 'YYCC') {
      return (cycle) => cycle.ident;
    }
    const match = /^(eff|expMinus|exp)\(([\s\S]*)\)$/.exec(fragment);
    if (match) {
      const formatter = CycleFormat.dateFormatter(match[2]);
      switch (match[1]) {
        case 'eff':
          return (cycle) => formatter(cycle.effectiveTimestamp);
        case 'exp':
          return (cycle) => formatter(cycle.expirationTimestamp);
        default:
          return (cycle) => formatter(cycle.expirationTimestamp - 1000);
      }
    }
    throw new Error(`Invalid AIRAC cycle format fragment: {${fragment}}`);
  }
}

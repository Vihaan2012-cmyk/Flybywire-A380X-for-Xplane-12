import { Readable, Watchable } from './types';

const roundWith = (x: number, mode: number): number =>
  mode < 0 ? Math.floor(x) : mode > 0 ? Math.ceil(x) : Math.round(x);

export class Mappers {
  public static identity<T>(): (input: T) => T {
    return (input) => input;
  }

  public static not<T extends boolean>(): (input: T) => boolean {
    return (input) => !input;
  }

  public static or(): (input: readonly boolean[]) => boolean {
    return (input) => input.some((v) => v);
  }

  public static nor(): (input: readonly boolean[]) => boolean {
    return (input) => !input.some((v) => v);
  }

  public static and(): (input: readonly boolean[]) => boolean {
    return (input) => input.length > 0 && input.every((v) => v);
  }

  public static nand(): (input: readonly boolean[]) => boolean {
    return (input) => !(input.length > 0 && input.every((v) => v));
  }

  public static negate<T extends number>(): (input: T) => number {
    return (input) => -input;
  }

  public static abs<T extends number>(): (input: T) => number {
    return (input) => Math.abs(input);
  }

  public static min(): (input: readonly number[]) => number {
    return (input) => Math.min(...input);
  }

  public static max(): (input: readonly number[]) => number {
    return (input) => Math.max(...input);
  }

  public static count<T>(predicate: (value: T) => boolean): (input: readonly T[]) => number {
    return (input) => {
      let n = 0;
      for (const v of input) {
        if (predicate(v)) {
          n++;
        }
      }
      return n;
    };
  }

  public static sum(): (input: readonly number[]) => number {
    return (input) => input.reduce((a, b) => a + b, 0);
  }

  public static average(): (inputs: readonly number[]) => number {
    return (inputs) => (inputs.length === 0 ? NaN : inputs.reduce((a, b) => a + b, 0) / inputs.length);
  }

  public static reduce<T>(
    callbackFn: (previousValue: T, currentInput: T, currentIndex: number, inputs: readonly T[]) => T,
    initialValue?: T,
  ): (input: readonly T[]) => T;
  public static reduce<T, U>(
    callbackFn: (previousValue: U, currentInput: T, currentIndex: number, inputs: readonly T[]) => U,
    initialValue: U,
  ): (input: readonly T[], currentVal?: U) => U;
  public static reduce(callbackFn: (...args: any[]) => any, initialValue?: any): (input: readonly any[]) => any {
    return (input) => (initialValue === undefined ? input.reduce(callbackFn) : input.reduce(callbackFn, initialValue));
  }

  public static rounded<T extends number>(precision: number | Readable<number>, round = 0): (input: T) => number {
    return (input) => {
      const p = typeof precision === 'number' ? precision : precision.get();
      return roundWith(input / p, round) * p;
    };
  }

  public static withPrecisionHysteresis<T extends number>(
    precision: number,
    hysteresis: number | readonly [number, number],
    round = 0,
  ): (input: T, currentVal?: T) => number {
    const lower = Math.max(0, typeof hysteresis === 'number' ? hysteresis : hysteresis[0]);
    const upper = Math.max(0, typeof hysteresis === 'number' ? hysteresis : hysteresis[1]);
    return (input, currentVal) => {
      if (currentVal !== undefined && !Number.isNaN(currentVal)) {
        const lo = round === 0 ? currentVal - precision / 2 : round < 0 ? currentVal : currentVal - precision;
        const hi = round === 0 ? currentVal + precision / 2 : round < 0 ? currentVal + precision : currentVal;
        if (input >= lo - lower && input <= hi + upper) {
          return currentVal;
        }
      }
      return roundWith(input / precision, round) * precision;
    };
  }

  public static changedBy<T extends number>(
    threshold: number | Watchable<number>,
  ): (input: T, currentVal?: T) => number {
    return (input, currentVal) => {
      const t = typeof threshold === 'number' ? threshold : threshold.get();
      return currentVal === undefined || Math.abs(input - currentVal) >= t ? input : currentVal;
    };
  }
}

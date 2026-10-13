import type { Quantity, QuantityLike, Measure } from '../math';
import { AbstractWatchable, DerivedCell } from './core';
import { DerivedWatchable, WritableWatchable, Watchable } from './types';

export class QuantityCell<F extends string, U extends Measure<F> = Measure<F>>
  extends AbstractWatchable<QuantityLike<F, U>>
  implements WritableWatchable<QuantityLike<F, U>>, WritableWatchable<QuantityLike<F, U>, number>
{
  public readonly isWritable = true;

  private constructor(private readonly value: Quantity<F, U>) {
    super();
  }

  public static create<F extends string, U extends Measure<F>>(initialVal: Quantity<F, U>): QuantityCell<F, U> {
    return new QuantityCell(initialVal);
  }

  public static createFromNumberUnit<F extends string, U extends Measure<F>>(
    initialVal: Quantity<F, U>,
  ): QuantityCell<F, U> {
    return new QuantityCell(initialVal);
  }

  public get(): QuantityLike<F, U> {
    return (this.value as any).readonly;
  }

  public set(value: QuantityLike<F>): void;
  public set(value: number, unit?: Measure<F>): void;
  public set(value: QuantityLike<F> | number, unit?: Measure<F>): void {
    const v: any = this.value;
    const same = typeof value === 'number' ? v.equals(value, unit) : v.equals(value);
    if (!same) {
      if (typeof value === 'number') {
        v.set(value, unit);
      } else {
        v.set(value);
      }
      this.notify();
    }
  }

  public asUnit(unit: Measure<F> | Watchable<Measure<F>>): DerivedWatchable<number> {
    if ((unit as any)?.isWatchable === true) {
      return DerivedCell.create(
        ([nu, u]: readonly [QuantityLike<F, U>, Measure<F>]) => (nu as any).asUnit(u) as number,
        this as Watchable<QuantityLike<F, U>>,
        unit as Watchable<Measure<F>>,
      );
    }
    return this.map((nu) => (nu as any).asUnit(unit) as number);
  }
}

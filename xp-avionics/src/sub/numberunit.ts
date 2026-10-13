import type { NumberUnit, NumberUnitInterface, Unit } from '../math';
import { AbstractSubscribable, MappedSubject } from './core';
import { MappedSubscribable, MutableSubscribable, Subscribable } from './types';

export class NumberUnitSubject<F extends string, U extends Unit<F> = Unit<F>>
  extends AbstractSubscribable<NumberUnitInterface<F, U>>
  implements MutableSubscribable<NumberUnitInterface<F, U>>, MutableSubscribable<NumberUnitInterface<F, U>, number>
{
  public readonly isMutableSubscribable = true;

  private constructor(private readonly value: NumberUnit<F, U>) {
    super();
  }

  public static create<F extends string, U extends Unit<F>>(initialVal: NumberUnit<F, U>): NumberUnitSubject<F, U> {
    return new NumberUnitSubject(initialVal);
  }

  public static createFromNumberUnit<F extends string, U extends Unit<F>>(
    initialVal: NumberUnit<F, U>,
  ): NumberUnitSubject<F, U> {
    return new NumberUnitSubject(initialVal);
  }

  public get(): NumberUnitInterface<F, U> {
    return (this.value as any).readonly;
  }

  public set(value: NumberUnitInterface<F>): void;
  public set(value: number, unit?: Unit<F>): void;
  public set(value: NumberUnitInterface<F> | number, unit?: Unit<F>): void {
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

  public asUnit(unit: Unit<F> | Subscribable<Unit<F>>): MappedSubscribable<number> {
    if ((unit as any)?.isSubscribable === true) {
      return MappedSubject.create(
        ([nu, u]: readonly [NumberUnitInterface<F, U>, Unit<F>]) => (nu as any).asUnit(u) as number,
        this as Subscribable<NumberUnitInterface<F, U>>,
        unit as Subscribable<Unit<F>>,
      );
    }
    return this.map((nu) => (nu as any).asUnit(unit) as number);
  }
}

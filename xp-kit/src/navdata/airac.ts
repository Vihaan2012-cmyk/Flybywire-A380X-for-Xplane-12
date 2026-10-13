export interface Cycle {
  ident: string;
  cycle: number;
  cycleString: string;
  effectiveTimestamp: number;
  expirationTimestamp: number;
}

const CYCLE_DURATION = 86400_000 * 28;
const DATE_RANGE = /([A-Z]{3})(\d\d?)([A-Z]{3})(\d\d?)\/(\d\d)/;

export class CycleKit {
  public static readonly CYCLE_DURATION = CYCLE_DURATION;

  public static parseFacilitiesCycle(dateRange: string): Cycle | undefined {
    const match = typeof dateRange === 'string' ? dateRange.match(DATE_RANGE) : null;
    if (match === null) {
      return undefined;
    }
    const [, effMonth, effDay, expMonth, expDay, expYear] = match;
    const effective = new Date(`${effMonth}-${effDay}-${expYear} UTC`);
    const expiry = new Date(`${expMonth}-${expDay}-${expYear} UTC`);
    if (Number.isNaN(effective.getTime()) || Number.isNaN(expiry.getTime())) {
      return undefined;
    }
    if (effective.getTime() > expiry.getTime()) {
      effective.setUTCFullYear(effective.getUTCFullYear() - 1);
    }
    return CycleKit.fromEffectiveTimestamp(effective.getTime());
  }

  public static getCycleNumber(effectiveTimestamp: number): number {
    const year = new Date(effectiveTimestamp).getUTCFullYear();
    return Math.trunc((effectiveTimestamp - Date.UTC(year, 0, 1)) / CYCLE_DURATION) + 1;
  }

  public static getOffsetCycle(base: Cycle, offset: number): Cycle {
    return CycleKit.fromEffectiveTimestamp(base.effectiveTimestamp + offset * CYCLE_DURATION);
  }

  public static getCurrentCycle(date: Date): Cycle {
    const anchor = Date.UTC(2020, 0, 2);
    const index = Math.floor((date.getTime() - anchor) / CYCLE_DURATION);
    return CycleKit.fromEffectiveTimestamp(anchor + index * CYCLE_DURATION);
  }

  private static fromEffectiveTimestamp(effectiveTimestamp: number): Cycle {
    const year = new Date(effectiveTimestamp).getUTCFullYear();
    const cycle = CycleKit.getCycleNumber(effectiveTimestamp);
    const cycleString = cycle.toString().padStart(2, '0');
    return {
      ident: `${(year % 100).toString().padStart(2, '0')}${cycleString}`,
      cycle,
      cycleString,
      effectiveTimestamp,
      expirationTimestamp: effectiveTimestamp + CYCLE_DURATION,
    };
  }
}

//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { BasicTap as BasicConsumer, Tap as Consumer, Callback as Handler, Watch as Subscription } from 'xp-kit';
import { Arinc429WordData } from './arinc429';

type Subscribe<T> = (handler: Handler<T>, paused: boolean) => Subscription;

export interface ArincConsumer<T> extends Consumer<T> {
  withArinc429Precision(precision: number): ArincConsumer<T>;

  whenArinc429SsmChanged(): ArincConsumer<T>;
}

export class BasicArincConsumer<T> extends BasicConsumer<T> implements ArincConsumer<T> {
  constructor(private readonly subscribeArinc: Subscribe<T>) {
    super(subscribeArinc);
  }

  public withArinc429Precision(precision: number): ArincConsumer<T> {
    const multiplier = 10 ** precision;
    let lastValue: number | undefined;
    let lastSsm: number | undefined;
    let lastNormal: boolean | undefined;
    return this.filtered((word, next) => {
      const data = word as unknown as Arinc429WordData;
      const normal = data.isNormalOperation();
      const value = Math.round(data.value * multiplier) / multiplier;
      if (value !== lastValue || data.ssm !== lastSsm || normal !== lastNormal) {
        lastValue = value;
        lastSsm = data.ssm;
        lastNormal = normal;
        next(word);
      }
    });
  }

  public whenArinc429SsmChanged(): ArincConsumer<T> {
    let lastSsm: number | undefined;
    return this.filtered((word, next) => {
      const ssm = (word as unknown as { ssm: number }).ssm;
      if (ssm !== lastSsm) {
        lastSsm = ssm;
        next(word);
      }
    });
  }

  private filtered(filter: (data: T, next: Handler<T>) => void): BasicArincConsumer<T> {
    return new BasicArincConsumer<T>((handler, paused) =>
      this.subscribeArinc((data) => filter(data, handler), paused),
    );
  }
}

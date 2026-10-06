//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { EventBus } from '@microsoft/msfs-sdk';
import { AocMessageStore } from './AocMessageStore';
import { buildLoadsheet, loadsheetFlightKnown } from './Loadsheet';

const POLL_MS = 10_000;

export class AocUplink {
  private timer: ReturnType<typeof setInterval> | undefined;

  private prelimSent = false;

  private finalSent = false;

  constructor(private readonly bus: EventBus) {}

  start(): void {
    if (this.timer === undefined) {
      this.timer = setInterval(() => this.poll(), POLL_MS);
    }
  }

  stop(): void {
    if (this.timer !== undefined) {
      clearInterval(this.timer);
      this.timer = undefined;
    }
  }

  private poll(): void {
    try {
      if (this.finalSent || !loadsheetFlightKnown()) {
        return;
      }
      const kind = this.prelimSent ? 'FINAL' : 'PRELIM';
      const sheet = buildLoadsheet(kind);
      if (!sheet) {
        return;
      }
      AocMessageStore.instance(this.bus).add('LOADSHEET', sheet.title, 'LOAD CONTROL', sheet.lines);
      if (kind === 'PRELIM') {
        this.prelimSent = true;
      } else {
        this.finalSent = true;
      }
    } catch (e) {
      console.warn('AOC uplink:', e);
    }
  }
}

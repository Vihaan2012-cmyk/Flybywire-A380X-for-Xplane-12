// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { ConsumerSubject, EventBus, MappedSubject, Subscription } from '@microsoft/msfs-sdk';
import { EfisSide } from '@flybywiresim/fbw-sdk';
import { NdWxrMessage, NdWxrMessageEvents } from '@flybywiresim/navigation-display';
import { NDSimvars } from './NDSimvarPublisher';

export class WxrNdMessagePublisher {
  private readonly subs: Subscription[] = [];

  private lastKey = '';

  constructor(
    private readonly bus: EventBus,
    side: EfisSide,
  ) {
    const sub = bus.getSubscriber<NDSimvars>();
    const l = side === 'L';
    const sysSelected = ConsumerSubject.create(sub.on('wxrTawsSysSelected'), 0);
    const failed1 = ConsumerSubject.create(sub.on('wxr1Failed'), true);
    const failed2 = ConsumerSubject.create(sub.on('wxr2Failed'), true);
    const off = ConsumerSubject.create(sub.on('wxrOff'), false);
    const overlay = ConsumerSubject.create(sub.on(l ? 'wxrOverlayL' : 'wxrOverlayR'), 0);
    const mode = ConsumerSubject.create(sub.on(l ? 'wxrElevnTiltL' : 'wxrElevnTiltR'), 0);
    const elevnFt = ConsumerSubject.create(sub.on(l ? 'wxrElevnFtL' : 'wxrElevnFtR'), 0);
    const tiltDeg = ConsumerSubject.create(sub.on(l ? 'wxrTiltDegL' : 'wxrTiltDegR'), 0);
    const gainMan = ConsumerSubject.create(sub.on(l ? 'wxrGainManL' : 'wxrGainManR'), false);
    const gainPct = ConsumerSubject.create(sub.on(l ? 'wxrGainPctL' : 'wxrGainPctR'), 50);
    const map = ConsumerSubject.create(sub.on(l ? 'wxrModeMapL' : 'wxrModeMapR'), false);
    const baroStd = ConsumerSubject.create(sub.on(l ? 'wxrBaroStdL' : 'wxrBaroStdR'), false);

    const message = MappedSubject.create(
      ([sel, f1, f2, isOff, ov, m, ft, tilt, man, pct, isMap, std]): NdWxrMessage | null => {
        if (Math.round(ov) !== 1) {
          return null;
        }
        const failed = sel === 1 ? f1 : sel === 2 ? f2 : true;
        if (failed) {
          return { text: 'WXR INOP', className: 'Amber' };
        }
        if (isOff) {
          return { text: 'WXR OFF', className: 'White' };
        }
        if (isMap) {
          return { text: 'MAP', className: 'Green' };
        }
        const gain = man ? `GAIN ${Math.round(pct)}%` : undefined;
        switch (Math.round(m)) {
          case 1:
            return {
              text: std ? `ELEVN FL${Math.round(ft / 100).toString().padStart(3, '0')}` : `ELEVN ${Math.round(ft)}FT`,
              className: 'Cyan',
              secondLine: gain,
            };
          case 2:
            return {
              text: `TILT ${tilt >= 0 ? '+' : '-'}${Math.abs(tilt).toFixed(1)}°`,
              className: 'Cyan',
              secondLine: gain,
            };
          default:
            return gain ? { text: gain, className: 'Cyan' } : { text: 'WX', className: 'Green' };
        }
      },
      sysSelected,
      failed1,
      failed2,
      off,
      overlay,
      mode,
      elevnFt,
      tiltDeg,
      gainMan,
      gainPct,
      map,
      baroStd,
    );

    const publisher = bus.getPublisher<NdWxrMessageEvents>();
    this.subs.push(
      message.sub((m) => {
        const key = m ? `${m.className}|${m.text}|${m.secondLine ?? ''}` : '';
        if (key !== this.lastKey) {
          this.lastKey = key;
          publisher.pub('ndWxrMessage', m, false, true);
        }
      }, true),
    );
  }
}

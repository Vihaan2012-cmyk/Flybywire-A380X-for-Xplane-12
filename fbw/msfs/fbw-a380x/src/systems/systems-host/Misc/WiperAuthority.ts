// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { Instrument } from '@microsoft/msfs-sdk';

export class WiperAuthority implements Instrument {
  private static readonly SIDES = [
    { circuit: 141, failureId: 8030049, selectedVar: 'L:A32NX_RAIN_REMOVAL_SELECTED_1' },
    { circuit: 143, failureId: 8030050, selectedVar: 'L:A32NX_RAIN_REMOVAL_SELECTED_2' },
  ];

  private static readonly STOPPED_AT = 0.5;

  private static readonly SETTLE_FRAMES = 10;

  private readonly heldOn = [false, false];

  private readonly settle = [0, 0];

  init(): void {}

  onUpdate(): void {
    WiperAuthority.SIDES.forEach((side, i) => {
      const on = !!SimVar.GetSimVarValue(`A:CIRCUIT SWITCH ON:${side.circuit}`, 'bool');
      const failed = SimVar.GetSimVarValue(`L:A32NX_DEEP_FAILURE_${side.failureId}_ACTIVE`, 'number') >= WiperAuthority.STOPPED_AT;

      if (this.settle[i] > 0) {
        this.settle[i]--;
      } else if (failed && on) {
        this.heldOn[i] = true;
        this.toggle(i, side.circuit);
      } else if (!failed && this.heldOn[i]) {
        this.heldOn[i] = false;
        if (!on) {
          this.toggle(i, side.circuit);
        }
      }

      SimVar.SetSimVarValue(side.selectedVar, 'number', on || this.heldOn[i] ? 1 : 0);
    });
  }

  private toggle(side: number, circuit: number): void {
    SimVar.SetSimVarValue('K:ELECTRICAL_CIRCUIT_TOGGLE', 'number', circuit);
    this.settle[side] = WiperAuthority.SETTLE_FRAMES;
  }
}

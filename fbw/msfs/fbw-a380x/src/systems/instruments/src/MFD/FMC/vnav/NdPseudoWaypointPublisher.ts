// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { EfisSide, GenericDataListenerSync, NdSymbol } from '@flybywiresim/fbw-sdk';

import { VnavAircraftState, VnavProfile } from './types';
import { buildVnavPseudoWaypointSymbols } from './NdPseudoWaypoints';

export class VnavNdPseudoWaypointPublisher {
  private readonly syncer = new GenericDataListenerSync();

  constructor(private readonly side: EfisSide) {}

  pseudoWaypointSymbols(profile: VnavProfile, aircraftState: VnavAircraftState): NdSymbol[] {
    return buildVnavPseudoWaypointSymbols(profile, aircraftState);
  }

  publish(profile: VnavProfile, aircraftState: VnavAircraftState): void {
    this.syncer.sendEvent(`A32NX_EFIS_${this.side}_SYMBOLS`, this.pseudoWaypointSymbols(profile, aircraftState));
  }
}

// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { NdPwpSymbolTypeFlags, NdSymbol, NdSymbolTypeFlags } from '@flybywiresim/fbw-sdk';

import { VnavAircraftState, VnavProfile, VnavPseudoWaypoint, VnavPseudoWaypointIdent } from './types';

const PWP_ICON_BY_IDENT: Partial<Record<VnavPseudoWaypointIdent, NdPwpSymbolTypeFlags>> = {
  '(T/C)': NdPwpSymbolTypeFlags.PwpStartOfClimb,
  '(T/D)': NdPwpSymbolTypeFlags.PwpTopOfDescent,
  '(DECEL)': NdPwpSymbolTypeFlags.PwpDecel,
  '(S/C)': NdPwpSymbolTypeFlags.PwpSpeedChange,
};

const BEHIND_TOLERANCE_NM = 0.5;

export function buildVnavPseudoWaypointSymbols(profile: VnavProfile, aircraftState: VnavAircraftState): NdSymbol[] {
  if (profile.invalidReason) {
    return [];
  }

  const symbols: NdSymbol[] = [];

  for (const pwp of profile.pseudoWaypoints) {
    const icon = PWP_ICON_BY_IDENT[pwp.ident];
    if (icon === undefined) {
      continue;
    }

    const distanceFromAirplane = pwp.distanceFromStartNm - aircraftState.distanceFromStartNm;
    if (distanceFromAirplane < -BEHIND_TOLERANCE_NM) {
      continue;
    }

    symbols.push(symbolForPwp(pwp, icon, distanceFromAirplane));
  }

  return symbols;
}

function symbolForPwp(pwp: VnavPseudoWaypoint, icon: NdPwpSymbolTypeFlags, distanceFromAirplane: number): NdSymbol {
  return {
    databaseId: `VNAV-PWP-${pwp.ident}-${Math.round(pwp.distanceFromStartNm * 10)}`,
    ident: pwp.ident.replace(/[()]/g, ''),
    location: null,
    distanceFromAirplane,
    type: NdSymbolTypeFlags.FlightPlan,
    typePwp: icon,
  };
}

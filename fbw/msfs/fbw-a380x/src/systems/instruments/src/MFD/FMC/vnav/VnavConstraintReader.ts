// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { AltitudeDescriptor, SpeedDescriptor } from '@flybywiresim/fbw-sdk';
import type { AltitudeConstraint, SpeedConstraint } from '@flybywiresim/fbw-sdk';
import type { BaseFlightPlan } from '@fmgc/flightplanning/plans/BaseFlightPlan';
import type { FlightPlanLeg } from '@fmgc/flightplanning/legs/FlightPlanLeg';
import { VnavLeg } from './types';

export class VnavConstraintReader {
  static read(plan: BaseFlightPlan | undefined | null): VnavLeg[] {
    if (!plan) {
      return [];
    }

    const allLegs = plan.allLegs;
    const legs: VnavLeg[] = [];

    for (let i = 0; i < allLegs.length; i++) {
      const element = allLegs[i];

      if (element.isDiscontinuity === true) {
        continue;
      }

      const leg = element;

      legs.push({
        index: i,
        ident: leg.ident,
        distanceFromStartNm: leg.calculated?.cumulativeDistanceWithTransitions ?? legs[i - 1]?.distanceFromStartNm ?? 0,
        altitudeConstraint: VnavConstraintReader.readAltitudeConstraint(leg.altitudeConstraint),
        speedConstraintKt: VnavConstraintReader.readSpeedConstraint(leg.speedConstraint),
        isMissedApproach: leg.segment === plan.missedApproachSegment,
        segment: VnavConstraintReader.readSegment(plan, leg),
      });
    }

    return legs;
  }

  static readExcludingMissedApproach(plan: BaseFlightPlan | undefined | null): VnavLeg[] {
    return VnavConstraintReader.read(plan).filter((leg) => !leg.isMissedApproach);
  }

  private static readAltitudeConstraint(constraint: AltitudeConstraint | undefined): VnavLeg['altitudeConstraint'] {
    if (!constraint || constraint.altitudeDescriptor === undefined || constraint.altitude1 === undefined) {
      return undefined;
    }

    switch (constraint.altitudeDescriptor) {
      case AltitudeDescriptor.AtAlt1:
      case AltitudeDescriptor.AtAlt1GsIntcptAlt2:
      case AltitudeDescriptor.AtAlt1AngleAlt2:
        return { type: 'at', ft1: constraint.altitude1 };
      case AltitudeDescriptor.AtOrAboveAlt1:
      case AltitudeDescriptor.AtOrAboveAlt1GsIntcptAlt2:
      case AltitudeDescriptor.AtOrAboveAlt1AngleAlt2:
        return { type: 'atOrAbove', ft1: constraint.altitude1 };
      case AltitudeDescriptor.AtOrAboveAlt2:
        return constraint.altitude2 === undefined ? undefined : { type: 'atOrAbove', ft1: constraint.altitude2 };
      case AltitudeDescriptor.AtOrBelowAlt1:
      case AltitudeDescriptor.AtOrBelowAlt1AngleAlt2:
        return { type: 'atOrBelow', ft1: constraint.altitude1 };
      case AltitudeDescriptor.BetweenAlt1Alt2:
        return constraint.altitude2 === undefined
          ? { type: 'atOrBelow', ft1: constraint.altitude1 }
          : { type: 'between', ft1: constraint.altitude1, ft2: constraint.altitude2 };
      default:
        return undefined;
    }
  }

  private static readSpeedConstraint(constraint: SpeedConstraint | undefined): number | undefined {
    if (!constraint || constraint.speed === undefined || constraint.speed <= 100) {
      return undefined;
    }

    switch (constraint.speedDescriptor) {
      case SpeedDescriptor.Mandatory:
      case SpeedDescriptor.Maximum:
        return constraint.speed;
      default:
        return undefined;
    }
  }

  private static readSegment(plan: BaseFlightPlan, leg: FlightPlanLeg): VnavLeg['segment'] {
    const segment = leg.segment;

    if (
      segment === plan.originSegment ||
      segment === plan.departureRunwayTransitionSegment ||
      segment === plan.departureSegment ||
      segment === plan.departureEnrouteTransitionSegment
    ) {
      return 'departure';
    }

    if (segment === plan.enrouteSegment) {
      return 'enroute';
    }

    if (
      segment === plan.arrivalEnrouteTransitionSegment ||
      segment === plan.arrivalSegment ||
      segment === plan.arrivalRunwayTransitionSegment
    ) {
      return 'arrival';
    }

    return 'approach';
  }
}

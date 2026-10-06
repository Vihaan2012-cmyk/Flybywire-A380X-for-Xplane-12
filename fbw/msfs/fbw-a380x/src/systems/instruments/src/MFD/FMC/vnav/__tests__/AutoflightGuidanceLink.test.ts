// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { describe, expect, it } from 'vitest';
import { RequestedVerticalMode } from '@fmgc/guidance/ControlLaws';
import { DescentGuidanceInputs, PathCaptureState, decideDescentGuidance, nextSpeedConditions } from '../AutoflightGuidanceLink';

function onIdlePath(overrides: Partial<DescentGuidanceInputs> = {}): DescentGuidanceInputs {
  return {
    linearDeviationFt: 0,
    pressureAltitudeFt: 25_000,
    pathVerticalSpeedFpm: -2_400,
    pathAngleDeg: -3,
    pathIsGeometric: false,
    pastTopOfDescent: true,
    verticalSpeedFpm: -2_400,
    isHoldActive: false,
    isSpeedAuto: true,
    isApproachPhase: false,
    isAboveSpeedLimitAltitude: true,
    isCloseToAirfieldElevation: false,
    isInOverspeed: false,
    isInUnderspeed: false,
    ...overrides,
  };
}

describe('decideDescentGuidance', () => {
  it('flies VPATH THRUST on the idle path, targeting the path altitude and its vertical speed', () => {
    const d = decideDescentGuidance(onIdlePath({ linearDeviationFt: 20 }), PathCaptureState.OnPath);
    expect(d.mode).toBe(RequestedVerticalMode.VpathThrust);
    expect(d.targetAltitudeFt).toBe(25_000 - 20);
    expect(d.targetVerticalSpeed).toBe(-2_400);
    expect(d.pathCaptureState).toBe(PathCaptureState.OnPath);
  });

  it('flies VPATH SPEED on a geometric segment', () => {
    expect(decideDescentGuidance(onIdlePath({ pathIsGeometric: true }), PathCaptureState.OnPath).mode).toBe(RequestedVerticalMode.VpathSpeed);
  });

  it('goes SPEED THRUST (idle, pitch on speed) well above the path', () => {
    const d = decideDescentGuidance(onIdlePath({ linearDeviationFt: 1_500, verticalSpeedFpm: -2_400 }), PathCaptureState.OffPath);
    expect(d.pathCaptureState).toBe(PathCaptureState.OffPath);
    expect(d.mode).toBe(RequestedVerticalMode.SpeedThrust);
  });

  it('descends at 1,000 ft/min below the path above the speed limit altitude, 500 ft/min under it', () => {
    const high = decideDescentGuidance(onIdlePath({ linearDeviationFt: -1_500 }), PathCaptureState.OffPath);
    expect(high.mode).toBe(RequestedVerticalMode.VsSpeed);
    expect(high.targetVerticalSpeed).toBe(-1_000);
    const low = decideDescentGuidance(onIdlePath({ linearDeviationFt: -1_500, isAboveSpeedLimitAltitude: false }), PathCaptureState.OffPath);
    expect(low.targetVerticalSpeed).toBe(-500);
  });

  it('holds half the path angle below a geometric segment', () => {
    const d = decideDescentGuidance(onIdlePath({ linearDeviationFt: -1_500, pathIsGeometric: true, pathAngleDeg: -3 }), PathCaptureState.OffPath);
    expect(d.mode).toBe(RequestedVerticalMode.FpaSpeed);
    expect(d.targetVerticalSpeed).toBe(-1.5);
  });

  it('flies an early descent before T/D at a fixed vertical speed', () => {
    const d = decideDescentGuidance(
      onIdlePath({ pastTopOfDescent: false, linearDeviationFt: -2_000, pathVerticalSpeedFpm: 0, pathAngleDeg: 0, pathIsGeometric: true }),
      PathCaptureState.OffPath,
    );
    expect(d.mode).toBe(RequestedVerticalMode.VsSpeed);
    expect(d.targetVerticalSpeed).toBe(-1_000);
  });

  it('descends at 1,000 ft/min in a hold, and on speed when overspeeding', () => {
    expect(decideDescentGuidance(onIdlePath({ isHoldActive: true }), PathCaptureState.OnPath).targetVerticalSpeed).toBe(-1_000);
    expect(decideDescentGuidance(onIdlePath({ isInOverspeed: true }), PathCaptureState.OnPath).mode).toBe(RequestedVerticalMode.SpeedThrust);
  });

  it('captures the path once the deviation is inside the capture window, and latches on it', () => {
    const far = decideDescentGuidance(onIdlePath({ linearDeviationFt: -300, verticalSpeedFpm: -3_400 }), PathCaptureState.OffPath);
    expect(far.pathCaptureState).toBe(PathCaptureState.OffPath);
    const near = decideDescentGuidance(onIdlePath({ linearDeviationFt: -80, verticalSpeedFpm: -3_400 }), PathCaptureState.OffPath);
    expect(near.pathCaptureState).toBe(PathCaptureState.InPathCapture);
    const on = decideDescentGuidance(onIdlePath({ linearDeviationFt: -30, verticalSpeedFpm: -2_600 }), near.pathCaptureState);
    expect(on.pathCaptureState).toBe(PathCaptureState.OnPath);
    expect(on.mode).toBe(RequestedVerticalMode.VpathThrust);
  });
});

describe('nextSpeedConditions', () => {
  it('needs 5 kt past the upper limit to call an overspeed and clears it back under the limit', () => {
    let c = { overspeed: false, underspeed: false };
    c = nextSpeedConditions(303, 260, 300, c);
    expect(c.overspeed).toBe(false);
    c = nextSpeedConditions(306, 260, 300, c);
    expect(c.overspeed).toBe(true);
    c = nextSpeedConditions(302, 260, 300, c);
    expect(c.overspeed).toBe(true);
    c = nextSpeedConditions(299, 260, 300, c);
    expect(c.overspeed).toBe(false);
  });

  it('calls an underspeed below the lower limit and clears it 5 kt above', () => {
    let c = nextSpeedConditions(258, 260, 300, { overspeed: false, underspeed: false });
    expect(c.underspeed).toBe(true);
    c = nextSpeedConditions(263, 260, 300, c);
    expect(c.underspeed).toBe(true);
    c = nextSpeedConditions(266, 260, 300, c);
    expect(c.underspeed).toBe(false);
  });
});

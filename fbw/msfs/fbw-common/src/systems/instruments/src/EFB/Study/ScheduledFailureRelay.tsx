// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { useEffect, useRef } from 'react';
import { useSimVar } from '@flybywiresim/fbw-sdk-react';
import { useFailuresOrchestrator } from '../failures-orchestrator-provider';

export const ScheduledFailureRelay = () => {
  const [due] = useSimVar('L:A32NX_DEEP_SCRIPTED_EXTERNAL_FIRE_ID', 'number', 250);
  const { activate } = useFailuresOrchestrator();
  const handled = useRef(0);

  useEffect(() => {
    const id = Math.round(due);
    if (id <= 0) {
      handled.current = 0;
      return;
    }
    if (id === handled.current) {
      return;
    }
    handled.current = id;
    void activate(id).finally(() => SimVar.SetSimVarValue('L:A32NX_DEEP_SCRIPTED_EXTERNAL_FIRE_ACK', 'number', id));
  }, [due]);

  return null;
};

//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { EventBus } from '@microsoft/msfs-sdk';
import { ArincEventSubscriber } from './ArincEventBusSubscriber';

export class ArincEventBus extends EventBus {
  public getArincSubscriber<E>(): ArincEventSubscriber<E> {
    return new ArincEventSubscriber<E>(this);
  }
}

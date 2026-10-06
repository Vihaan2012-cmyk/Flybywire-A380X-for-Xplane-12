// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { ArraySubject, ClockEvents, ConsumerSubject, EventBus } from '@microsoft/msfs-sdk';

export type AocFolder =
  | 'INBOX'
  | 'OPS'
  | 'METAR'
  | 'SENT'
  | 'REQUESTS'
  | 'LOADSHEET'
  | 'REPORTS'
  | 'ADC DELAY'
  | 'DIVERSION';

export interface AocMessage {
  id: number;
  folder: AocFolder;
  title: string;
  from: string;
  zulu: string;
  timeMs: number;
  lines: string[];
  isNew: boolean;
}

export class AocMessageStore {
  private static readonly instances = new Map<EventBus, AocMessageStore>();

  public readonly messages = ArraySubject.create<AocMessage>([]);

  private nextId = 1;

  private readonly zuluTime: ConsumerSubject<number>;

  private constructor(private readonly bus: EventBus) {
    const sub = this.bus.getSubscriber<ClockEvents>();
    this.zuluTime = ConsumerSubject.create(sub.on('simTime'), Date.now());
  }

  static instance(bus: EventBus): AocMessageStore {
    let inst = AocMessageStore.instances.get(bus);
    if (!inst) {
      inst = new AocMessageStore(bus);
      AocMessageStore.instances.set(bus, inst);
    }
    return inst;
  }

  private currentZulu(): string {
    const date = new Date(this.zuluTime.get());
    return `${String(date.getUTCHours()).padStart(2, '0')}${String(date.getUTCMinutes()).padStart(2, '0')}`;
  }

  add(folder: AocFolder, title: string, from: string, lines: string[]): AocMessage {
    const message: AocMessage = {
      id: this.nextId++,
      folder,
      title,
      from,
      zulu: this.currentZulu(),
      timeMs: this.zuluTime.get(),
      lines,
      isNew: true,
    };
    this.messages.insert(message, 0);
    return message;
  }

  markRead(id: number): void {
    const arr = this.messages.getArray();
    const index = arr.findIndex((m) => m.id === id);
    if (index >= 0 && arr[index].isNew) {
      const updated: AocMessage = { ...arr[index], isNew: false };
      this.messages.removeAt(index);
      this.messages.insert(updated, index);
    }
  }

  clear(folder: AocFolder): void {
    const arr = this.messages.getArray();
    for (let i = arr.length - 1; i >= 0; i--) {
      if (arr[i].folder === folder) {
        this.messages.removeAt(i);
      }
    }
  }
}

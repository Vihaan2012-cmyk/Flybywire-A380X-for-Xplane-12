//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { ArraySubject, Subject } from '@microsoft/msfs-sdk';

export interface MaintenanceEvent {
  readonly kind: MaintenanceEventKind;
  readonly id: number;
  readonly zuluS: number;
  readonly phase: number;
  readonly ata: number;
}

export enum MaintenanceEventKind {
  FailureArmed = 1,
  FailureCleared = 2,
  SystemFaultActive = 3,
  SystemFaultCleared = 4,
  BreakerTripped = 5,
  BreakerReset = 6,
  MelDeferred = 7,
  MelReleased = 8,
}

interface CatalogueFailure {
  id: number;
  name: string;
  component: string;
  ataChapterNumber: number;
  ataChapterName: string;
}

export interface MaintenanceBreaker {
  id: string;
  label: string;
  name: string;
  bus: string;
  ratingAmperes: number;
  ataChapterNumber: number;
}

type CatalogueBreaker = MaintenanceBreaker;

interface CatalogueMelEntry {
  melReference: string;
  ataChapterNumber: number;
  ataChapterName: string;
  failures: number[];
}

interface CatalogueChapter {
  number: number;
  name: string;
}

const CATALOGUE_URLS = ['/VFS/html_ui/Pages/VCockpit/Instruments/A380X/EFB/catalogue.json', 'catalogue.json'];

const LOG_CAPACITY = 64;
const MEL_ITEMS = 64;

const PHASES = ['PREF', 'T/O', 'CLB', 'CRZ', 'DES', 'APPR', 'G/A', 'DONE'];

export class MaintenanceData {
  private static shared: MaintenanceData | undefined;

  static instance(): MaintenanceData {
    if (!MaintenanceData.shared) {
      MaintenanceData.shared = new MaintenanceData();
    }
    return MaintenanceData.shared;
  }

  readonly events = ArraySubject.create<MaintenanceEvent>([]);

  readonly ataActive = Subject.create<ReadonlyMap<number, number>>(new Map());

  readonly melDeferred = Subject.create<ReadonlyMap<number, number>>(new Map());

  readonly catalogueLoaded = Subject.create(false);

  private failures = new Map<number, CatalogueFailure>();

  private breakers: CatalogueBreaker[] = [];

  private melEntries: CatalogueMelEntry[] = [];

  private chapters = new Map<number, string>();

  private lastSeq = -1;

  private users = 0;

  private timer: ReturnType<typeof setInterval> | undefined;

  private constructor() {
    this.loadCatalogue(CATALOGUE_URLS);
  }

  private loadCatalogue(urls: string[]): void {
    const [url, ...rest] = urls;
    if (!url) {
      return;
    }
    fetch(url)
      .then((response) => {
        if (!response.ok) {
          throw new Error(`${response.status}`);
        }
        return response.json();
      })
      .then((json) => {
        this.failures = new Map((json.failures as CatalogueFailure[]).map((f) => [f.id, f]));
        this.breakers = json.breakers as CatalogueBreaker[];
        this.melEntries = json.melEntries as CatalogueMelEntry[];
        this.chapters = new Map((json.chapters as CatalogueChapter[]).map((c) => [c.number, c.name]));
        this.catalogueLoaded.set(true);
      })
      .catch(() => this.loadCatalogue(rest));
  }

  acquire(): void {
    this.users++;
    if (this.timer === undefined) {
      this.refresh();
      this.timer = setInterval(() => this.refresh(), 1000);
    }
  }

  release(): void {
    this.users = Math.max(0, this.users - 1);
    if (this.users === 0 && this.timer !== undefined) {
      clearInterval(this.timer);
      this.timer = undefined;
    }
  }

  private refresh(): void {
    const seq = SimVar.GetSimVarValue('L:A32NX_DEEP_MAINT_LOG_SEQ', 'number') as number;
    if (seq !== this.lastSeq) {
      this.lastSeq = seq;
      const count = Math.min(LOG_CAPACITY, SimVar.GetSimVarValue('L:A32NX_DEEP_MAINT_LOG_COUNT', 'number') as number);
      const events: MaintenanceEvent[] = [];
      for (let k = 0; k < count; k++) {
        events.push({
          kind: SimVar.GetSimVarValue(`L:A32NX_DEEP_MAINT_LOG_${k}_KIND`, 'number'),
          id: SimVar.GetSimVarValue(`L:A32NX_DEEP_MAINT_LOG_${k}_ID`, 'number'),
          zuluS: SimVar.GetSimVarValue(`L:A32NX_DEEP_MAINT_LOG_${k}_TIME`, 'number'),
          phase: SimVar.GetSimVarValue(`L:A32NX_DEEP_MAINT_LOG_${k}_PHASE`, 'number'),
          ata: SimVar.GetSimVarValue(`L:A32NX_DEEP_MAINT_LOG_${k}_ATA`, 'number'),
        });
      }
      this.events.set(events);
    }

    const ata = new Map<number, number>();
    for (const chapter of this.chapters.keys()) {
      const n = SimVar.GetSimVarValue(`L:A32NX_DEEP_MAINT_ATA_${chapter.toString().padStart(2, '0')}_ACTIVE`, 'number');
      if (n > 0) {
        ata.set(chapter, n);
      }
    }
    this.ataActive.set(ata);

    const mel = new Map<number, number>();
    for (let k = 0; k < MEL_ITEMS; k++) {
      if (SimVar.GetSimVarValue(`L:A32NX_DEEP_MEL_ITEM_${k}_DEFERRED`, 'number') > 0) {
        mel.set(k, SimVar.GetSimVarValue(`L:A32NX_DEEP_MEL_ITEM_${k}_REMAINING_HOURS`, 'number'));
      }
    }
    this.melDeferred.set(mel);
  }

  breakerList(): readonly MaintenanceBreaker[] {
    return this.breakers;
  }

  messageFor(e: MaintenanceEvent): string {
    switch (e.kind) {
      case MaintenanceEventKind.FailureArmed:
      case MaintenanceEventKind.FailureCleared:
      case MaintenanceEventKind.SystemFaultActive:
      case MaintenanceEventKind.SystemFaultCleared: {
        const f = this.failures.get(e.id);
        return f ? f.name.toUpperCase() : `FAULT ${e.id}`;
      }
      case MaintenanceEventKind.BreakerTripped:
      case MaintenanceEventKind.BreakerReset: {
        const b = this.breakers[e.id];
        return b ? `C/B ${b.name.toUpperCase()}` : `C/B ${e.id}`;
      }
      case MaintenanceEventKind.MelDeferred:
      case MaintenanceEventKind.MelReleased:
        return this.melLabel(e.id);
      default:
        return '';
    }
  }

  static isOnset(e: MaintenanceEvent): boolean {
    return (
      e.kind === MaintenanceEventKind.FailureArmed ||
      e.kind === MaintenanceEventKind.SystemFaultActive ||
      e.kind === MaintenanceEventKind.BreakerTripped ||
      e.kind === MaintenanceEventKind.MelDeferred
    );
  }

  static kindLabel(kind: MaintenanceEventKind): string {
    switch (kind) {
      case MaintenanceEventKind.FailureArmed:
        return 'FAULT';
      case MaintenanceEventKind.FailureCleared:
        return 'CLEARED';
      case MaintenanceEventKind.SystemFaultActive:
        return 'SYS FAULT';
      case MaintenanceEventKind.SystemFaultCleared:
        return 'SYS CLEARED';
      case MaintenanceEventKind.BreakerTripped:
        return 'C/B TRIP';
      case MaintenanceEventKind.BreakerReset:
        return 'C/B RESET';
      case MaintenanceEventKind.MelDeferred:
        return 'MEL DEFER';
      case MaintenanceEventKind.MelReleased:
        return 'MEL RELEASE';
      default:
        return '';
    }
  }

  ataFor(e: MaintenanceEvent): number {
    if (e.ata > 0) {
      return e.ata;
    }
    if (e.kind === MaintenanceEventKind.MelDeferred || e.kind === MaintenanceEventKind.MelReleased) {
      return this.melEntries[e.id]?.ataChapterNumber ?? 0;
    }
    return 0;
  }

  chapterName(ata: number): string {
    return this.chapters.get(ata) ?? `${ata}`;
  }

  melLabel(item: number): string {
    const m = this.melEntries[item];
    return m ? `MEL ${m.melReference}` : `MEL ITEM ${item + 1}`;
  }

  melCovers(item: number): string[] {
    const m = this.melEntries[item];
    return m ? m.failures.map((id) => this.failures.get(id)?.name ?? `${id}`) : [];
  }

  static formatZulu(zuluS: number): string {
    const s = Math.max(0, Math.floor(zuluS)) % 86400;
    const hh = Math.floor(s / 3600);
    const mm = Math.floor((s % 3600) / 60);
    return `${hh.toString().padStart(2, '0')}${mm.toString().padStart(2, '0')}`;
  }

  static phaseName(phase: number): string {
    return PHASES[Math.round(phase)] ?? '';
  }
}

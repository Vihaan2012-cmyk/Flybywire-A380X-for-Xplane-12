//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { FSComponent, VNode } from '@microsoft/msfs-sdk';
import { DestroyableComponent } from '../../../../MsfsAvionicsCommon/DestroyableComponent';
import { AbstractOitAvncsPageProps } from '../../../OIT';
import { MaintenanceData, MaintenanceEvent, MaintenanceEventKind } from '../../../System/MaintenanceData';

interface OitAvncsMaintenanceReportProps extends AbstractOitAvncsPageProps {
  readonly mode: 'current' | 'post';
  readonly ata?: number;
}

function faultKey(e: MaintenanceEvent): string {
  switch (e.kind) {
    case MaintenanceEventKind.FailureArmed:
    case MaintenanceEventKind.FailureCleared:
      return `F${e.id}`;
    case MaintenanceEventKind.SystemFaultActive:
    case MaintenanceEventKind.SystemFaultCleared:
      return `S${e.id}`;
    case MaintenanceEventKind.BreakerTripped:
    case MaintenanceEventKind.BreakerReset:
      return `B${e.id}`;
    default:
      return `M${e.id}`;
  }
}

export class OitAvncsMaintenanceReport extends DestroyableComponent<OitAvncsMaintenanceReportProps> {
  private readonly data = MaintenanceData.instance();

  private readonly bodyRef = FSComponent.createRef<HTMLDivElement>();

  private rows(): MaintenanceEvent[] {
    const events = this.data.events.getArray();
    let rows: MaintenanceEvent[];
    if (this.props.mode === 'current') {
      const seen = new Set<string>();
      rows = [];
      for (const e of events) {
        const key = faultKey(e);
        if (seen.has(key)) {
          continue;
        }
        seen.add(key);
        if (MaintenanceData.isOnset(e)) {
          rows.push(e);
        }
      }
    } else {
      rows = [...events];
    }
    if (this.props.ata !== undefined) {
      rows = rows.filter((e) => this.data.ataFor(e) === this.props.ata);
    }
    return rows;
  }

  private redraw(): void {
    const body = this.bodyRef.getOrDefault();
    if (!body) {
      return;
    }
    while (body.firstChild) {
      body.removeChild(body.firstChild);
    }
    const rows = this.rows();
    if (rows.length === 0) {
      const empty = document.createElement('div');
      empty.className = 'oit-maint-empty';
      empty.textContent = this.props.mode === 'current' ? 'NO FAULT' : 'NO EVENT RECORDED';
      body.appendChild(empty);
      return;
    }
    for (const e of rows) {
      const onset = MaintenanceData.isOnset(e);
      const row = document.createElement('div');
      row.className = 'oit-maint-row';
      const cells: [string, string][] = [
        ['w8 tc', MaintenanceData.formatZulu(e.zuluS)],
        ['w8 tc', MaintenanceData.phaseName(e.phase)],
        ['w8 tc', `${this.data.ataFor(e) || '--'}`],
        ['f1', this.data.messageFor(e)],
        ['w15 tc', MaintenanceData.kindLabel(e.kind)],
      ];
      for (const [cls, text] of cells) {
        const cell = document.createElement('div');
        cell.className = `oit-ccom-inbox-msg-table-line ${cls} ${onset ? 'oit-amber-text' : 'oit-green-text'}`;
        cell.textContent = text;
        row.appendChild(cell);
      }
      body.appendChild(row);
    }
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);
    this.subscriptions.push(
      this.data.events.sub(() => this.redraw(), true),
      this.data.catalogueLoaded.sub(() => this.redraw()),
    );
  }

  private title(): string {
    const base = this.props.mode === 'current' ? 'Current Flight Report' : 'Post Flight Report';
    return this.props.ata !== undefined ? `${base} - ATA ${this.data.chapterName(this.props.ata)}` : base;
  }

  render(): VNode {
    return (
      <>
        <div class="oit-ccom-headline">{this.title()}</div>
        <div class="oit-ccom-inbox-msg-table">
          <div class="fr ass">
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w8">UTC</div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w8">PH</div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w8">ATA</div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib f1">Message</div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w15">Type</div>
          </div>
          <div class="oit-maint-table-body" ref={this.bodyRef} />
        </div>
        <div style="flex-grow: 1" />
      </>
    );
  }
}

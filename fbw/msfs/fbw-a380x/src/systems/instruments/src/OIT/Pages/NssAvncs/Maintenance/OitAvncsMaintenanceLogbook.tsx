//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { FSComponent, VNode } from '@microsoft/msfs-sdk';
import { DestroyableComponent } from '../../../../MsfsAvionicsCommon/DestroyableComponent';
import { AbstractOitAvncsPageProps } from '../../../OIT';
import { MaintenanceData } from '../../../System/MaintenanceData';

interface OitAvncsMaintenanceLogbookProps extends AbstractOitAvncsPageProps {}

export class OitAvncsMaintenanceLogbook extends DestroyableComponent<OitAvncsMaintenanceLogbookProps> {
  private readonly data = MaintenanceData.instance();

  private readonly bodyRef = FSComponent.createRef<HTMLDivElement>();

  private redraw(deferred: ReadonlyMap<number, number>): void {
    const body = this.bodyRef.getOrDefault();
    if (!body) {
      return;
    }
    while (body.firstChild) {
      body.removeChild(body.firstChild);
    }
    if (deferred.size === 0) {
      const empty = document.createElement('div');
      empty.className = 'oit-maint-empty';
      empty.textContent = 'NO DEFERRED ITEM';
      body.appendChild(empty);
      return;
    }
    for (const [item, hours] of [...deferred.entries()].sort(([a], [b]) => a - b)) {
      const row = document.createElement('div');
      row.className = 'oit-maint-row';
      const ref = document.createElement('div');
      ref.className = 'oit-ccom-inbox-msg-table-line w15 oit-amber-text';
      ref.textContent = this.data.melLabel(item);
      const covers = document.createElement('div');
      covers.className = 'oit-ccom-inbox-msg-table-line f1 oit-amber-text';
      covers.textContent = this.data.melCovers(item).join(', ').toUpperCase();
      const left = document.createElement('div');
      left.className = 'oit-ccom-inbox-msg-table-line w12 tc oit-amber-text';
      left.textContent = hours > 0 ? `${hours.toFixed(1)} H` : 'EXPIRED';
      row.append(ref, covers, left);
      body.appendChild(row);
    }
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);
    this.subscriptions.push(
      this.data.melDeferred.sub((deferred) => this.redraw(deferred), true),
      this.data.catalogueLoaded.sub(() => this.redraw(this.data.melDeferred.get())),
    );
  }

  render(): VNode {
    return (
      <>
        <div class="oit-ccom-headline">Logbook - MEL Deferred Items</div>
        <div class="oit-ccom-inbox-msg-table">
          <div class="fr ass">
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w15">MEL Ref</div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib f1">Placarded</div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w12">Remaining</div>
          </div>
          <div class="oit-maint-table-body" ref={this.bodyRef} />
        </div>
        <div style="flex-grow: 1" />
      </>
    );
  }
}

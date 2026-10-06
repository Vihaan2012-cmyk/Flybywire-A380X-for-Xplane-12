//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { FSComponent, VNode } from '@microsoft/msfs-sdk';
import { DestroyableComponent } from '../../../../MsfsAvionicsCommon/DestroyableComponent';
import { AbstractOitAvncsPageProps } from '../../../OIT';
import { MaintenanceData } from '../../../System/MaintenanceData';

interface OitAvncsMaintenanceSystemStatusProps extends AbstractOitAvncsPageProps {}

export class OitAvncsMaintenanceSystemStatus extends DestroyableComponent<OitAvncsMaintenanceSystemStatusProps> {
  private readonly data = MaintenanceData.instance();

  private readonly bodyRef = FSComponent.createRef<HTMLDivElement>();

  private redraw(active: ReadonlyMap<number, number>): void {
    const body = this.bodyRef.getOrDefault();
    if (!body) {
      return;
    }
    while (body.firstChild) {
      body.removeChild(body.firstChild);
    }
    const chapters = [...active.entries()].sort(([a], [b]) => a - b);
    if (chapters.length === 0) {
      const empty = document.createElement('div');
      empty.className = 'oit-maint-empty';
      empty.textContent = 'ALL SYSTEMS NORMAL';
      body.appendChild(empty);
      return;
    }
    for (const [ata, count] of chapters) {
      const row = document.createElement('div');
      row.className = 'oit-maint-row';
      const name = document.createElement('div');
      name.className = 'oit-ccom-inbox-msg-table-line f1 oit-amber-text';
      name.textContent = this.data.chapterName(ata).toUpperCase();
      const n = document.createElement('div');
      n.className = 'oit-ccom-inbox-msg-table-line w15 tc oit-amber-text';
      n.textContent = `${count}`;
      row.append(name, n);
      row.addEventListener('click', () => this.props.uiService.navigateTo(`nss-avncs/maintenance/system-status/${ata}`));
      body.appendChild(row);
    }
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);
    this.subscriptions.push(
      this.data.ataActive.sub((active) => this.redraw(active), true),
      this.data.catalogueLoaded.sub(() => this.redraw(this.data.ataActive.get())),
    );
  }

  render(): VNode {
    return (
      <>
        <div class="oit-ccom-headline">System Status</div>
        <div class="oit-ccom-inbox-msg-table">
          <div class="fr ass">
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib f1">ATA Chapter</div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w15">Active Faults</div>
          </div>
          <div class="oit-maint-table-body" ref={this.bodyRef} />
        </div>
        <div style="flex-grow: 1" />
      </>
    );
  }
}

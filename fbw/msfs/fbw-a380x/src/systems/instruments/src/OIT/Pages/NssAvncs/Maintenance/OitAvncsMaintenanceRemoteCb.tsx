//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { FSComponent, MappedSubject, Subject, VNode } from '@microsoft/msfs-sdk';
import { DestroyableComponent } from '../../../../MsfsAvionicsCommon/DestroyableComponent';
import { Button } from '../../../../MsfsAvionicsCommon/UiWidgets/Button';
import { InputField } from '../../../../MsfsAvionicsCommon/UiWidgets/InputField';
import { DataEntryFormat } from '../../../../MFD/pages/common/DataEntryFormats';
import { AbstractOitAvncsPageProps } from '../../../OIT';
import { MaintenanceBreaker, MaintenanceData } from '../../../System/MaintenanceData';

interface OitAvncsMaintenanceRemoteCbProps extends AbstractOitAvncsPageProps {}

const REMOTE_CB_CTL_PB = 'L:A380X_REMOTE_CB_CTRL';

const STATE_LABEL: Record<number, string> = {
  0: 'CLOSED',
  1: 'OPEN',
  2: 'TRIPPED THERMAL',
  3: 'TRIPPED MAGNETIC',
  4: 'TRIPPED ARC FAULT',
  5: 'LOCKED OUT',
};

const STATE_CLASS: Record<number, string> = {
  0: 'oit-green-text',
  1: 'oit-cyan-text',
  2: 'oit-amber-text',
  3: 'oit-amber-text',
  4: 'oit-amber-text',
  5: 'oit-amber-text',
};

const ROWS_PER_PAGE = 12;

function lvarKey(id: string): string {
  return id.replace(/[^A-Za-z0-9]/g, '_').toUpperCase();
}

class CbSearchFormat implements DataEntryFormat<string> {
  public readonly placeholder = '--------------------';

  public readonly maxDigits = 20;

  public format(value: string | null): [string | null, string | null, string | null] {
    return [value ? value : this.placeholder, null, null];
  }

  public async parse(input: string) {
    const trimmed = input.trim();
    return trimmed === '' || trimmed === this.placeholder ? null : trimmed.toUpperCase();
  }
}

export class OitAvncsMaintenanceRemoteCb extends DestroyableComponent<OitAvncsMaintenanceRemoteCbProps> {
  private readonly data = MaintenanceData.instance();

  private readonly bodyRef = FSComponent.createRef<HTMLDivElement>();

  private readonly ctlOn = Subject.create(false);

  private readonly showAll = Subject.create(false);

  private readonly search = Subject.create<string | null>(null);

  private readonly selected = Subject.create<MaintenanceBreaker | null>(null);

  private readonly selectedText = this.selected.map((b) => (b ? b.name.toUpperCase() : 'NO C/B SELECTED'));

  private readonly ctlText = this.ctlOn.map((on) =>
    on ? 'REMOTE C/B CTL ON' : 'REMOTE C/B CTL OFF - SELECT ELEC REMOTE C/B CTL ON (OVHD MAINT PANEL)',
  );

  private readonly commandsDisabled = MappedSubject.create(([on, b]) => !on || b === null, this.ctlOn, this.selected);

  private readonly page = Subject.create(0);

  private readonly pageCount = Subject.create(1);

  private readonly matchCount = Subject.create(0);

  private readonly pageText = MappedSubject.create(([p, n, m]) => `${m} C/B  PAGE ${p + 1}/${n}`, this.page, this.pageCount, this.matchCount);

  private readonly prevDisabled = this.page.map((p) => p <= 0);

  private readonly nextDisabled = MappedSubject.create(([p, n]) => p >= n - 1, this.page, this.pageCount);

  private readonly clearDisabled = this.search.map((s) => s === null);

  private timer: ReturnType<typeof setInterval> | undefined;

  private states = new Map<string, number>();

  private refresh(): void {
    this.ctlOn.set(SimVar.GetSimVarValue(REMOTE_CB_CTL_PB, 'number') > 0);
    const states = new Map<string, number>();
    for (const b of this.data.breakerList()) {
      states.set(b.id, SimVar.GetSimVarValue(`L:A32NX_BKR_${lvarKey(b.id)}_STATUS`, 'number'));
    }
    this.states = states;
    this.redraw();
  }

  private command(cmd: 1 | 2): void {
    const b = this.selected.get();
    if (!b || !this.ctlOn.get()) {
      return;
    }
    SimVar.SetSimVarValue(`L:A32NX_BKR_${lvarKey(b.id)}_CMD`, 'number', cmd);
  }

  private matches(b: MaintenanceBreaker, query: string | null): boolean {
    if (query === null) {
      return this.showAll.get() || (this.states.get(b.id) ?? 0) !== 0;
    }
    const words = query.split(/\s+/).filter((w) => w !== '');
    const haystack = `${b.name} ${b.label} ${b.bus} ATA ${b.ataChapterNumber} ${b.ataChapterNumber}`.toUpperCase();
    return words.every((w) => haystack.includes(w));
  }

  private cell(cls: string, text: string, stateClass: string): HTMLDivElement {
    const cell = document.createElement('div');
    cell.className = `oit-cb-cell ${cls} ${stateClass}`;
    cell.textContent = text;
    return cell;
  }

  private redraw(): void {
    const body = this.bodyRef.getOrDefault();
    if (!body) {
      return;
    }
    while (body.firstChild) {
      body.removeChild(body.firstChild);
    }
    const query = this.search.get();
    const matching = this.data.breakerList().filter((b) => this.matches(b, query));
    this.matchCount.set(matching.length);
    const pages = Math.max(1, Math.ceil(matching.length / ROWS_PER_PAGE));
    this.pageCount.set(pages);
    if (this.page.get() > pages - 1) {
      this.page.set(pages - 1);
    }
    const first = this.page.get() * ROWS_PER_PAGE;
    const rows = matching.slice(first, first + ROWS_PER_PAGE);
    if (rows.length === 0) {
      const empty = document.createElement('div');
      empty.className = 'oit-maint-empty';
      if (this.data.breakerList().length === 0) {
        empty.textContent = 'C/B DATA NOT AVAILABLE';
      } else if (query !== null) {
        empty.textContent = `NO C/B MATCHES ${query}`;
      } else {
        empty.textContent = 'ALL C/B CLOSED';
      }
      body.appendChild(empty);
      return;
    }
    const selectedId = this.selected.get()?.id;
    for (const b of rows) {
      const state = this.states.get(b.id) ?? 0;
      const stateClass = STATE_CLASS[state] ?? '';
      const row = document.createElement('div');
      row.className = `oit-cb-row${b.id === selectedId ? ' selected' : ''}`;
      row.appendChild(this.cell('oit-cb-name', b.name.toUpperCase(), stateClass));
      row.appendChild(this.cell('oit-cb-ata', `${b.ataChapterNumber}`, stateClass));
      row.appendChild(this.cell('oit-cb-bus', b.bus, stateClass));
      row.appendChild(this.cell('oit-cb-rating', `${b.ratingAmperes} A`, stateClass));
      row.appendChild(this.cell('oit-cb-state', STATE_LABEL[state] ?? `${state}`, stateClass));
      row.addEventListener('click', () => {
        this.selected.set(b);
        this.redraw();
      });
      body.appendChild(row);
    }
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);
    this.subscriptions.push(
      this.selectedText,
      this.ctlText,
      this.commandsDisabled,
      this.pageText,
      this.prevDisabled,
      this.nextDisabled,
      this.clearDisabled,
      this.showAll.sub(() => {
        this.page.set(0);
        this.redraw();
      }),
      this.search.sub(() => {
        this.page.set(0);
        this.redraw();
      }),
      this.page.sub(() => this.redraw()),
      this.data.catalogueLoaded.sub(() => this.refresh()),
    );
    this.refresh();
    this.timer = setInterval(() => this.refresh(), 1000);
  }

  destroy(): void {
    if (this.timer !== undefined) {
      clearInterval(this.timer);
    }
    super.destroy();
  }

  render(): VNode {
    return (
      <>
        <div class="oit-ccom-headline">Remote C/B Control</div>
        <div class="oit-maint-ctl-line">{this.ctlText}</div>
        <div class="fr oit-cb-toolbar">
          <div class="oit-cb-search-label">SEARCH</div>
          <InputField<string>
            dataEntryFormat={new CbSearchFormat()}
            value={this.search}
            containerStyle="width: 360px; margin-right: 10px;"
            alignText="flex-start"
            hEventConsumer={this.props.container.hEventConsumer}
            interactionMode={this.props.container.interactionMode}
            errorHandler={() => {}}
          />
          <Button label="CLEAR" disabled={this.clearDisabled} onClick={() => this.search.set(null)} />
          <div style="width: 20px" />
          <Button label="ABNORMAL" selected={this.showAll.map((a) => !a)} onClick={() => this.showAll.set(false)} />
          <Button label="ALL" selected={this.showAll} onClick={() => this.showAll.set(true)} />
          <div style="flex-grow: 1" />
          <Button label="PREV" disabled={this.prevDisabled} onClick={() => this.page.set(Math.max(0, this.page.get() - 1))} />
          <div class="oit-maint-pager">{this.pageText}</div>
          <Button
            label="NEXT"
            disabled={this.nextDisabled}
            onClick={() => this.page.set(Math.min(this.pageCount.get() - 1, this.page.get() + 1))}
          />
        </div>
        <div class="oit-cb-table">
          <div class="oit-cb-row oit-cb-header">
            <div class="oit-cb-cell oit-cb-name">C/B</div>
            <div class="oit-cb-cell oit-cb-ata">ATA</div>
            <div class="oit-cb-cell oit-cb-bus">BUS</div>
            <div class="oit-cb-cell oit-cb-rating">RATING</div>
            <div class="oit-cb-cell oit-cb-state">STATE</div>
          </div>
          <div class="oit-maint-table-body" ref={this.bodyRef} />
        </div>
        <div class="fr" style="margin-top: 8px; align-items: center;">
          <div class="oit-maint-selected">{this.selectedText}</div>
          <Button label="OPEN" disabled={this.commandsDisabled} onClick={() => this.command(1)} />
          <Button label="CLOSE / RESET" disabled={this.commandsDisabled} onClick={() => this.command(2)} />
        </div>
        <div style="flex-grow: 1" />
      </>
    );
  }
}

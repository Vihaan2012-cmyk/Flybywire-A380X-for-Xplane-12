//  Copyright (c) 2025 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { FSComponent, Subject, Subscribable, VNode } from '@microsoft/msfs-sdk';
import { AbstractOitAvncsPageProps } from '../../../OIT';
import { DestroyableComponent } from '../../../../MsfsAvionicsCommon/DestroyableComponent';
import { AnsuOps } from '../../../System/AnsuOps';
import { AocMessage, AocMessageStore } from '../../../System/AocMessageStore';

interface OitAvncsCompanyComInboxProps extends AbstractOitAvncsPageProps {}

export interface OitAvncsCompanyComMessages {
  read: Subscribable<boolean>;
  subject: string;
  message: string;
  date: number;
}

export abstract class OitAvncsCompanyComInbox extends DestroyableComponent<OitAvncsCompanyComInboxProps> {
  private readonly sci = this.props.container.ansu.sci;

  private readonly store = AocMessageStore.instance(this.props.bus);

  private readonly messageContainerRef = FSComponent.createRef<HTMLDivElement>();

  private selectedId: number | null = null;

  private readonly toText = this.sci.fltNumber.map((fltNumber) => (fltNumber ? `To : ${fltNumber}` : 'To : <unknown>'));
  private readonly readText = Subject.create('');
  private readonly subjectText = Subject.create('Subject :');
  private readonly dateText = Subject.create('');
  private readonly messageText = Subject.create('NO MESSAGE');

  private received(): AocMessage[] {
    return this.store.messages.getArray().filter((m) => m.folder !== 'SENT');
  }

  private select(message: AocMessage | undefined): void {
    if (!message) {
      this.selectedId = null;
      this.readText.set('');
      this.subjectText.set('Subject :');
      this.dateText.set('');
      this.messageText.set('NO MESSAGE');
      return;
    }
    this.selectedId = message.id;
    this.readText.set('Message is read');
    this.subjectText.set(`Subject : ${message.title}`);
    this.dateText.set(AnsuOps.formatDateTime(message.timeMs));
    this.messageText.set(message.lines.join('\n'));
    this.store.markRead(message.id);
  }

  private redraw(): void {
    const container = this.messageContainerRef.getOrDefault();
    if (!container) {
      return;
    }
    while (container.firstChild) {
      container.removeChild(container.firstChild);
    }
    const messages = this.received();
    if (this.selectedId === null && messages.length > 0) {
      this.select(messages[0]);
    }
    for (const m of messages) {
      const row = document.createElement('div');
      row.className = 'fr ass';
      const selected = m.id === this.selectedId;
      const cells: [string, string][] = [
        ['tc w5', m.isNew ? '\u2709' : ''],
        ['f1', `${m.title}  (${m.from})`],
        ['w15', AnsuOps.formatDateTime(m.timeMs)],
      ];
      for (const [cls, text] of cells) {
        const cell = document.createElement('div');
        cell.className = `oit-ccom-inbox-msg-table-line oit-green-text ${cls}${selected ? ' selected' : ''}`;
        cell.textContent = text;
        row.appendChild(cell);
      }
      row.addEventListener('click', () => {
        this.select(m);
        this.redraw();
      });
      container.appendChild(row);
    }
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    this.subscriptions.push(this.toText, this.store.messages.sub(() => this.redraw(), true));
  }

  destroy(): void {
    super.destroy();
  }

  render(): VNode {
    return (
      <>
        <div class="oit-ccom-headline">Inbox</div>
        <div class="oit-ccom-inbox-msg-table">
          <div class="fr ass">
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w5"></div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib f1">Subject</div>
            <div class="oit-ccom-inbox-msg-table-header oit-table-ib w15">Date</div>
          </div>
          <div ref={this.messageContainerRef} />
          <div style="flex-grow: 1" />
        </div>
        <div class="oit-ccom-inbox-msg-details">
          <div>{this.readText}</div>
          <div />
          <div style="text-align: right;">{this.dateText}</div>
          <div>{this.toText}</div>
          <div>{this.subjectText}</div>
          <div />
        </div>
        <div class="oit-ccom-inbox-msg-box" style="white-space: pre-wrap; font-family: monospace;">
          {this.messageText}
        </div>
        <div style="flex-grow: 1" />
      </>
    );
  }
}

interface OitAvncsCompanyComMessageLineProps {
  readonly read: Subscribable<boolean>;
  readonly subject: string;
  readonly date: number;
  readonly selected: Subscribable<boolean>;
  readonly onClick?: () => void;
}

export class OitAvncsCompanyComMessageLine extends DestroyableComponent<OitAvncsCompanyComMessageLineProps> {
  private readonly refs = [
    FSComponent.createRef<HTMLDivElement>(),
    FSComponent.createRef<HTMLDivElement>(),
    FSComponent.createRef<HTMLDivElement>(),
  ];

  private onClickHandler = this.props.onClick?.bind(this);

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    for (const ref of this.refs) {
      if (this.onClickHandler) {
        ref.instance.addEventListener('click', this.onClickHandler);
      }
    }
  }

  render(): VNode {
    return (
      <div class="fr ass">
        <div class={{ 'oit-ccom-inbox-msg-table-line': true, tc: true, w5: true, selected: this.props.selected }}>
          ✉️
        </div>
        <div
          class={{
            'oit-ccom-inbox-msg-table-line': true,
            'oit-green-text': true,
            f1: true,
            selected: this.props.selected,
          }}
        >
          {this.props.subject}
        </div>
        <div
          class={{
            'oit-ccom-inbox-msg-table-line': true,
            'oit-green-text': true,
            w15: true,
            selected: this.props.selected,
          }}
        >
          {AnsuOps.formatDateTime(this.props.date)}
        </div>
      </div>
    );
  }
}

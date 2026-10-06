//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { DisplayComponent, FSComponent, Subject, Subscription, VNode } from '@microsoft/msfs-sdk';
import { AbstractOitFltOpsPageProps } from '../../OIT';
import { Button } from '../../../MsfsAvionicsCommon/UiWidgets/Button';
import { AocFolder, AocMessage, AocMessageStore } from '../../System/AocMessageStore';
import { OitFolder } from '../NssAvncs/OitAvncsFolderNavigator';
import { buildLoadsheet, initLoadsheetBus } from '../../System/Loadsheet';

interface OitFltOpsAocProps extends AbstractOitFltOpsPageProps {}

const COMPOSE_FOLDERS: readonly AocFolder[] = ['REQUESTS', 'REPORTS', 'ADC DELAY', 'DIVERSION'];

const DELAY_CODES = ['11', '12', '13', '14', '15', '16', '17', '18', '19'];

export class OitFltOpsAoc extends DisplayComponent<OitFltOpsAocProps> {
  private readonly subs = [] as Subscription[];

  private readonly store = AocMessageStore.instance(this.props.bus);

  private readonly selectedFolder = Subject.create<AocFolder>('INBOX');

  private readonly selectedMessage = Subject.create<AocMessage | null>(null);

  private readonly folderListRef = FSComponent.createRef<HTMLDivElement>();

  private readonly messageListRef = FSComponent.createRef<HTMLDivElement>();

  private readonly viewerRef = FSComponent.createRef<HTMLDivElement>();

  private renderFolderTree(): void {
    const root = this.folderListRef.getOrDefault();
    if (!root) {
      return;
    }
    while (root.firstChild) {
      root.removeChild(root.firstChild);
    }
    FSComponent.render(this.folderTreeVNode(), root);
  }

  private folderEntry(label: string, folder: AocFolder): VNode {
    const selected = this.selectedFolder.map((f) => f === folder);
    this.subs.push(selected);
    return (
      <div
        class="oit-avncs-navigator-file"
        style="cursor: pointer;"
        onClick={() => {
          this.selectedFolder.set(folder);
          this.selectedMessage.set(null);
          this.renderMessageList();
          this.renderViewer();
        }}
      >
        <span class={{ 'oit-avncs-navigator-file-name': true, selected }}>{label}</span>
      </div>
    );
  }

  private folderTreeVNode(): VNode {
    return (
      <>
        <OitFolder name="INBOXES" initExpanded>
          {this.folderEntry('INBOX', 'INBOX')}
          {this.folderEntry('OPS', 'OPS')}
          {this.folderEntry('METAR', 'METAR')}
        </OitFolder>
        {this.folderEntry('SENT', 'SENT')}
        {this.folderEntry('REQUESTS', 'REQUESTS')}
        {this.folderEntry('LOADSHEET', 'LOADSHEET')}
        {this.folderEntry('REPORTS', 'REPORTS')}
        {this.folderEntry('ADC DELAY', 'ADC DELAY')}
        {this.folderEntry('DIVERSION', 'DIVERSION')}
      </>
    );
  }

  private renderMessageList(): void {
    const root = this.messageListRef.getOrDefault();
    if (!root) {
      return;
    }
    while (root.firstChild) {
      root.removeChild(root.firstChild);
    }

    const folder = this.selectedFolder.get();
    if (COMPOSE_FOLDERS.includes(folder)) {
      FSComponent.render(this.composeVNode(folder), root);
      return;
    }

    const messages = this.store.messages.getArray().filter((m) => m.folder === folder);
    FSComponent.render(
      <>
        <div class="fr ass">
          <div class="oit-ccom-inbox-msg-table-header oit-table-ib f1">MSG</div>
          <div class="oit-ccom-inbox-msg-table-header oit-table-ib w15">ZULU</div>
          <div class="oit-ccom-inbox-msg-table-header oit-table-ib w5">NEW</div>
        </div>
        {messages.map((m) => (
          <div
            class="fr ass"
            style="cursor: pointer;"
            onClick={() => {
              this.store.markRead(m.id);
              this.selectedMessage.set({ ...m, isNew: false });
              this.renderViewer();
              this.renderMessageList();
            }}
          >
            <div class="oit-ccom-inbox-msg-table-line oit-green-text f1">{m.title}</div>
            <div class="oit-ccom-inbox-msg-table-line oit-green-text w15">{m.zulu}</div>
            <div class="oit-ccom-inbox-msg-table-line tc w5">{m.isNew ? '*' : ''}</div>
          </div>
        ))}
      </>,
      root,
    );
  }

  private renderViewer(): void {
    const root = this.viewerRef.getOrDefault();
    if (!root) {
      return;
    }
    while (root.firstChild) {
      root.removeChild(root.firstChild);
    }

    const folder = this.selectedFolder.get();
    if (COMPOSE_FOLDERS.includes(folder)) {
      return;
    }

    const msg = this.selectedMessage.get();
    if (!msg) {
      FSComponent.render(<></>, root);
      return;
    }

    FSComponent.render(
      <>
        <div class="fr ass">
          <div class="oit-label">FROM</div>
          <div class="oit-label cyan" style="margin-left: 10px;">
            {msg.from}
          </div>
          <div style="flex-grow: 1" />
          <div class="oit-label">RECVD</div>
          <div class="oit-label cyan" style="margin-left: 10px;">
            {msg.zulu}
          </div>
        </div>
        <div class="oit-ccom-inbox-msg-box" style="white-space: pre-wrap; font-family: monospace;">
          {msg.lines.join('\n')}
        </div>
      </>,
      root,
    );
  }

  private sendToSent(title: string, lines: string[]): void {
    this.store.add('SENT', title, 'CAPT', lines);
    this.renderMessageList();
  }

  private requestLoadsheet(kind: 'PRELIM' | 'FINAL'): void {
    this.sendToSent(`LOADSHEET REQUEST ${kind}`, [`LOADSHEET ${kind} REQUESTED`]);
    if (buildLoadsheet) {
      const result = buildLoadsheet(kind);
      if (result) {
        this.store.add('LOADSHEET', result.title, 'LOADSHEET', result.lines);
      }
    }
    this.renderMessageList();
  }

  private composeVNode(folder: AocFolder): VNode {
    if (folder === 'REQUESTS') {
      const icaoRef = FSComponent.createRef<HTMLInputElement>();
      return (
        <div class="oit-centered" style="flex-direction: column; align-items: flex-start; padding: 20px;">
          <div class="oit-label bigger">LOADSHEET</div>
          <div class="fr" style="margin: 10px 0 20px 0;">
            <Button label="REQUEST PRELIM" onClick={() => this.requestLoadsheet('PRELIM')} containerStyle="width: 260px; margin-right: 20px;" />
            <Button label="REQUEST FINAL" onClick={() => this.requestLoadsheet('FINAL')} containerStyle="width: 260px;" />
          </div>
          <div class="oit-label bigger">METAR</div>
          <div class="fr" style="margin: 10px 0 20px 0; align-items: center;">
            <input ref={icaoRef} placeholder="ICAO" maxlength={4} style="width: 120px; font-size: 24px; margin-right: 20px; text-transform: uppercase;" />
            <Button
              label="SEND"
              onClick={() => {
                const icao = (icaoRef.getOrDefault()?.value ?? '').toUpperCase() || '----';
                this.sendToSent(`METAR ${icao}`, [`METAR REQUEST FOR ${icao} SENT`, 'AWAITING AIRLINE REPLY']);
              }}
              containerStyle="width: 150px;"
            />
          </div>
          <div class="oit-label bigger">WX</div>
          <div class="fr" style="margin: 10px 0; align-items: center;">
            <Button
              label="SEND WX REQUEST"
              onClick={() => this.sendToSent('WX REQUEST', ['WX REQUEST SENT', 'AWAITING AIRLINE REPLY'])}
              containerStyle="width: 260px;"
            />
          </div>
        </div>
      );
    }

    if (folder === 'REPORTS') {
      const textRef = FSComponent.createRef<HTMLTextAreaElement>();
      return (
        <div class="oit-centered" style="flex-direction: column; align-items: flex-start; padding: 20px;">
          <div class="oit-label bigger">REPORT TEXT</div>
          <textarea ref={textRef} rows={10} style="width: 100%; font-size: 22px; margin: 10px 0;" />
          <Button
            label="SEND"
            onClick={() => {
              const text = textRef.getOrDefault()?.value ?? '';
              this.sendToSent('REPORT', text.length > 0 ? text.split('\n') : ['(EMPTY REPORT)']);
            }}
            containerStyle="width: 150px;"
          />
        </div>
      );
    }

    if (folder === 'ADC DELAY') {
      const codeRef = FSComponent.createRef<HTMLSelectElement>();
      const minRef = FSComponent.createRef<HTMLInputElement>();
      return (
        <div class="oit-centered" style="flex-direction: column; align-items: flex-start; padding: 20px;">
          <div class="fr" style="align-items: center; margin-bottom: 20px;">
            <div class="oit-label bigger" style="width: 200px;">
              DELAY CODE
            </div>
            <select ref={codeRef} style="font-size: 24px; width: 120px;">
              {DELAY_CODES.map((c) => (
                <option value={c}>{c}</option>
              ))}
            </select>
          </div>
          <div class="fr" style="align-items: center; margin-bottom: 20px;">
            <div class="oit-label bigger" style="width: 200px;">
              DELAY MINUTES
            </div>
            <input ref={minRef} type="number" min="0" style="font-size: 24px; width: 120px;" />
          </div>
          <Button
            label="SEND"
            onClick={() => {
              const code = codeRef.getOrDefault()?.value ?? '--';
              const mins = minRef.getOrDefault()?.value ?? '0';
              this.sendToSent('ADC DELAY REPORT', [`DELAY CODE ${code}`, `DELAY ${mins} MIN`]);
            }}
            containerStyle="width: 150px;"
          />
        </div>
      );
    }

    const apRef = FSComponent.createRef<HTMLInputElement>();
    const reasonRef = FSComponent.createRef<HTMLInputElement>();
    return (
      <div class="oit-centered" style="flex-direction: column; align-items: flex-start; padding: 20px;">
        <div class="fr" style="align-items: center; margin-bottom: 20px;">
          <div class="oit-label bigger" style="width: 200px;">
            DIVERSION AIRPORT
          </div>
          <input ref={apRef} maxlength={4} style="font-size: 24px; width: 150px; text-transform: uppercase;" />
        </div>
        <div class="fr" style="align-items: center; margin-bottom: 20px;">
          <div class="oit-label bigger" style="width: 200px;">
            REASON
          </div>
          <input ref={reasonRef} style="font-size: 24px; width: 400px;" />
        </div>
        <Button
          label="SEND"
          onClick={() => {
            const ap = (apRef.getOrDefault()?.value ?? '').toUpperCase() || '----';
            const reason = reasonRef.getOrDefault()?.value ?? '';
            this.sendToSent('DIVERSION REPORT', [`DIVERSION TO ${ap}`, `REASON: ${reason || 'N/A'}`]);
          }}
          containerStyle="width: 150px;"
        />
      </div>
    );
  }

  private clearAll(): void {
    this.store.clear(this.selectedFolder.get());
    this.selectedMessage.set(null);
    this.renderMessageList();
    this.renderViewer();
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    initLoadsheetBus(this.props.bus);

    this.subs.push(
      this.store.messages.sub(() => {
        this.renderMessageList();
      }),
    );

    this.renderFolderTree();
    this.renderMessageList();
    this.renderViewer();
  }

  public destroy(): void {
    for (const s of this.subs) {
      s.destroy();
    }
    super.destroy();
  }

  render(): VNode {
    return (
      <>
        <div class="oit-page-container">
          <div class="oit-avncs-navigator-container">
            <div class="oit-avncs-navigator-left">
              <div class="oit-ccom-headline">COMPANY COM</div>
              <div ref={this.folderListRef} />
              <div style="flex-grow: 1" />
              <Button label="CLEAR ALL" onClick={() => this.clearAll()} containerStyle="width: 100%; margin-top: 10px;" />
            </div>
            <div class="oit-avncs-navigator-right" style="display: flex; flex-direction: row;">
              <div style="width: 45%; border-right: 1px solid #5a5a5a; overflow-y: auto;" ref={this.messageListRef} />
              <div style="flex-grow: 1; padding: 10px;" ref={this.viewerRef} />
            </div>
          </div>
        </div>
      </>
    );
  }
}

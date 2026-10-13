//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { ClockEvents, DisplayComponent, FSComponent, Subject, Subscription, VNode } from '@microsoft/msfs-sdk';
import { AbstractOitFltOpsPageProps } from '../../OIT';
import { Button } from '../../../MsfsAvionicsCommon/UiWidgets/Button';
import { buildLoadsheet } from '../../System/Loadsheet';
import { AocMessageStore } from '../../System/AocMessageStore';

interface OitFltOpsLoadsheetPageProps extends AbstractOitFltOpsPageProps {}

export class OitFltOpsLoadsheet extends DisplayComponent<OitFltOpsLoadsheetPageProps> {
  private readonly subs = [] as Subscription[];

  private readonly kind = Subject.create<'PRELIM' | 'FINAL'>('PRELIM');

  private readonly contentRef = FSComponent.createRef<HTMLDivElement>();

  private readonly store = AocMessageStore.instance(this.props.bus);

  private render_(): void {
    const root = this.contentRef.getOrDefault();
    if (!root) {
      return;
    }
    while (root.firstChild) {
      root.removeChild(root.firstChild);
    }

    const result = buildLoadsheet(this.kind.get());

    if (!result) {
      FSComponent.render(
        <div class="oit-label bigger">
          NO LOADSHEET AVAILABLE
          <br />
          {this.kind.get() === 'FINAL'
            ? 'MAIN DOOR MUST BE CLOSED FOR A FINAL LOADSHEET'
            : 'AIRCRAFT WEIGHT NOT YET COMPUTED'}
        </div>,
        root,
      );
      return;
    }

    FSComponent.render(
      <div style="white-space: pre-wrap; font-family: monospace;">
        <div class="oit-label bigger cyan">{result.title}</div>
        {result.lines.map((l) => (
          <div class="oit-label bigger">{l}</div>
        ))}
      </div>,
      root,
    );
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    const sub = this.props.bus.getSubscriber<ClockEvents>();
    this.subs.push(
      sub
        .on('realTime')
        .atFrequency(0.5)
        .handle(() => this.render_()),
      this.kind.sub(() => this.render_()),
    );

    this.render_();
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
        <div class="oit-page-container framed" style="flex-direction: column;">
          <div class="fr ass" style="margin-bottom: 15px;">
            <Button
              label="PRELIM"
              onClick={() => this.kind.set('PRELIM')}
              selected={this.kind.map((k) => k === 'PRELIM')}
              containerStyle="width: 200px; margin-right: 20px;"
            />
            <Button
              label="FINAL"
              onClick={() => this.kind.set('FINAL')}
              selected={this.kind.map((k) => k === 'FINAL')}
              containerStyle="width: 200px; margin-right: 20px;"
            />
            <div style="flex-grow: 1" />
            <Button
              label="SEND TO AOC"
              onClick={() => {
                const result = buildLoadsheet(this.kind.get());
                if (result) {
                  this.store.add('LOADSHEET', result.title, 'LOADSHEET', result.lines);
                }
              }}
              containerStyle="width: 220px;"
            />
          </div>
          <div ref={this.contentRef} style="overflow-y: auto; flex-grow: 1;" />
        </div>
      </>
    );
  }
}

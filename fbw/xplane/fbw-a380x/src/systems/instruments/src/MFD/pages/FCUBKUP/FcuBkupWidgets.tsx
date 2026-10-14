// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { ComponentProps, DisplayComponent, FSComponent, Subscribable, VNode } from '@microsoft/msfs-sdk';

export function fcuKey(event: string, value = 0) {
  SimVar.SetSimVarValue(`K:${event}`, 'number', value);
}

interface FcuBkupLightProps extends ComponentProps {
  label: string;
  on: Subscribable<boolean>;
  onClick: () => void;
  labelBelow?: boolean;
}

export class FcuBkupLight extends DisplayComponent<FcuBkupLightProps> {
  private readonly ref = FSComponent.createRef<HTMLDivElement>();

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);
    this.ref.instance.addEventListener('click', () => this.props.onClick());
  }

  render(): VNode {
    return (
      <div class={{ 'mfd-fcubkup-light-group': true, 'label-below': !!this.props.labelBelow }}>
        <div ref={this.ref} class={{ 'mfd-fcubkup-light': true, wide: !!this.props.labelBelow }}>
          <div class={{ 'mfd-fcubkup-bars': true, on: this.props.on }}>
            <div />
            <div />
            <div />
          </div>
          <div class="mfd-fcubkup-light-label" style={this.props.labelBelow ? 'display: none' : ''}>
            {this.props.label}
          </div>
        </div>
        <div class="mfd-fcubkup-light-caption" style={this.props.labelBelow ? '' : 'display: none'}>
          {this.props.label}
        </div>
      </div>
    );
  }
}

interface FcuBkupWindowProps extends ComponentProps {
  left: number;
  top?: number;
  width?: number;
  text: Subscribable<string>;
  unit: Subscribable<string>;
  inc: string;
  dec: string;
}

export class FcuBkupWindow extends DisplayComponent<FcuBkupWindowProps> {
  private readonly ref = FSComponent.createRef<HTMLDivElement>();

  private readonly onWheel = (e: WheelEvent) => {
    e.preventDefault();
    const clicks = Math.max(1, Math.round(Math.abs(e.deltaY) / 100));
    for (let i = 0; i < clicks; i++) {
      fcuKey(e.deltaY < 0 ? this.props.inc : this.props.dec);
    }
  };

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);
    this.ref.instance.addEventListener('wheel', this.onWheel);
  }

  render(): VNode {
    return (
      <div
        ref={this.ref}
        class="mfd-fcubkup-window"
        style={`left: ${this.props.left}px; top: ${this.props.top ?? 305}px; width: ${this.props.width ?? 150}px;`}
      >
        <span class="mfd-fcubkup-window-value">{this.props.text}</span>
        <span class="mfd-fcubkup-window-unit">{this.props.unit}</span>
      </div>
    );
  }
}

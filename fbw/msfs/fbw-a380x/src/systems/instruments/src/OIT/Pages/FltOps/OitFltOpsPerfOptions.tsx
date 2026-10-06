//  Copyright (c) 2025 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { ComponentProps, FSComponent, MutableSubscribable, Subject, VNode } from '@microsoft/msfs-sdk';
import { DestroyableComponent } from '../../../MsfsAvionicsCommon/DestroyableComponent';
import { RadioButtonGroup } from '../../../MsfsAvionicsCommon/UiWidgets/RadioButtonGroup';
import { OisUnits } from './OitFltOpsPerfCommon';

interface OitFltOpsPerfOptionsProps extends ComponentProps {
  readonly units: MutableSubscribable<OisUnits>;
}

export class OitFltOpsPerfOptions extends DestroyableComponent<OitFltOpsPerfOptionsProps> {
  private readonly weightIndex = Subject.create<number | null>(this.props.units.get().weight === 't' ? 0 : 1);
  private readonly altimeterIndex = Subject.create<number | null>(this.props.units.get().altimeter === 'hpa' ? 0 : 1);
  private readonly distanceIndex = Subject.create<number | null>(this.props.units.get().distance === 'm' ? 0 : 1);

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    this.subscriptions.push(
      this.weightIndex.sub((i) => this.props.units.set({ ...this.props.units.get(), weight: i === 1 ? 'klb' : 't' })),
      this.altimeterIndex.sub((i) =>
        this.props.units.set({ ...this.props.units.get(), altimeter: i === 1 ? 'inhg' : 'hpa' }),
      ),
      this.distanceIndex.sub((i) => this.props.units.set({ ...this.props.units.get(), distance: i === 1 ? 'ft' : 'm' })),
    );
  }

  render(): VNode {
    return (
      <div class="oit-labeled-box-container oit-perf-options">
        <span class="oit-labeled-box-label">OPTIONS</span>
        <div class="fr">
          <div class="oit-perf-options-column">
            <div class="oit-label">WEIGHT</div>
            <RadioButtonGroup values={['T', 'KLB']} selectedIndex={this.weightIndex} idPrefix="oit-perf-opt-weight" />
          </div>
          <div class="oit-perf-options-column">
            <div class="oit-label">ALTIMETER</div>
            <RadioButtonGroup
              values={['HPA', 'IN HG']}
              selectedIndex={this.altimeterIndex}
              idPrefix="oit-perf-opt-alt"
            />
          </div>
          <div class="oit-perf-options-column">
            <div class="oit-label">DISTANCE</div>
            <RadioButtonGroup values={['M', 'FT']} selectedIndex={this.distanceIndex} idPrefix="oit-perf-opt-dist" />
          </div>
        </div>
      </div>
    );
  }
}

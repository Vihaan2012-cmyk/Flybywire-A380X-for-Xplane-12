// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { DisplayComponent, FSComponent, MappedSubject, Subject, Subscribable, Subscription, VNode } from '@microsoft/msfs-sdk';

import './MfdFcuBkup.scss';

import { ActivePageTitleBar } from '../common/ActivePageTitleBar';
import { AbstractMfdPageProps } from '../../MFD';
import { Footer } from '../common/Footer';
import { Button } from '../../../MsfsAvionicsCommon/UiWidgets/Button';
import { FcuBkupLight, FcuBkupWindow, fcuKey } from './FcuBkupWidgets';

interface MfdFcuBkupAfsProps extends AbstractMfdPageProps {}

export class MfdFcuBkupAfs extends DisplayComponent<MfdFcuBkupAfsProps> {
  private readonly subs = [] as Subscription[];

  private timer: ReturnType<typeof setInterval> | undefined;

  private readonly cpActive = Subject.create(true);

  private readonly backupSelected = Subject.create(false);

  private readonly backupOn = MappedSubject.create(([cp, sel]) => sel || !cp, this.cpActive, this.backupSelected);

  private readonly machMode = Subject.create(false);

  private readonly trkFpaMode = Subject.create(false);

  private readonly trueMode = Subject.create(false);

  private readonly metric = Subject.create(false);

  private readonly spd = Subject.create(100);

  private readonly spdDashes = Subject.create(false);

  private readonly hdg = Subject.create(0);

  private readonly hdgDashes = Subject.create(false);

  private readonly alt = Subject.create(0);

  private readonly vs = Subject.create(0);

  private readonly vsDashes = Subject.create(true);

  private readonly locLight = Subject.create(false);

  private readonly altLight = Subject.create(false);

  private readonly apprLight = Subject.create(false);

  private readonly ap1Light = Subject.create(false);

  private readonly ap2Light = Subject.create(false);

  private readonly athrLight = Subject.create(false);

  private readonly fdLight = Subject.create(false);

  private readonly spdText = MappedSubject.create(
    ([v, dashes, mach]) => (dashes ? '---' : mach ? v.toFixed(2).replace(/^0/, '') : Math.round(v).toString().padStart(3, '0')),
    this.spd,
    this.spdDashes,
    this.machMode,
  );

  private readonly hdgText = MappedSubject.create(
    ([v, dashes]) => (dashes ? '---' : (Math.round(v) % 360).toString().padStart(3, '0')),
    this.hdg,
    this.hdgDashes,
  );

  private readonly altText = this.alt.map((v) => Math.round(v).toString());

  private readonly vsText = MappedSubject.create(
    ([v, dashes, fpa]) => {
      if (dashes) {
        return '---';
      }
      if (fpa) {
        return `${v >= 0 ? '+' : '-'}${Math.abs(v).toFixed(1)}`;
      }
      const hundreds = Math.round(Math.abs(v) / 100);
      return `${v >= 0 ? '+' : '-'}${hundreds.toString().padStart(2, '0')}`;
    },
    this.vs,
    this.vsDashes,
    this.trkFpaMode,
  );

  private readonly read = (name: string) => SimVar.GetSimVarValue(name, 'number') as number;

  private poll() {
    this.cpActive.set(this.read('L:A32NX_FCU_AFS_CP_ACTIVE') !== 0);
    this.backupSelected.set(this.read('L:A380X_FCU_BKUP_AFS_ACTIVE') !== 0);
    this.machMode.set(this.read('L:A32NX_FCU_AFS_DISPLAY_MACH_MODE') !== 0);
    this.trkFpaMode.set(this.read('L:A32NX_FCU_AFS_DISPLAY_TRK_FPA_MODE') !== 0);
    this.trueMode.set(this.read('L:A32NX_FCU_AFS_DISPLAY_TRUE_MODE') !== 0);
    this.metric.set(this.read('L:A32NX_METRIC_ALT_TOGGLE') !== 0);
    this.spd.set(this.read('L:A32NX_FCU_AFS_DISPLAY_SPD_MACH_VALUE'));
    this.spdDashes.set(this.read('L:A32NX_FCU_AFS_DISPLAY_SPD_MACH_DASHES') !== 0);
    this.hdg.set(this.read('L:A32NX_FCU_AFS_DISPLAY_HDG_TRK_VALUE'));
    this.hdgDashes.set(this.read('L:A32NX_FCU_AFS_DISPLAY_HDG_TRK_DASHES') !== 0);
    this.alt.set(this.read('L:A32NX_FCU_AFS_DISPLAY_ALT_VALUE'));
    this.vs.set(this.read('L:A32NX_FCU_AFS_DISPLAY_VS_FPA_VALUE'));
    this.vsDashes.set(this.read('L:A32NX_FCU_AFS_DISPLAY_VS_FPA_DASHES') !== 0);
    this.locLight.set(this.read('L:A32NX_FCU_LOC_LIGHT_ON') !== 0);
    this.altLight.set(this.read('L:A32NX_FCU_ALT_LIGHT_ON') !== 0);
    this.apprLight.set(this.read('L:A32NX_FCU_APPR_LIGHT_ON') !== 0);
    this.ap1Light.set(this.read('L:A32NX_FCU_AP_1_LIGHT_ON') !== 0);
    this.ap2Light.set(this.read('L:A32NX_FCU_AP_2_LIGHT_ON') !== 0);
    this.athrLight.set(this.read('L:A32NX_FCU_ATHR_LIGHT_ON') !== 0);
    this.fdLight.set(this.read('L:A32NX_FCU_FD_LIGHT_ON') !== 0);
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);
    this.poll();
    this.timer = setInterval(() => this.poll(), 100);
    this.subs.push(this.backupOn, this.spdText, this.hdgText, this.altText, this.vsText);
  }

  public destroy(): void {
    if (this.timer) {
      clearInterval(this.timer);
    }
    for (const s of this.subs) {
      s.destroy();
    }
    super.destroy();
  }

  private toggle(label: Subscribable<boolean>, whenOff: string, whenOn: string) {
    return label.map((on) => (on ? whenOn : whenOff));
  }

  render(): VNode {
    return (
      <>
        <ActivePageTitleBar activePage={Subject.create('AFS CONTROL PANEL')} offset={Subject.create('')} />
        <div class="mfd-page-container">
          <div class="mfd-fcubkup-canvas">
            <svg class="mfd-fcubkup-lines" width="768" height="780">
              <polyline points="120,225 120,262" />
              <polyline points="300,225 300,250 230,250 230,262" />
              <polyline points="470,80 470,140 300,140 300,175" />
              <polyline points="620,80 620,262" />
              <polyline points="520,138 520,262" />
              <rect x="20" y="170" width="728" height="390" />
              <rect x="190" y="590" width="390" height="185" />
            </svg>

            <div class="mfd-fcubkup-item" style="left: 150px; top: 5px;">
              <FcuBkupLight label="AFS CP BACKUP" on={this.backupOn} onClick={() => this.toggleBackup()} labelBelow />
            </div>

            <div class="mfd-fcubkup-item" style="left: 360px; top: 40px;">
              <Button
                label={this.toggle(this.trkFpaMode, 'HDG-V/S → TRK-FPA', 'TRK-FPA → HDG-V/S')}
                onClick={() => fcuKey('A32NX.FCU_TRK_FPA_TOGGLE_PUSH')}
                buttonStyle="width: 230px;"
              />
            </div>
            <div class="mfd-fcubkup-item" style="left: 470px; top: 98px;">
              <Button
                label={this.toggle(this.metric, 'METER', 'METER')}
                selected={this.metric}
                onClick={() => fcuKey('A32NX.FCU_METRIC_ALT_TOGGLE_PUSH')}
                buttonStyle="width: 100px;"
              />
            </div>

            <div class="mfd-fcubkup-item" style="left: 40px; top: 182px;">
              <Button
                label={this.toggle(this.machMode, 'SPD → MACH', 'MACH → SPD')}
                onClick={() => fcuKey('A32NX.FCU_SPD_MACH_TOGGLE_PUSH')}
                buttonStyle="width: 160px;"
              />
            </div>
            <div class="mfd-fcubkup-item" style="left: 220px; top: 182px;">
              <Button
                label={this.toggle(this.trueMode, 'MAG → TRUE', 'TRUE → MAG')}
                onClick={() => fcuKey('A32NX.FCU_TRUE_TOGGLE_PUSH')}
                buttonStyle="width: 160px;"
              />
            </div>

            <div class="mfd-fcubkup-label" style="left: 45px; top: 266px; width: 150px;">
              {this.toggle(this.machMode, 'SPD', 'MACH')}
            </div>
            <div class="mfd-fcubkup-label" style="left: 225px; top: 266px; width: 150px;">
              {MappedSubject.create(
                ([trk, tru]) => `${tru ? 'TRUE' : 'MAG'} ${trk ? 'TRK' : 'HDG'}`,
                this.trkFpaMode,
                this.trueMode,
              )}
            </div>
            <div class="mfd-fcubkup-label" style="left: 405px; top: 266px; width: 150px;">
              ALT
            </div>
            <div class="mfd-fcubkup-label" style="left: 575px; top: 266px; width: 150px;">
              {this.toggle(this.trkFpaMode, 'V/S', 'FPA')}
            </div>

            <FcuBkupWindow
              left={45}
              text={this.spdText}
              unit={this.toggle(this.machMode, 'KT', '')}
              inc="A32NX.FCU_SPD_INC"
              dec="A32NX.FCU_SPD_DEC"
            />
            <FcuBkupWindow
              left={225}
              text={this.hdgText}
              unit={Subject.create('°')}
              inc="A32NX.FCU_HDG_INC"
              dec="A32NX.FCU_HDG_DEC"
            />
            <FcuBkupWindow
              left={405}
              text={this.altText}
              unit={Subject.create('FT')}
              inc="A32NX.FCU_ALT_INC"
              dec="A32NX.FCU_ALT_DEC"
            />
            <FcuBkupWindow
              left={575}
              text={this.vsText}
              unit={this.toggle(this.trkFpaMode, 'FT/MN', '°')}
              inc="A32NX.FCU_VS_INC"
              dec="A32NX.FCU_VS_DEC"
            />

            <div class="mfd-fcubkup-stack" style="left: 45px;">
              <Button label="SELECTED" onClick={() => fcuKey('A32NX.FCU_SPD_PULL')} buttonStyle="width: 150px;" />
              <Button label="MANAGED" onClick={() => fcuKey('A32NX.FCU_SPD_PUSH')} buttonStyle="width: 150px;" />
            </div>
            <div class="mfd-fcubkup-stack" style="left: 225px;">
              <Button label="SELECTED" onClick={() => fcuKey('A32NX.FCU_HDG_PULL')} buttonStyle="width: 150px;" />
              <Button label="MANAGED" onClick={() => fcuKey('A32NX.FCU_HDG_PUSH')} buttonStyle="width: 150px;" />
            </div>
            <div class="mfd-fcubkup-stack" style="left: 405px;">
              <Button label="OPEN" onClick={() => fcuKey('A32NX.FCU_ALT_PULL')} buttonStyle="width: 150px;" />
              <Button label="MANAGED" onClick={() => fcuKey('A32NX.FCU_ALT_PUSH')} buttonStyle="width: 150px;" />
            </div>
            <div class="mfd-fcubkup-stack" style="left: 575px;">
              <Button label="SELECTED" onClick={() => fcuKey('A32NX.FCU_VS_PULL')} buttonStyle="width: 150px;" />
            </div>

            <div class="mfd-fcubkup-item" style="left: 260px; top: 470px;">
              <FcuBkupLight label="LOC" on={this.locLight} onClick={() => fcuKey('A32NX.FCU_LOC_PUSH')} />
            </div>
            <div class="mfd-fcubkup-item" style="left: 440px; top: 470px;">
              <FcuBkupLight label="ALT" on={this.altLight} onClick={() => fcuKey('A32NX.FCU_ALT_BUTTON_PUSH')} />
            </div>
            <div class="mfd-fcubkup-item" style="left: 610px; top: 470px;">
              <FcuBkupLight label="APPR" on={this.apprLight} onClick={() => fcuKey('A32NX.FCU_APPR_PUSH')} />
            </div>

            <div class="mfd-fcubkup-item" style="left: 345px; top: 600px;">
              <FcuBkupLight label="FD" on={this.fdLight} onClick={() => fcuKey('A32NX.FCU_FD_PUSH')} />
            </div>
            <div class="mfd-fcubkup-item" style="left: 465px; top: 612px;">
              <FcuBkupLight label="AP2" on={this.ap2Light} onClick={() => fcuKey('A32NX.FCU_AP_2_PUSH')} />
            </div>
            <div class="mfd-fcubkup-item" style="left: 225px; top: 680px;">
              <FcuBkupLight label="AP1" on={this.ap1Light} onClick={() => fcuKey('A32NX.FCU_AP_1_PUSH')} />
            </div>
            <div class="mfd-fcubkup-item" style="left: 345px; top: 685px;">
              <FcuBkupLight label="A/THR" on={this.athrLight} onClick={() => fcuKey('A32NX.FCU_ATHR_PUSH')} />
            </div>
          </div>
        </div>
        <Footer
          bus={this.props.bus}
          mfd={this.props.mfd}
          fmcService={this.props.fmcService}
          flightPlanInterface={this.props.fmcService.master.flightPlanInterface}
        />
      </>
    );
  }

  private toggleBackup() {
    SimVar.SetSimVarValue('L:A380X_FCU_BKUP_AFS_ACTIVE', 'number', this.backupSelected.get() ? 0 : 1);
  }
}

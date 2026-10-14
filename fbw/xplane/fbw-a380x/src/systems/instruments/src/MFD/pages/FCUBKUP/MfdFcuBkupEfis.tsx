// Copyright (c) 2026 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { DisplayComponent, FSComponent, MappedSubject, Subject, Subscribable, Subscription, VNode } from '@microsoft/msfs-sdk';

import './MfdFcuBkup.scss';

import { ActivePageTitleBar } from '../common/ActivePageTitleBar';
import { AbstractMfdPageProps } from '../../MFD';
import { Footer } from '../common/Footer';
import { Button } from '../../../MsfsAvionicsCommon/UiWidgets/Button';
import { RadioButtonColor, RadioButtonGroup } from '../../../MsfsAvionicsCommon/UiWidgets/RadioButtonGroup';
import { FcuBkupLight, FcuBkupWindow, fcuKey } from './FcuBkupWidgets';

interface MfdFcuBkupEfisProps extends AbstractMfdPageProps {}

const ND_MODES = ['LS', 'VOR', 'NAV', 'ARC', 'PLAN'];

const ND_RANGES = ['10', '20', '40', '80', '160', '320', '640'];

const NAVAID_MODES = ['VOR', 'OFF', 'ADF'];

const NAVAID_VALUE = [2, 0, 1];

export class MfdFcuBkupEfis extends DisplayComponent<MfdFcuBkupEfisProps> {
  private readonly subs = [] as Subscription[];

  private timer: ReturnType<typeof setInterval> | undefined;

  private readonly side: 'L' | 'R' = this.props.mfd.uiService.captOrFo === 'CAPT' ? 'L' : 'R';

  private readonly cpActive = Subject.create(true);

  private readonly backupSelected = Subject.create(false);

  private readonly backupOn = MappedSubject.create(([cp, sel]) => sel || !cp, this.cpActive, this.backupSelected);

  private readonly baroValue = Subject.create(1013);

  private readonly baroStd = Subject.create(false);

  private readonly baroInhg = Subject.create(false);

  private readonly ls = Subject.create(false);

  private readonly vv = Subject.create(false);

  private readonly cstr = Subject.create(false);

  private readonly arpt = Subject.create(false);

  private readonly traf = Subject.create(false);

  private readonly filter = Subject.create(0);

  private readonly overlay = Subject.create(0);

  private readonly ndMode = Subject.create(3);

  private readonly ndRange = Subject.create(1);

  private readonly navaid1 = Subject.create(0);

  private readonly navaid2 = Subject.create(0);

  private readonly targets = { mode: null as number | null, range: null as number | null, nav1: null as number | null, nav2: null as number | null };

  private readonly baroText = MappedSubject.create(
    ([value, std, inhg]) => (std ? 'STD' : inhg ? value.toFixed(2) : Math.round(value).toString().padStart(4, '0')),
    this.baroValue,
    this.baroStd,
    this.baroInhg,
  );

  private readonly baroUnit = MappedSubject.create(
    ([std, inhg]) => (std ? '' : inhg ? 'IN.HG' : 'HPA'),
    this.baroStd,
    this.baroInhg,
  );

  private readonly modeIndex = this.ndMode.map((m) => {
    const index = [0, 1, 2, 3, 4].indexOf(Math.round(m));
    return index >= 0 ? index : null;
  });

  private readonly rangeIndex = this.ndRange.map((r) => (Math.round(r) >= 1 ? Math.round(r) - 1 : null));

  private readonly nav1Index = this.navaid1.map((v) => NAVAID_VALUE.indexOf(Math.round(v)));

  private readonly nav2Index = this.navaid2.map((v) => NAVAID_VALUE.indexOf(Math.round(v)));

  private readonly read = (name: string) => SimVar.GetSimVarValue(name, 'number') as number;

  private readonly event = (name: string) => `A32NX.FCU_EFIS_${this.side}_${name}`;

  private poll() {
    const s = this.side;
    this.cpActive.set(this.read(`L:A32NX_FCU_EFIS_${s}_CP_ACTIVE`) !== 0);
    this.backupSelected.set(this.read(`L:A380X_FCU_BKUP_EFIS_${s}_ACTIVE`) !== 0);
    this.baroValue.set(this.read(`L:A32NX_FCU_EFIS_${s}_DISPLAY_BARO_VALUE`));
    this.baroStd.set(this.read(`L:A32NX_FCU_EFIS_${s}_DISPLAY_BARO_IS_STD`) !== 0);
    this.baroInhg.set(this.read(`L:A32NX_FCU_EFIS_${s}_DISPLAY_BARO_IS_INHG`) !== 0);
    const lit = (name: string) => this.read(`L:A32NX_FCU_EFIS_${s}_${name}_LIGHT_ON`) !== 0;
    this.ls.set(lit('LS'));
    this.vv.set(lit('VV'));
    this.cstr.set(lit('CSTR'));
    this.arpt.set(lit('ARPT'));
    this.traf.set(lit('TRAF'));
    this.filter.set(lit('WPT') ? 1 : lit('VORD') ? 2 : lit('NDB') ? 3 : 0);
    this.overlay.set(lit('WX') ? 1 : lit('TERR') ? 2 : 0);
    this.ndMode.set(this.read(`L:A32NX_EFIS_${s}_ND_MODE`));
    this.ndRange.set(this.read(`L:A32NX_EFIS_${s}_ND_RANGE`));
    this.navaid1.set(this.read(`L:A32NX_EFIS_${s}_NAVAID_1_MODE`));
    this.navaid2.set(this.read(`L:A32NX_EFIS_${s}_NAVAID_2_MODE`));
    this.chase();
  }

  private chase() {
    const t = this.targets;
    const step = (target: number | null, now: number, inc: string, dec: string | null) => {
      if (target === null || Math.round(now) === target) {
        return null;
      }
      fcuKey(dec === null || target > Math.round(now) ? inc : dec);
      return target;
    };
    t.mode = step(t.mode, this.ndMode.get(), this.event('MODE_INC'), this.event('MODE_DEC'));
    t.range = step(t.range, this.ndRange.get(), this.event('RANGE_INC'), this.event('RANGE_DEC'));
    t.nav1 = step(t.nav1, this.navaid1.get(), this.event('NAVAID_1_PUSH'), null);
    t.nav2 = step(t.nav2, this.navaid2.get(), this.event('NAVAID_2_PUSH'), null);
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);
    this.poll();
    this.timer = setInterval(() => this.poll(), 100);
    this.subs.push(this.backupOn, this.baroText, this.baroUnit, this.modeIndex, this.rangeIndex, this.nav1Index, this.nav2Index);
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

  private light(label: string, on: Subscribable<boolean>, push: string, left: number, top: number): VNode {
    return (
      <div class="mfd-fcubkup-item" style={`left: ${left}px; top: ${top}px;`}>
        <FcuBkupLight label={label} on={on} onClick={() => fcuKey(this.event(push))} />
      </div>
    );
  }

  render(): VNode {
    const side = this.side === 'L' ? 'CAPT' : 'F/O';
    const prefix = `${this.props.mfd.uiService.captOrFo}_MFD_fcuBkupEfis`;
    return (
      <>
        <ActivePageTitleBar activePage={Subject.create(`${side} EFIS CONTROL PANEL`)} offset={Subject.create('')} />
        <div class="mfd-page-container">
          <div class="mfd-fcubkup-canvas">
            <svg class="mfd-fcubkup-lines" width="768" height="780">
              <rect x="20" y="120" width="300" height="250" />
              <rect x="340" y="120" width="408" height="250" />
              <rect x="20" y="390" width="728" height="200" />
              <rect x="20" y="610" width="728" height="160" />
            </svg>

            <div class="mfd-fcubkup-item" style="left: 40px; top: 5px;">
              <FcuBkupLight
                label="EFIS CP BACKUP"
                on={this.backupOn}
                onClick={() =>
                  SimVar.SetSimVarValue(`L:A380X_FCU_BKUP_EFIS_${this.side}_ACTIVE`, 'number', this.backupSelected.get() ? 0 : 1)
                }
                labelBelow
              />
            </div>

            <div class="mfd-fcubkup-label" style="left: 95px; top: 132px; width: 150px;">
              BARO REF
            </div>
            <FcuBkupWindow
              left={70}
              top={172}
              width={200}
              text={this.baroText}
              unit={this.baroUnit}
              inc={this.event('BARO_INC')}
              dec={this.event('BARO_DEC')}
            />
            <div class="mfd-fcubkup-item" style="left: 45px; top: 245px;">
              <Button label="QNH" onClick={() => fcuKey(this.event('BARO_PUSH'))} buttonStyle="width: 120px;" />
            </div>
            <div class="mfd-fcubkup-item" style="left: 175px; top: 245px;">
              <Button label="STD" selected={this.baroStd} onClick={() => fcuKey(this.event('BARO_PULL'))} buttonStyle="width: 120px;" />
            </div>

            <div class="mfd-fcubkup-label" style="left: 360px; top: 132px; width: 120px;">
              ND MODE
            </div>
            <div class="mfd-fcubkup-item" style="left: 370px; top: 165px;">
              <RadioButtonGroup
                values={ND_MODES}
                selectedIndex={this.modeIndex}
                onModified={(i) => (this.targets.mode = i)}
                idPrefix={`${prefix}Mode`}
                color={Subject.create(RadioButtonColor.Green)}
              />
            </div>
            <div class="mfd-fcubkup-label" style="left: 520px; top: 132px; width: 120px;">
              ND RANGE
            </div>
            <div class="mfd-fcubkup-item mfd-fcubkup-range" style="left: 530px; top: 165px;">
              <RadioButtonGroup
                values={ND_RANGES}
                selectedIndex={this.rangeIndex}
                onModified={(i) => (this.targets.range = i + 1)}
                idPrefix={`${prefix}Range`}
                color={Subject.create(RadioButtonColor.Green)}
              />
            </div>

            {this.light('LS', this.ls, 'LS_PUSH', 60, 410)}
            {this.light('VV', this.vv, 'VV_PUSH', 160, 410)}
            {this.light('CSTR', this.cstr, 'CSTR_PUSH', 300, 410)}
            {this.light('WPT', this.filter.map((f) => f === 1), 'WPT_PUSH', 390, 410)}
            {this.light('VOR.D', this.filter.map((f) => f === 2), 'VORD_PUSH', 480, 410)}
            {this.light('NDB', this.filter.map((f) => f === 3), 'NDB_PUSH', 570, 410)}
            {this.light('ARPT', this.arpt, 'ARPT_PUSH', 660, 410)}
            {this.light('WX', this.overlay.map((o) => o === 1), 'WX_PUSH', 300, 500)}
            {this.light('TERR', this.overlay.map((o) => o === 2), 'TERR_PUSH', 390, 500)}
            {this.light('TRAF', this.traf, 'TRAF_PUSH', 480, 500)}

            <div class="mfd-fcubkup-label" style="left: 60px; top: 625px; width: 200px;">
              ADF/VOR 1
            </div>
            <div class="mfd-fcubkup-item" style="left: 90px; top: 660px;">
              <RadioButtonGroup
                values={NAVAID_MODES}
                selectedIndex={this.nav1Index}
                onModified={(i) => (this.targets.nav1 = NAVAID_VALUE[i])}
                idPrefix={`${prefix}Nav1`}
                color={Subject.create(RadioButtonColor.Green)}
              />
            </div>
            <div class="mfd-fcubkup-label" style="left: 480px; top: 625px; width: 200px;">
              ADF/VOR 2
            </div>
            <div class="mfd-fcubkup-item" style="left: 510px; top: 660px;">
              <RadioButtonGroup
                values={NAVAID_MODES}
                selectedIndex={this.nav2Index}
                onModified={(i) => (this.targets.nav2 = NAVAID_VALUE[i])}
                idPrefix={`${prefix}Nav2`}
                color={Subject.create(RadioButtonColor.Green)}
              />
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
}

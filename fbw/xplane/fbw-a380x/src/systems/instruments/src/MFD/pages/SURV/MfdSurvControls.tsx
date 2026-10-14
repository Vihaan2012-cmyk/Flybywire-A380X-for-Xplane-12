import {
  ConsumerSubject,
  DisplayComponent,
  FSComponent,
  MappedSubject,
  SimVarValueType,
  Subject,
  SubscribableMapFunctions,
  Subscription,
  VNode,
} from '@microsoft/msfs-sdk';

import './MfdSurvControls.scss';

import { MfdSurvEvents } from '../../../MsfsAvionicsCommon/providers/MfdSurvPublisher';
import { ActivePageTitleBar } from '../common/ActivePageTitleBar';
import { AbstractMfdPageProps } from '../../MFD';
import { Footer } from '../common/Footer';
import { InputField } from '../../../MsfsAvionicsCommon/UiWidgets/InputField';
import {
  AltitudeFormat,
  FlightLevelFormat,
  SquawkFormat,
  WxrGainFormat,
  WxrTiltFormat,
} from '../common/DataEntryFormats';
import { ConfirmationDialog } from '../../../MsfsAvionicsCommon/UiWidgets/ConfirmationDialog';
import { Button } from '../../../MsfsAvionicsCommon/UiWidgets/Button';
import { RadioButtonColor, RadioButtonGroup } from '../../../MsfsAvionicsCommon/UiWidgets/RadioButtonGroup';
import { MfdSimvars } from '../../shared/MFDSimvarPublisher';
import { SurvButton } from '../../../MsfsAvionicsCommon/UiWidgets/SurvButton';
import { NXSystemMessages } from '../../shared/NXSystemMessages';
import { FmsErrorType } from '@fmgc/FmsError';

interface MfdSurvControlsProps extends AbstractMfdPageProps {}

export enum TransponderState {
  Off = 0,
  Standby = 1,
  Test = 2,
  ModeA = 3,
  ModeC = 4,
  ModeS = 5,
}

export class MfdSurvControls extends DisplayComponent<MfdSurvControlsProps> {
  // Make sure to collect all subscriptions here, otherwise page navigation doesn't work.
  private readonly subs = [] as Subscription[];

  private readonly sub = this.props.bus.getSubscriber<MfdSimvars & MfdSurvEvents>();

  private readonly xpdrFailed = Subject.create<boolean>(false);

  private readonly squawkCode = Subject.create<number | null>(null);

  private readonly xpdrAltRptgAvailable = Subject.create<boolean>(true);

  private readonly xpdrSetAltReportingRequest = ConsumerSubject.create(this.sub.on('mfd_xpdr_set_alt_reporting'), true);

  private readonly xpdrState = ConsumerSubject.create(this.sub.on('xpdrState'), TransponderState.Off);

  private readonly xpdrAltRptgOn = Subject.create<boolean>(true);
  private readonly xpdrAltRptgDisabled = MappedSubject.create(
    ([failed, avail]) => failed || !avail,
    this.xpdrFailed,
    this.xpdrAltRptgAvailable,
  );

  private readonly xpdrStatusSelectedIndex = Subject.create<number | null>(0);
  private readonly xpdrStatusRadioColor = this.xpdrStatusSelectedIndex.map((it) =>
    it === 0 ? RadioButtonColor.Green : RadioButtonColor.White,
  );

  private readonly tcasFailed = ConsumerSubject.create(this.sub.on('tcasFail'), true);

  private readonly tcasRadioGroupDisabled = MappedSubject.create(
    ([tcasFailed, xpdrState]) =>
      Array(3).fill(tcasFailed || xpdrState === TransponderState.Off || xpdrState === TransponderState.Standby),
    this.tcasFailed,
    this.xpdrState,
  );

  private readonly tcasTaraSelectedIndex = Subject.create<number | null>(2);
  private readonly tcasTaraRadioColor = this.tcasTaraSelectedIndex.map((it) =>
    it === 0 ? RadioButtonColor.Green : RadioButtonColor.White,
  );

  private readonly tcasNormAbvBlwSelectedIndex = Subject.create<number | null>(0);
  private readonly tcasNormAbvBlwRadioColor = this.tcasNormAbvBlwSelectedIndex.map((it) =>
    it === 0 ? RadioButtonColor.Green : RadioButtonColor.Cyan,
  );

  private readonly activeSystemGroupWxrTaws = ConsumerSubject.create(this.sub.on('wxrTawsSysSelected'), 0);

  private readonly side: 'L' | 'R' = this.props.mfd.uiService.captOrFo === 'CAPT' ? 'L' : 'R';

  private readonly wxr1Failed = ConsumerSubject.create(this.sub.on('wxr1Failed'), true);

  private readonly wxr2Failed = ConsumerSubject.create(this.sub.on('wxr2Failed'), true);

  private readonly wxrFailed = MappedSubject.create(
    ([selected, f1, f2]) => (selected === 1 ? f1 : selected === 2 ? f2 : true),
    this.activeSystemGroupWxrTaws,
    this.wxr1Failed,
    this.wxr2Failed,
  );

  private readonly wxrOff = ConsumerSubject.create(this.sub.on('wxrOff'), false);

  private readonly wxrPwsOff = ConsumerSubject.create(this.sub.on('wxrPwsOff'), false);

  private readonly wxrTurbOff = ConsumerSubject.create(this.sub.on('wxrTurbOff'), false);

  private readonly wxrElevnTilt = ConsumerSubject.create(
    this.sub.on(this.side === 'L' ? 'wxrElevnTiltL' : 'wxrElevnTiltR'),
    0,
  );

  private readonly wxrElevnFt = ConsumerSubject.create(this.sub.on(this.side === 'L' ? 'wxrElevnFtL' : 'wxrElevnFtR'), 0);

  private readonly wxrTiltDeg = ConsumerSubject.create(this.sub.on(this.side === 'L' ? 'wxrTiltDegL' : 'wxrTiltDegR'), 0);

  private readonly wxrGainMan = ConsumerSubject.create(
    this.sub.on(this.side === 'L' ? 'wxrGainManL' : 'wxrGainManR'),
    false,
  );

  private readonly wxrGainPct = ConsumerSubject.create(this.sub.on(this.side === 'L' ? 'wxrGainPctL' : 'wxrGainPctR'), 50);

  private readonly wxrModeMap = ConsumerSubject.create(
    this.sub.on(this.side === 'L' ? 'wxrModeMapL' : 'wxrModeMapR'),
    false,
  );

  private readonly wxrVdOff = ConsumerSubject.create(this.sub.on(this.side === 'L' ? 'wxrVdOffL' : 'wxrVdOffR'), false);

  private readonly baroStd = ConsumerSubject.create(this.sub.on(this.side === 'L' ? 'baroStdL' : 'baroStdR'), false);

  private readonly wxrInhibited = MappedSubject.create(SubscribableMapFunctions.or(), this.wxrFailed, this.wxrOff);

  private readonly wxrAuto = this.wxrOff.map(SubscribableMapFunctions.not());

  private readonly wxrPredWsAuto = MappedSubject.create(([off, pwsOff]) => !off && !pwsOff, this.wxrOff, this.wxrPwsOff);

  private readonly wxrTurbAuto = MappedSubject.create(([off, turbOff]) => !off && !turbOff, this.wxrOff, this.wxrTurbOff);

  private readonly wxrElevnTiltSelectedIndex = MappedSubject.create(
    ([off, map, mode]) => (off || map ? null : Math.round(mode)),
    this.wxrOff,
    this.wxrModeMap,
    this.wxrElevnTilt,
  );

  private readonly wxrElevnTiltDisabled = MappedSubject.create(
    ([inhibited, map]) => Array(3).fill(inhibited || map),
    this.wxrInhibited,
    this.wxrModeMap,
  );

  private readonly wxrElevnTiltColor = this.wxrElevnTiltSelectedIndex.map((it) =>
    it === 0 ? RadioButtonColor.Green : RadioButtonColor.Cyan,
  );

  private readonly wxrElevnFieldShown = MappedSubject.create(
    ([index, std]) => (index === 1 ? (std ? 'FL' : 'FT') : null),
    this.wxrElevnTiltSelectedIndex,
    this.baroStd,
  );

  private readonly wxrElevnFtValue = Subject.create<number | null>(null);

  private readonly wxrElevnFlValue = Subject.create<number | null>(null);

  private readonly wxrTiltValue = Subject.create<number | null>(null);

  private readonly wxrGainValue = Subject.create<number | null>(null);

  private readonly wxrGainAuto = this.wxrGainMan.map(SubscribableMapFunctions.not());

  private readonly wxrModeWx = this.wxrModeMap.map(SubscribableMapFunctions.not());

  private readonly wxrOnVd = MappedSubject.create(([off, vdOff]) => !off && !vdOff, this.wxrOff, this.wxrVdOff);

  private readonly wxrGainFieldShown = MappedSubject.create(([off, man]) => !off && man, this.wxrOff, this.wxrGainMan);

  private readonly wxrConfirmVisible = Subject.create(false);

  private readonly wxrConfirmLabel = Subject.create('');

  private wxrConfirmAction: () => void = () => {};
  private readonly terr1Failed = ConsumerSubject.create(this.sub.on('terr1Failed'), false);
  private readonly gpws1Failed = ConsumerSubject.create(this.sub.on('gpws1Failed'), false);
  private readonly terr2Failed = ConsumerSubject.create(this.sub.on('terr2Failed'), false);
  private readonly gpws2Failed = ConsumerSubject.create(this.sub.on('gpws2Failed'), false);

  private readonly tawsTerrFailed = MappedSubject.create(
    ([selected, f1, f2]) => (selected === 1 ? f1 : selected === 2 ? f2 : true),
    this.activeSystemGroupWxrTaws,
    this.terr1Failed,
    this.terr2Failed,
  );

  private readonly tawsGpwsFailed = MappedSubject.create(
    ([selected, f1, f2]) => (selected === 1 ? f1 : selected === 2 ? f2 : true),
    this.activeSystemGroupWxrTaws,
    this.gpws1Failed,
    this.gpws2Failed,
  );

  private readonly allTawsFailed = MappedSubject.create(
    SubscribableMapFunctions.and(),
    this.tawsTerrFailed,
    this.tawsGpwsFailed,
  );

  private readonly tawsTerrSysOn = Subject.create<boolean>(true);

  private readonly tawsGpwsOn = Subject.create<boolean>(true);

  private readonly tawsGsModeOn = Subject.create<boolean>(true);

  private readonly tawsFlapModeOn = Subject.create<boolean>(true);

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    const sub = this.props.bus.getSubscriber<MfdSimvars & MfdSurvEvents>();

    this.subs.push(
      sub
        .on('xpdrCode')
        .whenChanged()
        .handle((code) => {
          this.squawkCode.set(code);
        }),
    );

    this.subs.push(this.xpdrState.sub(() => this.xpdrStatusChanged(), true));

    this.subs.push(
      this.wxrElevnFt.sub((ft) => {
        this.wxrElevnFtValue.set(ft);
        this.wxrElevnFlValue.set(Math.round(ft / 100));
      }, true),
      this.wxrTiltDeg.sub((deg) => this.wxrTiltValue.set(deg), true),
      this.wxrGainPct.sub((pct) => this.wxrGainValue.set(pct), true),
    );
    this.subs.push(this.xpdrSetAltReportingRequest.sub(() => this.xpdrStatusChanged(), true));

    this.subs.push(sub.on('mfd_tcas_alert_level').handle((val) => this.tcasTaraSelectedIndex.set(2 - val)));

    this.subs.push(sub.on('mfd_tcas_alt_select').handle((val) => this.tcasNormAbvBlwSelectedIndex.set(val)));

    this.subs.push(
      sub
        .on('gpwsTerrOff')
        .whenChanged()
        .handle((it) => this.tawsTerrSysOn.set(!it)),
    );

    this.subs.push(
      sub
        .on('gpwsSysOff')
        .whenChanged()
        .handle((it) => this.tawsGpwsOn.set(!it)),
    );

    this.subs.push(
      sub
        .on('gpwsGsInhibit')
        .whenChanged()
        .handle((it) => this.tawsGsModeOn.set(!it)),
    );

    this.subs.push(
      sub
        .on('gpwsFlapsInhibit')
        .whenChanged()
        .handle((it) => this.tawsFlapModeOn.set(!it)),
    );

    this.subs.push(
      this.xpdrSetAltReportingRequest,
      this.xpdrState,
      this.xpdrAltRptgDisabled,
      this.xpdrStatusRadioColor,
      this.tcasTaraRadioColor,
      this.tcasNormAbvBlwRadioColor,
      this.tcasFailed,
      this.tcasRadioGroupDisabled,
      this.activeSystemGroupWxrTaws,
      this.terr1Failed,
      this.terr2Failed,
      this.gpws1Failed,
      this.gpws2Failed,
      this.tawsTerrFailed,
      this.tawsGpwsFailed,
      this.allTawsFailed,
      this.wxr1Failed,
      this.wxr2Failed,
      this.wxrFailed,
      this.wxrOff,
      this.wxrPwsOff,
      this.wxrTurbOff,
      this.wxrElevnTilt,
      this.wxrElevnFt,
      this.wxrTiltDeg,
      this.wxrGainMan,
      this.wxrGainPct,
      this.wxrModeMap,
      this.wxrVdOff,
      this.baroStd,
      this.wxrInhibited,
      this.wxrAuto,
      this.wxrPredWsAuto,
      this.wxrTurbAuto,
      this.wxrElevnTiltSelectedIndex,
      this.wxrElevnTiltDisabled,
      this.wxrElevnTiltColor,
      this.wxrElevnFieldShown,
      this.wxrGainAuto,
      this.wxrModeWx,
      this.wxrOnVd,
      this.wxrGainFieldShown,
    );
  }

  private setWxrVar(name: string, value: number) {
    SimVar.SetSimVarValue(`L:A380X_WXR_${name}`, SimVarValueType.Number, value);
  }

  private setWxrSideVar(side: 'L' | 'R', name: string, value: number) {
    this.setWxrVar(`${side}_${name}`, value);
  }

  private setWxrSideDefaults(side: 'L' | 'R') {
    this.setWxrSideVar(side, 'ELEVN_TILT', 0);
    this.setWxrSideVar(side, 'GAIN_MAN', 0);
    this.setWxrSideVar(side, 'MODE_MAP', 0);
    this.setWxrSideVar(side, 'VD_OFF', 0);
  }

  private setWxrOn(on: boolean) {
    this.setWxrVar('OFF', on ? 0 : 1);
    this.setWxrVar('PWS_OFF', on ? 0 : 1);
    this.setWxrVar('TURB_OFF', on ? 0 : 1);
    for (const side of ['L', 'R'] as const) {
      if (on) {
        this.setWxrSideDefaults(side);
      } else {
        this.setWxrSideVar(side, 'ELEVN_TILT', 0);
        this.setWxrSideVar(side, 'VD_OFF', 1);
      }
    }
  }

  private selectElevnTilt(index: number) {
    if (index === 1 && Math.round(this.wxrElevnTilt.get()) !== 1) {
      const altitude = SimVar.GetSimVarValue('INDICATED ALTITUDE', 'feet');
      this.setWxrSideVar(this.side, 'ELEVN_FT', Math.max(0, Math.round(altitude / 100) * 100));
    } else if (index === 2 && Math.round(this.wxrElevnTilt.get()) !== 2) {
      this.setWxrSideVar(this.side, 'TILT_DEG', this.props.fmcService.master.fmgc.isOnGround() ? 3 : 0);
    }
    this.setWxrSideVar(this.side, 'ELEVN_TILT', index);
  }

  private askConfirmation(label: string, action: () => void) {
    this.wxrConfirmLabel.set(label);
    this.wxrConfirmAction = action;
    this.wxrConfirmVisible.set(true);
  }

  public destroy(): void {
    // Destroy all subscriptions to remove all references to this instance.
    for (const s of this.subs) {
      s.destroy();
    }

    super.destroy();
  }

  private xpdrStatusChanged() {
    const state = this.xpdrState.get();
    const isOnGround = this.props.fmcService.master.fmgc.isOnGround();

    this.xpdrStatusSelectedIndex.set(
      state === TransponderState.ModeA || state === TransponderState.ModeC || state === TransponderState.ModeS ? 0 : 1,
    );

    // On ground, Mode C is inhibited, we can only update from transponder state once we're in the air
    this.xpdrAltRptgOn.set(
      isOnGround
        ? this.xpdrSetAltReportingRequest.get()
        : state === TransponderState.ModeC || state === TransponderState.ModeS,
    );
    this.xpdrAltRptgAvailable.set(
      state === TransponderState.ModeA || state === TransponderState.ModeC || state === TransponderState.ModeS,
    );
  }

  private setDefaultSettings() {
    if (!this.xpdrFailed.get()) {
      this.props.bus.getPublisher<MfdSurvEvents>().pub('mfd_xpdr_set_auto', true, true);
      this.props.bus.getPublisher<MfdSurvEvents>().pub('mfd_xpdr_set_alt_reporting', true, true);
    }

    if (!this.tcasFailed.get()) {
      this.props.bus.getPublisher<MfdSurvEvents>().pub('mfd_tcas_alert_level', 2, true); // TA/RA
      this.props.bus.getPublisher<MfdSurvEvents>().pub('mfd_tcas_alt_select', 0, true); // NORM
    }

    if (!this.wxrFailed.get()) {
      this.setWxrVar('OFF', 0);
      this.setWxrVar('PWS_OFF', 0);
      this.setWxrVar('TURB_OFF', 0);
      this.setWxrSideDefaults(this.side);
    }

    if (!this.tawsTerrFailed.get()) {
      SimVar.SetSimVarValue('L:A32NX_GPWS_TERR_OFF', SimVarValueType.Bool, false);
    }

    if (!this.tawsGpwsFailed.get()) {
      SimVar.SetSimVarValue('L:A32NX_GPWS_SYS_OFF', SimVarValueType.Bool, false);
      SimVar.SetSimVarValue('L:A32NX_GPWS_GS_OFF', SimVarValueType.Bool, false);
      SimVar.SetSimVarValue('L:A32NX_GPWS_FLAPS_OFF', SimVarValueType.Bool, false);
    }
  }

  render(): VNode {
    return (
      <>
        <ActivePageTitleBar activePage={Subject.create('CONTROLS')} offset={Subject.create('')} />
        {/* begin page content */}
        <div class="mfd-page-container">
          <div class="mfd-surv-controls-first-section">
            <div class="mfd-surv-controls-xpdr-section">
              <div class="mfd-surv-controls-xpdr-left">
                <div class={{ 'mfd-surv-heading': true, 'mfd-surv-xpdr-label': true, failed: this.xpdrFailed }}>
                  XPDR
                </div>
                <div class="mfd-label bigger" style="margin-top: 30px;">
                  SQWK
                </div>
                <InputField<number>
                  dataEntryFormat={new SquawkFormat()}
                  dataHandlerDuringValidation={async (v) =>
                    v ? SimVar.SetSimVarValue('K:XPNDR_SET', 'number', parseInt(v.toString(), 16)) : false
                  }
                  value={this.squawkCode}
                  containerStyle="width: 100px; margin-bottom: 5px;"
                  errorHandler={(e) => {
                    if (e.type == FmsErrorType.FormatError) {
                      this.props.fmcService.master.addMessageToQueue(
                        NXSystemMessages.sqwkCodeNotValid,
                        undefined,
                        undefined,
                      );
                    } else {
                      this.props.fmcService.master.showFmsErrorMessage(e.type, e.details);
                    }
                  }}
                  hEventConsumer={this.props.mfd.hEventConsumer}
                  interactionMode={this.props.mfd.interactionMode}
                  alignText={'center'}
                />
                <Button
                  label={'IDENT'}
                  onClick={() => SimVar.SetSimVarValue('K:XPNDR_IDENT_ON', SimVarValueType.Bool, true)}
                  buttonStyle="width: 100px;"
                />
                <div class="mfd-label bigger" style="margin-top: 20px; margin-bottom: 5px;">
                  ALT RPTG
                </div>
                <SurvButton
                  state={this.xpdrAltRptgOn}
                  disabled={this.xpdrAltRptgDisabled}
                  labelFalse={'OFF'}
                  labelTrue={'ON'}
                  onChanged={(v) => {
                    this.props.bus.getPublisher<MfdSurvEvents>().pub('mfd_xpdr_set_alt_reporting', v, true);
                  }}
                />
              </div>
              <div class="mfd-surv-controls-xpdr-right">
                <RadioButtonGroup
                  values={['AUTO', 'STBY']}
                  onModified={(val) =>
                    this.props.bus.getPublisher<MfdSurvEvents>().pub('mfd_xpdr_set_auto', val === 0, true)
                  }
                  selectedIndex={this.xpdrStatusSelectedIndex}
                  idPrefix={`${this.props.mfd.uiService.captOrFo}_MFD_survControlsXpdrStatus`}
                  additionalVerticalSpacing={50}
                  color={this.xpdrStatusRadioColor}
                />
              </div>
            </div>
            <div class="mfd-surv-controls-tcas-section">
              <div class={{ 'mfd-surv-heading': true, 'mfd-surv-tcas-label': true, failed: this.tcasFailed }}>TCAS</div>
              <div class="mfd-surv-controls-tcas-left">
                <RadioButtonGroup
                  values={['TA/RA', 'TA ONLY', 'STBY']}
                  onModified={(val) =>
                    this.props.bus.getPublisher<MfdSurvEvents>().pub('mfd_tcas_alert_level', 2 - val, true)
                  }
                  selectedIndex={this.tcasTaraSelectedIndex}
                  idPrefix={`${this.props.mfd.uiService.captOrFo}_MFD_survControlsTcasTara`}
                  additionalVerticalSpacing={10}
                  valuesDisabled={this.tcasRadioGroupDisabled}
                  color={this.tcasTaraRadioColor}
                />
              </div>
              <div class="mfd-surv-controls-tcas-right">
                <RadioButtonGroup
                  values={['NORM', 'ABV', 'BLW']}
                  selectedIndex={this.tcasNormAbvBlwSelectedIndex}
                  onModified={(val) =>
                    this.props.bus.getPublisher<MfdSurvEvents>().pub('mfd_tcas_alt_select', val, true)
                  }
                  valuesDisabled={this.tcasRadioGroupDisabled}
                  idPrefix={`${this.props.mfd.uiService.captOrFo}_MFD_survControlsTcasNormAbvBlw`}
                  additionalVerticalSpacing={10}
                  color={this.tcasNormAbvBlwRadioColor}
                />
              </div>
            </div>
          </div>
          <div class="mfd-surv-controls-second-section">
            <div class="mfd-surv-controls-wxr-left">
              <div class={{ 'mfd-surv-heading': true, 'mfd-surv-wxr-label': true, failed: this.wxrFailed }}>WXR</div>
              <div class="mfd-label bigger" style="margin-top: 80px;">
                ELEVN/TILT
              </div>
              <RadioButtonGroup
                values={['AUTO', 'ELEVN', 'TILT']}
                selectedIndex={this.wxrElevnTiltSelectedIndex}
                onModified={(val) => this.selectElevnTilt(val)}
                idPrefix={`${this.props.mfd.uiService.captOrFo}_MFD_survControlswxrElevnTilt`}
                additionalVerticalSpacing={10}
                color={this.wxrElevnTiltColor}
                valuesDisabled={this.wxrElevnTiltDisabled}
              />
              <div class="mfd-surv-controls-wxr-entry">
                <div style={this.wxrElevnFieldShown.map((u) => (u === 'FT' ? 'display: block' : 'display: none'))}>
                  <InputField<number>
                    dataEntryFormat={new AltitudeFormat()}
                    dataHandlerDuringValidation={async (v) => {
                      if (v === null) {
                        return false;
                      }
                      this.setWxrSideVar(this.side, 'ELEVN_FT', Math.round(v / 100) * 100);
                      return true;
                    }}
                    value={this.wxrElevnFtValue}
                    containerStyle="width: 130px;"
                    errorHandler={(e) => this.props.fmcService.master.showFmsErrorMessage(e.type, e.details)}
                    hEventConsumer={this.props.mfd.hEventConsumer}
                    interactionMode={this.props.mfd.interactionMode}
                    alignText={'center'}
                  />
                </div>
                <div style={this.wxrElevnFieldShown.map((u) => (u === 'FL' ? 'display: block' : 'display: none'))}>
                  <InputField<number>
                    dataEntryFormat={new FlightLevelFormat()}
                    dataHandlerDuringValidation={async (v) => {
                      if (v === null) {
                        return false;
                      }
                      this.setWxrSideVar(this.side, 'ELEVN_FT', Math.round(v) * 100);
                      return true;
                    }}
                    value={this.wxrElevnFlValue}
                    containerStyle="width: 130px;"
                    errorHandler={(e) => this.props.fmcService.master.showFmsErrorMessage(e.type, e.details)}
                    hEventConsumer={this.props.mfd.hEventConsumer}
                    interactionMode={this.props.mfd.interactionMode}
                    alignText={'center'}
                  />
                </div>
                <div style={this.wxrElevnTiltSelectedIndex.map((i) => (i === 2 ? 'display: block' : 'display: none'))}>
                  <InputField<number>
                    dataEntryFormat={new WxrTiltFormat()}
                    dataHandlerDuringValidation={async (v) => {
                      if (v === null) {
                        return false;
                      }
                      this.setWxrSideVar(this.side, 'TILT_DEG', v);
                      return true;
                    }}
                    value={this.wxrTiltValue}
                    containerStyle="width: 130px;"
                    errorHandler={(e) => this.props.fmcService.master.showFmsErrorMessage(e.type, e.details)}
                    hEventConsumer={this.props.mfd.hEventConsumer}
                    interactionMode={this.props.mfd.interactionMode}
                    alignText={'center'}
                  />
                </div>
              </div>
            </div>
            <div class="mfd-surv-controls-wxr-right">
              <div class="mfd-surv-controls-wxr-grid">
                <div class="mfd-surv-controls-wxr-grid-cell">
                  <div class="mfd-surv-label">WXR</div>
                  <SurvButton
                    state={this.wxrAuto}
                    disabled={this.wxrFailed}
                    labelFalse={'OFF'}
                    labelTrue={'AUTO'}
                    onChanged={(v) => this.askConfirmation(v ? 'WXR AUTO ?' : 'WXR OFF ?', () => this.setWxrOn(v))}
                  />
                </div>
                <div class="mfd-surv-controls-wxr-grid-cell">
                  <div class="mfd-surv-label">PRED W/S</div>
                  <SurvButton
                    state={this.wxrPredWsAuto}
                    disabled={this.wxrInhibited}
                    labelFalse={'OFF'}
                    labelTrue={'AUTO'}
                    onChanged={(v) => this.setWxrVar('PWS_OFF', v ? 0 : 1)}
                  />
                </div>
                <div class="mfd-surv-controls-wxr-grid-cell">
                  <div class="mfd-surv-label">TURB</div>
                  <SurvButton
                    state={this.wxrTurbAuto}
                    disabled={this.wxrInhibited}
                    labelFalse={'OFF'}
                    labelTrue={'AUTO'}
                    onChanged={(v) => this.setWxrVar('TURB_OFF', v ? 0 : 1)}
                  />
                </div>
                <div
                  class="mfd-surv-controls-wxr-grid-cell"
                  style={this.wxrOff.map((off) => (off ? 'visibility: hidden' : 'visibility: visible'))}
                >
                  <div class="mfd-surv-label">GAIN</div>
                  <SurvButton
                    state={this.wxrGainAuto}
                    disabled={this.wxrFailed}
                    labelFalse={'MAN'}
                    labelTrue={'AUTO'}
                    onChanged={(v) => {
                      if (!v) {
                        this.setWxrSideVar(this.side, 'GAIN_PCT', 50);
                      }
                      this.setWxrSideVar(this.side, 'GAIN_MAN', v ? 0 : 1);
                    }}
                  />
                  <div
                    class="mfd-surv-controls-wxr-gain-entry"
                    style={this.wxrGainFieldShown.map((shown) => (shown ? 'display: block' : 'display: none'))}
                  >
                    <InputField<number>
                      dataEntryFormat={new WxrGainFormat()}
                      dataHandlerDuringValidation={async (v) => {
                        if (v === null) {
                          return false;
                        }
                        this.setWxrSideVar(this.side, 'GAIN_PCT', v);
                        return true;
                      }}
                      value={this.wxrGainValue}
                      containerStyle="width: 90px;"
                      errorHandler={(e) => this.props.fmcService.master.showFmsErrorMessage(e.type, e.details)}
                      hEventConsumer={this.props.mfd.hEventConsumer}
                      interactionMode={this.props.mfd.interactionMode}
                      alignText={'center'}
                    />
                  </div>
                </div>
                <div
                  class="mfd-surv-controls-wxr-grid-cell"
                  style={this.wxrOff.map((off) => (off ? 'visibility: hidden' : 'visibility: visible'))}
                >
                  <div class="mfd-surv-label">MODE</div>
                  <SurvButton
                    state={this.wxrModeWx}
                    disabled={this.wxrFailed}
                    labelFalse={'MAP'}
                    labelTrue={'WX'}
                    onChanged={(v) => this.setWxrSideVar(this.side, 'MODE_MAP', v ? 0 : 1)}
                  />
                </div>
                <div class="mfd-surv-controls-wxr-grid-cell">
                  <div class="mfd-surv-label">WX ON VD</div>
                  <SurvButton
                    state={this.wxrOnVd}
                    disabled={this.wxrInhibited}
                    labelFalse={'OFF'}
                    labelTrue={'AUTO'}
                    onChanged={(v) => this.setWxrSideVar(this.side, 'VD_OFF', v ? 0 : 1)}
                  />
                </div>
              </div>
              <ConfirmationDialog
                visible={this.wxrConfirmVisible}
                cancelAction={() => this.wxrConfirmVisible.set(false)}
                confirmAction={() => {
                  this.wxrConfirmVisible.set(false);
                  this.wxrConfirmAction();
                }}
                contentContainerStyle="width: 300px; height: 140px; left: -560px; top: -10px;"
              >
                {this.wxrConfirmLabel}
              </ConfirmationDialog>
            </div>
          </div>
          <div class="mfd-surv-controls-third-section">
            <div class={{ 'mfd-surv-heading': true, 'mfd-surv-taws-label': true, failed: this.allTawsFailed }}>
              TAWS
            </div>
            <div class="mfd-surv-controls-taws-section">
              <div class="mfd-surv-controls-taws-element" style="padding-right: 50px;">
                <div class="mfd-surv-label">TERR SYS</div>
                <SurvButton
                  state={this.tawsTerrSysOn}
                  disabled={this.tawsTerrFailed}
                  labelFalse={'OFF'}
                  labelTrue={'ON'}
                  onChanged={(v) => SimVar.SetSimVarValue('L:A32NX_GPWS_TERR_OFF', SimVarValueType.Bool, !v)}
                />
              </div>
              <div class="mfd-surv-controls-taws-element">
                <div class="mfd-surv-label">GPWS</div>
                <SurvButton
                  state={this.tawsGpwsOn}
                  disabled={this.tawsGpwsFailed}
                  labelFalse={'OFF'}
                  labelTrue={'ON'}
                  onChanged={(v) => SimVar.SetSimVarValue('L:A32NX_GPWS_SYS_OFF', SimVarValueType.Bool, !v)}
                />
              </div>
              <div class="mfd-surv-controls-taws-element">
                <div class="mfd-surv-label">G/S MODE</div>
                <SurvButton
                  state={this.tawsGsModeOn}
                  disabled={this.tawsGpwsFailed}
                  labelFalse={'OFF'}
                  labelTrue={'ON'}
                  onChanged={(v) => SimVar.SetSimVarValue('L:A32NX_GPWS_GS_OFF', SimVarValueType.Bool, !v)}
                />
              </div>
              <div class="mfd-surv-controls-taws-element">
                <div class="mfd-surv-label">FLAP MODE</div>
                <SurvButton
                  state={this.tawsFlapModeOn}
                  disabled={this.tawsGpwsFailed}
                  labelFalse={'OFF'}
                  labelTrue={'ON'}
                  onChanged={(v) => SimVar.SetSimVarValue('L:A32NX_GPWS_FLAPS_OFF', SimVarValueType.Bool, !v)}
                />
              </div>
            </div>
            <div class="mfd-surv-controls-def-settings-container">
              <div class="mfd-surv-controls-def-settings">
                <div class="mfd-surv-label">SURV</div>
                <Button
                  label={'DEFAULT SETTINGS'}
                  onClick={() => this.askConfirmation('DEFAULT SETTINGS ?', () => this.setDefaultSettings())}
                  buttonStyle="width: 140px;"
                />
              </div>
            </div>
          </div>
        </div>
        {/* end page content */}
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

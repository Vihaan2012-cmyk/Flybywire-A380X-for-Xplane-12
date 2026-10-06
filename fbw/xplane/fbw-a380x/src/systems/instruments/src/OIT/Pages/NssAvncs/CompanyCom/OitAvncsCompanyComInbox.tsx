//  Copyright (c) 2025 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { ArraySubject, FSComponent, Subject, Subscribable, VNode } from '@microsoft/msfs-sdk';
import { AbstractOitAvncsPageProps } from '../../../OIT';
import { DestroyableComponent } from '@flybywiresim/msfs-avionics-common';
import { OitSimvars } from '../../../OitSimvarPublisher';
import { AnsuOps } from '../../../System/AnsuOps';
import { Arinc429Register, SeatFlags } from '@flybywiresim/fbw-sdk';

interface OitAvncsCompanyComInboxProps extends AbstractOitAvncsPageProps {}

export interface OitAvncsCompanyComMessages {
  read: Subscribable<boolean>;
  subject: string;
  message: string;
  date: number;
}

/**
 * Zero fuel weight, take-off weight and landing weight structural limits (kg), FlyByWire's
 * own (`flight_model.cfg`'s `[WEIGHT_AND_BALANCE]` comment: MZFW 373 000 kg, MTOW 510 000 kg,
 * MLW 395 000 kg, for this port's WV003 weight variant).
 */
const MZFW_KG = 373_000;
const MTOW_KG = 510_000;
const MLW_KG = 395_000;

/**
 * The fourteen cabin zones FlyByWire's payload module publishes: simvar name and seat count,
 * from `config/a380x/a380-842/cabin.json5`'s `seatMap` (the same table the X-Plane port's own
 * loadsheet page mirrors in `study/loadsheet.rs`'s `PAX_ZONES`). Each zone's simvar is a
 * seat-occupancy bitmask (see `A380Payload.tsx`'s "Calculate Total Pax from Pax Flags"), not
 * a plain count.
 */
const LOADSHEET_PAX_ZONES: ReadonlyArray<{ simVar: string; capacity: number }> = [
  { simVar: 'A32NX_PAX_MAIN_FWD_A', capacity: 28 },
  { simVar: 'A32NX_PAX_MAIN_FWD_B', capacity: 28 },
  { simVar: 'A32NX_PAX_MAIN_MID_1A', capacity: 39 },
  { simVar: 'A32NX_PAX_MAIN_MID_1B', capacity: 50 },
  { simVar: 'A32NX_PAX_MAIN_MID_1C', capacity: 43 },
  { simVar: 'A32NX_PAX_MAIN_MID_2A', capacity: 48 },
  { simVar: 'A32NX_PAX_MAIN_MID_2B', capacity: 40 },
  { simVar: 'A32NX_PAX_MAIN_MID_2C', capacity: 36 },
  { simVar: 'A32NX_PAX_MAIN_AFT_A', capacity: 42 },
  { simVar: 'A32NX_PAX_MAIN_AFT_B', capacity: 40 },
  { simVar: 'A32NX_PAX_UPPER_FWD', capacity: 14 },
  { simVar: 'A32NX_PAX_UPPER_MID_A', capacity: 30 },
  { simVar: 'A32NX_PAX_UPPER_MID_B', capacity: 28 },
  { simVar: 'A32NX_PAX_UPPER_AFT', capacity: 18 },
];

/**
 * A real LOADSHEET inbox message, built from the same weights the flight deck loadsheet
 * itself reports: ZFW/TOW and their %MAC (`A32NX_AIRFRAME_*`, the plain weight/balance
 * simvars `weight_balance.rs` and `study/loadsheet.rs` read under the same names), take-off
 * fuel (`A32NX_FQMS_TOTAL_FUEL_ON_BOARD`, a raw ARINC 429 word per `FqmsBusPublisher.ts`, so
 * decoded like any other one rather than read raw), the FMC's own trip fuel prediction
 * (`L:A32NX_FM_TRIP_FUEL_AT_PREFLIGHT`, newly published by `FmcAircraftInterface.ts` -- see
 * that edit), and the resulting landing weight and tightest underload against the airframe's
 * structural limits.
 */
function loadsheetMessageText(): string {
  const zfw = SimVar.GetSimVarValue('L:A32NX_AIRFRAME_ZFW', 'number');
  const macZfw = SimVar.GetSimVarValue('L:A32NX_AIRFRAME_ZFW_CG_PERCENT_MAC', 'number');
  const tow = SimVar.GetSimVarValue('L:A32NX_AIRFRAME_TOW', 'number');
  const macTow = SimVar.GetSimVarValue('L:A32NX_AIRFRAME_TO_CG_PERCENT_MAC', 'number');

  const fob = Arinc429Register.empty().setFromSimVar('L:A32NX_FQMS_TOTAL_FUEL_ON_BOARD');
  const tof = fob.isNormalOperation() ? fob.value : NaN;

  const tif = SimVar.GetSimVarValue('L:A32NX_FM_TRIP_FUEL_AT_PREFLIGHT', 'number');
  const law = tow - tif;

  const paxTtl = LOADSHEET_PAX_ZONES.reduce(
    (sum, z) =>
      sum + new SeatFlags(SimVar.GetSimVarValue(`L:${z.simVar}`, 'number'), z.capacity).getTotalFilledSeats(),
    0,
  );

  const undld = Math.min(MZFW_KG - zfw, MTOW_KG - tow, MLW_KG - law);

  const kg = (v: number) => (Number.isFinite(v) ? `${Math.round(v)} KG` : '--- KG');

  return [
    `ZFW    ${kg(zfw)}      MACZFW  ${macZfw.toFixed(1)}`,
    `TOW    ${kg(tow)}      MACTOW  ${macTow.toFixed(1)}`,
    `TOF    ${kg(tof)}      TIF     ${kg(tif)}`,
    `LAW    ${kg(law)}`,
    `UNDLD  ${kg(undld)}      PAX TTL ${paxTtl}`,
  ].join('\n');
}

export abstract class OitAvncsCompanyComInbox extends DestroyableComponent<OitAvncsCompanyComInboxProps> {
  private readonly sci = this.props.container.ansu.sci;

  private readonly sub = this.props.bus.getSubscriber<OitSimvars>();

  private readonly messageContainerRef = FSComponent.createRef<HTMLDivElement>();

  // A real message, built from the same live weight/balance and fuel figures the flight deck
  // loadsheet itself reports (loadsheetMessageText(), above). Computed once, like the rest of
  // this inbox's content: a loadsheet message does not update itself once it has arrived.
  private readonly loadsheetMessage = loadsheetMessageText();

  private readonly messages = ArraySubject.create<OitAvncsCompanyComMessages>([
    {
      read: Subject.create(true),
      subject: 'LOADSHEET',
      message: this.loadsheetMessage,
      date: Date.now(),
    },
    {
      read: Subject.create(true),
      subject: 'FBW A380X TEST MESSAGE',
      message:
        'This is a test message for the inbox. As soon as the communication backend is implemented, this will be replaced by real messages.',
      date: Date.now(),
    },
  ]);

  private readonly toText = this.sci.fltNumber.map((fltNumber) => (fltNumber ? `To : ${fltNumber}` : 'To : <unknown>'));
  private readonly subjectText = Subject.create('Subject : LOADSHEET');
  private readonly dateText = Subject.create(AnsuOps.formatDateTime(Date.now()));

  private readonly messageText = Subject.create(this.loadsheetMessage);

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    this.subscriptions.push(this.toText);
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
          <div ref={this.messageContainerRef}>
            {this.messages.getArray().map((message, index) => (
              <OitAvncsCompanyComMessageLine
                date={message.date}
                subject={message.subject}
                read={message.read}
                selected={Subject.create(index === 0)}
              />
            ))}
          </div>
          <div style="flex-grow: 1" />
        </div>
        <div class="oit-ccom-inbox-msg-details">
          <div>Message is read</div>
          <div />
          <div style="text-align: right;">{this.dateText}</div>
          <div>{this.toText}</div>
          <div>{this.subjectText}</div>
          <div />
        </div>
        <div class="oit-ccom-inbox-msg-box">{this.messageText}</div>
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

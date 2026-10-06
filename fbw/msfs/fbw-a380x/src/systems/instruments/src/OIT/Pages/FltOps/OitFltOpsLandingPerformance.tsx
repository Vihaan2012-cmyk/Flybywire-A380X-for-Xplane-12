//  Copyright (c) 2025 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { ArraySubject, FSComponent, MappedSubject, Subject, VNode } from '@microsoft/msfs-sdk';
import { DestroyableComponent } from '../../../MsfsAvionicsCommon/DestroyableComponent';
import { FmsData, getSimBriefOfp, ISimbriefData, ISimbriefTlrPhase } from '@flybywiresim/fbw-sdk';
import { AbstractOitFltOpsPageProps } from '../../OIT';
import { Button } from '../../../MsfsAvionicsCommon/UiWidgets/Button';
import { DropdownMenu } from '../../../MsfsAvionicsCommon/UiWidgets/DropdownMenu';
import { InputField } from '../../../MsfsAvionicsCommon/UiWidgets/InputField';
import { AirportFormat, WeightFormat } from '../../../MFD/pages/common/DataEntryFormats';
import { OitFltOpsPerfOptions } from './OitFltOpsPerfOptions';
import {
  DEFAULT_UNITS,
  OisRunwayEnd,
  OisUnits,
  UNAVAILABLE,
  UNAVAILABLE_REASON,
  distanceUnitLabel,
  formatDistance,
  formatPressure,
  formatWeight,
  loadAirport,
  pressureAltitude,
  pressureUnitLabel,
  readWeather,
  readWeights,
  runwayEnds,
  weightUnitLabel,
  windComponents,
} from './OitFltOpsPerfCommon';

const RUNWAY_CONDITIONS = ['DRY', 'GOOD', 'MEDIUM'];
const FLAPS = ['CONF FULL', 'CONF 3'];
const TECHNIQUE = ['MANUAL', 'AUTOLAND'];
const BRAKE_MODE = ['LO', '2', '3', 'HI'];
const REVERSE = ['YES', 'NO'];

export class OitFltOpsLandingPerformance extends DestroyableComponent<AbstractOitFltOpsPageProps> {
  private readonly units = Subject.create<OisUnits>(DEFAULT_UNITS);

  private readonly sub = this.props.bus.getSubscriber<FmsData>();

  private readonly airport = Subject.create<string | null>(null);
  private readonly runwayIdents = ArraySubject.create<string>([]);
  private readonly selectedRunway = Subject.create<number | null>(null);
  private readonly runways = Subject.create<OisRunwayEnd[]>([]);
  private readonly databaseMessage = Subject.create('');

  private readonly runway = MappedSubject.create(
    ([list, index]) => (index === null ? null : (list[index] ?? null)),
    this.runways,
    this.selectedRunway,
  );

  private readonly landingWeight = Subject.create<number | null>(null);
  private readonly landingWeightSource = Subject.create('NOT ENTERED');

  private readonly oat = Subject.create<number | null>(null);
  private readonly qnh = Subject.create<number | null>(null);
  private readonly windText = Subject.create('---°/---');
  private readonly headwindText = Subject.create('---');
  private readonly crosswindText = Subject.create('---');
  private readonly pressureAltText = Subject.create('-----');

  private readonly runwayCondition = Subject.create<number | null>(0);
  private readonly flaps = Subject.create<number | null>(0);
  private readonly technique = Subject.create<number | null>(0);
  private readonly brakeMode = Subject.create<number | null>(0);
  private readonly reverse = Subject.create<number | null>(0);
  private readonly vPilot = Subject.create<number | null>(0);

  private readonly resultRunwayText = Subject.create('----/---');

  private tlrLanding: ISimbriefTlrPhase | null = null;
  private tlrWeightUnits = 'KG';
  private readonly tlrVrefText = Subject.create(UNAVAILABLE);
  private readonly tlrMaxDryText = Subject.create(UNAVAILABLE);
  private readonly tlrMaxWetText = Subject.create(UNAVAILABLE);
  private readonly tlrNote = Subject.create('SYNC SIMBRIEF FOR THE SIMBRIEF LANDING REPORT (OFP GENERATED WITH RUNWAY ANALYSIS ON)');
  private readonly fromSimbrief = Subject.create(false);

  private fmsDestinationValue: string | null = null;
  private fmsLandingRunwayValue: string | null = null;

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    this.subscriptions.push(
      this.runway,
      this.units.sub(() => {
        if (this.fromSimbrief.get()) {
          this.showTlr(this.runway.get()?.ident ?? null);
        }
      }),
      this.sub.on('fmsDestination').handle((destination) => {
        this.fmsDestinationValue = destination;
        if (destination && this.airport.get() === null) {
          this.airport.set(destination);
        }
      }),
      this.sub.on('fmsLandingRunway').handle((runway) => {
        this.fmsLandingRunwayValue = runway;
        if (runway && this.selectedRunway.get() === null) {
          this.selectRunway(this.fmsLandingRunwayIdent);
        }
      }),
      this.airport.sub((ident) => {
        if (ident) {
          this.loadRunways(ident);
        } else {
          this.runways.set([]);
          this.runwayIdents.clear();
          this.selectedRunway.set(null);
        }
      }),
      this.runway.sub(() => this.refreshConditions()),
    );

    this.refreshConditions();
  }

  private get fmsLandingRunwayIdent(): string | null {
    const raw = this.fmsLandingRunwayValue;
    return raw && raw.length > 4 ? raw.substring(4) : null;
  }

  private selectRunway(ident: string | null): void {
    const index = ident === null ? -1 : this.runways.get().findIndex((r) => r.ident === ident);
    this.selectedRunway.set(index >= 0 ? index : null);
  }

  private loadRunways(ident: string): void {
    this.databaseMessage.set('LOADING RUNWAY DATA...');
    loadAirport(ident)
      .then((airport) => {
        if (this.airport.get() !== ident) {
          return;
        }
        if (airport === null) {
          this.runways.set([]);
          this.runwayIdents.clear();
          this.selectedRunway.set(null);
          this.databaseMessage.set(`${ident} NOT IN NAVIGATION DATABASE`);
          return;
        }
        const ends = runwayEnds(airport);
        this.runways.set(ends);
        this.runwayIdents.set(ends.map((r) => r.ident));
        this.databaseMessage.set(ends.length === 0 ? `${ident} HAS NO RUNWAY DATA` : '');
        this.selectRunway(this.fmsLandingRunwayIdent);
      })
      .catch(() => this.databaseMessage.set('NAVIGATION DATABASE UNAVAILABLE'));
  }

  private refreshConditions(): void {
    const weather = readWeather();
    this.oat.set(Math.round(weather.oat));
    this.qnh.set(weather.qnh);
    this.windText.set(
      `${Math.round(weather.windDirectionTrue).toString().padStart(3, '0')}°T/${Math.round(weather.windKt)
        .toString()
        .padStart(3, '0')}KT`,
    );

    const rwy = this.runway.get();
    if (rwy === null) {
      this.headwindText.set('---');
      this.crosswindText.set('---');
      this.pressureAltText.set('-----');
      return;
    }
    const { headwind, crosswind } = windComponents(rwy.bearingTrue, weather);
    this.headwindText.set(`${headwind >= 0 ? 'HEAD' : 'TAIL'} ${Math.abs(headwind).toFixed(0)}KT`);
    this.crosswindText.set(`${crosswind >= 0 ? 'R' : 'L'} ${Math.abs(crosswind).toFixed(0)}KT`);
    this.pressureAltText.set(`${Math.round(pressureAltitude(rwy.elevationFt, weather.qnh))} FT`);
  }

  private syncFms(): void {
    if (this.fmsDestinationValue) {
      if (this.airport.get() === this.fmsDestinationValue) {
        this.selectRunway(this.fmsLandingRunwayIdent);
      } else {
        this.airport.set(this.fmsDestinationValue);
      }
    } else {
      this.databaseMessage.set('FMS HAS NO DESTINATION');
    }

    const gw = readWeights().grossWeightKg;
    if (gw === null) {
      this.landingWeightSource.set('FQMS GROSS WEIGHT INVALID');
    } else {
      this.landingWeight.set(gw);
      this.landingWeightSource.set('CURRENT GROSS WEIGHT (NO FMS LANDING WEIGHT PREDICTION AVAILABLE)');
    }
    this.refreshConditions();
  }

  private compute(): void {
    this.refreshConditions();
    const rwy = this.runway.get();
    this.resultRunwayText.set(rwy === null ? '----/---' : `${this.airport.get() ?? '----'}/${rwy.ident}`);
    this.showTlr(rwy === null ? null : rwy.ident);
  }

  private async syncSimbrief(): Promise<void> {
    this.databaseMessage.set('SIMBRIEF REQUEST SENT');
    let ofp: ISimbriefData;
    try {
      ofp = await getSimBriefOfp();
    } catch (e) {
      this.databaseMessage.set(`SIMBRIEF: ${String(e instanceof Error ? e.message : e).toUpperCase()}`);
      return;
    }
    this.tlrLanding = ofp.tlr?.landing ?? null;
    this.tlrWeightUnits = ofp.units === 'lbs' ? 'LB' : 'KG';
    const destination = ofp.destination?.icao ?? null;
    if (!destination) {
      this.databaseMessage.set('SIMBRIEF OFP HAS NO DESTINATION');
      return;
    }
    if (this.airport.get() === destination) {
      this.selectRunway(ofp.destination.runway || null);
    } else {
      this.airport.set(destination);
    }
    this.databaseMessage.set(`SIMBRIEF ${destination}/${ofp.destination.runway || '--'}`);
    this.showTlr(this.runway.get()?.ident ?? null);
  }

  private showTlr(ident: string | null): void {
    for (const t of [this.tlrVrefText, this.tlrMaxDryText, this.tlrMaxWetText]) {
      t.set(UNAVAILABLE);
    }
    this.fromSimbrief.set(false);
    if (this.tlrLanding === null) {
      return;
    }
    const runways = this.tlrLanding.runway;
    const list = runways === undefined ? [] : Array.isArray(runways) ? runways : [runways];
    const entry = ident === null ? undefined : list.find((r) => r.identifier === ident);
    if (!entry) {
      this.tlrNote.set(
        list.length === 0
          ? 'THE SIMBRIEF OFP HAS NO LANDING REPORT: GENERATE IT WITH RUNWAY ANALYSIS ON'
          : `THE SIMBRIEF LANDING REPORT HAS NO RWY ${ident ?? '---'}`,
      );
      return;
    }
    const num = (v: string | undefined): number | null => {
      const n = v === undefined || v.trim() === '' ? NaN : Number(v);
      return Number.isFinite(n) ? n : null;
    };
    const units = this.units.get();
    const toKg = (v: number) => (this.tlrWeightUnits === 'LB' ? v * 0.45359237 : v);
    const weight = (v: string | undefined) => {
      const n = num(v);
      return n === null ? UNAVAILABLE : `${formatWeight(toKg(n), units)} ${weightUnitLabel(units)}`;
    };
    const vref = num(entry.speeds_vref);
    this.tlrVrefText.set(vref === null ? UNAVAILABLE : `${Math.round(vref)} KT`);
    this.tlrMaxDryText.set(weight(entry.max_weight_dry));
    this.tlrMaxWetText.set(weight(entry.max_weight_wet));
    this.fromSimbrief.set(true);
    const c = this.tlrLanding.conditions ?? {};
    const assumed = [
      c.flap_setting ? `FLAPS ${c.flap_setting}` : '',
      c.temperature ? `${c.temperature}°C` : '',
      c.wind_direction && c.wind_speed ? `WIND ${c.wind_direction.padStart(3, '0')}/${c.wind_speed}KT` : '',
      c.surface_condition ? c.surface_condition.toUpperCase() : '',
    ].filter((x) => x !== '');
    this.tlrNote.set(
      `SIMBRIEF LANDING REPORT, RWY ${entry.identifier}${assumed.length ? ` (SIMBRIEF ASSUMED ${assumed.join(', ')})` : ''}: SIMBRIEF PLANNING DATA, NOT AIRBUS-CERTIFIED A380 PERFORMANCE`,
    );
  }

  private tlrClass() {
    return {
      'oit-label': true,
      bigger: true,
      cyan: this.fromSimbrief,
      amber: this.fromSimbrief.map((s) => !s),
    };
  }

  render(): VNode {
    const units = this.units;
    return (
      <>
        <div class="oit-page-container oit-perf-page">
          <div class="oit-labeled-box-container">
            <span class="oit-labeled-box-label">ARRIVAL</span>
            <div class="fr oit-perf-row">
              <div class="oit-label bigger oit-perf-key">AIRPORT</div>
              <InputField<string>
                dataEntryFormat={new AirportFormat()}
                mandatory={Subject.create(true)}
                value={this.airport}
                containerStyle="width: 160px; margin-right: 20px;"
                alignText="center"
                hEventConsumer={this.props.container.hEventConsumer}
                interactionMode={this.props.container.interactionMode}
                errorHandler={() => {}}
              />
              <div class="oit-label bigger oit-perf-key">RWY</div>
              <DropdownMenu
                idPrefix={`${this.props.uiService.captOrFo}_OIT_ldgPerfRunway`}
                selectedIndex={this.selectedRunway}
                values={this.runwayIdents}
                freeTextAllowed={false}
                containerStyle="width: 180px; margin-right: 20px;"
                alignLabels="center"
                numberOfDigitsForInputField={4}
                tmpyActive={Subject.create(false)}
                hEventConsumer={this.props.container.hEventConsumer}
                interactionMode={this.props.container.interactionMode}
              />
              <Button label="SYNC FMS" onClick={() => this.syncFms()} containerStyle="width: 175px; margin-right: 20px;" />
              <Button label="SYNC SIMBRIEF" onClick={() => this.syncSimbrief()} containerStyle="width: 230px;" />
            </div>
            <div class="oit-label amber oit-perf-note">{this.databaseMessage}</div>
            <div class="oit-perf-grid">
              <div class="oit-label bigger">LDA</div>
              <div class="oit-label bigger oit-green-text">
                {MappedSubject.create(
                  ([r, u]) => (r === null ? '----' : `${formatDistance(r.lda, u)} ${distanceUnitLabel(u)}`),
                  this.runway,
                  units,
                )}
              </div>
              <div class="oit-label bigger">ELEVN</div>
              <div class="oit-label bigger oit-green-text">
                {this.runway.map((r) => (r === null ? '-----' : `${Math.round(r.elevationFt)} FT`))}
              </div>

              <div class="oit-label bigger">SLOPE</div>
              <div class="oit-label bigger oit-green-text">
                {this.runway.map((r) =>
                  r === null ? '----' : `${r.slopePercent >= 0 ? 'UP' : 'DOWN'} ${Math.abs(r.slopePercent).toFixed(2)}%`,
                )}
              </div>
              <div class="oit-label bigger">WIDTH</div>
              <div class="oit-label bigger oit-green-text">
                {MappedSubject.create(
                  ([r, u]) => (r === null ? '----' : `${formatDistance(r.width, u)} ${distanceUnitLabel(u)}`),
                  this.runway,
                  units,
                )}
              </div>
            </div>
          </div>

          <div class="oit-labeled-box-container">
            <span class="oit-labeled-box-label">CONDITIONS</span>
            <div class="fr oit-perf-row">
              <div class="oit-label bigger oit-perf-key">LW</div>
              <InputField<number>
                dataEntryFormat={new WeightFormat()}
                mandatory={Subject.create(true)}
                value={this.landingWeight}
                containerStyle="width: 220px; margin-right: 20px;"
                alignText="center"
                hEventConsumer={this.props.container.hEventConsumer}
                interactionMode={this.props.container.interactionMode}
                errorHandler={() => {}}
              />
              <div class="oit-label">{this.landingWeightSource}</div>
            </div>
            <div class="oit-perf-grid">
              <div class="oit-label bigger">OAT</div>
              <div class="oit-label bigger oit-green-text">{this.oat.map((v) => (v === null ? '---' : `${v}°C`))}</div>
              <div class="oit-label bigger">QNH</div>
              <div class="oit-label bigger oit-green-text">
                {MappedSubject.create(
                  ([q, u]) => (q === null ? '----' : `${formatPressure(q, u)} ${pressureUnitLabel(u)}`),
                  this.qnh,
                  units,
                )}
              </div>

              <div class="oit-label bigger">WIND</div>
              <div class="oit-label bigger oit-green-text">{this.windText}</div>
              <div class="oit-label bigger">PRESS ALT</div>
              <div class="oit-label bigger oit-green-text">{this.pressureAltText}</div>

              <div class="oit-label bigger">HEADWIND</div>
              <div class="oit-label bigger oit-green-text">{this.headwindText}</div>
              <div class="oit-label bigger">CROSSWIND</div>
              <div class="oit-label bigger oit-green-text">{this.crosswindText}</div>
            </div>
            <div class="fr oit-perf-row">
              {this.selector('RWY COND', RUNWAY_CONDITIONS, this.runwayCondition, 'ldgPerfRwyCond', 180)}
              {this.selector('FLAPS', FLAPS, this.flaps, 'ldgPerfFlaps', 200)}
              {this.selector('VPILOT', ['0', '5', '10', '15', '20'], this.vPilot, 'ldgPerfVpilot', 130)}
            </div>
            <div class="fr oit-perf-row">
              {this.selector('TECHNIQUE', TECHNIQUE, this.technique, 'ldgPerfTechnique', 200)}
              {this.selector('BRAKE', BRAKE_MODE, this.brakeMode, 'ldgPerfBrake', 130)}
              {this.selector('REVERSE', REVERSE, this.reverse, 'ldgPerfReverse', 130)}
            </div>
          </div>

          <div class="oit-labeled-box-container">
            <span class="oit-labeled-box-label">RESULTS</span>
            <div class="fr oit-perf-row">
              <Button
                label="COMPUTE"
                onClick={() => this.compute()}
                containerStyle="width: 200px; margin-right: 20px;"
              />
              <div style="flex-grow: 1" />
              <div class="oit-label bigger">RWY</div>
              <div class="oit-label bigger oit-green-text" style="margin-left: 15px;">
                {this.resultRunwayText}
              </div>
            </div>
            <div class="oit-perf-grid">
              <div class="oit-label bigger">LW USED</div>
              <div class="oit-label bigger oit-green-text">
                {MappedSubject.create(
                  ([w, u]) => `${formatWeight(w, u)} ${weightUnitLabel(u)}`,
                  this.landingWeight,
                  units,
                )}
              </div>
              <div class="oit-label bigger">LDA</div>
              <div class="oit-label bigger oit-green-text">
                {MappedSubject.create(
                  ([r, u]) => (r === null ? '----' : `${formatDistance(r.lda, u)} ${distanceUnitLabel(u)}`),
                  this.runway,
                  units,
                )}
              </div>

              <div class="oit-label bigger">LD</div>
              <div class="oit-label bigger amber">{UNAVAILABLE}</div>
              <div class="oit-label bigger">FLD</div>
              <div class="oit-label bigger amber">{UNAVAILABLE}</div>

              <div class="oit-label bigger">STOP MARGIN / LD</div>
              <div class="oit-label bigger amber">{UNAVAILABLE}</div>
              <div class="oit-label bigger">STOP MARGIN / FLD</div>
              <div class="oit-label bigger amber">{UNAVAILABLE}</div>

              <div class="oit-label bigger">VAPP</div>
              <div class="oit-label bigger amber">{UNAVAILABLE}</div>
              <div class="oit-label bigger">MAX CROSSWIND</div>
              <div class="oit-label bigger amber">{UNAVAILABLE}</div>
            </div>
            <div class="oit-label amber oit-perf-note">{UNAVAILABLE_REASON}</div>
          </div>

          <div class="oit-labeled-box-container">
            <span class="oit-labeled-box-label">SIMBRIEF LANDING REPORT</span>
            <div class="oit-perf-grid">
              <div class="oit-label bigger">VREF</div>
              <div class={this.tlrClass()}>{this.tlrVrefText}</div>
              <div class="oit-label bigger">MAX LW DRY</div>
              <div class={this.tlrClass()}>{this.tlrMaxDryText}</div>

              <div class="oit-label bigger" />
              <div class="oit-label bigger" />
              <div class="oit-label bigger">MAX LW WET</div>
              <div class={this.tlrClass()}>{this.tlrMaxWetText}</div>
            </div>
            <div class="oit-label amber oit-perf-note">{this.tlrNote}</div>
          </div>

          <OitFltOpsPerfOptions units={this.units} />
        </div>
      </>
    );
  }

  private selector(label: string, values: string[], selected: Subject<number | null>, id: string, width: number): VNode {
    return (
      <>
        <div class="oit-label bigger oit-perf-key">{label}</div>
        <DropdownMenu
          idPrefix={`${this.props.uiService.captOrFo}_OIT_${id}`}
          selectedIndex={selected}
          values={ArraySubject.create(values)}
          freeTextAllowed={false}
          containerStyle={`width: ${width}px; margin-right: 20px;`}
          alignLabels="center"
          numberOfDigitsForInputField={16}
          tmpyActive={Subject.create(false)}
          hEventConsumer={this.props.container.hEventConsumer}
          interactionMode={this.props.container.interactionMode}
        />
      </>
    );
  }
}

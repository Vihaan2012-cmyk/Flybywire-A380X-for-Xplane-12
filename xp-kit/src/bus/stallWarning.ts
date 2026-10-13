import type { Watchable } from '../sub';
import type { Hub } from './eventBus';
import { SourceBase } from './publisher';

export interface StallSignals {
  stall_warning_on: boolean;
}

export class StallSource extends SourceBase<StallSignals> {
  private aoa = 0;
  private onGround = true;
  private stallAoA?: number;
  private stallWarningOn?: boolean;
  private debounceTimeRemaining = 0;
  private previousTime = Date.now();

  public constructor(
    bus: Hub,
    private readonly aoaThreshold: number,
    private readonly debounceMs = 500,
    aoa?: Watchable<number>,
  ) {
    super(bus);
    if (aoa !== undefined) {
      aoa.sub((v: number) => {
        this.aoa = v;
      }, true);
    } else {
      bus
        .reader<{ aoa: number }>()
        .on('aoa')
        .handle((v) => {
          this.aoa = v;
        });
    }
    bus
      .reader<{ on_ground: boolean }>()
      .on('on_ground')
      .handle((v) => {
        this.onGround = v;
      });
  }

  public onUpdate(): void {
    const now = Date.now();
    const dt = now - this.previousTime;
    this.previousTime = now;

    if (this.stallAoA === undefined) {
      this.stallAoA = SimVar.GetSimVarValue('STALL ALPHA', 'degrees');
    }

    const exceeded = !this.onGround && this.aoa >= (this.stallAoA ?? Infinity) * this.aoaThreshold;
    if (exceeded) {
      this.debounceTimeRemaining = this.debounceMs;
      this.setStallWarningOn(true);
    } else if (this.stallWarningOn) {
      this.debounceTimeRemaining -= dt;
      if (this.debounceTimeRemaining <= 0) {
        this.setStallWarningOn(false);
      }
    } else {
      this.setStallWarningOn(false);
    }
  }

  private setStallWarningOn(isOn: boolean): void {
    if (isOn !== this.stallWarningOn) {
      this.stallWarningOn = isOn;
      this.publish('stall_warning_on', isOn);
    }
  }
}

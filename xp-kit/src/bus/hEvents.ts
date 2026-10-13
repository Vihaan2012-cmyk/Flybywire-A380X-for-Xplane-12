import { SourceBase } from './publisher';

export interface HSignalData {
  readonly name: string;
  readonly args: readonly string[];
}

export interface HSignal {
  hEvent: string;
  hEvent_with_args: HSignalData;
}

export class HSignalSource extends SourceBase<HSignal> {
  public fireHSignal(hEvent: readonly string[], sync?: boolean): void;
  public fireHSignal(hEvent: string, sync?: boolean): void;
  public fireHSignal(hEvent: readonly string[] | string, sync = false): void {
    if (typeof hEvent === 'string') {
      this.publish('hEvent', hEvent, sync, false);
    } else if (hEvent.length > 1) {
      this.publish('hEvent_with_args', { name: hEvent[0], args: hEvent.slice(1) }, sync, false);
    } else {
      this.publish('hEvent', hEvent[0], sync, false);
    }
  }
}

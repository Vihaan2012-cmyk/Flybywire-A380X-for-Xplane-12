import { BasePublisher } from './publisher';

export interface HEventData {
  readonly name: string;
  readonly args: readonly string[];
}

export interface HEvent {
  hEvent: string;
  hEvent_with_args: HEventData;
}

export class HEventPublisher extends BasePublisher<HEvent> {
  public dispatchHEvent(hEvent: readonly string[], sync?: boolean): void;
  public dispatchHEvent(hEvent: string, sync?: boolean): void;
  public dispatchHEvent(hEvent: readonly string[] | string, sync = false): void {
    if (typeof hEvent === 'string') {
      this.publish('hEvent', hEvent, sync, false);
    } else if (hEvent.length > 1) {
      this.publish('hEvent_with_args', { name: hEvent[0], args: hEvent.slice(1) }, sync, false);
    } else {
      this.publish('hEvent', hEvent[0], sync, false);
    }
  }
}

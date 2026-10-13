import type { EventBus, IndexedEventType } from './eventBus';
import type { PublishPacer } from './publisher';
import { SimVarPublisher, SimVarValueType } from './simVarPublisher';

export interface BaseWeightBalanceEvents {
  cg_percent: number;
  total_weight: number;
  payload_station_weight: number;
}

type WeightBalanceIndexedEvents = {
  [P in keyof Pick<BaseWeightBalanceEvents, 'payload_station_weight'> as IndexedEventType<P>]: BaseWeightBalanceEvents[P];
};

export interface WeightBalanceEvents extends BaseWeightBalanceEvents, WeightBalanceIndexedEvents {}

export class WeightBalanceSimvarPublisher extends SimVarPublisher<WeightBalanceEvents> {
  public constructor(bus: EventBus, pacer?: PublishPacer<WeightBalanceEvents>) {
    super(
      new Map<keyof WeightBalanceEvents, any>([
        ['cg_percent', { name: 'CG PERCENT', type: SimVarValueType.Percent }],
        ['total_weight', { name: 'TOTAL WEIGHT', type: SimVarValueType.LBS }],
        [
          'payload_station_weight',
          { name: 'PAYLOAD STATION WEIGHT:#index#', type: SimVarValueType.LBS, indexed: true },
        ],
      ]) as any,
      bus,
      pacer,
    );
  }
}

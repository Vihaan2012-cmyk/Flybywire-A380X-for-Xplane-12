import type { Hub, IndexedSignal } from './eventBus';
import type { Rate } from './publisher';
import { VarSource, VarKind } from './simVarPublisher';

export interface BaseMassSignals {
  cg_percent: number;
  total_weight: number;
  payload_station_weight: number;
}

type WeightBalanceIndexedEvents = {
  [P in keyof Pick<BaseMassSignals, 'payload_station_weight'> as IndexedSignal<P>]: BaseMassSignals[P];
};

export interface MassSignals extends BaseMassSignals, WeightBalanceIndexedEvents {}

export class MassSimvarSource extends VarSource<MassSignals> {
  public constructor(bus: Hub, pacer?: Rate<MassSignals>) {
    super(
      new Map<keyof MassSignals, any>([
        ['cg_percent', { name: 'CG PERCENT', type: VarKind.Percent }],
        ['total_weight', { name: 'TOTAL WEIGHT', type: VarKind.LBS }],
        [
          'payload_station_weight',
          { name: 'PAYLOAD STATION WEIGHT:#index#', type: VarKind.LBS, indexed: true },
        ],
      ]) as any,
      bus,
      pacer,
    );
  }
}

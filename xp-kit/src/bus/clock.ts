import type { Hub, HubMeta } from './eventBus';
import { SourceBase, Rate } from './publisher';

export interface TickerSignals {
  realTime: number;
  simTime: number;
  activeSimDuration: number;
  realTimeHiFreq: number;
  simTimeHiFreq: number;
  activeSimDurationHiFreq: number;
  simRate: number;
  zulu_sunrise: number;
  zulu_sunset: number;
}

const UNIX_EPOCH_ABSOLUTE_SECONDS = 62135596800;

export class TickerSource extends SourceBase<TickerSignals> {
  private readonly topics = new Set<string>();
  private hiFreqInterval?: ReturnType<typeof setInterval>;

  public constructor(bus: Hub, pacer?: Rate<TickerSignals>) {
    super(bus, pacer);
    bus
      .reader<HubMeta>()
      .on('event_bus_topic_first_sub')
      .handle((topic) => this.onTopicFirstSub(topic));
    bus.forEachSubscribedTopic((topic) => this.onTopicFirstSub(topic));
  }

  private onTopicFirstSub(topic: string): void {
    switch (topic) {
      case 'realTime':
      case 'simTime':
      case 'activeSimDuration':
      case 'simRate':
      case 'zulu_sunrise':
      case 'zulu_sunset':
        if (!this.topics.has(topic)) {
          this.topics.add(topic);
          if (this.publishActive) {
            this.publishLowFreq(topic);
          }
        }
        break;
      case 'realTimeHiFreq':
      case 'simTimeHiFreq':
      case 'activeSimDurationHiFreq':
        this.topics.add(topic);
        break;
    }
  }

  private startHiFreq(): void {
    if (this.hiFreqInterval === undefined) {
      this.hiFreqInterval = setInterval(() => this.publishHiFreq(), 0);
    }
  }

  public startFeed(): void {
    super.startFeed();
    this.startHiFreq();
  }

  public stopPublish(): void {
    super.stopPublish();
    if (this.hiFreqInterval !== undefined) {
      clearInterval(this.hiFreqInterval);
      this.hiFreqInterval = undefined;
    }
  }

  public onUpdate(): void {
    if (!this.publishActive) {
      return;
    }
    for (const topic of this.topics) {
      this.publishLowFreq(topic);
    }
  }

  private publishLowFreq(topic: string): void {
    switch (topic) {
      case 'realTime':
        this.publish('realTime', Date.now());
        break;
      case 'simTime':
        this.publish('simTime', TickerSource.readSimTime());
        break;
      case 'activeSimDuration':
        this.publish('activeSimDuration', TickerSource.readActiveSimDuration());
        break;
      case 'simRate':
        this.publish('simRate', SimVar.GetGlobalVarValue('SIMULATION RATE', 'number'));
        break;
      case 'zulu_sunrise':
        this.publish('zulu_sunrise', SimVar.GetSimVarValue('E:ZULU SUNRISE', 'seconds'));
        break;
      case 'zulu_sunset':
        this.publish('zulu_sunset', SimVar.GetSimVarValue('E:ZULU SUNSET', 'seconds'));
        break;
    }
  }

  private publishHiFreq(): void {
    if (this.topics.has('realTimeHiFreq')) {
      this.publish('realTimeHiFreq', Date.now());
    }
    if (this.topics.has('simTimeHiFreq')) {
      this.publish('simTimeHiFreq', TickerSource.readSimTime());
    }
    if (this.topics.has('activeSimDurationHiFreq')) {
      this.publish('activeSimDurationHiFreq', TickerSource.readActiveSimDuration());
    }
  }

  private static readSimTime(): number {
    return (SimVar.GetSimVarValue('E:ABSOLUTE TIME', 'seconds') - UNIX_EPOCH_ABSOLUTE_SECONDS) * 1000;
  }

  private static readActiveSimDuration(): number {
    return SimVar.GetSimVarValue('E:SIMULATION TIME', 'seconds') * 1000;
  }
}

export class Ticker {
  private readonly publisher: TickerSource;

  public constructor(bus: Hub) {
    this.publisher = new TickerSource(bus);
  }

  public init(): void {
    this.publisher.startFeed();
  }

  public onUpdate(): void {
    this.publisher.onUpdate();
  }
}

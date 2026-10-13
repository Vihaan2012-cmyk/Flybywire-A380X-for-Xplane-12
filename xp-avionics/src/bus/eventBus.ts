import type { Subscription } from '../sub';
import { BasicConsumer, Consumer, Handler } from './consumer';
import { HandlerSubscription } from './handlerSubscription';

export type WildcardHandler = (topic: string, data: any) => void;

export interface EventBusMetaEvents {
  event_bus: string;
  event_bus_topic_first_sub: string;
}

export type IndexedEventType<T extends string> = `${T}_${number}`;

export type IndexedEvents<Events extends { [event: string]: any }, Index extends number> = {
  [Event in keyof Events as `${Event & string}_${Index}`]: Events[Event];
};

export interface Publisher<E> {
  pub<K extends keyof E>(topic: K, data: E[K], sync?: boolean, isCached?: boolean): void;
}

export interface EventBusSyncHandler {
  sendSyncedEvent(topic: string, data: unknown, isCached: boolean): void;
}

export type EventBusSyncHandlerFactory = (
  busId: number,
  onSyncedEventReceived: (topic: string, data: unknown, isCached: boolean) => void,
) => EventBusSyncHandler;

interface CachedEvent {
  data: any;
  synced: boolean;
}

const SYNC_KEY = 'wt.eb.evt';

class GenericDataSyncHandler implements EventBusSyncHandler {
  private readonly listener: BusViewListener;
  private ready = false;
  private queue: { topic: string; data: unknown; isCached: boolean }[] = [];

  public constructor(
    private readonly busId: number,
    onReceived: (topic: string, data: unknown, isCached: boolean) => void,
  ) {
    this.listener = RegisterGenericDataListener(() => {
      this.listener.onDataReceived?.(SYNC_KEY, (packet: any) => {
        if (packet && packet.busId !== this.busId && Array.isArray(packet.events)) {
          for (const event of packet.events) {
            onReceived(event.topic, event.data, event.isCached);
          }
        }
      });
      this.ready = true;
    });
    requestAnimationFrame(this.flush);
  }

  public sendSyncedEvent(topic: string, data: unknown, isCached: boolean): void {
    this.queue.push({ topic, data, isCached });
  }

  private readonly flush = (): void => {
    if (this.ready && this.queue.length > 0) {
      const events = this.queue;
      this.queue = [];
      this.listener.send?.(SYNC_KEY, { busId: this.busId, events });
    }
    requestAnimationFrame(this.flush);
  };
}

export class EventBus {
  private readonly topicSubs = new Map<string, HandlerSubscription<Handler<any>>[]>();
  private wildcardSubs: HandlerSubscription<WildcardHandler>[] = [];
  private readonly eventCache = new Map<string, CachedEvent>();
  private readonly busId = Math.floor(Math.random() * 2_147_483_647);
  private readonly busSync: EventBusSyncHandler;

  public constructor(syncHandlerFactory?: EventBusSyncHandlerFactory | boolean, shouldResync = true) {
    const factory: EventBusSyncHandlerFactory =
      typeof syncHandlerFactory === 'function'
        ? syncHandlerFactory
        : (id, received) => new GenericDataSyncHandler(id, received);
    this.busSync = factory(this.busId, (topic, data, isCached) => this.onSyncedEventReceived(topic, data, isCached));

    this.on('event_bus', (data: string) => {
      if (data === 'resync_request') {
        this.resyncEvents();
      }
    });
    if (shouldResync) {
      this.pub('event_bus', 'resync_request', true, false);
    }
  }

  public on(topic: string, handler: Handler<any>, paused = false): Subscription {
    let subs = this.topicSubs.get(topic);
    const isNewTopic = subs === undefined;
    if (subs === undefined) {
      subs = [];
      this.topicSubs.set(topic, subs);
    }

    const sub = new HandlerSubscription<Handler<any>>(
      handler,
      (s) => {
        const cached = this.eventCache.get(topic);
        if (cached !== undefined) {
          s.handler(cached.data);
        }
      },
      (s) => this.removeTopicSub(topic, s),
    );
    subs.push(sub);

    if (paused) {
      sub.pause();
    }

    if (isNewTopic) {
      this.pub('event_bus_topic_first_sub', topic, false, false);
    }

    if (!paused) {
      sub.initialNotify();
    }
    return sub as unknown as Subscription;
  }

  public onAll(handler: WildcardHandler): Subscription {
    const sub = new HandlerSubscription<WildcardHandler>(handler, undefined, (s) => {
      this.wildcardSubs = this.wildcardSubs.filter((x) => x !== s);
    });
    this.wildcardSubs = [...this.wildcardSubs, sub];
    return sub as unknown as Subscription;
  }

  public pub(topic: string, data: any, sync = false, isCached = true): void {
    if (isCached) {
      this.eventCache.set(topic, { data, synced: sync });
    }

    const subs = this.topicSubs.get(topic);
    if (subs !== undefined) {
      const count = subs.length;
      for (let i = 0; i < count && i < subs.length; i++) {
        const sub = subs[i];
        if (sub.isAlive && !sub.isPaused) {
          try {
            sub.handler(data);
          } catch (e) {
            console.error(`EventBus: error in handler: ${e}. topic: ${topic}. data: ${data}. sub: ${sub}`);
            if (e instanceof Error) {
              console.error(e.stack);
            }
          }
        }
      }
    }

    const wildcards = this.wildcardSubs;
    for (let i = 0; i < wildcards.length; i++) {
      const sub = wildcards[i];
      if (sub.isAlive && !sub.isPaused) {
        try {
          sub.handler(topic, data);
        } catch (e) {
          console.error(`EventBus: error in wildcard handler: ${e}. topic: ${topic}. data: ${data}`);
          if (e instanceof Error) {
            console.error(e.stack);
          }
        }
      }
    }

    if (sync) {
      this.busSync.sendSyncedEvent(topic, data, isCached);
    }
  }

  public getPublisher<E>(): Publisher<E> {
    return this as unknown as Publisher<E>;
  }

  public getSubscriber<E>(): EventSubscriber<E> {
    return new EventSubscriber<E>(this);
  }

  public getTopicSubscriberCount(topic: string): number {
    return this.topicSubs.get(topic)?.length ?? 0;
  }

  public forEachSubscribedTopic(fn: (topic: string, subscriberCount: number) => void): void {
    for (const [topic, subs] of [...this.topicSubs]) {
      if (subs.length > 0) {
        fn(topic, subs.length);
      }
    }
  }

  private removeTopicSub(topic: string, sub: HandlerSubscription<Handler<any>>): void {
    const subs = this.topicSubs.get(topic);
    if (subs === undefined) {
      return;
    }
    const index = subs.indexOf(sub);
    if (index >= 0) {
      subs.splice(index, 1);
    }
  }

  private resyncEvents(): void {
    for (const [topic, cached] of this.eventCache) {
      if (cached.synced) {
        this.busSync.sendSyncedEvent(topic, cached.data, true);
      }
    }
  }

  private onSyncedEventReceived(topic: string, data: unknown, isCached: boolean): void {
    this.pub(topic, data, false, isCached);
  }
}

export class EventSubscriber<E> {
  public constructor(private bus: EventBus) {}

  public on<K extends keyof E & string>(topic: K): Consumer<E[K]> {
    return new BasicConsumer<E[K]>((handler, paused) => this.bus.on(topic, handler, paused));
  }
}

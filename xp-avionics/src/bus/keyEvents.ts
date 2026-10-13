import type { EventBus } from './eventBus';

export type KeyEventData = {
  key: string;
  value0?: number;
  value1?: number;
  value2?: number;
};

export type KeyEvents = {
  key_intercept: KeyEventData;
};

export class KeyEventManager {
  private static instance?: KeyEventManager;
  private static isCreatingInstance = false;
  private static readonly pendingResolves: ((manager: KeyEventManager) => void)[] = [];

  private constructor(
    private readonly keyListener: BusViewListener,
    private readonly bus: EventBus,
  ) {
    keyListener.on('keyIntercepted', this.onKeyIntercepted.bind(this));
  }

  private onKeyIntercepted(key: string, value1?: number, value0?: number, value2?: number): void {
    this.bus.getPublisher<KeyEvents>().pub('key_intercept', { key, value0, value1, value2 }, false, false);
  }

  public triggerKey(key: string, bypass: boolean, value0 = 0, value1 = 0, value2 = 0): Promise<void> {
    return Coherent.call('TRIGGER_KEY_EVENT', key, bypass, value0, value1, value2);
  }

  public interceptKey(key: string, passThrough: boolean): void {
    Coherent.call('INTERCEPT_KEY_EVENT', key, passThrough ? 0 : 1);
  }

  public static getManager(bus: EventBus): Promise<KeyEventManager> {
    if (KeyEventManager.instance !== undefined) {
      return Promise.resolve(KeyEventManager.instance);
    }
    return new Promise<KeyEventManager>((resolve) => {
      KeyEventManager.pendingResolves.push(resolve);
      if (!KeyEventManager.isCreatingInstance) {
        KeyEventManager.isCreatingInstance = true;
        KeyEventManager.createInstance(bus);
      }
    });
  }

  private static async createInstance(bus: EventBus): Promise<void> {
    const instance = await KeyEventManager.create(bus);
    KeyEventManager.instance = instance;
    KeyEventManager.isCreatingInstance = false;
    for (const resolve of KeyEventManager.pendingResolves.splice(0)) {
      resolve(instance);
    }
  }

  private static create(bus: EventBus): Promise<KeyEventManager> {
    return new Promise<KeyEventManager>((resolve) => {
      const listener: BusViewListener = RegisterViewListener(
        'JS_LISTENER_KEYEVENT',
        () => resolve(new KeyEventManager(listener, bus)),
        true,
      );
    });
  }
}

import type { Hub } from './eventBus';

export type KeySignalData = {
  key: string;
  value0?: number;
  value1?: number;
  value2?: number;
};

export type KeySignals = {
  key_intercept: KeySignalData;
};

export class KeyPort {
  private static instance?: KeyPort;
  private static isCreatingInstance = false;
  private static readonly pendingResolves: ((manager: KeyPort) => void)[] = [];

  private constructor(
    private readonly keyListener: BusViewListener,
    private readonly bus: Hub,
  ) {
    keyListener.on('keyIntercepted', this.onKeyIntercepted.bind(this));
  }

  private onKeyIntercepted(key: string, value1?: number, value0?: number, value2?: number): void {
    this.bus.writer<KeySignals>().send('key_intercept', { key, value0, value1, value2 }, false, false);
  }

  public fireKey(key: string, bypass: boolean, value0 = 0, value1 = 0, value2 = 0): Promise<void> {
    return Coherent.call('TRIGGER_KEY_EVENT', key, bypass, value0, value1, value2);
  }

  public grabKey(key: string, passThrough: boolean): void {
    Coherent.call('INTERCEPT_KEY_EVENT', key, passThrough ? 0 : 1);
  }

  public static port(bus: Hub): Promise<KeyPort> {
    if (KeyPort.instance !== undefined) {
      return Promise.resolve(KeyPort.instance);
    }
    return new Promise<KeyPort>((resolve) => {
      KeyPort.pendingResolves.push(resolve);
      if (!KeyPort.isCreatingInstance) {
        KeyPort.isCreatingInstance = true;
        KeyPort.createInstance(bus);
      }
    });
  }

  private static async createInstance(bus: Hub): Promise<void> {
    const instance = await KeyPort.create(bus);
    KeyPort.instance = instance;
    KeyPort.isCreatingInstance = false;
    for (const resolve of KeyPort.pendingResolves.splice(0)) {
      resolve(instance);
    }
  }

  private static create(bus: Hub): Promise<KeyPort> {
    return new Promise<KeyPort>((resolve) => {
      const listener: BusViewListener = RegisterViewListener(
        'JS_LISTENER_KEYEVENT',
        () => resolve(new KeyPort(listener, bus)),
        true,
      );
    });
  }
}

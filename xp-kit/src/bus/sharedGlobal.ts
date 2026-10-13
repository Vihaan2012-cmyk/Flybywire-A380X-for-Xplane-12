import { Cell } from '../sub';
import type { Watchable } from '../sub';

export interface GlobalShareRef<T extends object> {
  readonly instance: T;
  readonly isViewOwner: boolean;
  readonly isDetached: Watchable<boolean>;
}

interface GlobalEntry {
  attached: boolean;
  isDetached: Cell<boolean>;
  resolve: (ref: GlobalShareRef<any>) => void;
  promise: Promise<GlobalShareRef<any>>;
}

export class GlobalShare {
  private static listenerPromise?: Promise<BusViewListener>;
  private static readonly entries = new Map<string, GlobalEntry>();

  private static initListener(): Promise<BusViewListener> {
    if (GlobalShare.listenerPromise === undefined) {
      GlobalShare.listenerPromise = new Promise<BusViewListener>((resolve) => {
        const listener: BusViewListener = RegisterViewListener(
          'JS_LISTENER_SHAREDGLOBAL',
          () => {
            listener.on('SharedGlobalAttached', GlobalShare.onGlobalAttached);
            listener.on('SharedGlobalDetached', GlobalShare.onGlobalDetached);
            resolve(listener);
          },
          true,
        );
      });
    }
    return GlobalShare.listenerPromise;
  }

  public static async get<T extends object = object>(name: string): Promise<GlobalShareRef<T>> {
    const listener = await GlobalShare.initListener();
    const [entry, isNew] = GlobalShare.pendRequest(name);
    if (isNew) {
      await listener.call('REQUEST_SHARED_GLOBAL', name);
      await new Promise<void>((resolve) => setTimeout(resolve, 0));
      if (!entry.attached) {
        await listener.call('CREATE_SHARED_GLOBAL', name);
      }
    }
    return entry.promise;
  }

  public static async await<T extends object = object>(name: string): Promise<GlobalShareRef<T>> {
    const listener = await GlobalShare.initListener();
    const [entry, isNew] = GlobalShare.pendRequest(name);
    if (isNew) {
      await listener.call('REQUEST_SHARED_GLOBAL', name);
    }
    return entry.promise;
  }

  private static pendRequest(name: string): [GlobalEntry, boolean] {
    const existing = GlobalShare.entries.get(name);
    if (existing !== undefined) {
      return [existing, false];
    }
    let resolve!: (ref: GlobalShareRef<any>) => void;
    const promise = new Promise<GlobalShareRef<any>>((r) => {
      resolve = r;
    });
    const entry: GlobalEntry = { attached: false, isDetached: Cell.create(false), resolve, promise };
    GlobalShare.entries.set(name, entry);
    return [entry, true];
  }

  private static onGlobalAttached(name: string, isViewOwner: boolean): void {
    const entry = GlobalShare.entries.get(name);
    if (entry === undefined || entry.attached) {
      return;
    }
    entry.attached = true;
    entry.resolve({
      instance: (globalThis as any)[name],
      isViewOwner: !!isViewOwner,
      isDetached: entry.isDetached,
    });
  }

  private static onGlobalDetached(name: string): void {
    const entry = GlobalShare.entries.get(name);
    if (entry === undefined) {
      return;
    }
    entry.isDetached.set(true);
    GlobalShare.entries.delete(name);
  }
}

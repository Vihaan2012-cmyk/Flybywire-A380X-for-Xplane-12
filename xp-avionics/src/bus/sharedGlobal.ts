import { Subject } from '../sub';
import type { Subscribable } from '../sub';

export interface SharedGlobalObjectRef<T extends object> {
  readonly instance: T;
  readonly isViewOwner: boolean;
  readonly isDetached: Subscribable<boolean>;
}

interface GlobalEntry {
  attached: boolean;
  isDetached: Subject<boolean>;
  resolve: (ref: SharedGlobalObjectRef<any>) => void;
  promise: Promise<SharedGlobalObjectRef<any>>;
}

export class SharedGlobal {
  private static listenerPromise?: Promise<BusViewListener>;
  private static readonly entries = new Map<string, GlobalEntry>();

  private static initListener(): Promise<BusViewListener> {
    if (SharedGlobal.listenerPromise === undefined) {
      SharedGlobal.listenerPromise = new Promise<BusViewListener>((resolve) => {
        const listener: BusViewListener = RegisterViewListener(
          'JS_LISTENER_SHAREDGLOBAL',
          () => {
            listener.on('SharedGlobalAttached', SharedGlobal.onGlobalAttached);
            listener.on('SharedGlobalDetached', SharedGlobal.onGlobalDetached);
            resolve(listener);
          },
          true,
        );
      });
    }
    return SharedGlobal.listenerPromise;
  }

  public static async get<T extends object = object>(name: string): Promise<SharedGlobalObjectRef<T>> {
    const listener = await SharedGlobal.initListener();
    const [entry, isNew] = SharedGlobal.pendRequest(name);
    if (isNew) {
      await listener.call('REQUEST_SHARED_GLOBAL', name);
      await new Promise<void>((resolve) => setTimeout(resolve, 0));
      if (!entry.attached) {
        await listener.call('CREATE_SHARED_GLOBAL', name);
      }
    }
    return entry.promise;
  }

  public static async await<T extends object = object>(name: string): Promise<SharedGlobalObjectRef<T>> {
    const listener = await SharedGlobal.initListener();
    const [entry, isNew] = SharedGlobal.pendRequest(name);
    if (isNew) {
      await listener.call('REQUEST_SHARED_GLOBAL', name);
    }
    return entry.promise;
  }

  private static pendRequest(name: string): [GlobalEntry, boolean] {
    const existing = SharedGlobal.entries.get(name);
    if (existing !== undefined) {
      return [existing, false];
    }
    let resolve!: (ref: SharedGlobalObjectRef<any>) => void;
    const promise = new Promise<SharedGlobalObjectRef<any>>((r) => {
      resolve = r;
    });
    const entry: GlobalEntry = { attached: false, isDetached: Subject.create(false), resolve, promise };
    SharedGlobal.entries.set(name, entry);
    return [entry, true];
  }

  private static onGlobalAttached(name: string, isViewOwner: boolean): void {
    const entry = SharedGlobal.entries.get(name);
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
    const entry = SharedGlobal.entries.get(name);
    if (entry === undefined) {
      return;
    }
    entry.isDetached.set(true);
    SharedGlobal.entries.delete(name);
  }
}

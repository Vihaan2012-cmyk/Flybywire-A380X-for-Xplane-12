import type { Subscribable, Subscription } from '../sub';
import type { Consumer } from './consumer';

export class Wait {
  public static awaitDelay(delay: number): Promise<void> {
    return new Promise<void>((resolve) => {
      setTimeout(resolve, delay);
    });
  }

  public static awaitFrames(count: number, _glassCockpitRefresh = false): Promise<void> {
    return new Promise<void>((resolve) => {
      let remaining = count;
      const step = (): void => {
        if (--remaining <= 0) {
          resolve();
        } else {
          requestAnimationFrame(step);
        }
      };
      requestAnimationFrame(step);
    });
  }

  public static awaitCondition(predicate: () => boolean, interval = 0, timeout = 0): Promise<void> {
    return new Promise<void>((resolve, reject) => {
      const startTime = Date.now();
      const check = (): boolean => {
        if (predicate()) {
          resolve();
          return true;
        }
        if (timeout > 0 && Date.now() - startTime >= timeout) {
          reject('Wait.awaitCondition() timed out');
          return true;
        }
        return false;
      };

      if (check()) {
        return;
      }
      if (interval > 0) {
        const id = setInterval(() => {
          if (check()) {
            clearInterval(id);
          }
        }, interval);
      } else {
        const step = (): void => {
          if (!check()) {
            requestAnimationFrame(step);
          }
        };
        requestAnimationFrame(step);
      }
    });
  }

  public static awaitSubscribable<T>(
    subscribable: Subscribable<T>,
    predicate?: (value: T) => boolean,
    initialCheck = false,
    timeout = 0,
  ): Promise<T> {
    return new Promise<T>((resolve, reject) => {
      let subscription: Subscription | undefined;
      let timeoutId: ReturnType<typeof setTimeout> | undefined;
      let done = false;

      const finish = (): void => {
        done = true;
        if (timeoutId !== undefined) {
          clearTimeout(timeoutId);
        }
        subscription?.destroy();
      };

      subscription = subscribable.sub(
        (value: T) => {
          if (!done && (predicate === undefined || predicate(value))) {
            finish();
            resolve(value);
          }
        },
        initialCheck,
      );
      if (done) {
        subscription.destroy();
      } else if (timeout > 0) {
        timeoutId = setTimeout(() => {
          finish();
          reject('Wait.awaitSubscribable() timed out');
        }, timeout);
      }
    });
  }

  public static awaitConsumer<T>(
    consumer: Consumer<T>,
    predicate?: (value: T) => boolean,
    initialCheck = false,
    timeout = 0,
  ): Promise<T> {
    return new Promise<T>((resolve, reject) => {
      let timeoutId: ReturnType<typeof setTimeout> | undefined;
      let done = false;
      let subscription: Subscription | undefined;

      const finish = (): void => {
        done = true;
        if (timeoutId !== undefined) {
          clearTimeout(timeoutId);
        }
        subscription?.destroy();
      };

      subscription = consumer.handle((value: T) => {
        if (!done && (predicate === undefined || predicate(value))) {
          finish();
          resolve(value);
        }
      }, !initialCheck);
      if (done) {
        subscription.destroy();
        return;
      }
      if (!initialCheck) {
        subscription.resume(false);
      }
      if (!done && timeout > 0) {
        timeoutId = setTimeout(() => {
          finish();
          reject('Wait.awaitConsumer() timed out');
        }, timeout);
      }
    });
  }

  public static awaitSubEvent<SenderType, DataType>(
    event: { on(handler: (sender: SenderType, data: DataType) => void): Subscription },
    predicate?: (data: DataType, sender: SenderType) => boolean,
    timeout = 0,
  ): Promise<DataType> {
    return new Promise<DataType>((resolve, reject) => {
      let timeoutId: ReturnType<typeof setTimeout> | undefined;
      let subscription: Subscription | undefined;

      subscription = event.on((sender: SenderType, data: DataType) => {
        if (predicate === undefined || predicate(data, sender)) {
          if (timeoutId !== undefined) {
            clearTimeout(timeoutId);
          }
          subscription?.destroy();
          resolve(data);
        }
      });
      if (timeout > 0) {
        timeoutId = setTimeout(() => {
          subscription?.destroy();
          reject('Wait.awaitSubEvent() timed out');
        }, timeout);
      }
    });
  }
}

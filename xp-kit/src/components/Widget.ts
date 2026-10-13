import type { Life, Watch } from '../sub';
import type { Ref } from './Ref';

export type WidgetChildren = KitNode | string | number | { isWatchable: true; get(): unknown } | WidgetChildren[] | null | undefined;

export interface KitNode {
  instance: unknown;
  root?: Node;
  props: any;
  children: KitNode[] | null;
}

export class WidgetProps {
  public children?: WidgetChildren[];
  public ref?: Ref<any>;
}

export abstract class Widget<P = any, Contexts extends unknown[] = []> {
  public props: P & WidgetProps;
  public context?: unknown[];
  public readonly contextType?: readonly unknown[];

  public constructor(props: P) {
    this.props = props as P & WidgetProps;
  }

  public onBeforeRender(): void {}

  public mounted(node: KitNode): void {}

  public abstract render(): KitNode | null;

  public destroy(): void {}

  protected getContext(context: unknown): unknown {
    const index = this.contextType?.indexOf(context) ?? -1;
    return index >= 0 && this.context ? this.context[index] : undefined;
  }
}

class ComponentLifecycle implements Life {
  private readonly subs: Watch[] = [];

  public register(sub: Watch): void {
    if (!this.subs.includes(sub)) {
      this.subs.push(sub);
    }
  }

  public unregister(sub: Watch): void {
    const index = this.subs.indexOf(sub);
    if (index >= 0) {
      this.subs.splice(index, 1);
    }
  }

  public pause(): void {
    for (const sub of this.subs) {
      if (sub.isAlive) {
        sub.pause();
      }
    }
  }

  public resume(): void {
    for (const sub of this.subs) {
      if (sub.isAlive) {
        sub.resume(true);
      }
    }
  }

  public destroy(): void {
    for (const sub of this.subs.splice(0)) {
      sub.destroy();
    }
  }
}

export abstract class LifeWidget<P = any, Contexts extends unknown[] = []> extends Widget<P, Contexts> {
  protected readonly defaultLifecycle: Life = this.createDefaultLifecycle();
  protected thisNode?: KitNode;

  protected createDefaultLifecycle(): Life {
    return new ComponentLifecycle();
  }

  public pause(): void {
    this.defaultLifecycle.pause();
    this.forChildNodes((child) => child.pause());
  }

  public resume(): void {
    this.defaultLifecycle.resume();
    this.forChildNodes((child) => child.resume());
  }

  public destroy(): void {
    this.defaultLifecycle.destroy();
    this.forChildNodes((child) => child.destroy());
    super.destroy();
  }

  public mounted(node: KitNode): void {
    this.thisNode = node;
  }

  protected register<T extends Watch>(sub: T): T {
    this.defaultLifecycle.register(sub);
    return sub;
  }

  protected forChildNodes(fn: (child: LifeWidget<any, any>) => void): void {
    const visit = (node: KitNode, isRoot: boolean): void => {
      if (!isRoot && node.instance instanceof LifeWidget) {
        fn(node.instance);
        return;
      }
      for (const child of node.children ?? []) {
        visit(child, false);
      }
    };
    if (this.thisNode) {
      visit(this.thisNode, true);
    }
  }
}

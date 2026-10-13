import type { Lifecycle, Subscription } from '../sub';
import type { NodeReference } from './NodeReference';

export type DisplayChildren = VNode | string | number | { isSubscribable: true; get(): unknown } | DisplayChildren[] | null | undefined;

export interface VNode {
  instance: unknown;
  root?: Node;
  props: any;
  children: VNode[] | null;
}

export class ComponentProps {
  public children?: DisplayChildren[];
  public ref?: NodeReference<any>;
}

export abstract class DisplayComponent<P = any, Contexts extends unknown[] = []> {
  public props: P & ComponentProps;
  public context?: unknown[];
  public readonly contextType?: readonly unknown[];

  public constructor(props: P) {
    this.props = props as P & ComponentProps;
  }

  public onBeforeRender(): void {}

  public onAfterRender(node: VNode): void {}

  public abstract render(): VNode | null;

  public destroy(): void {}

  protected getContext(context: unknown): unknown {
    const index = this.contextType?.indexOf(context) ?? -1;
    return index >= 0 && this.context ? this.context[index] : undefined;
  }
}

class ComponentLifecycle implements Lifecycle {
  private readonly subs: Subscription[] = [];

  public register(sub: Subscription): void {
    if (!this.subs.includes(sub)) {
      this.subs.push(sub);
    }
  }

  public unregister(sub: Subscription): void {
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

export abstract class LifecycleComponent<P = any, Contexts extends unknown[] = []> extends DisplayComponent<P, Contexts> {
  protected readonly defaultLifecycle: Lifecycle = this.createDefaultLifecycle();
  protected thisNode?: VNode;

  protected createDefaultLifecycle(): Lifecycle {
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

  public onAfterRender(node: VNode): void {
    this.thisNode = node;
  }

  protected register<T extends Subscription>(sub: T): T {
    this.defaultLifecycle.register(sub);
    return sub;
  }

  protected forChildNodes(fn: (child: LifecycleComponent<any, any>) => void): void {
    const visit = (node: VNode, isRoot: boolean): void => {
      if (!isRoot && node.instance instanceof LifecycleComponent) {
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

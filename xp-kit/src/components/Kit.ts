import type { MutableWatchableMap, MutableWatchableSet, Watchable, WatchableMap, WatchableSet, Watch } from '../sub';
import { WidgetProps, WidgetChildren, Widget, KitNode } from './Widget';
import { Ref } from './Ref';

export type ToggleableClassNameRecordX = Record<string, boolean | Watchable<boolean>>;
export type StyleRecordX = Record<string, string | number | undefined | null | Watchable<string | number | undefined | null>>;

const SVG_NAMESPACE = 'http://www.w3.org/2000/svg';
const XLINK_NAMESPACE = 'http://www.w3.org/1999/xlink';
const SVG_TAGS = new Set([
  'svg', 'g', 'defs', 'symbol', 'use', 'path', 'line', 'polyline', 'polygon', 'rect', 'circle', 'ellipse', 'text', 'tspan',
  'textPath', 'clipPath', 'mask', 'pattern', 'marker', 'image', 'linearGradient', 'radialGradient', 'stop', 'filter',
  'feBlend', 'feColorMatrix', 'feComponentTransfer', 'feComposite', 'feConvolveMatrix', 'feDiffuseLighting',
  'feDisplacementMap', 'feDistantLight', 'feDropShadow', 'feFlood', 'feFuncA', 'feFuncB', 'feFuncG', 'feFuncR',
  'feGaussianBlur', 'feImage', 'feMerge', 'feMergeNode', 'feMorphology', 'feOffset', 'fePointLight',
  'feSpecularLighting', 'feSpotLight', 'feTile', 'feTurbulence', 'foreignObject', 'switch', 'desc', 'metadata', 'view',
  'animate', 'animateMotion', 'animateTransform', 'mpath', 'set',
]);

function isWatchable(value: unknown): value is Watchable<any> {
  return typeof value === 'object' && value !== null && (value as { isWatchable?: unknown }).isWatchable === true;
}

function isSubscribableSet(value: unknown): value is WatchableSet<string> {
  return typeof value === 'object' && value !== null && (value as { isSubscribableSet?: unknown }).isSubscribableSet === true;
}

function isSubscribableMap(value: unknown): value is WatchableMap<string, any> {
  return typeof value === 'object' && value !== null && (value as { isSubscribableMap?: unknown }).isSubscribableMap === true;
}

function isVNode(value: unknown): value is KitNode {
  return typeof value === 'object' && value !== null && 'instance' in value && 'children' in value && 'props' in value;
}

function isComponentClass(type: unknown): type is new (props: any) => Widget<any> {
  return typeof type === 'function' && type.prototype instanceof Widget;
}

function setStyle(element: HTMLElement | SVGElement, name: string, value: unknown): void {
  if (value === undefined || value === null) {
    element.style.removeProperty(name);
  } else {
    element.style.setProperty(name, String(value));
  }
}

function removeStyle(element: HTMLElement | SVGElement, name: string): void {
  element.style.removeProperty(name);
}

function setAttr(element: HTMLElement | SVGElement, name: string, value: unknown): void {
  if (name.startsWith('xlink:')) {
    element.setAttributeNS(XLINK_NAMESPACE, name, value as string);
  } else {
    element.setAttribute(name, value as string);
  }
}

function applyClass(element: HTMLElement | SVGElement, value: unknown): void {
  if (isSubscribableSet(value)) {
    for (const cssClass of value.get()) {
      element.classList.add(cssClass);
    }
    value.sub((_set, type, key) => {
      if ((type as unknown as string) === 'Added') {
        element.classList.add(key);
      } else {
        element.classList.remove(key);
      }
    });
  } else if (isWatchable(value)) {
    element.setAttribute('class', value.get());
    value.sub((v) => element.setAttribute('class', v));
  } else if (typeof value === 'object' && value !== null) {
    for (const [cssClass, enabled] of Object.entries(value as ToggleableClassNameRecordX)) {
      if (isWatchable(enabled)) {
        element.classList.toggle(cssClass, !!enabled.get());
        enabled.sub((v) => element.classList.toggle(cssClass, !!v));
      } else {
        element.classList.toggle(cssClass, !!enabled);
      }
    }
  } else {
    element.setAttribute('class', value as string);
  }
}

function applyStyle(element: HTMLElement | SVGElement, value: unknown): void {
  if (isSubscribableMap(value)) {
    for (const [name, v] of value.get()) {
      setStyle(element, name, v);
    }
    value.sub((_map, type, key, v) => {
      if ((type as unknown as string) === 'Deleted') {
        removeStyle(element, key);
      } else {
        setStyle(element, key, v);
      }
    });
  } else if (isWatchable(value)) {
    const current = value.get();
    if (typeof current === 'object' && current !== null) {
      for (const [name, v] of Object.entries(current)) {
        setStyle(element, name, v);
      }
      (value as unknown as { sub(handler: (obj: any, key: string, v: unknown) => void): Watch }).sub((_obj, key, v) => setStyle(element, key, v));
    } else {
      element.setAttribute('style', current);
      value.sub((v) => element.setAttribute('style', v));
    }
  } else if (typeof value === 'object' && value !== null) {
    for (const [name, v] of Object.entries(value as StyleRecordX)) {
      if (isWatchable(v)) {
        setStyle(element, name, v.get());
        v.sub((nv) => setStyle(element, name, nv));
      } else {
        setStyle(element, name, v);
      }
    }
  } else {
    element.setAttribute('style', value as string);
  }
}

function appendRoots(parent: Node, node: KitNode): void {
  if (node.root) {
    parent.appendChild(node.root);
  } else if (node.children) {
    for (const child of node.children) {
      appendRoots(parent, child);
    }
  }
}

function collectRoots(node: KitNode, out: Node[]): Node[] {
  if (node.root) {
    out.push(node.root);
  } else if (node.children) {
    for (const child of node.children) {
      collectRoots(child, out);
    }
  }
  return out;
}

function initialText(value: unknown): string {
  const text = String(value);
  return text === '' ? ' ' : text;
}

function updatedText(value: unknown): string {
  return value === null || value === undefined || value === '' ? ' ' : String(value);
}

function staticNodes(content: string): KitNode[] {
  const container = document.createElement('div');
  container.innerHTML = content;
  return Array.from(container.childNodes, (child) => ({ instance: child, root: child, props: null, children: null }));
}

function flatten(children: WidgetChildren[], out: WidgetChildren[] = []): WidgetChildren[] {
  for (const child of children) {
    if (Array.isArray(child)) {
      flatten(child, out);
    } else {
      out.push(child);
    }
  }
  return out;
}

let afterRenderParentsFirst = (globalThis as { __XP_AFTER_RENDER_ORDER__?: string }).__XP_AFTER_RENDER_ORDER__ === 'parents-first';

function callAfterRender(node: KitNode): void {
  const instance = node.instance;
  const isComponent = instance instanceof Widget;
  if (isComponent && afterRenderParentsFirst) {
    instance.mounted(node);
  }
  if (node.children) {
    for (const child of node.children) {
      callAfterRender(child);
    }
  }
  if (isComponent && !afterRenderParentsFirst) {
    instance.mounted(node);
  }
}

export namespace Kit {
  export namespace JSX {
    export interface IntrinsicElements {
      [elementName: string]: {
        [attributeName: string]: any;
        ref?: Ref<any>;
        style?: string | Watchable<string> | WatchableMap<string, string> | StyleRecordX | any;
        class?: string | Watchable<string> | WatchableSet<string> | ToggleableClassNameRecordX | any;
      };
    }
  }

  export const EmptyHandler = (): void => {};

  export function Fragment(props: WidgetProps): WidgetChildren[] | undefined {
    return props.children;
  }

  export function h(type: any, props: any, ...children: WidgetChildren[]): KitNode | null {
    if (typeof type === 'string') {
      const element: HTMLElement | SVGElement = SVG_TAGS.has(type)
        ? (document.createElementNS(SVG_NAMESPACE, type) as SVGElement)
        : document.createElement(type);
      const node: KitNode = { instance: element, root: element, props, children: null };
      if (props !== null && props !== undefined) {
        for (const key of Object.keys(props)) {
          const value = props[key];
          if (key === 'ref') {
            if (value) {
              (value as Ref<any>).instance = element;
            }
          } else if (key === 'children') {
            continue;
          } else if (key === 'class') {
            applyClass(element, value);
          } else if (key === 'style') {
            applyStyle(element, value);
          } else if (typeof value === 'function') {
            continue;
          } else if (isWatchable(value)) {
            setAttr(element, key, value.get());
            value.sub((v) => setAttr(element, key, v));
          } else {
            setAttr(element, key, value);
          }
        }
      }
      node.children = createChildNodes(node, children);
      if (node.children) {
        for (const child of node.children) {
          appendRoots(element, child);
        }
      }
      return node;
    }

    if (isComponentClass(type)) {
      const componentProps = props ?? {};
      componentProps.children = children;
      const instance = new type(componentProps);
      if (componentProps.ref) {
        (componentProps.ref as Ref<any>).instance = instance;
      }
      const node: KitNode = { instance, props: componentProps, children: null };
      instance.onBeforeRender();
      const rendered = instance.render();
      if (rendered) {
        node.children = [rendered];
        node.root = rendered.root;
      }
      return node;
    }

    if (typeof type === 'function') {
      const fragmentProps = props ?? {};
      fragmentProps.children = children;
      const result = type(fragmentProps) as WidgetChildren[] | WidgetChildren | undefined;
      const node: KitNode = { instance: null, props: fragmentProps, children: null };
      if (result !== undefined && result !== null) {
        node.children = createChildNodes(node, Array.isArray(result) ? result : [result]);
      }
      return node;
    }

    return null;
  }

  export function createChildNodes(parent: KitNode, children: WidgetChildren[]): KitNode[] | null {
    const nodes: KitNode[] = [];
    for (const child of flatten(children)) {
      if (child === null || child === undefined || typeof child === 'boolean') {
        continue;
      }
      if (typeof child === 'string' || typeof child === 'number') {
        if (child === '') {
          continue;
        }
        if (typeof child === 'string' && /[<&]/.test(child)) {
          nodes.push(...staticNodes(child));
        } else {
          nodes.push(createStaticContentNode(child));
        }
      } else if (isWatchable(child)) {
        const text = document.createTextNode(initialText(child.get()));
        child.sub((v) => {
          text.textContent = updatedText(v);
        });
        nodes.push({ instance: child, root: text, props: null, children: null });
      } else if (isVNode(child)) {
        nodes.push(child);
      }
    }
    return nodes.length > 0 ? nodes : null;
  }

  export function createStaticContentNode(content: string | number): KitNode {
    return { instance: content, root: document.createTextNode(String(content)), props: null, children: null };
  }

  export function render(node: KitNode, element: HTMLElement | SVGElement | null, position: Place = Place.In): void {
    if (element === null || element === undefined || node === null || node === undefined) {
      return;
    }
    const roots = collectRoots(node, []);
    if (position === Place.In) {
      for (const root of roots) {
        element.appendChild(root);
      }
    } else if (position === Place.Before) {
      for (const root of roots) {
        element.parentNode?.insertBefore(root, element);
      }
    } else {
      let anchor: Node | null = element.nextSibling;
      for (const root of roots) {
        element.parentNode?.insertBefore(root, anchor);
        anchor = root.nextSibling;
      }
    }
    callAfterRender(node);
  }

  export function renderBefore(node: KitNode, element: HTMLElement | SVGElement | null): void {
    render(node, element, Place.Before);
  }

  export function renderAfter(node: KitNode, element: HTMLElement | SVGElement | null): void {
    render(node, element, Place.After);
  }

  export function remove(element: HTMLElement | SVGElement | null): void {
    element?.remove();
  }

  export function createRef<T>(): Ref<T> {
    return new Ref<T>();
  }

  export function visitNodes(node: KitNode, visitor: (node: KitNode) => boolean): void {
    if (visitor(node)) {
      return;
    }
    if (node.children) {
      for (const child of node.children) {
        visitNodes(child, visitor);
      }
    }
  }

  export function shallowDestroy(root: KitNode): void {
    visitNodes(root, (node) => {
      if (node !== root && node.instance instanceof Widget) {
        node.instance.destroy();
        return true;
      }
      return false;
    });
  }

  export function parseCssClassesFromString(classString: string, filter?: (cssClass: string) => boolean): string[] {
    return classString.split(/\s+/).filter((cssClass) => cssClass.length > 0 && (filter === undefined || filter(cssClass)));
  }

  export function bindCssClassSet(setToBind: MutableWatchableSet<string>, classesToSubscribe: WatchableSet<string> | ToggleableClassNameRecordX, reservedClasses: Iterable<string>): Watch | Watch[] {
    const reserved = new Set(reservedClasses);
    if (isSubscribableSet(classesToSubscribe)) {
      return classesToSubscribe.sub((set, type, key) => {
        if (reserved.has(key)) {
          return;
        }
        if ((type as unknown as string) === 'Added') {
          setToBind.add(key);
        } else {
          setToBind.delete(key);
        }
      }, true);
    }
    const subs: Watch[] = [];
    for (const [cssClass, enabled] of Object.entries(classesToSubscribe)) {
      if (reserved.has(cssClass)) {
        continue;
      }
      if (isWatchable(enabled)) {
        subs.push(enabled.sub((v) => setToBind.toggle(cssClass, !!v), true));
      } else {
        setToBind.toggle(cssClass, !!enabled);
      }
    }
    return subs;
  }

  export function addCssClassesToRecord(record: ToggleableClassNameRecordX, classesToAdd: string | Iterable<string> | ToggleableClassNameRecordX, allowOverwrite = true, filter?: (cssClass: string) => boolean): ToggleableClassNameRecordX {
    const add = (cssClass: string, value: boolean | Watchable<boolean>): void => {
      if ((filter === undefined || filter(cssClass)) && (allowOverwrite || !(cssClass in record))) {
        record[cssClass] = value;
      }
    };
    if (typeof classesToAdd === 'string') {
      for (const cssClass of parseCssClassesFromString(classesToAdd)) {
        add(cssClass, true);
      }
    } else if (typeof (classesToAdd as Iterable<string>)[Symbol.iterator] === 'function') {
      for (const cssClass of classesToAdd as Iterable<string>) {
        add(cssClass, true);
      }
    } else {
      for (const [cssClass, value] of Object.entries(classesToAdd as ToggleableClassNameRecordX)) {
        add(cssClass, value);
      }
    }
    return record;
  }

  export function bindStyleMap(mapToBind: MutableWatchableMap<string, string>, stylesToSubscribe: WatchableMap<string, string> | StyleRecordX | any, reservedStyles: Iterable<string>): Watch | Watch[] {
    const reserved = new Set(reservedStyles);
    if (isSubscribableMap(stylesToSubscribe)) {
      return stylesToSubscribe.sub((_map, type, key, value) => {
        if (reserved.has(key)) {
          return;
        }
        if ((type as unknown as string) === 'Deleted') {
          mapToBind.delete(key);
        } else {
          mapToBind.setValue(key, value);
        }
      }, true);
    }
    if (isWatchable(stylesToSubscribe)) {
      const objectSubject = stylesToSubscribe as unknown as { get(): Record<string, any>; sub(handler: (obj: any, key: string, v: any) => void, initialNotify?: boolean): Watch };
      for (const [key, value] of Object.entries(objectSubject.get())) {
        if (!reserved.has(key)) {
          mapToBind.setValue(key, value);
        }
      }
      return objectSubject.sub((_obj, key, value) => {
        if (!reserved.has(key)) {
          mapToBind.setValue(key, value);
        }
      });
    }
    const subs: Watch[] = [];
    for (const [key, value] of Object.entries(stylesToSubscribe as StyleRecordX)) {
      if (reserved.has(key)) {
        continue;
      }
      if (isWatchable(value)) {
        subs.push(value.sub((v) => mapToBind.setValue(key, v as string), true));
      } else if (value !== undefined && value !== null) {
        mapToBind.setValue(key, String(value));
      }
    }
    return subs;
  }

  export function __setAfterRenderOrder(order: 'parents-first' | 'children-first'): void {
    afterRenderParentsFirst = order === 'parents-first';
  }
}

export enum Place {
  Before = 0,
  After = 1,
  In = 2,
}

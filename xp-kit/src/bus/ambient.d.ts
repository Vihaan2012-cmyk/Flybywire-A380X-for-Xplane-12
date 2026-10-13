declare const SimVar: {
  GetSimVarValue(name: string, unit: string, dataSource?: string): any;
  SetSimVarValue(name: string, unit: string, value: any, dataSource?: string): Promise<void>;
  GetGlobalVarValue(name: string, unit: string): any;
  GetRegisteredId(name: string, unit: string, dataSource?: string): number;
  GetSimVarValueFastReg(id: number): any;
  GetSimVarValueFastRegString(id: number): string;
};
declare const Coherent: {
  call(name: string, ...args: any[]): Promise<any>;
  on(name: string, callback: (...args: any[]) => void, context?: any): { clear(): void };
  trigger(name: string, ...args: any[]): void;
};
interface BusViewListener {
  on(event: string, callback: (...args: any[]) => void, context?: any): void;
  off(event: string, callback: (...args: any[]) => void, context?: any): void;
  call(name: string, ...args: any[]): Promise<any>;
  trigger(event: string, ...args: any[]): void;
  onDataReceived?(key: string, callback: (data: any) => void): void;
  send?(key: string, data: any): void;
}
declare function RegisterViewListener(name: string, callback?: (() => void) | null, requiresSingleton?: boolean): BusViewListener;
declare function RegisterGenericDataListener(callback?: () => void): BusViewListener;
declare enum GameState {
  mainmenu = 0,
  loading = 1,
  briefing = 2,
  ingame = 3,
}
type Name_Z = any;
declare class BaseInstrument extends HTMLElement {
  protected Update(): void;
  connectedCallback(): void;
  onInteractionEvent(args: string[]): void;
  protected onGameStateChanged(oldState: GameState, newState: GameState): void;
  onFlightStart(): void;
  onSoundEnd(soundEventId: Name_Z): void;
  get isInteractive(): boolean;
}

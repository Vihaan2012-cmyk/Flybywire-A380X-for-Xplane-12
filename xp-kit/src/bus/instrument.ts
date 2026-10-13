export interface GaugeLike {
  readonly instrument: BaseInstrument;
  Update(): void;
  onInteractionEvent(args: Array<string>): void;
  onFlightStart(): void;
  onGameStateChanged(oldState: GameState, newState: GameState): void;
  onSoundEnd(soundEventId: Name_Z): void;
}

export abstract class GaugeBase<T extends GaugeLike> extends BaseInstrument {
  protected fsInstrument!: T;

  public abstract get templateID(): string;

  public abstract constructInstrument(): T;

  public connectedCallback(): void {
    super.connectedCallback();
    this.fsInstrument = this.constructInstrument();
  }

  protected Update(): void {
    super.Update();
    this.fsInstrument?.Update();
  }

  public onInteractionEvent(args: Array<string>): void {
    super.onInteractionEvent(args);
    this.fsInstrument?.onInteractionEvent(args);
  }

  protected onGameStateChanged(oldState: GameState, newState: GameState): void {
    super.onGameStateChanged(oldState, newState);
    this.fsInstrument?.onGameStateChanged(oldState, newState);
  }

  public onFlightStart(): void {
    super.onFlightStart();
    this.fsInstrument?.onFlightStart();
  }

  public onSoundEnd(soundEventId: Name_Z): void {
    super.onSoundEnd(soundEventId);
    this.fsInstrument?.onSoundEnd(soundEventId);
  }

  public get isInteractive(): boolean {
    return true;
  }
}

import { Subject } from '../sub';
import type { Subscribable } from '../sub';

export class GameStateProvider {
  private static instance?: GameStateProvider;

  private readonly gameState = Subject.create<GameState | undefined>(undefined);

  private constructor() {
    setTimeout(() => this.onAttributesChanged(), 0);
    document.addEventListener('OnVCockpitPanelAttributesChanged', () => this.onAttributesChanged());
  }

  private onAttributesChanged(): void {
    const attribute = document.body.getAttribute('gamestate');
    if (attribute !== null) {
      const state = (GameState as any)[attribute] as GameState | undefined;
      if (state !== undefined) {
        this.gameState.set(state);
      }
    }
  }

  public static get(): Subscribable<GameState | undefined> {
    if (GameStateProvider.instance === undefined) {
      GameStateProvider.instance = new GameStateProvider();
    }
    return GameStateProvider.instance.gameState;
  }
}

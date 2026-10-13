import { Cell } from '../sub';
import type { Watchable } from '../sub';

export class GameStateWatch {
  private static instance?: GameStateWatch;

  private readonly gameState = Cell.create<GameState | undefined>(undefined);

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

  public static get(): Watchable<GameState | undefined> {
    if (GameStateWatch.instance === undefined) {
      GameStateWatch.instance = new GameStateWatch();
    }
    return GameStateWatch.instance.gameState;
  }
}

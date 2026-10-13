export class Debouncer {
  private timer: ReturnType<typeof setTimeout> | null = null;

  public isPending(): boolean {
    return this.timer !== null;
  }

  public schedule(action: () => void, delay: number): void {
    this.clear();
    this.timer = setTimeout(() => {
      this.timer = null;
      action();
    }, delay);
  }

  public clear(): void {
    if (this.timer !== null) {
      clearTimeout(this.timer);
      this.timer = null;
    }
  }
}

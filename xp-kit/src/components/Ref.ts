export class Ref<T> {
  private _instance: T | null = null;

  public get instance(): T {
    if (this._instance === null) {
      throw new Error('Instance was null.');
    }
    return this._instance;
  }

  public set instance(value: T) {
    this._instance = value;
  }

  public getOrDefault(): T | null {
    return this._instance;
  }
}

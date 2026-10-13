function roundTo(value: number, precision: number): number {
  return precision > 0 ? Math.round(value / precision) * precision : value;
}

class CssTranslate3dTransform {
  private x = 0;
  private y = 0;
  private z = 0;

  public constructor(
    private readonly unitX: string,
    private readonly unitY: string,
    private readonly unitZ: string,
  ) {}

  public set(x: number, y: number, z: number, precisionX = 0, precisionY = precisionX, precisionZ = precisionX): void {
    this.x = roundTo(x, precisionX);
    this.y = roundTo(y, precisionY);
    this.z = roundTo(z, precisionZ);
  }

  public resolve(): string {
    return `translate3d(${this.x}${this.unitX}, ${this.y}${this.unitY}, ${this.z}${this.unitZ})`;
  }
}

class CssTranslateTransform {
  private x = 0;
  private y = 0;

  public constructor(
    private readonly unitX: string,
    private readonly unitY: string,
  ) {}

  public set(x: number, y: number, precisionX = 0, precisionY = precisionX): void {
    this.x = roundTo(x, precisionX);
    this.y = roundTo(y, precisionY);
  }

  public resolve(): string {
    return `translate(${this.x}${this.unitX}, ${this.y}${this.unitY})`;
  }
}

export class CssTransform {
  public static translate3d(unitX: string, unitY = unitX, unitZ = unitX): CssTranslate3dTransform {
    return new CssTranslate3dTransform(unitX, unitY, unitZ);
  }

  public static translate(unitX: string, unitY = unitX): CssTranslateTransform {
    return new CssTranslateTransform(unitX, unitY);
  }
}

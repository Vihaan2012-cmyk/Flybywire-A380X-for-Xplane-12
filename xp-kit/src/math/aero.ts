interface IsaLayer {
  baseAltitude: number;
  baseTemperature: number;
  basePressure: number;
  lapseRate: number;
}

const SEA_LEVEL_TEMPERATURE = 288.15;
const SEA_LEVEL_PRESSURE_PA = 101325;
const GRAVITY = 9.80665;
const AIR_GAS_CONSTANT = 287.057;
const GAMMA = 1.4;
const KELVIN_OFFSET = 273.15;
const MIN_ALTITUDE = -610;
const MAX_ALTITUDE = 80000;
const SEA_LEVEL_PRESSURE_HPA = 1013.25;

const LAPSE_RATES = [
  { base: 0, lapse: -0.0065 },
  { base: 11000, lapse: 0 },
  { base: 20000, lapse: 0.001 },
  { base: 32000, lapse: 0.0028 },
  { base: 47000, lapse: 0 },
  { base: 51000, lapse: -0.0028 },
  { base: 71000, lapse: -0.002 },
];

function buildLayers(): IsaLayer[] {
  const layers: IsaLayer[] = [];
  let temperature = SEA_LEVEL_TEMPERATURE;
  let pressure = SEA_LEVEL_PRESSURE_PA;
  for (let i = 0; i < LAPSE_RATES.length; i++) {
    const { base, lapse } = LAPSE_RATES[i];
    layers.push({ baseAltitude: base, baseTemperature: temperature, basePressure: pressure, lapseRate: lapse });
    if (i + 1 < LAPSE_RATES.length) {
      const next = LAPSE_RATES[i + 1].base;
      pressure = layerPressure(layers[i], next);
      temperature = temperature + lapse * (next - base);
    }
  }
  return layers;
}

function layerPressure(layer: IsaLayer, altitude: number): number {
  const delta = altitude - layer.baseAltitude;
  if (layer.lapseRate === 0) {
    return layer.basePressure * Math.exp((-GRAVITY * delta) / (AIR_GAS_CONSTANT * layer.baseTemperature));
  }
  const temperature = layer.baseTemperature + layer.lapseRate * delta;
  return (
    layer.basePressure * Math.pow(temperature / layer.baseTemperature, -GRAVITY / (AIR_GAS_CONSTANT * layer.lapseRate))
  );
}

const LAYERS = buildLayers();

function layerForAltitude(altitude: number): IsaLayer {
  for (let i = LAYERS.length - 1; i >= 0; i--) {
    if (altitude >= LAYERS[i].baseAltitude) {
      return LAYERS[i];
    }
  }
  return LAYERS[0];
}

export class Aero {
  public static readonly R = 8.314462618153;
  public static readonly R_AIR = 287.057;
  public static readonly GAMMA_AIR = 1.4;
  public static readonly SOUND_SPEED_SEA_LEVEL_ISA = 340.2964;
  public static readonly DENSITY_SEA_LEVEL_ISA = SEA_LEVEL_PRESSURE_PA / (AIR_GAS_CONSTANT * SEA_LEVEL_TEMPERATURE);

  public static pressureAir(temperature: number, density: number): number {
    return (density * AIR_GAS_CONSTANT * (temperature + KELVIN_OFFSET)) / 100;
  }

  public static densityAir(pressure: number, temperature: number): number {
    return (pressure * 100) / (AIR_GAS_CONSTANT * (temperature + KELVIN_OFFSET));
  }

  public static temperatureAir(pressure: number, density: number): number {
    return (pressure * 100) / (AIR_GAS_CONSTANT * density) - KELVIN_OFFSET;
  }

  public static soundSpeedAir(temperature: number): number {
    return Math.sqrt(GAMMA * AIR_GAS_CONSTANT * (temperature + KELVIN_OFFSET));
  }

  public static totalPressureRatioAir(mach: number): number {
    return Math.pow(1 + 0.2 * mach * mach, 3.5);
  }

  public static totalTemperatureRatioAir(mach: number, recovery = 1): number {
    return 1 + recovery * 0.2 * mach * mach;
  }

  public static isaTemperature(altitude: number): number {
    const clamped = Math.min(MAX_ALTITUDE, Math.max(MIN_ALTITUDE, altitude));
    const layer = layerForAltitude(clamped);
    return layer.baseTemperature + layer.lapseRate * (clamped - layer.baseAltitude) - KELVIN_OFFSET;
  }

  public static isaPressure(altitude: number): number {
    const clamped = Math.min(MAX_ALTITUDE, Math.max(MIN_ALTITUDE, altitude));
    return layerPressure(layerForAltitude(clamped), clamped) / 100;
  }

  public static isaAltitude(pressure: number): number {
    const minPressure = Aero.isaPressure(MAX_ALTITUDE);
    const maxPressure = Aero.isaPressure(MIN_ALTITUDE);
    if (pressure >= maxPressure) {
      return MIN_ALTITUDE;
    }
    if (pressure <= minPressure) {
      return MAX_ALTITUDE;
    }
    let low = MIN_ALTITUDE;
    let high = MAX_ALTITUDE;
    for (let i = 0; i < 100; i++) {
      const mid = (low + high) / 2;
      if (Aero.isaPressure(mid) > pressure) {
        low = mid;
      } else {
        high = mid;
      }
    }
    return (low + high) / 2;
  }

  public static isaDensity(altitude: number, deltaIsa = 0): number {
    return Aero.densityAir(Aero.isaPressure(altitude), Aero.isaTemperature(altitude) + deltaIsa);
  }

  public static soundSpeedIsa(altitude: number, deltaIsa = 0): number {
    return Aero.soundSpeedAir(Aero.isaTemperature(altitude) + deltaIsa);
  }

  public static baroPressureAltitudeOffset(baro: number): number {
    return -Aero.isaAltitude(baro);
  }

  public static altitudeOffsetBaroPressure(offset: number): number {
    return Aero.isaPressure(-offset);
  }

  public static tasToMach(tas: number, soundSpeed: number): number {
    return tas / soundSpeed;
  }

  public static tasToMachIsa(tas: number, altitude: number, deltaIsa = 0): number {
    return Aero.tasToMach(tas, Aero.soundSpeedIsa(altitude, deltaIsa));
  }

  public static machToTas(mach: number, soundSpeed: number): number {
    return mach * soundSpeed;
  }

  public static machToTasIsa(mach: number, altitude: number, deltaIsa = 0): number {
    return Aero.machToTas(mach, Aero.soundSpeedIsa(altitude, deltaIsa));
  }

  public static casToMach(cas: number, pressure: number): number {
    const speedRatio = cas / Aero.SOUND_SPEED_SEA_LEVEL_ISA;
    const impactRatio = (SEA_LEVEL_PRESSURE_HPA / pressure) * (Math.pow(1 + 0.2 * speedRatio * speedRatio, 3.5) - 1);
    return Math.sqrt(5 * (Math.pow(impactRatio + 1, 2 / 7) - 1));
  }

  public static casToMachIsa(cas: number, altitude: number): number {
    return Aero.casToMach(cas, Aero.isaPressure(altitude));
  }

  public static machToCas(mach: number, pressure: number): number {
    const impactRatio = Math.pow(1 + 0.2 * mach * mach, 3.5) - 1;
    const sea = (pressure / SEA_LEVEL_PRESSURE_HPA) * impactRatio + 1;
    const speedRatio = Math.sqrt(5 * (Math.pow(sea, 2 / 7) - 1));
    return speedRatio * Aero.SOUND_SPEED_SEA_LEVEL_ISA;
  }

  public static machToCasIsa(mach: number, altitude: number): number {
    return Aero.machToCas(mach, Aero.isaPressure(altitude));
  }

  public static casToTas(cas: number, pressure: number, temperature: number): number {
    return Aero.machToTas(Aero.casToMach(cas, pressure), Aero.soundSpeedAir(temperature));
  }

  public static casToTasIsa(cas: number, altitude: number, deltaIsa = 0): number {
    return Aero.casToTas(cas, Aero.isaPressure(altitude), Aero.isaTemperature(altitude) + deltaIsa);
  }

  public static tasToCas(tas: number, pressure: number, temperature: number): number {
    return Aero.machToCas(Aero.tasToMach(tas, Aero.soundSpeedAir(temperature)), pressure);
  }

  public static tasToCasIsa(tas: number, altitude: number, deltaIsa = 0): number {
    return Aero.tasToCas(tas, Aero.isaPressure(altitude), Aero.isaTemperature(altitude) + deltaIsa);
  }

  public static tasToEas(tas: number, density: number): number {
    return tas * Math.sqrt(density / Aero.DENSITY_SEA_LEVEL_ISA);
  }

  public static tasToEasIsa(tas: number, altitude: number, deltaIsa = 0): number {
    return Aero.tasToEas(tas, Aero.isaDensity(altitude, deltaIsa));
  }

  public static easToTas(eas: number, density: number): number {
    return eas / Math.sqrt(density / Aero.DENSITY_SEA_LEVEL_ISA);
  }

  public static easToTasIsa(eas: number, altitude: number, deltaIsa = 0): number {
    return Aero.easToTas(eas, Aero.isaDensity(altitude, deltaIsa));
  }

  public static machToEas(mach: number, pressure: number): number {
    return mach * Math.sqrt((GAMMA * pressure * 100) / Aero.DENSITY_SEA_LEVEL_ISA);
  }

  public static machToEasIsa(mach: number, altitude: number): number {
    return Aero.machToEas(mach, Aero.isaPressure(altitude));
  }

  public static easToMach(eas: number, pressure: number): number {
    return eas / Math.sqrt((GAMMA * pressure * 100) / Aero.DENSITY_SEA_LEVEL_ISA);
  }

  public static easToMachIsa(eas: number, altitude: number): number {
    return Aero.easToMach(eas, Aero.isaPressure(altitude));
  }

  public static casToEas(cas: number, pressure: number): number {
    return Aero.machToEas(Aero.casToMach(cas, pressure), pressure);
  }

  public static casToEasIsa(cas: number, altitude: number): number {
    return Aero.casToEas(cas, Aero.isaPressure(altitude));
  }

  public static easToCas(eas: number, pressure: number): number {
    return Aero.machToCas(Aero.easToMach(eas, pressure), pressure);
  }

  public static easToCasIsa(eas: number, altitude: number): number {
    return Aero.easToCas(eas, Aero.isaPressure(altitude));
  }

  private static flowDynamicPressureHpa(args: number[]): number {
    if (args.length >= 2) {
      return (0.5 * args[0] * args[1] * args[1]) / 100;
    }
    return args[0];
  }

  private static flowCoefFromForce(force: number, area: number, flowArgs: number[]): number {
    return force / (area * Aero.flowDynamicPressureHpa(flowArgs) * 100);
  }

  private static flowForceFromCoef(coef: number, area: number, flowArgs: number[]): number {
    return coef * area * Aero.flowDynamicPressureHpa(flowArgs) * 100;
  }

  public static liftCoefficient: {
    (lift: number, area: number, density: number, flowSpeed: number): number;
    (lift: number, area: number, dynamicPressure: number): number;
  } = (lift: number, area: number, ...flow: number[]): number => Aero.flowCoefFromForce(lift, area, flow);

  public static lift: {
    (cl: number, area: number, density: number, flowSpeed: number): number;
    (cl: number, area: number, dynamicPressure: number): number;
  } = (cl: number, area: number, ...flow: number[]): number => Aero.flowForceFromCoef(cl, area, flow);

  public static dragCoefficient: {
    (drag: number, area: number, density: number, flowSpeed: number): number;
    (drag: number, area: number, dynamicPressure: number): number;
  } = (drag: number, area: number, ...flow: number[]): number => Aero.flowCoefFromForce(drag, area, flow);

  public static drag: {
    (cd: number, area: number, density: number, flowSpeed: number): number;
    (cd: number, area: number, dynamicPressure: number): number;
  } = (cd: number, area: number, ...flow: number[]): number => Aero.flowForceFromCoef(cd, area, flow);

  public static thrustCorrectionFactor(mach: number, ambientPressure: number): number {
    return SEA_LEVEL_PRESSURE_HPA / (ambientPressure * Aero.totalPressureRatioAir(mach));
  }
}

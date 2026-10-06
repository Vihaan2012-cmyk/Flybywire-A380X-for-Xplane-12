import { useSimVar } from '@instruments/common/simVars';
import { EngineNumber, IgnitionActive, Position } from '@instruments/common/types';
import React, { FC } from 'react';
import DecimalValues from './DecimalValues';
import IgnitionBorder from './IgnitionBorder';
import NacelleTemperatureGauge from './NacelleTemperatureGauge';
import OilPressureGauge from './OilPressureGauge';
import OilQuantityGauge from './OilQuantityGauge';
import StartValve from './StartValve';
import { NXUnits } from '@flybywiresim/fbw-sdk-react';
import { AutoThrustMode } from '@shared/autopilot';

const VIB_ADVISORY_THRESHOLD_UNITS = 5;
const OIL_QTY_ADVISORY_LIMIT_QUARTS = 1.2;
const OIL_TEMP_ADVISORY_THRESHOLD_C = 163;
const OIL_TEMP_ABNORMAL_THRESHOLD_C = 177;

interface EngineColumnProps {
  anyEngineRunning: boolean;
  useMetric: boolean;
}

const EngineColumn: FC<Position & EngineNumber & IgnitionActive & EngineColumnProps> = ({
  x,
  y,
  engine,
  ignition,
  anyEngineRunning,
}) => {
  const [N2] = useSimVar(`L:A32NX_ENGINE_N2:${engine}`, 'number', 100); // TODO: Update with correct SimVars
  const [N3] = useSimVar(`L:A32NX_ENGINE_N3:${engine}`, 'number', 100); // TODO: Update with correct SimVars
  const [starterValveOpen] = useSimVar(`L:A32NX_PNEU_ENG_${engine}_STARTER_VALVE_OPEN`, 'number', 500); // TODO: Update with correct SimVars
  const starting = !!(N2 < 58.5 && ignition && starterValveOpen); // TODO Should be N3
  const [fadecManuallyPowered] = useSimVar(`L:A32NX_OVHD_FADEC_${engine}`, 'bool', 500);
  const [engineFirePbReleased] = useSimVar(`L:A32NX_FIRE_BUTTON_ENG${engine}`, 'bool', 500);

  const fadecPowered = (ignition || anyEngineRunning || fadecManuallyPowered) && !engineFirePbReleased;

  const [fuelFlow] = useSimVar(`L:A32NX_ENGINE_FF:${engine}`, 'number', 100);

  const [n1Vibration] = useSimVar(`L:A32NX_ENG_${engine}_N1_VIB_INDEX`, 'number', 100);
  const [n2Vibration] = useSimVar(`L:A32NX_ENG_${engine}_N2_VIB_INDEX`, 'number', 100);
  const [n3Vibration] = useSimVar(`L:A32NX_ENG_${engine}_N3_VIB_INDEX`, 'number', 100);
  const n1VibrationPulse = n1Vibration > VIB_ADVISORY_THRESHOLD_UNITS;
  const n2VibrationPulse = n2Vibration > VIB_ADVISORY_THRESHOLD_UNITS;
  const n3VibrationPulse = n3Vibration > VIB_ADVISORY_THRESHOLD_UNITS;

  const [oilQuantityFrac] = useSimVar(`L:A32NX_DEEP_ENG_${engine}_OIL_QTY_SENSED_FRAC`, 'number', 500);
  const oilQuantity = oilQuantityFrac * 18.3;
  const [engineOilTemperature] = useSimVar(`L:A32NX_DEEP_ENG_${engine}_OIL_TEMP_SENSED_C`, 'number', 100);

  const [autoThrustMode] = useSimVar('L:A32NX_AUTOTHRUST_MODE', 'enum', 500);
  const [reverserDeploying] = useSimVar(`L:A32NX_REVERSER_${engine}_DEPLOYING`, 'bool', 500);
  const takeoffOrGoAround =
    autoThrustMode === AutoThrustMode.MAN_TOGA ||
    autoThrustMode === AutoThrustMode.MAN_GA_SOFT ||
    autoThrustMode === AutoThrustMode.TOGA_LK;
  const alphaFloorActive = autoThrustMode === AutoThrustMode.A_FLOOR;
  const oilQuantityAdvisoryInhibited = takeoffOrGoAround || alphaFloorActive || !!reverserDeploying;
  const oilQuantityAdvisory = oilQuantity < OIL_QTY_ADVISORY_LIMIT_QUARTS && !oilQuantityAdvisoryInhibited;

  const oilTempAbnormal = engineOilTemperature > OIL_TEMP_ABNORMAL_THRESHOLD_C;
  const oilTempAdvisory = engineOilTemperature > OIL_TEMP_ADVISORY_THRESHOLD_C && !oilTempAbnormal;

  return (
    <>
      <IgnitionBorder x={x} y={y} engine={engine} ignition={ignition} />
      {/* N2 */}
      <DecimalValues x={x} y={y} value={N2} active={fadecPowered} />
      <path className="White SW2" d={`M${engine > 2 ? x - 96 : x + 64},${y - 10} l 26, 0`} />
      {/* N3 */}
      <rect x={x - 55} y={y + 10} width={98} height={34} className={`LightGreyBox ${starting ? 'Show' : 'Hide'}`} />
      <DecimalValues x={x} y={y + 38} value={N3} active={fadecPowered} />
      <path className="White SW2" d={`M${engine > 2 ? x - 96 : x + 64},${y + 28} l 26, 0`} />
      {/* Fuel Flow */}
      {!fadecPowered && (
        <text x={x} y={y + 86} className="Amber F29 MiddleAlign">
          XX
        </text>
      )}
      {fadecPowered && (
        <text x={x + 30} y={y + 92} className="Green EndAlign F29">
          {Math.ceil(NXUnits.kgToUser(fuelFlow) / 10) * 10}
        </text>
      )}
      {/* OIL */}
      <OilQuantityGauge
        x={x}
        y={y + 206}
        engine={engine}
        active={fadecPowered}
        value={oilQuantity}
        pulse={oilQuantityAdvisory}
      />
      <DecimalValues x={x} y={y + 206} value={oilQuantity} active={fadecPowered} pulse={oilQuantityAdvisory} />
      {!fadecPowered && (
        <text x={x} y={y + 242} className="Amber F29 MiddleAlign">
          XX
        </text>
      )}
      {fadecPowered && (
        <text
          x={x + 20}
          y={y + 248}
          className={`${oilTempAbnormal ? 'Amber' : oilTempAdvisory ? 'FillPulse' : 'Green'} EndAlign F29`}
        >
          {engineOilTemperature < 0 ? 0 : Math.round(engineOilTemperature)}
        </text>
      )}
      <path className="White SW2" d={`M${engine > 2 ? x - 96 : x + 64},${y + 82} l 26, 0`} />
      {/* Oil Pressure */}
      <OilPressureGauge x={x} y={y + 320} engine={engine} active={fadecPowered} />
      {/* VIB N1 */}
      <DecimalValues x={x} y={y + 380} value={n1Vibration} active={fadecPowered} shift={-14} pulse={n1VibrationPulse} />
      <path className="White SW2" d={`M${engine > 2 ? x - 96 : x + 64},${y + 370} l 26, 0`} />
      {/* VIB N2 */}
      <DecimalValues x={x} y={y + 416} value={n2Vibration} active={fadecPowered} shift={-14} pulse={n2VibrationPulse} />
      <path className="White SW2" d={`M${engine > 2 ? x - 96 : x + 64},${y + 406} l 26, 0`} />
      {/* VIB N3 */}
      <DecimalValues x={x} y={y + 450} value={n3Vibration} active={fadecPowered} shift={-14} pulse={n3VibrationPulse} />
      <path className="White SW2" d={`M${engine > 2 ? x - 96 : x + 64},${y + 440} l 26, 0`} />

      {/* NAC / Ignition */}
      {(starting || ignition) && <StartValve x={x} y={y + 536} engine={engine} />}
      {!starterValveOpen && !ignition && (
        <NacelleTemperatureGauge x={x} y={y + 536} engine={engine} active={fadecPowered} value={240} />
      )}
    </>
  );
};

export default EngineColumn;

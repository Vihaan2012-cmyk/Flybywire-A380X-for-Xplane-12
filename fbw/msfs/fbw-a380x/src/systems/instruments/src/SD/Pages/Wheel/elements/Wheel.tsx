import { useSimVar } from '@flybywiresim/fbw-sdk-react';
import React, { FC } from 'react';

interface WheelProps {
  x: number;
  y: number;
  number: number | null;
  noseIndex?: number;
  isLeftSide: boolean;
  hasBrake: boolean;
  moreActive: boolean;
}

const roundTemperature = (rawTemp: number): number => Math.min(995, Math.max(0, Math.round(rawTemp / 5) * 5));

const maxStaleness = 300;

const PA_PER_PSI = 6894.757;
const TYRE_SERVICE_PA = 1_550_000;
const TPIS_SENSOR_OF_BRAKED_WHEEL = [1, 2, 5, 6, 3, 4, 7, 8, 9, 10, 13, 14, 11, 12, 15, 16];

const tpisVars = (number: number | null, noseIndex?: number): [string, string] => {
  if (noseIndex) {
    return [`L:A32NX_DEEP_TYRE_PRESSURE_SENSED_PA_NOSE_${noseIndex}`, `L:A32NX_DEEP_TYRE_PRESSURE_COLD_EQUIV_PA_NOSE_${noseIndex}`];
  }
  if (number !== null && number <= 16) {
    const sensor = TPIS_SENSOR_OF_BRAKED_WHEEL[number - 1];
    return [`L:A32NX_DEEP_TYRE_PRESSURE_SENSED_PA_${sensor}`, `L:A32NX_DEEP_TYRE_PRESSURE_COLD_EQUIV_PA_${sensor}`];
  }
  const own = `L:A32NX_TYRE_PRESSURE_PA:${(number ?? 0) + 2}`;
  return [own, own];
};

export const Wheel: FC<WheelProps> = ({ x, y, number, noseIndex, isLeftSide, hasBrake }) => {
  const negativeSign = isLeftSide ? '-' : '';
  const rightNegativeSign = !isLeftSide ? '-' : '';

  const [brakeTemp] = useSimVar(`L:A32NX_REPORTED_BRAKE_TEMPERATURE_${number}`, 'celsius', maxStaleness);
  const [pressureVar, coldEquivVar] = tpisVars(number, noseIndex);
  const [tyrePa] = useSimVar(pressureVar, 'number', maxStaleness);
  const [coldEquivPa] = useSimVar(coldEquivVar, 'number', maxStaleness);
  const tyreLow = coldEquivPa < 0.9 * TYRE_SERVICE_PA;

  return (
    <g id={`wheel-${number ?? 'nose'}`} transform={`translate(${x} ${y})`}>
      <path
        className="Grey NoFill SW2"
        d={`m${negativeSign}51,-28 v-10 c${rightNegativeSign}10,-6 ${rightNegativeSign}30,-5, ${rightNegativeSign}35,0 v10`}
      />
      <path
        className="Grey NoFill SW2"
        d={`m${negativeSign}51,40 c${rightNegativeSign}10,4 ${rightNegativeSign}30,3, ${rightNegativeSign}35,-1 v-10`}
      />
      {hasBrake && (
        <>
          <path
            className="Grey NoFill SW2"
            d={`m ${negativeSign}15,18 v -36 M ${negativeSign}21,18 v -36 M ${negativeSign}27,18 v -36 M ${negativeSign}33,18 v -36`}
          />
          <path className="BackgroundFill" d={`m ${isLeftSide ? -18 : 70},-13 h -50 v -24 h 50 z`} />
          <text className={`F26 EndAlign ${brakeTemp > 300 ? 'Amber' : 'Green'}`} x={isLeftSide ? -16 : 72} y={-16}>
            {roundTemperature(brakeTemp)}
          </text>
        </>
      )}
      <text className={`F22 EndAlign ${tyreLow ? 'Amber' : 'Green'}`} x={isLeftSide ? -16 : 65} y={34}>
        {Math.round(Math.max(0, tyrePa) / PA_PER_PSI)}
      </text>
      {number && (
        <text className={`F22 White ${isLeftSide ? 'EndAlign' : ''}`} x={isLeftSide ? -38 : 42} y={7}>
          {number}
        </text>
      )}
    </g>
  );
};

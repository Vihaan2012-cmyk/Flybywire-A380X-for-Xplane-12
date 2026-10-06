// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React from 'react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import { DepthGrid, DepthGroup, DepthLamp, DepthNum } from './DepthField';
import { LiveDiagram } from './SystemDiagrams';

const CIRCUITS: { color: string; edps: string[] }[] = [
  { color: 'GREEN', edps: ['1A', '1B', '2A', '2B'] },
  { color: 'YELLOW', edps: ['3A', '3B', '4A', '4B'] },
];

export const HydraulicDepth = () => (
  <ScrollableContainer height={43}>

    <LiveDiagram titles={['Hydraulic Network']} />

    <DepthGrid>
      {CIRCUITS.map(({ color, edps }) => (
        <DepthGroup key={color} title={`${color} circuit`}>
          <DepthNum label="Manifold pressure" lvar={`HYD_${color}_MANIFOLD_PRESSURE_PSI`} unit="psi" decimals={0} />
          <DepthNum label="Essential pressure" lvar={`HYD_${color}_ESSENTIAL_PRESSURE_PSI`} unit="psi" decimals={0} />
          <DepthNum label="Accumulator pressure" lvar={`HYD_${color}_ACCUMULATOR_PRESSURE_PSI`} unit="psi" decimals={0} />
          <DepthNum label="Reservoir level" lvar={`HYD_${color}_RESERVOIR_LEVEL_FRACTION`} unit="" decimals={2} />
          <DepthNum label="Fluid temperature" lvar={`HYD_${color}_FLUID_TEMP_C`} unit="C" decimals={1} />
          <DepthNum label="Manifold temperature" lvar={`HYD_${color}_MANIFOLD_TEMP_C`} unit="C" decimals={1} />
          <DepthLamp label="Reservoir level low" lvar={`HYD_${color}_RESERVOIR_LEVEL_IS_LOW`} />
          <DepthLamp label="Reservoir air pressure low" lvar={`HYD_${color}_RESERVOIR_AIR_PRESSURE_IS_LOW`} />
          <DepthLamp label="Reservoir overheat" lvar={`HYD_${color}_RESERVOIR_OVHT`} />
          <DepthLamp label="Sys temp hi" lvar={`HYD_${color}_SYS_TEMP_HI`} />
          <DepthLamp label="Fuel HX valve fault" lvar={`HYD_${color}_FUEL_HX_VALVE_FAULT`} />
          <DepthLamp label="Fuel HX air leak" lvar={`HYD_${color}_FUEL_HX_AIR_LEAK`} />
          <DepthLamp label="Fuel HX air leak detector fault" lvar={`HYD_${color}_FUEL_HX_AIR_LEAK_DET_FAULT`} />
          <DepthLamp label="Overheat detector A fault" lvar={`HYD_${color}_SYS_CHAN_A_OVHT_DET_FAULT`} />
          <DepthLamp label="Overheat detector B fault" lvar={`HYD_${color}_SYS_CHAN_B_OVHT_DET_FAULT`} />
          <DepthNum label="Reservoir leak" lvar={`DEEP_HYD_${color}_RESERVOIR_LEAK_M3_S`} unit="m3/s" decimals={5} />
          <DepthNum label="Reservoir pressurization loss" lvar={`DEEP_HYD_${color}_RESERVOIR_PRESSURIZATION_LOSS`} unit="" decimals={3} />
          <DepthNum label="Accumulator precharge loss" lvar={`DEEP_HYD_${color}_ACCUMULATOR_PRECHARGE_LOSS`} unit="" decimals={3} />
          <DepthLamp label="Priority valve stuck" lvar={`DEEP_HYD_${color}_PRIORITY_VALVE_STUCK`} />
          {['A', 'B'].map((letter) => (
            <DepthNum
              key={letter}
              label={`Electric pump ${letter} flow`}
              lvar={`HYD_${color}_ELEC_PUMP_${letter}_FLOW_L_MIN`}
              unit="L/min"
              decimals={1}
            />
          ))}
        </DepthGroup>
      ))}

      {CIRCUITS.map(({ color, edps }) => (
        <DepthGroup key={`${color}-edps`} title={`${color} engine-driven pumps`}>
          {edps.map((edp) => (
            <div key={edp} className="mb-2 border-b border-theme-accent/40 pb-1 last:border-b-0">
              <p className="text-xs font-bold uppercase text-theme-highlight">EDP {edp}</p>
              <DepthNum label="Flow" lvar={`HYD_${color}_EDP_${edp}_FLOW_L_MIN`} unit="L/min" decimals={1} />
              <DepthNum label="Case drain" lvar={`HYD_${color}_EDP_${edp}_CASE_DRAIN_L_MIN`} unit="L/min" decimals={2} />
              <DepthNum label="Volumetric efficiency" lvar={`HYD_${color}_EDP_${edp}_VOLUMETRIC_EFFICIENCY`} unit="" decimals={3} />
              <DepthNum label="Capability loss" lvar={`DEEP_HYD_EDP_${edp}_CAPABILITY_LOSS`} unit="" decimals={3} />
              <DepthLamp label="Fire SOV stuck" lvar={`DEEP_HYD_EDP_${edp}_FIRE_SOV_STUCK`} />
            </div>
          ))}
        </DepthGroup>
      ))}

      <DepthGroup title="Electric pump capability loss">
        {['GREEN_A', 'GREEN_B', 'YELLOW_A', 'YELLOW_B'].map((p) => (
          <DepthNum key={p} label={p.replace('_', ' ')} lvar={`DEEP_HYD_ELEC_PUMP_${p}_CAPABILITY_LOSS`} unit="" decimals={3} />
        ))}
      </DepthGroup>
    </DepthGrid>
  </ScrollableContainer>
);

// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React from 'react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import { DepthGrid, DepthGroup, DepthLamp, DepthNum } from './DepthField';
import { LiveDiagram } from './SystemDiagrams';

export const ElectricalDepth = () => {
  const buses = ['AC_1', 'AC_2', 'AC_3', 'AC_4', 'AC_ESS', 'AC_ESS_SHED', 'AC_EMER', 'AC_GND_FLT_SVC'];
  const dcBuses = ['DC_1', 'DC_2', 'DC_ESS', 'DC_ESS_SHED', 'DC_BAT', 'DC_HOT_1', 'DC_HOT_2', 'DC_APU', 'DC_GND_FLT_SVC'];

  return (
    <ScrollableContainer height={43}>

      <LiveDiagram titles={['Electrical Network']} />

      <DepthGrid>
        <DepthGroup title="AC buses">
          {buses.map((b) => (
            <div key={b} className="mb-2 border-b border-theme-accent/40 pb-1 last:border-b-0">
              <p className="text-xs font-bold uppercase text-theme-highlight">{b}</p>
              <DepthNum label="Potential" lvar={`ELEC_${b}_BUS_POTENTIAL`} unit="V" decimals={1} />
              <DepthNum label="Frequency" lvar={`ELEC_${b}_BUS_FREQUENCY`} unit="Hz" decimals={0} />
              <DepthLamp label="Powered" lvar={`ELEC_${b}_BUS_IS_POWERED`} />
            </div>
          ))}
        </DepthGroup>

        <DepthGroup title="DC buses">
          {dcBuses.map((b) => (
            <div key={b} className="mb-2 border-b border-theme-accent/40 pb-1 last:border-b-0">
              <p className="text-xs font-bold uppercase text-theme-highlight">{b}</p>
              <DepthNum label="Potential" lvar={`ELEC_${b}_BUS_POTENTIAL`} unit="V" decimals={1} />
              <DepthLamp label="Powered" lvar={`ELEC_${b}_BUS_IS_POWERED`} />
            </div>
          ))}
        </DepthGroup>

        {[1, 2, 3, 4].map((n) => (
          <DepthGroup key={`gen-${n}`} title={`Engine generator ${n}`}>
            <DepthNum label="Load" lvar={`ELEC_ENG_GEN_${n}_LOAD_W`} unit="W" decimals={0} />
            <DepthLamp label="Fault" lvar={`ELEC_GEN_${n}_FAULT`} />
            <DepthNum label="Impedance degradation" lvar={`ELEC_ENG_GEN_${n}_IMPEDANCE_DEGRADATION`} unit="" decimals={3} />
            <DepthNum label="Regulator drift" lvar={`ELEC_ENG_GEN_${n}_REGULATOR_DRIFT`} unit="" decimals={3} />
          </DepthGroup>
        ))}

        {[1, 2].map((n) => (
          <DepthGroup key={`apu-gen-${n}`} title={`APU generator ${n}`}>
            <DepthNum label="Load" lvar={`ELEC_APU_GEN_${n}_LOAD_W`} unit="W" decimals={0} />
            <DepthLamp label="Fault" lvar={`ELEC_APU_GEN_${n}_FAULT`} />
          </DepthGroup>
        ))}

        {['1', '2', 'ESS', 'APU'].map((tr) => (
          <DepthGroup key={`tr-${tr}`} title={`TR ${tr}`}>
            <DepthLamp label="Fault" lvar={`ELEC_TR_${tr}_FAULT`} />
            {(tr === '1' || tr === '2') && (
              <DepthNum label="Resistance degradation" lvar={`ELEC_TR_${tr}_RESISTANCE_DEGRADATION`} unit="" decimals={3} />
            )}
          </DepthGroup>
        ))}

        {[1, 2].map((n) => (
          <DepthGroup key={`bat-${n}`} title={`Battery ${n}`}>
            <DepthNum label="Charge" lvar={`ELEC_BAT_${n}_CHARGE_FRACTION`} unit="" decimals={3} />
            <DepthLamp label="Fault" lvar={`ELEC_BAT_${n}_FAULT`} />
            <DepthNum label="Resistance growth" lvar={`ELEC_BAT_${n}_RESISTANCE_GROWTH`} unit="" decimals={3} />
            <DepthNum label="Capacity fade" lvar={`ELEC_BAT_${n}_CAPACITY_FADE`} unit="" decimals={3} />
          </DepthGroup>
        ))}

        <DepthGroup title="Static inverter / RAT">
          <DepthLamp label="Static inverter fault" lvar="ELEC_STATIC_INV_FAULT" />
          <DepthNum label="Static inv. efficiency loss" lvar="ELEC_STAT_INV_EFFICIENCY_DEGRADATION" unit="" decimals={3} />
          <DepthLamp label="RAT deployed" lvar="ELEC_RAT_DEPLOYED" />
          <DepthLamp label="RAT fault" lvar="ELEC_RAT_FAULT" />
          <DepthNum label="Emergency gen load" lvar="ELEC_EMER_GEN_LOAD" unit="%" decimals={0} />
          <DepthNum label="RAT output capability loss" lvar="ELEC_EMER_GEN_OUTPUT_CAPABILITY_LOSS" unit="" decimals={3} />
        </DepthGroup>

        {[1, 2, 3, 4].map((n) => (
          <DepthGroup key={`ext-${n}`} title={`External power ${n}`}>
            <DepthLamp label="On line" lvar={`ELEC_EXT_PWR_${n}_ON_LINE`} />
            <DepthLamp label="Fault" lvar={`ELEC_EXT_PWR_${n}_FAULT`} />
            <DepthNum label="Regulation degradation" lvar={`ELEC_EXT_PWR_${n}_REGULATION_DEGRADATION`} unit="" decimals={3} />
          </DepthGroup>
        ))}

        <DepthGroup title="Network totals">
          <DepthNum label="Total demand" lvar="ELEC_TOTAL_DEMAND_W" unit="W" decimals={0} />
          <DepthNum label="Available capacity" lvar="ELEC_AVAILABLE_CAPACITY_W" unit="W" decimals={0} />
          <DepthNum label="Total delivered" lvar="ELEC_TOTAL_DELIVERED_W" unit="W" decimals={0} />
          <DepthLamp label="Emergency config active" lvar="ELEC_EMER_CONFIG_ACTIVE" />
          <DepthLamp label="Bus tie off" lvar="ELEC_BUS_TIE_OFF" />
          <DepthLamp label="Galley shed" lvar="ELEC_GALLEY_SHED_ACTIVE" />
          <DepthLamp label="Commercial shed" lvar="ELEC_COMMERCIAL_SHED_ACTIVE" />
        </DepthGroup>
      </DepthGrid>
    </ScrollableContainer>
  );
};

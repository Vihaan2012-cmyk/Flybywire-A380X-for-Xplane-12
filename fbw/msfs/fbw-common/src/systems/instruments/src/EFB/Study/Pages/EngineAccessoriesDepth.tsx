// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import React from 'react';
import { useSimVar } from '@flybywiresim/fbw-sdk-react';
import { ScrollableContainer } from '../../UtilComponents/ScrollableContainer';
import { DEPTH_REFRESH_MS, DepthGrid, DepthGroup, DepthLamp, DepthNum } from './DepthField';
import { LiveDiagram } from './SystemDiagrams';

const HotSectionLife = ({ engine }: { engine: number }) => {
  const [used] = useSimVar(`L:A32NX_DEEP_ENG_${engine}_HOT_SECTION_LIFE_USED`, 'number', DEPTH_REFRESH_MS);
  let colour = '';
  if (used >= 1) {
    colour = 'text-utility-red';
  } else if (used > 0) {
    colour = 'text-utility-amber';
  }
  return (
    <div className="flex flex-row items-center justify-between space-x-4 border-b border-theme-accent/40 py-1 text-sm last:border-b-0">
      <span className="text-theme-unselected">Life used beyond the over-temperature allowance</span>
      <span className="flex flex-row items-center space-x-2">
        <span className={`font-mono ${colour}`}>{(used * 100).toFixed(1)}%</span>
        {used > 0 && (
          <button
            type="button"
            onClick={() => SimVar.SetSimVarValue('L:A32NX_DEEP_ENG_HOT_SECTION_REPAIR', 'number', engine)}
            className="rounded-md border border-utility-green px-2 py-0.5 text-xs text-utility-green transition duration-100 hover:bg-utility-green hover:text-theme-body"
          >
            Repair
          </button>
        )}
      </span>
    </div>
  );
};

export const EngineAccessoriesDepth = () => (
  <ScrollableContainer height={43}>

    <LiveDiagram titles={['Engine 1 State', 'Engine 2 State', 'Engine 3 State', 'Engine 4 State']} labels={['Engine 1', 'Engine 2', 'Engine 3', 'Engine 4']} />

    <DepthGrid>
      {[1, 2, 3, 4].map((n) => (
        <DepthGroup key={n} title={`Engine ${n}`}>
          <p className="mb-1 text-xs font-bold uppercase text-theme-highlight">Fuel</p>
          <DepthLamp label="Strainer clogged" lvar={`ENG_${n}_FUEL_STRAINER_CLOGGED`} />
          <DepthLamp label="Filter impending bypass" lvar={`ENG_${n}_FUEL_FILTER_IMPENDING_BYPASS`} />
          <DepthLamp label="Filter bypassed" lvar={`ENG_${n}_FUEL_FILTER_BYPASSED`} />
          <DepthNum label="Filter dP" lvar={`ENG_${n}_FUEL_FILTER_DP_PA`} unit="Pa" decimals={0} />
          <DepthLamp label="Filter monitor fault" lvar={`ENG_${n}_FUEL_FILTER_MONITOR_FAULT`} />
          <DepthLamp label="HP pump low flow" lvar={`ENG_${n}_HP_PUMP_LOW_FLOW`} />
          <DepthNum label="HP pump flow" lvar={`ENG_${n}_HP_PUMP_FLOW_KG_S`} unit="kg/s" decimals={3} />
          <DepthLamp label="LP pump cavitating" lvar={`ENG_${n}_LP_PUMP_CAVITATING`} />
          <DepthNum label="Metered fuel" lvar={`ENG_${n}_FMU_METERED_KG_S`} unit="kg/s" decimals={3} />
          <DepthLamp label="FMU fault" lvar={`ENG_${n}_FMU_FAULT`} />
          <DepthLamp label="Thrust abnormal" lvar={`ENG_${n}_THRUST_ABNORMAL`} />
          <DepthNum label="HP SOV position" lvar={`ENG_${n}_HP_SOV_POSITION`} unit="" decimals={2} />
          <DepthLamp label="HP SOV disagree" lvar={`ENG_${n}_HP_SOV_DISAGREE`} />
          <DepthNum label="Fuel flow (indicated)" lvar={`ENG_${n}_FF_INDICATED_KG_S`} unit="kg/s" decimals={3} />
          <DepthNum label="Fuel flow (true)" lvar={`ENG_${n}_FF_TRUE_KG_S`} unit="kg/s" decimals={3} />
          <DepthLamp label="Fuel flow disagree" lvar={`ENG_${n}_FF_DISAGREE`} />
          <DepthLamp label="Nozzle imbalance" lvar={`ENG_${n}_NOZZLE_IMBALANCE`} />

          <p className="mb-1 mt-2 text-xs font-bold uppercase text-theme-highlight">Oil</p>
          <DepthLamp label="Oil filter bypassed" lvar={`ENG_${n}_OIL_FILTER_BYPASSED`} />
          <DepthLamp label="Oil system contamination" lvar={`ENG_${n}_OIL_SYSTEM_CONTAMINATION`} />

          <p className="mb-1 mt-2 text-xs font-bold uppercase text-theme-highlight">EEC / FADEC</p>
          <DepthLamp label="Channel fault (single-channel)" lvar={`ENG_${n}_EEC_CHANNEL_FAULT`} />
          <DepthLamp label="No valid channel" lvar={`ENG_${n}_EEC_NO_VALID_CHANNEL`} />
          <DepthLamp label="Sensor disagree" lvar={`ENG_${n}_EEC_SENSOR_DISAGREE`} />
          <DepthLamp label="Maintenance fault" lvar={`ENG_${n}_EEC_MAINTENANCE_FAULT`} />
          <DepthNum label="TLA" lvar={`ENG_${n}_TLA_DEG`} unit="deg" decimals={1} />
          <DepthLamp label="Thrust lever disagree" lvar={`ENG_${n}_THR_LEVER_DISAGREE`} />

          <p className="mb-1 mt-2 text-xs font-bold uppercase text-theme-highlight">Hot section</p>
          <HotSectionLife engine={n} />

          <p className="mb-1 mt-2 text-xs font-bold uppercase text-theme-highlight">Ignition &amp; starting</p>
          <DepthLamp label="Ignition powered" lvar={`ENG_${n}_IGN_POWERED`} />
          <DepthLamp label="No ignition available" lvar={`ENG_${n}_NO_IGNITION_AVAILABLE`} />
          <DepthNum label="Start valve position" lvar={`ENG_${n}_START_VALVE_POSITION`} unit="" decimals={2} />
          <DepthLamp label="Start valve disagree" lvar={`ENG_${n}_START_VALVE_DISAGREE`} />
          <DepthNum label="Starter torque" lvar={`ENG_${n}_STARTER_TORQUE_NM`} unit="Nm" decimals={0} />
          <DepthNum label="Starter rotor speed" lvar={`ENG_${n}_STARTER_ROTOR_RPM`} unit="rpm" decimals={0} />
          <DepthLamp label="Starter disengage fault" lvar={`ENG_${n}_STARTER_DISENGAGE_FAULT`} />
          <DepthLamp label="Starter disintegrated" lvar={`ENG_${n}_STARTER_DISINTEGRATED`} />
          <DepthLamp label="Starter overheat" lvar={`ENG_${n}_STARTER_OVERHEAT`} />
          <DepthNum label="Starter housing temp rise" lvar={`ENG_${n}_STARTER_HOUSING_RISE_K`} unit="K" decimals={1} />

          <p className="mb-1 mt-2 text-xs font-bold uppercase text-theme-highlight">Variable geometry &amp; ice</p>
          <DepthNum label="VSV angle" lvar={`ENG_${n}_VSV_ANGLE_DEG`} unit="deg" decimals={1} />
          <DepthNum label="VSV schedule error" lvar={`ENG_${n}_VSV_SCHEDULE_ERROR_DEG`} unit="deg" decimals={2} />
          <DepthNum label="VSV stall margin delta" lvar={`ENG_${n}_VSV_STALL_MARGIN_DELTA_PCT`} unit="%" decimals={2} />
          <DepthNum label="Anti-ice position" lvar={`ENG_${n}_ANTI_ICE_POSITION`} unit="" decimals={2} />
          <DepthLamp label="Anti-ice disagree" lvar={`ENG_${n}_ANTI_ICE_DISAGREE`} />
          <DepthNum label="Anti-ice lip temp" lvar={`ENG_${n}_ANTI_ICE_LIP_TEMP_K`} unit="K" decimals={0} />
          <DepthNum label="Nacelle vent flow" lvar={`ENG_${n}_NACELLE_VENT_FLOW_KG_S`} unit="kg/s" decimals={3} />
          <DepthLamp label="Nacelle vapour risk" lvar={`ENG_${n}_NACELLE_VAPOUR_RISK`} />
        </DepthGroup>
      ))}
    </DepthGrid>
  </ScrollableContainer>
);

// Stand-in for FlyByWire's SimConnectInterface.h (fbw_a380/src/interface), so
// their Prim wrapper (fbw_a380/src/prim/Prim.cpp) compiles unchanged without
// the MSFS SDK.
//
// Prim::update only calls these client-data methods when one of its model
// parts is disabled from the MSFS debug configuration (the four `...Disabled`
// arguments, FlyByWireInterface.cpp:1712-1715 with PRIM_DISABLED etc. read
// from the INI at FlyByWireInterface.cpp:184-188, default off). This port
// always passes false for all four, so none of them is ever called; they are
// defined only so the wrapper links, and keep whatever they are handed in
// plain members the way client data would.
//
// build.rs copies Prim.cpp next to a copy of this file's directory, so the
// wrapper's `#include "../interface/SimConnectInterface.h"` finds this file
// before FlyByWire's.
#pragma once

#include "A380PrimComputerGeneralLogic_types.h"

class SimConnectInterface {
 public:
  bool setClientDataPrimDiscretes(base_prim_discrete_inputs& output) {
    discreteInputs = output;
    return true;
  }
  bool setClientDataPrimAnalog(base_prim_analog_inputs& output) {
    analogInputs = output;
    return true;
  }
  bool setClientDataFms(base_fms_inputs& output) {
    fms = output;
    return true;
  }
  bool setClientDataPrimGeneralLogicOutput(const base_prim_general_logic_outputs& output) {
    generalLogic = output;
    return true;
  }
  bool setClientDataPrimFlightEnvelopeOutput(const base_prim_flight_envelope_outputs& output) {
    flightEnvelope = output;
    return true;
  }
  bool setClientDataPrimFgLogicOutput(const base_prim_fg_logic_output& output) {
    fgLogic = output;
    return true;
  }
  bool setClientDataPrimFgModeLogicOutput(const base_prim_ap_fd_logic_outputs& output) {
    fgModeLogic = output;
    return true;
  }
  bool setClientDataPrimFgLawsOutput(const base_prim_fg_laws_outputs& output) {
    fgLaws = output;
    return true;
  }
  bool setClientDataPrimFctlLogicOutput(const base_prim_fctl_logic_outputs& output) {
    fctlLogic = output;
    return true;
  }

  base_prim_discrete_outputs& getClientDataPrimDiscretesOutput() { return discreteOutputs; }
  base_prim_analog_outputs& getClientDataPrimAnalogsOutput() { return analogOutputs; }
  base_prim_out_bus& getClientDataPrimBusOutput() { return busOutput; }
  base_prim_general_logic_outputs& getClientDataPrimGeneralLogicOutput() { return generalLogic; }
  base_prim_flight_envelope_outputs& getClientDataPrimFlightEnvelopeOutput() { return flightEnvelope; }
  base_prim_fg_logic_output& getClientDataPrimFgLogicOutput() { return fgLogic; }
  base_prim_ap_fd_logic_outputs& getClientDataPrimFgModeLogicOutput() { return fgModeLogic; }
  base_prim_fg_laws_outputs& getClientDataPrimFgLawsOutput() { return fgLaws; }
  base_prim_fctl_logic_outputs& getClientDataPrimFctlLogicOutput() { return fctlLogic; }

 private:
  base_prim_discrete_inputs discreteInputs{};
  base_prim_analog_inputs analogInputs{};
  base_fms_inputs fms{};
  base_prim_general_logic_outputs generalLogic{};
  base_prim_flight_envelope_outputs flightEnvelope{};
  base_prim_fg_logic_output fgLogic{};
  base_prim_ap_fd_logic_outputs fgModeLogic{};
  base_prim_fg_laws_outputs fgLaws{};
  base_prim_fctl_logic_outputs fctlLogic{};
  base_prim_discrete_outputs discreteOutputs{};
  base_prim_analog_outputs analogOutputs{};
  base_prim_out_bus busOutput{};
};

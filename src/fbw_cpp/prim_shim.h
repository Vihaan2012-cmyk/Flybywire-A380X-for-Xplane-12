// C ABI over FlyByWire's A380 PRIM, SEC and FCU computers, as their own
// wrappers run them: fbw_a380/src/prim/Prim.cpp, sec/Sec.cpp and fcu/Fcu.cpp,
// compiled unchanged (see build.rs) around the generated models
// A380PrimComputerGeneralLogic/Fe/Fg/Fctl, A380SecComputer and
// A380FcuComputer.
//
// Each wrapper owns its model's input bus; FlyByWireInterface writes into it
// field by field before calling update(). Here the whole input bus is copied
// in instead, except `sim_data.computer_running`, which the wrappers set
// themselves (Prim.cpp:150, Sec.cpp:46-60, Fcu.cpp:22) and which is left as the
// wrapper last wrote it. The structs are mirrored in src/fbw_types.rs; both
// sides assert the same layout (src/fbw_cpp/fbw_types_layout.h).
#ifndef FBW_XP_PRIM_SHIM_H_
#define FBW_XP_PRIM_SHIM_H_

#include <stdbool.h>
#include <stddef.h>

#include "A380FcuComputer_types.h"
#include "A380PrimComputerGeneralLogic_types.h"
#include "A380SecComputer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fbw_prim fbw_prim;
typedef struct fbw_sec fbw_sec;
typedef struct fbw_fcu fbw_fcu;

// unit: 0, 1 or 2 for PRIM 1, 2, 3 (Prim(isUnit1, isUnit2, isUnit3),
// FlyByWireInterface.h:132). NULL if allocation fails.
fbw_prim *fbw_prim_create(int unit);
void fbw_prim_destroy(fbw_prim *prim);
void fbw_prim_set_inputs(fbw_prim *prim, const prim_inputs *inputs);
// Prim::update with every model part enabled (FlyByWireInterface.cpp:1712).
void fbw_prim_update(fbw_prim *prim, double dt, double simulation_time, bool fault_active, bool is_powered);
void fbw_prim_bus_outputs(fbw_prim *prim, base_prim_out_bus *out);
void fbw_prim_discrete_outputs(fbw_prim *prim, base_prim_discrete_outputs *out);
void fbw_prim_analog_outputs(fbw_prim *prim, base_prim_analog_outputs *out);
// getDebugOutputs().fg_laws.ap_fd_1.flare_law (FlyByWireInterface.cpp:1980-1986).
void fbw_prim_flare_law(fbw_prim *prim, ap_raw_laws_flare *out);

// A few internal flags of the last step (getDebugOutputs()), for tests and
// diagnostics: [on_ground, engine_running, triple_adr_failure,
// triple_ir_failure, all_sfcc_lost, speed_scale_lost, is_master_prim,
// manual_spd_control_active, auto_spd_control_active, athr_engaged,
// ra_computation_data_ft, all_ra_failure, fd_1_engaged, athr_active,
// alpha_floor_condition, v_ias_kn].
#define FBW_PRIM_DIAGNOSTICS 16
void fbw_prim_diagnostics(fbw_prim *prim, double *out);

fbw_sec *fbw_sec_create(int unit);
void fbw_sec_destroy(fbw_sec *sec);
void fbw_sec_set_inputs(fbw_sec *sec, const sec_inputs *inputs);
void fbw_sec_update(fbw_sec *sec, double dt, double simulation_time, bool fault_active, bool is_powered);
void fbw_sec_bus_outputs(fbw_sec *sec, base_sec_out_bus *out);
void fbw_sec_discrete_outputs(fbw_sec *sec, base_sec_discrete_outputs *out);
void fbw_sec_analog_outputs(fbw_sec *sec, base_sec_analog_outputs *out);

fbw_fcu *fbw_fcu_create(void);
void fbw_fcu_destroy(fbw_fcu *fcu);
void fbw_fcu_set_inputs(fbw_fcu *fcu, const fcu_inputs *inputs);
void fbw_fcu_update(fbw_fcu *fcu, double dt, double simulation_time, bool fault_active, bool is_powered);
void fbw_fcu_bus_outputs(fbw_fcu *fcu, base_fcu_bus *out);
// Fcu::getDiscreteOutputs, which also reads the current input bus
// (efis_backup_activated, Fcu.cpp:77).
void fbw_fcu_discrete_outputs(fbw_fcu *fcu, base_fcu_discrete_outputs *out);

// sizeof checks for the Rust side, as this compiler laid the structs out.
size_t fbw_prim_inputs_size(void);
size_t fbw_sec_inputs_size(void);
size_t fbw_fcu_inputs_size(void);

#ifdef __cplusplus
}
#endif

#endif

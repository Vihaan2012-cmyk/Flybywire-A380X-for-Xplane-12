// C ABI over FlyByWire's generated A380 FADEC computer. See shim.h.
//
// Built without exceptions and RTTI and without linking libstdc++: the model
// needs nothing from the C++ runtime but <cmath>, and the instance is placed
// into malloc'd memory, so only the C runtime Rust already links is used.
#include "shim.h"

#include <cstddef>
#include <cstdlib>
#include <new>

#include "A380FadecComputer.h"

struct fbw_fadec {
  A380FadecComputer model;
};

// Layout contract with src/fbw_controllers.rs (same numbers asserted there).
static_assert(sizeof(boolean_T) == 1, "boolean_T");
static_assert(sizeof(athr_thrust_limit_type) == 4, "athr_thrust_limit_type");
static_assert(sizeof(base_arinc_429) == 8 && alignof(base_arinc_429) == 4, "base_arinc_429");
static_assert(sizeof(base_prim_fctl_out_bus) == 456, "base_prim_fctl_out_bus");
static_assert(sizeof(base_prim_fe_out_bus) == 96, "base_prim_fe_out_bus");
static_assert(sizeof(base_prim_fg_out_bus) == 264, "base_prim_fg_out_bus");
static_assert(sizeof(base_prim_out_bus) == 816, "base_prim_out_bus");
static_assert(offsetof(base_prim_out_bus, fe) == 456 && offsetof(base_prim_out_bus, fg) == 552, "prim bus");
static_assert(offsetof(base_prim_fg_out_bus, ats_discrete_word) == 160, "fg.ats_discrete_word");
static_assert(offsetof(base_prim_fg_out_bus, n1_command_percent) == 240, "fg.n1_command_percent");
static_assert(offsetof(base_prim_fg_out_bus, flx_to_temp_deg_c) == 248, "fg.flx_to_temp_deg_c");
static_assert(offsetof(base_prim_fctl_out_bus, fctl_law_status_word) == 416, "fctl.fctl_law_status_word");
static_assert(sizeof(athr_time) == 16, "athr_time");
static_assert(sizeof(athr_data) == 152 && alignof(athr_data) == 8, "athr_data");
static_assert(offsetof(athr_data, on_ground) == 72, "athr_data.on_ground");
static_assert(offsetof(athr_data, flap_handle_index) == 80, "athr_data.flap_handle_index");
static_assert(offsetof(athr_data, is_engine_operative) == 88, "athr_data.is_engine_operative");
static_assert(offsetof(athr_data, commanded_engine_N1_percent) == 96, "athr_data.commanded_engine_N1_percent");
static_assert(offsetof(athr_data, ambient_density_kg_per_m3) == 144, "athr_data.ambient_density_kg_per_m3");
static_assert(sizeof(athr_input) == 72, "athr_input");
static_assert(offsetof(athr_input, TLA_deg) == 8, "athr_input.TLA_deg");
static_assert(offsetof(athr_input, is_anti_ice_active) == 64, "athr_input.is_anti_ice_active");
static_assert(offsetof(athr_input, tracking_mode_on_override) == 67, "athr_input.tracking_mode_on_override");
static_assert(sizeof(athr_in) == 2688 && alignof(athr_in) == 8, "athr_in");
static_assert(offsetof(athr_in, data) == 16 && offsetof(athr_in, input) == 168, "athr_in head");
static_assert(offsetof(athr_in, prim_1) == 240 && offsetof(athr_in, prim_2) == 1056 &&
                  offsetof(athr_in, prim_3) == 1872,
              "athr_in prims");
static_assert(sizeof(athr_data_computed) == 16, "athr_data_computed");
static_assert(sizeof(base_eec) == 104, "base_eec");
static_assert(sizeof(athr_output) == 56, "athr_output");
static_assert(offsetof(athr_output, is_in_reverse) == 24 && offsetof(athr_output, thrust_limit_type) == 28 &&
                  offsetof(athr_output, thrust_limit_percent) == 32 && offsetof(athr_output, N1_c_percent) == 40 &&
                  offsetof(athr_output, athr_control_active) == 48 && offsetof(athr_output, memo_thrust_active) == 49,
              "athr_output fields");
static_assert(sizeof(athr_out) == 1232 && alignof(athr_out) == 8, "athr_out");
static_assert(offsetof(athr_out, data_computed) == 168 && offsetof(athr_out, input) == 184 &&
                  offsetof(athr_out, prim_input) == 256 && offsetof(athr_out, output) == 1072 &&
                  offsetof(athr_out, fadec_bus_output) == 1128,
              "athr_out fields");
static_assert(sizeof(A380FadecComputer::ExternalInputs_A380FadecComputer_T) == sizeof(athr_in), "ExternalInputs");
static_assert(sizeof(A380FadecComputer::ExternalOutputs_A380FadecComputer_T) == sizeof(athr_out), "ExternalOutputs");

extern "C" {

fbw_fadec *fbw_fadec_create(void) {
  void *memory = std::malloc(sizeof(fbw_fadec));
  if (memory == nullptr) {
    return nullptr;
  }
  fbw_fadec *fadec = new (memory) fbw_fadec{};
  // FlyByWireInterface never calls initialize() on its FADECs; the
  // value-initialised work vector it relies on holds the same zeros and
  // falses initialize() writes, so either way the start state is identical.
  fadec->model.initialize();
  return fadec;
}

void fbw_fadec_destroy(fbw_fadec *fadec) {
  if (fadec == nullptr) {
    return;
  }
  fadec->~fbw_fadec();
  std::free(fadec);
}

void fbw_fadec_set_inputs(fbw_fadec *fadec, const athr_in *inputs) {
  A380FadecComputer::ExternalInputs_A380FadecComputer_T external{};
  external.in = *inputs;
  fadec->model.setExternalInputs(&external);
}

void fbw_fadec_step(fbw_fadec *fadec) {
  fadec->model.step();
}

void fbw_fadec_get_outputs(const fbw_fadec *fadec, athr_out *outputs) {
  *outputs = fadec->model.getExternalOutputs().out;
}

size_t fbw_fadec_athr_in_size(void) {
  return sizeof(athr_in);
}

size_t fbw_fadec_athr_out_size(void) {
  return sizeof(athr_out);
}
}

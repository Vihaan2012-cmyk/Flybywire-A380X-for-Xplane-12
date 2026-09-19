// C ABI over FlyByWire's PRIM, SEC and FCU wrappers. See prim_shim.h.
//
// FlyByWire holds these computers in a global FlyByWireInterface
// (main.cpp:7), so every member their constructors leave unset (the
// self-test timers, power-outage timers, monitoring flags) starts at zero.
// The instances here are placed into calloc'd memory to start from the same
// zeros; build.rs compiles with -fno-lifetime-dse so the compiler cannot drop
// that zeroing on the grounds that a constructor follows.
#include "prim_shim.h"

#include <cstddef>
#include <cstdlib>
#include <new>

#include "fcu/Fcu.h"
#include "prim/Prim.h"
#include "sec/Sec.h"

#include "fbw_types_layout.h"

struct fbw_prim {
  Prim prim;
  SimConnectInterface client_data;
};

struct fbw_sec {
  Sec sec;
};

struct fbw_fcu {
  Fcu fcu;
};

namespace {

void *zeroed(size_t size) {
  return std::calloc(1, size);
}

}  // namespace

extern "C" {

fbw_prim *fbw_prim_create(int unit) {
  void *memory = zeroed(sizeof(fbw_prim));
  if (memory == nullptr) {
    return nullptr;
  }
  auto *p = static_cast<fbw_prim *>(memory);
  new (&p->prim) Prim(unit == 0, unit == 1, unit == 2);
  new (&p->client_data) SimConnectInterface();
  return p;
}

void fbw_prim_destroy(fbw_prim *p) {
  if (p == nullptr) {
    return;
  }
  p->client_data.~SimConnectInterface();
  p->prim.~Prim();
  std::free(p);
}

void fbw_prim_set_inputs(fbw_prim *p, const prim_inputs *inputs) {
  auto &in = p->prim.externalInputs().in;
  const boolean_T running = in.sim_data.computer_running;
  in = *inputs;
  in.sim_data.computer_running = running;
}

void fbw_prim_update(fbw_prim *p, double dt, double simulation_time, bool fault_active, bool is_powered) {
  p->prim.update(dt, simulation_time, fault_active, is_powered, p->client_data, false, false, false, false);
}

void fbw_prim_bus_outputs(fbw_prim *p, base_prim_out_bus *out) {
  *out = p->prim.getBusOutputs();
}

void fbw_prim_discrete_outputs(fbw_prim *p, base_prim_discrete_outputs *out) {
  *out = p->prim.getDiscreteOutputs();
}

void fbw_prim_analog_outputs(fbw_prim *p, base_prim_analog_outputs *out) {
  *out = p->prim.getAnalogOutputs();
}

void fbw_prim_flare_law(fbw_prim *p, ap_raw_laws_flare *out) {
  *out = p->prim.getDebugOutputs().fg_laws.ap_fd_1.flare_law;
}

void fbw_prim_diagnostics(fbw_prim *p, double *out) {
  const prim_outputs &o = p->prim.getDebugOutputs();
  const double values[FBW_PRIM_DIAGNOSTICS] = {
      static_cast<double>(o.general_logic.on_ground),
      static_cast<double>(o.general_logic.engine_running),
      static_cast<double>(o.general_logic.triple_adr_failure),
      static_cast<double>(o.general_logic.triple_ir_failure),
      static_cast<double>(o.general_logic.all_sfcc_lost),
      static_cast<double>(o.flight_envelope.speed_scale_lost),
      static_cast<double>(o.fctl_logic.is_master_prim),
      static_cast<double>(o.fg_mode_logic.manual_spd_control_active),
      static_cast<double>(o.fg_mode_logic.auto_spd_control_active),
      static_cast<double>(o.fg_logic.athr_engaged),
      static_cast<double>(o.general_logic.ra_computation_data_ft),
      static_cast<double>(o.general_logic.all_ra_failure),
      static_cast<double>(o.fg_logic.fd_1_engaged),
      static_cast<double>(o.fg_mode_logic.athr_active),
      static_cast<double>(o.flight_envelope.alpha_floor_condition),
      static_cast<double>(o.general_logic.adr_computation_data.V_ias_kn),
  };
  for (int i = 0; i < FBW_PRIM_DIAGNOSTICS; i++) {
    out[i] = values[i];
  }
}

fbw_sec *fbw_sec_create(int unit) {
  void *memory = zeroed(sizeof(fbw_sec));
  if (memory == nullptr) {
    return nullptr;
  }
  auto *s = static_cast<fbw_sec *>(memory);
  new (&s->sec) Sec(unit == 0, unit == 1, unit == 2);
  return s;
}

void fbw_sec_destroy(fbw_sec *s) {
  if (s == nullptr) {
    return;
  }
  s->sec.~Sec();
  std::free(s);
}

void fbw_sec_set_inputs(fbw_sec *s, const sec_inputs *inputs) {
  auto &in = s->sec.modelInputs.in;
  const boolean_T running = in.sim_data.computer_running;
  in = *inputs;
  in.sim_data.computer_running = running;
}

void fbw_sec_update(fbw_sec *s, double dt, double simulation_time, bool fault_active, bool is_powered) {
  s->sec.update(dt, simulation_time, fault_active, is_powered);
}

void fbw_sec_bus_outputs(fbw_sec *s, base_sec_out_bus *out) {
  *out = s->sec.getBusOutputs();
}

void fbw_sec_discrete_outputs(fbw_sec *s, base_sec_discrete_outputs *out) {
  *out = s->sec.getDiscreteOutputs();
}

void fbw_sec_analog_outputs(fbw_sec *s, base_sec_analog_outputs *out) {
  *out = s->sec.getAnalogOutputs();
}

fbw_fcu *fbw_fcu_create(void) {
  void *memory = zeroed(sizeof(fbw_fcu));
  if (memory == nullptr) {
    return nullptr;
  }
  auto *f = static_cast<fbw_fcu *>(memory);
  new (&f->fcu) Fcu();
  return f;
}

void fbw_fcu_destroy(fbw_fcu *f) {
  if (f == nullptr) {
    return;
  }
  f->fcu.~Fcu();
  std::free(f);
}

void fbw_fcu_set_inputs(fbw_fcu *f, const fcu_inputs *inputs) {
  auto &in = f->fcu.modelInputs.in;
  const boolean_T running = in.sim_data.computer_running;
  in = *inputs;
  in.sim_data.computer_running = running;
}

void fbw_fcu_update(fbw_fcu *f, double dt, double simulation_time, bool fault_active, bool is_powered) {
  f->fcu.update(dt, simulation_time, fault_active, is_powered);
}

void fbw_fcu_bus_outputs(fbw_fcu *f, base_fcu_bus *out) {
  *out = f->fcu.getBusOutputs();
}

void fbw_fcu_discrete_outputs(fbw_fcu *f, base_fcu_discrete_outputs *out) {
  *out = f->fcu.getDiscreteOutputs();
}

size_t fbw_prim_inputs_size(void) {
  return sizeof(prim_inputs);
}

size_t fbw_sec_inputs_size(void) {
  return sizeof(sec_inputs);
}

size_t fbw_fcu_inputs_size(void) {
  return sizeof(fcu_inputs);
}
}

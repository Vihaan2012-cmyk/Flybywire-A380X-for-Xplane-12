// C ABI over FlyByWire's generated A380 FADEC computer
// (fbw-a380x/src/wasm/fbw_a380/src/model/A380FadecComputer.*).
//
// The model's own bus structs (athr_in, athr_out from
// A380FadecComputer_types.h) cross the boundary unchanged. They are plain
// C-layout structs of doubles, floats, uint32, unsigned char and int32 enums;
// src/fbw_controllers.rs mirrors them field for field and both sides assert
// the same sizes and offsets.
#ifndef FBW_XP_SHIM_H_
#define FBW_XP_SHIM_H_

#include <stddef.h>

#include "A380FadecComputer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fbw_fadec fbw_fadec;

// Allocates one FADEC computer (one per engine in FlyByWire's build) and runs
// the model's initialize(). Returns NULL if allocation fails.
fbw_fadec *fbw_fadec_create(void);
void fbw_fadec_destroy(fbw_fadec *fadec);

// Copies the whole input bus, as FlyByWireInterface::updateFadec does with
// setExternalInputs().
void fbw_fadec_set_inputs(fbw_fadec *fadec, const athr_in *inputs);

// One model step. The model is variable-step: in.time.dt carries the step.
void fbw_fadec_step(fbw_fadec *fadec);

// Copies the whole output bus (getExternalOutputs().out).
void fbw_fadec_get_outputs(const fbw_fadec *fadec, athr_out *outputs);

// sizeof(athr_in) / sizeof(athr_out) as this compiler laid them out.
size_t fbw_fadec_athr_in_size(void);
size_t fbw_fadec_athr_out_size(void);

#ifdef __cplusplus
}
#endif

#endif

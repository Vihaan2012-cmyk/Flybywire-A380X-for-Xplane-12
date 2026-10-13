//! FlyByWire's A380 PRIM, SEC and FCU computers, compiled from their C++.
//!
//! Each is FlyByWire's own wrapper class (`fbw_a380/src/prim/Prim.cpp`,
//! `sec/Sec.cpp`, `fcu/Fcu.cpp`) around the Simulink models, built unchanged
//! by `build.rs` and reached through `src/fbw_cpp/prim_shim.cpp`. Nothing of
//! the control laws is re-derived here. The bus structs are the exact mirrors
//! in [`crate::fbw_types`].
//!
//! Each call follows what FlyByWireInterface does: fill the model's input bus
//! (`externalInputs()` / `modelInputs`), `update(...)`, then read
//! `getBusOutputs()`, `getDiscreteOutputs()`, `getAnalogOutputs()`.

#![allow(dead_code)]

use std::mem::size_of;

use crate::fbw_types::*;

#[repr(C)]
struct RawPrim {
    _private: [u8; 0],
}
#[repr(C)]
struct RawSec {
    _private: [u8; 0],
}
#[repr(C)]
struct RawFcu {
    _private: [u8; 0],
}

#[link(name = "fbw_controllers", kind = "static")]
extern "C" {
    fn fbw_prim_create(unit: i32) -> *mut RawPrim;
    fn fbw_prim_destroy(prim: *mut RawPrim);
    fn fbw_prim_set_inputs(prim: *mut RawPrim, inputs: *const PrimInputs);
    fn fbw_prim_update(prim: *mut RawPrim, dt: f64, simulation_time: f64, fault_active: bool, is_powered: bool);
    fn fbw_prim_bus_outputs(prim: *mut RawPrim, out: *mut BasePrimOutBus);
    fn fbw_prim_discrete_outputs(prim: *mut RawPrim, out: *mut BasePrimDiscreteOutputs);
    fn fbw_prim_analog_outputs(prim: *mut RawPrim, out: *mut BasePrimAnalogOutputs);
    fn fbw_prim_flare_law(prim: *mut RawPrim, out: *mut ApRawLawsFlare);
    fn fbw_prim_fctl_logic_outputs(prim: *mut RawPrim, out: *mut BasePrimFctlLogicOutputs);
    fn fbw_prim_diagnostics(prim: *mut RawPrim, out: *mut f64);
    fn fbw_prim_health_diagnostics(prim: *mut RawPrim, out: *mut bool);

    fn fbw_sec_create(unit: i32) -> *mut RawSec;
    fn fbw_sec_destroy(sec: *mut RawSec);
    fn fbw_sec_set_inputs(sec: *mut RawSec, inputs: *const SecInputs);
    fn fbw_sec_update(sec: *mut RawSec, dt: f64, simulation_time: f64, fault_active: bool, is_powered: bool);
    fn fbw_sec_bus_outputs(sec: *mut RawSec, out: *mut BaseSecOutBus);
    fn fbw_sec_discrete_outputs(sec: *mut RawSec, out: *mut BaseSecDiscreteOutputs);
    fn fbw_sec_analog_outputs(sec: *mut RawSec, out: *mut BaseSecAnalogOutputs);

    fn fbw_fcu_create() -> *mut RawFcu;
    fn fbw_fcu_destroy(fcu: *mut RawFcu);
    fn fbw_fcu_set_inputs(fcu: *mut RawFcu, inputs: *const FcuInputs);
    fn fbw_fcu_update(fcu: *mut RawFcu, dt: f64, simulation_time: f64, fault_active: bool, is_powered: bool);
    fn fbw_fcu_bus_outputs(fcu: *mut RawFcu, out: *mut BaseFcuBus);
    fn fbw_fcu_discrete_outputs(fcu: *mut RawFcu, out: *mut BaseFcuDiscreteOutputs);

    fn fbw_prim_inputs_size() -> usize;
    fn fbw_sec_inputs_size() -> usize;
    fn fbw_fcu_inputs_size() -> usize;
}

fn check_layout() {
    unsafe {
        assert_eq!(fbw_prim_inputs_size(), size_of::<PrimInputs>());
        assert_eq!(fbw_sec_inputs_size(), size_of::<SecInputs>());
        assert_eq!(fbw_fcu_inputs_size(), size_of::<FcuInputs>());
    }
}

/// One PRIM (`Prim` in FlyByWireInterface.h:132): general logic, flight
/// envelope, flight guidance and flight controls.
pub struct PrimComputer {
    raw: *mut RawPrim,
}

unsafe impl Send for PrimComputer {}

impl PrimComputer {
    /// `index` 0, 1, 2 for PRIM 1, 2, 3.
    pub fn new(index: usize) -> Self {
        check_layout();
        assert!(index < 3);
        let raw = unsafe { fbw_prim_create(index as i32) };
        assert!(!raw.is_null(), "could not allocate a PRIM");
        Self { raw }
    }

    /// Copies the input bus into `externalInputs().in` (all of it but
    /// `sim_data.computer_running`, which Prim::update sets itself).
    pub fn set_inputs(&mut self, inputs: &PrimInputs) {
        unsafe { fbw_prim_set_inputs(self.raw, inputs) }
    }

    /// `Prim::update` with every model part enabled.
    pub fn update(&mut self, dt: f64, simulation_time: f64, fault_active: bool, is_powered: bool) {
        unsafe { fbw_prim_update(self.raw, dt, simulation_time, fault_active, is_powered) }
    }

    pub fn bus_outputs(&mut self) -> BasePrimOutBus {
        let mut out = BasePrimOutBus::default();
        unsafe { fbw_prim_bus_outputs(self.raw, &mut out) };
        out
    }

    pub fn discrete_outputs(&mut self) -> BasePrimDiscreteOutputs {
        let mut out = BasePrimDiscreteOutputs::default();
        unsafe { fbw_prim_discrete_outputs(self.raw, &mut out) };
        out
    }

    pub fn analog_outputs(&mut self) -> BasePrimAnalogOutputs {
        let mut out = BasePrimAnalogOutputs::default();
        unsafe { fbw_prim_analog_outputs(self.raw, &mut out) };
        out
    }

    pub fn flare_law(&mut self) -> ApRawLawsFlare {
        let mut out = ApRawLawsFlare::default();
        unsafe { fbw_prim_flare_law(self.raw, &mut out) };
        out
    }

    /// `getDebugOutputs().fctl_logic`: the real per-channel avail/engaged
    /// discretes, aileron droop/anti-droop and sidestick disabled/priority-
    /// locked bits this PRIM instance's compiled Simulink already computes
    /// every tick (E-FCTL, ECAM completeness pass; see `prim_shim.h`'s own
    /// doc comment on `fbw_prim_fctl_logic_outputs`).
    pub fn fctl_logic_outputs(&mut self) -> BasePrimFctlLogicOutputs {
        let mut out = BasePrimFctlLogicOutputs::default();
        unsafe { fbw_prim_fctl_logic_outputs(self.raw, &mut out) };
        out
    }
}

/// Internal flags of a PRIM's last step (`fbw_prim_diagnostics`).
#[derive(Clone, Copy, Debug, Default)]
pub struct PrimDiagnostics {
    pub on_ground: bool,
    pub engine_running: bool,
    pub triple_adr_failure: bool,
    pub triple_ir_failure: bool,
    pub all_sfcc_lost: bool,
    pub speed_scale_lost: bool,
    pub is_master_prim: bool,
    pub manual_spd_control_active: bool,
    pub auto_spd_control_active: bool,
    pub athr_engaged: bool,
    pub ra_ft: f64,
    pub all_ra_failure: bool,
    pub fd_1_engaged: bool,
    pub athr_active: bool,
    pub alpha_floor_condition: bool,
    pub v_ias_kn: f64,
}

impl PrimComputer {
    pub fn diagnostics(&mut self) -> PrimDiagnostics {
        let mut v = [0f64; 16];
        unsafe { fbw_prim_diagnostics(self.raw, v.as_mut_ptr()) };
        let b = |x: f64| x != 0.;
        PrimDiagnostics {
            on_ground: b(v[0]),
            engine_running: b(v[1]),
            triple_adr_failure: b(v[2]),
            triple_ir_failure: b(v[3]),
            all_sfcc_lost: b(v[4]),
            speed_scale_lost: b(v[5]),
            is_master_prim: b(v[6]),
            manual_spd_control_active: b(v[7]),
            auto_spd_control_active: b(v[8]),
            athr_engaged: b(v[9]),
            ra_ft: v[10],
            all_ra_failure: b(v[11]),
            fd_1_engaged: b(v[12]),
            athr_active: b(v[13]),
            alpha_floor_condition: b(v[14]),
            v_ias_kn: v[15],
        }
    }
}

/// Self-monitoring state behind `prim_healthy` (`fbw_prim_health_diagnostics`,
/// Prim.h `isSelfTestInProgress`/`isMonitoringHealthy`/`isPowerSupplyFault`).
/// None of `PrimDiagnostics` above (triple ADR/IR/SFCC/RA loss, speed-scale-
/// lost) feeds `prim_healthy`; this struct is what actually does (W104).
#[derive(Clone, Copy, Debug, Default)]
pub struct PrimHealthDiagnostics {
    /// The self test after a power interruption/button press hasn't finished
    /// yet: `prim_healthy` legitimately toggles with the FAULT test-light
    /// blink pattern during this window, which is not itself a fault.
    pub self_test_in_progress: bool,
    pub monitoring_healthy: bool,
    pub power_supply_fault: bool,
}

impl PrimComputer {
    pub fn health_diagnostics(&mut self) -> PrimHealthDiagnostics {
        let mut v = [false; 3];
        unsafe { fbw_prim_health_diagnostics(self.raw, v.as_mut_ptr()) };
        PrimHealthDiagnostics { self_test_in_progress: v[0], monitoring_healthy: v[1], power_supply_fault: v[2] }
    }
}

impl Drop for PrimComputer {
    fn drop(&mut self) {
        unsafe { fbw_prim_destroy(self.raw) }
    }
}

/// One SEC (`Sec` in FlyByWireInterface.h:137).
pub struct SecComputer {
    raw: *mut RawSec,
}

unsafe impl Send for SecComputer {}

impl SecComputer {
    pub fn new(index: usize) -> Self {
        check_layout();
        assert!(index < 3);
        let raw = unsafe { fbw_sec_create(index as i32) };
        assert!(!raw.is_null(), "could not allocate a SEC");
        Self { raw }
    }

    pub fn set_inputs(&mut self, inputs: &SecInputs) {
        unsafe { fbw_sec_set_inputs(self.raw, inputs) }
    }

    pub fn update(&mut self, dt: f64, simulation_time: f64, fault_active: bool, is_powered: bool) {
        unsafe { fbw_sec_update(self.raw, dt, simulation_time, fault_active, is_powered) }
    }

    pub fn bus_outputs(&mut self) -> BaseSecOutBus {
        let mut out = BaseSecOutBus::default();
        unsafe { fbw_sec_bus_outputs(self.raw, &mut out) };
        out
    }

    pub fn discrete_outputs(&mut self) -> BaseSecDiscreteOutputs {
        let mut out = BaseSecDiscreteOutputs::default();
        unsafe { fbw_sec_discrete_outputs(self.raw, &mut out) };
        out
    }

    pub fn analog_outputs(&mut self) -> BaseSecAnalogOutputs {
        let mut out = BaseSecAnalogOutputs::default();
        unsafe { fbw_sec_analog_outputs(self.raw, &mut out) };
        out
    }
}

impl Drop for SecComputer {
    fn drop(&mut self) {
        unsafe { fbw_sec_destroy(self.raw) }
    }
}

/// One FCU channel (`Fcu` in FlyByWireInterface.h:146).
pub struct FcuComputer {
    raw: *mut RawFcu,
}

unsafe impl Send for FcuComputer {}

impl FcuComputer {
    pub fn new() -> Self {
        check_layout();
        let raw = unsafe { fbw_fcu_create() };
        assert!(!raw.is_null(), "could not allocate an FCU");
        Self { raw }
    }

    pub fn set_inputs(&mut self, inputs: &FcuInputs) {
        unsafe { fbw_fcu_set_inputs(self.raw, inputs) }
    }

    pub fn update(&mut self, dt: f64, simulation_time: f64, fault_active: bool, is_powered: bool) {
        unsafe { fbw_fcu_update(self.raw, dt, simulation_time, fault_active, is_powered) }
    }

    pub fn bus_outputs(&mut self) -> BaseFcuBus {
        let mut out = BaseFcuBus::default();
        unsafe { fbw_fcu_bus_outputs(self.raw, &mut out) };
        out
    }

    /// `Fcu::getDiscreteOutputs`; reads the last update's model outputs and
    /// the input bus as it is now.
    pub fn discrete_outputs(&mut self) -> BaseFcuDiscreteOutputs {
        let mut out = BaseFcuDiscreteOutputs::default();
        unsafe { fbw_fcu_discrete_outputs(self.raw, &mut out) };
        out
    }
}

impl Default for FcuComputer {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for FcuComputer {
    fn drop(&mut self) {
        unsafe { fbw_fcu_destroy(self.raw) }
    }
}

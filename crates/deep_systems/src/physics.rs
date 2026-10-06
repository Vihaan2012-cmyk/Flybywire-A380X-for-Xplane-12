//! The plugin physics the areas name.

/// Jet A density, the plugin's own file (no dependencies).
#[path = "../../../src/physics/fluids.rs"]
pub mod fluids;

/// The ideal gas law's constants, the plugin's own file.
#[path = "../../../src/physics/gas.rs"]
pub mod gas;

/// The per-wheel tyre model (`physics/tyre_model.rs`, which the plugin's
/// `tyre` re-exports), under the name the areas use.
#[path = "../../../src/physics/tyre_model.rs"]
pub mod tyre;

/// `physics/damage.rs`'s fuse-plug melt point, which the tyre model reads.
pub mod damage {
    pub const FUSE_PLUG_MELT_C: f64 = 177.0;
}

/// `physics/engine/oil.rs`'s unit.
pub mod engine {
    pub mod oil {
        pub const PSI_PA: f64 = 6894.757;
    }
}

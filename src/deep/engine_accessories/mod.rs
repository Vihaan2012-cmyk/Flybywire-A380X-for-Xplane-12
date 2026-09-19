//! Engine accessories: the systems that hang off the Trent 972B-84 gas path
//! being rebuilt in `physics::engine` (read-only reference from this
//! directory, never edited here) -- fuel, ignition, starting, compressor
//! airflow control, rotor dynamics, the thrust reverser, the EEC, and the
//! nacelle. Each is self-contained; `registry.rs` is the only file that
//! reaches outside this directory (into `deep::api`), to declare every
//! failure, component and ECAM alert this area owns.

pub mod airflow_control;
pub mod eec;
pub mod fuel;
pub mod ignition;
pub mod nacelle;
pub mod registry;
pub mod rotor_dynamics;
pub mod starting;
pub mod thrust_reverser;

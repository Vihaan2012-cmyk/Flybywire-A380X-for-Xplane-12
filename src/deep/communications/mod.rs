//! ATA 23 -- Communications: CIDS (the Cabin Intercommunication Data
//! System), the three cockpit PTT switches, the ATSU/datalink router, and
//! the HF/SATCOM/VHF transceivers' own LRU faults. New with the ECAM-
//! completeness pass (`E:/fbw-debug/ecam/E-AIR-DESIGN.md`'s ATA 23 section).
//!
//! No `deep::` area models comms radios before this: this plugin's own
//! `src/radios.rs` already models VHF/COM frequency tuning and the RMP's
//! intercept behaviour (real, not a stub) but not the radios' own LRU
//! fault/stuck-transmitting states, and (per the design sheet) extending it
//! directly would cross this area's self-containment boundary for little
//! benefit, so the VHF stuck-emitting/datalink faults below are modelled as
//! their own components here instead, referencing `radios.rs`'s real
//! tuning model only in doc comments. Every one of these 18 ids names a
//! real, physical LRU on the aircraft (PTT switch, HF/VHF/SATCOM
//! transceiver, CIDS computer); every failure below is a binary component-
//! broken flag, no threshold invented, the same shape `fbw/ata24.rs`'s
//! `ELEC_GEN_n_FAULT` uses.

pub mod live;
pub mod registry;

//! Rotor dynamics: synchronous vibration from rotor imbalance (blade loss,
//! ice, bird strike) per spool, and the distinct frequency signature a
//! damaged rolling-element bearing rings at. Both feed the same kind of
//! N1/N2/N3 vibration indication a real EICAS/ECAM vibration page shows,
//! from two physically different causes with two physically different
//! diagnostic signatures (one tone at 1x shaft speed vs. four tones at
//! fixed multiples of it).

pub mod bearing;
pub mod bearings;
pub mod imbalance;

//! Oxygen (ATA 35): the flight crew's high-pressure gaseous supply, the
//! passengers' chemical generators, and the cabin's first-aid bottle.
//!
//! Before this area, nothing in the aeroplane modelled oxygen at all as a
//! deep system. `deep::sensors` had bottle-pressure transducers registered
//! against a variable nobody published; `deep::breakers` and
//! `deep::electrical` carried the crew shutoff valve, the generator
//! control circuit and the transducer supply as real loads with nothing
//! on the other end of them. This closes that.
//!
//! ## Three supplies, three different machines
//!
//! The one design decision worth stating up front is that these are *not*
//! three copies of one model:
//!
//! * **Crew** ([`crew`]) is a high-pressure gaseous cylinder group behind
//!   a shutoff valve and a pressure reducer, feeding diluter-demand mask
//!   regulators. It has a pressure, a temperature, a quantity, a gauge and
//!   a shutoff. Its state is continuous and reversible: it can be
//!   serviced, and it is drawn on in proportion to what the crew breathe.
//! * **Passengers** ([`pax`]) is a couple of hundred one-shot sodium
//!   chlorate candles ([`generator`]). There is no pressure anywhere in
//!   it, no quantity gauge, no shutoff and no way back -- lighting one is
//!   irreversible and its output is set by chemistry rather than by
//!   demand. Modelling it as a bottle with a different capacity would get
//!   every one of those wrong, and would miss the thing that actually
//!   matters about it: the tens of kilowatts of exothermic chemistry it
//!   puts into the cabin when it runs.
//! * **First aid** ([`therapeutic`]) is a third machine again: a gaseous
//!   cylinder, but on *continuous* flow rather than demand, and sized by
//!   an operating regulation rather than by a cylinder catalogue.
//!
//! [`cylinder`], [`gas`] and [`regulator`] are what the first and third
//! share; nothing at all is shared with the second beyond the gas
//! constants.
//!
//! ## The thing the gauges do
//!
//! A bottle gauge reads a pressure, and pressure is a function of mass
//! *and* temperature. Every cylinder here therefore carries a real
//! two-node thermal model -- the gas and the steel around it -- so that a
//! cold-soaked full bottle reads several hundred psi low without being
//! faulty, a bottle being discharged sags further than its remaining
//! contents justify and then recovers over the next few minutes, and a
//! bottle in a bay fire relieves through its burst disc because the
//! pressure genuinely got there. All of that falls out of the equation of
//! state; none of it is scripted. The temperature-corrected reading is
//! published beside the raw one, and the low-pressure caution uses the
//! corrected one, because that is the distinction a dispatch chart exists
//! to make.
//!
//! ## Files
//!
//! | file | what is in it |
//! |---|---|
//! | [`gas`] | oxygen as a real gas: van der Waals, choked orifices, blowdown cooling, the standard atmosphere |
//! | [`cylinder`] | a charged cylinder group: mass, two-node thermal state, leaks, the overpressure discharge disc |
//! | [`regulator`] | the pressure reducer, and the diluter-demand schedule solved from the alveolar gas equation |
//! | [`crew`] | 35-10: cylinder, shutoff valve, reducer, distribution, four mask regulators |
//! | [`generator`] | one chemical oxygen generator, its chemistry, its burn and its case temperature |
//! | [`pax`] | 35-20: two decks of generators, the deployment logic, the cabin heat load |
//! | [`therapeutic`] | 35-30: the first-aid cylinder and its continuous-flow outlets |
//! | [`registry`] | 14 components, 21 failures, 3 ECAM alerts |
//! | [`live`] | the [`crate::deep::live::Area`] implementation |
//!
//! Every constant in this directory is either sourced in the module that
//! declares it or labelled **GENERIC** with how it was derived, per
//! `docs/deep/BRIEF.md` hard rule 3. Nothing here depends on crate
//! internals (hard rule 2): this area re-derives the gas physics it needs
//! rather than reaching for `crate::physics::gas`, and reads the rest of
//! the aeroplane only through `Truth` and the previous frame's published
//! variables.

pub mod crew;
pub mod cylinder;
pub mod gas;
pub mod generator;
pub mod live;
pub mod pax;
pub mod regulator;
pub mod registry;
pub mod therapeutic;

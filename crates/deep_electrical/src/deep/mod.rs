//! The plugin's own `src/deep` files, compiled here unchanged.
//!
//! Each `#[path]` points at the X-Plane plugin's source, so there is one copy
//! of the physics. `tools/sync-deep-electrical.sh` copies these files into
//! FlyByWire's tree and drops the `#[path]` lines, which leaves every module
//! at the conventional path beside this file.
#[path = "../../../../src/deep/api.rs"]
pub mod api;
pub mod apu;
#[path = "../../../../src/deep/breakers/mod.rs"]
pub mod breakers;
#[path = "../../../../src/deep/electrical/mod.rs"]
pub mod electrical;
#[path = "../../../../src/deep/frame.rs"]
pub mod frame;
pub mod live;
#[path = "../../../../src/deep/wiring/mod.rs"]
pub mod wiring;

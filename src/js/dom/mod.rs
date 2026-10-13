//! The DOM, SVG, CSS and Canvas2D FlyByWire's instruments draw with,
//! painting each screen's document to the display stream
//! (docs/display-stream.md). It is JavaScript run in the engine; this module
//! only carries the scripts and puts them in an engine.
//!
//! After running [`SCRIPTS`] in order (after the engine's prelude, which
//! has the clock), scripts have the DOM's classes as globals and
//! `__createDocument(screen, width?, height?)`, which returns a document
//! that paints to `__host.submitDisplay(screen, ...)` after each tick when
//! something on it changed. The runtime makes one per screen and sets
//! `globalThis.document` to it. Load before anything that subclasses
//! `HTMLElement` at load time (the MSFS instrument classes).
//!
//! What the instruments use, and so what is implemented, is in
//! docs/dom-survey.md.

/// The scripts, in the order they load.
pub const SCRIPTS: [(&str, &str); 9] = [
    ("dom/selectors.js", include_str!("selectors.js")),
    ("dom/css.js", include_str!("css.js")),
    ("dom/geometry.js", include_str!("geometry.js")),
    ("dom/core.js", include_str!("core.js")),
    ("dom/document.js", include_str!("document.js")),
    ("dom/layout.js", include_str!("layout.js")),
    ("dom/paint.js", include_str!("paint.js")),
    ("dom/canvas.js", include_str!("canvas.js")),
    ("dom/install.js", include_str!("install.js")),
];

#[cfg(test)]
mod tests;

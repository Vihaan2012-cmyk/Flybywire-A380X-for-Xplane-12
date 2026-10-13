//! The order of GPU state changes and draws for a mesh, worked out once per
//! stream. The OpenGL renderer carries these steps out, and so does the
//! software renderer the tests draw with, so the tests exercise the same
//! clipping and stencil logic that runs in X-Plane.
//!
//! Stencil bits: the low seven count how many clip paths cover a sample;
//! the top bit marks samples a translucent stroke has already painted.

use super::tessellate::{Mesh, Paint};

/// The top stencil bit.
pub const MARK: u8 = 0x80;
/// The clip-path count bits.
pub const LEVELS: u8 = 0x7f;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StencilTest {
    Off,
    /// Pass where the stencil, masked, equals the reference.
    Equal { reference: u8, mask: u8, mark: bool },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// x0, y0, x1, y1 in device pixels, y down; `None` turns scissoring off.
    Scissor(Option<[i32; 4]>),
    /// Every stencil bit to zero, over the whole target.
    ClearStencil,
    /// The mark bit to zero, inside the scissor.
    ClearMark,
    /// Count a clip path into the stencil where the count equals `level`,
    /// with colour writes off.
    ClipPath { level: u8, first: u32, count: u32 },
    Draw { paint: Paint, first: u32, count: u32, stencil: StencilTest },
    /// Darken a dimming region to its brightness: untextured, no stencil,
    /// no scissor, colour black with alpha one minus the brightness.
    Dim { region: usize, first: u32, count: u32 },
}

pub fn plan(mesh: &Mesh) -> Vec<Step> {
    let mut steps = Vec::with_capacity(mesh.batches.len() * 2);
    let mut scissor: Option<Option<[i32; 4]>> = None;
    // Which clip's paths are in the stencil now.
    let mut in_stencil: Option<usize> = None;
    for batch in &mesh.batches {
        let clip = &mesh.clips[batch.clip];
        if matches!(clip.scissor, Some([x0, y0, x1, y1]) if x0 >= x1 || y0 >= y1) {
            continue;
        }
        let needs_stencil = batch.once || !clip.paths.is_empty();
        if needs_stencil && in_stencil != Some(batch.clip) {
            if scissor != Some(None) {
                steps.push(Step::Scissor(None));
                scissor = Some(None);
            }
            steps.push(Step::ClearStencil);
            for (level, &(first, count)) in clip.paths.iter().enumerate() {
                steps.push(Step::ClipPath { level: level as u8, first, count });
            }
            in_stencil = Some(batch.clip);
        }
        if scissor != Some(clip.scissor) {
            steps.push(Step::Scissor(clip.scissor));
            scissor = Some(clip.scissor);
        }
        let levels = clip.paths.len() as u8;
        let stencil = if batch.once {
            StencilTest::Equal { reference: levels, mask: LEVELS | MARK, mark: true }
        } else if levels > 0 {
            StencilTest::Equal { reference: levels, mask: LEVELS, mark: false }
        } else {
            StencilTest::Off
        };
        steps.push(Step::Draw { paint: batch.paint, first: batch.first, count: batch.count, stencil });
        if batch.once {
            steps.push(Step::ClearMark);
        }
    }
    if !mesh.dims.is_empty() && scissor != Some(None) {
        steps.push(Step::Scissor(None));
    }
    for (region, &(first, count)) in mesh.dims.iter().enumerate() {
        steps.push(Step::Dim { region, first, count });
    }
    steps
}

/// Draw calls a plan makes: its draws, clip-path passes and dimming.
pub fn draw_calls(steps: &[Step]) -> usize {
    steps.iter().filter(|s| matches!(s, Step::Draw { .. } | Step::ClipPath { .. } | Step::Dim { .. })).count()
}

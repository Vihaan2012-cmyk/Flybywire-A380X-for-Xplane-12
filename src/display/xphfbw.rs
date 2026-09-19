//! Pure logic for the XPHFBW screen bridge (docs/briefs/xphfbw-js-bridge.md,
//! agent D's section and race rule 6), kept apart from the X-Plane and
//! OpenGL calls in `mod.rs` and `gl.rs` so it can be unit tested without a
//! plugin or a GL context.

/// Rule 6: which rectangles the plugin uploads this tick, already clamped to
/// the screen's bounds.
///
/// `frame` is the [`crate::xphfbw_bridge::ScreenHeader`]'s `frame` as read
/// before the upload; `last_uploaded` is the frame this screen was last
/// caught up to; `force_full` is set after a previous upload raced a new
/// publish (rule 6: "if it moved during upload, upload the whole screen next
/// frame"). A gap between `last_uploaded` and `frame` (more than one frame
/// published since the last upload) means at least one paint's dirty rects
/// were never read, so the whole screen is uploaded instead of trusting the
/// latest dirty rects alone.
pub fn plan_upload(frame: u64, last_uploaded: u64, force_full: bool, dirty: &[[u32; 4]], width: u32, height: u32) -> Vec<[u32; 4]> {
    if width == 0 || height == 0 {
        return Vec::new();
    }
    let missed_a_frame = frame > last_uploaded.saturating_add(1);
    if force_full || missed_a_frame {
        return vec![[0, 0, width, height]];
    }
    dirty.iter().filter_map(|&r| clamp_rect(r, width, height)).collect()
}

/// Whether an upload raced a new publish: the header's `frame` moved between
/// the read that planned the upload and the read taken right after it (rule
/// 6). When true, the caller's next upload must force a full one, since the
/// rectangles just read may already be stale for what is now in `pixels`.
pub fn torn_by_a_new_publish(frame_before: u64, frame_after: u64) -> bool {
    frame_after != frame_before
}

/// Clamp a dirty rectangle to the screen's bounds; `None` if it is empty or
/// starts outside them (an app bug, or a screen that shrank mid-flight,
/// should not walk off the plugin's pixel buffer).
pub fn clamp_rect(rect: [u32; 4], width: u32, height: u32) -> Option<[u32; 4]> {
    let [x, y, w, h] = rect;
    if x >= width || y >= height || w == 0 || h == 0 {
        return None;
    }
    let w = w.min(width - x);
    let h = h.min(height - y);
    Some([x, y, w, h])
}

/// A device's texel (origin bottom-left, `y` down the CSS screen once
/// converted) as a point on its screen, in CSS pixels. Shared by the
/// QuickJS path's `ScreenEvent`s ([`super::Displays::to_screen`]) and the
/// bridge's `Input` records, so both name the same point the same way.
pub fn device_to_css(x: f64, y: f64, device_height: f64, scale: (f64, f64)) -> (f64, f64) {
    let (sx, sy) = scale;
    ((x + 0.5) / sx, (device_height - (y + 0.5)) / sy)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamp_rect_drops_empty_and_out_of_bounds_and_clips_to_the_edge() {
        assert_eq!(clamp_rect([0, 0, 10, 10], 100, 100), Some([0, 0, 10, 10]));
        assert_eq!(clamp_rect([95, 0, 10, 10], 100, 100), Some([95, 0, 5, 10]));
        assert_eq!(clamp_rect([0, 95, 10, 10], 100, 100), Some([0, 95, 10, 5]));
        assert_eq!(clamp_rect([100, 0, 10, 10], 100, 100), None, "starts past the right edge");
        assert_eq!(clamp_rect([0, 100, 10, 10], 100, 100), None, "starts past the bottom edge");
        assert_eq!(clamp_rect([0, 0, 0, 10], 100, 100), None, "zero width");
        assert_eq!(clamp_rect([0, 0, 10, 0], 100, 100), None, "zero height");
    }

    #[test]
    fn plan_upload_uses_the_dirty_rects_on_a_consecutive_frame() {
        // One in bounds, one that needs clamping to the edge, one entirely
        // outside (dropped).
        let dirty = [[10, 10, 20, 20], [90, 90, 20, 20], [1000, 1000, 5, 5]];
        let rects = plan_upload(5, 4, false, &dirty, 100, 100);
        assert_eq!(rects, vec![[10, 10, 20, 20], [90, 90, 10, 10]]);
    }

    #[test]
    fn plan_upload_uploads_the_whole_screen_after_a_missed_frame() {
        // frame jumped from 4 (last upload) to 7: at least one paint's dirty
        // rects were never read.
        let rects = plan_upload(7, 4, false, &[[10, 10, 20, 20]], 100, 60);
        assert_eq!(rects, vec![[0, 0, 100, 60]]);
        // Exactly one frame ahead is not a miss.
        let rects = plan_upload(5, 4, false, &[[10, 10, 20, 20]], 100, 60);
        assert_eq!(rects, vec![[10, 10, 20, 20]]);
    }

    #[test]
    fn plan_upload_forces_a_full_upload_after_a_torn_read() {
        // Even on an otherwise-consecutive frame, a prior race forces one
        // full upload before trusting dirty rects again.
        let rects = plan_upload(5, 4, true, &[[10, 10, 20, 20]], 100, 60);
        assert_eq!(rects, vec![[0, 0, 100, 60]]);
    }

    #[test]
    fn plan_upload_ignores_a_zero_sized_screen() {
        assert_eq!(plan_upload(5, 4, true, &[[10, 10, 20, 20]], 0, 60), Vec::<[u32; 4]>::new());
    }

    #[test]
    fn torn_by_a_new_publish_detects_the_frame_moving_mid_upload() {
        assert!(!torn_by_a_new_publish(4, 4));
        assert!(torn_by_a_new_publish(4, 5));
    }

    #[test]
    fn device_to_css_flips_y_and_undoes_the_device_scale() {
        // A 768x1024 CSS screen drawn into a 1536x2048 device (2x scale, a
        // Retina-like device). The texel origin is bottom-left, so a texel
        // near the top of the device (high y) lands near the screen's top.
        let scale = (2.0, 2.0);
        let device_height = 2048.0;

        // Near the top-left texel.
        let (x, y) = device_to_css(0.0, 2047.0, device_height, scale);
        assert!((x - 0.25).abs() < 1e-9);
        assert!((y - 0.25).abs() < 1e-9);

        // Near the bottom-right texel (max x, min y).
        let (x, y) = device_to_css(1535.0, 0.0, device_height, scale);
        assert!((x - 767.75).abs() < 1e-9);
        assert!((y - 1023.75).abs() < 1e-9);

        // 1x scale is the identity mapping up to the half-texel centring.
        let (x, y) = device_to_css(9.0, 9.0, 20.0, (1.0, 1.0));
        assert!((x - 9.5).abs() < 1e-9);
        assert!((y - 10.5).abs() < 1e-9);
    }
}

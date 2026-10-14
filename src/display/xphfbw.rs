//! Pure logic for the XPHFBW screen bridge (docs/briefs/xphfbw-js-bridge.md,
//! agent D's section and race rule 6), kept apart from the X-Plane and
//! OpenGL calls in `mod.rs` and `gl.rs` so it can be unit tested without a
//! plugin or a GL context.

/// Rule 6: which rectangles the plugin uploads this tick, clamped to the
/// screen's bounds: `changed`, every rectangle painted since the last upload
/// ([`crate::xphfbw_bridge::ScreenHeader::changed_since`]), or the whole
/// screen when that is `None` or when the rectangles cover at least its area.
pub fn plan_upload(changed: Option<Vec<[u32; 4]>>, width: u32, height: u32) -> Vec<[u32; 4]> {
    if width == 0 || height == 0 {
        return Vec::new();
    }
    let whole = vec![[0, 0, width, height]];
    let Some(changed) = changed else { return whole };
    let mut rects: Vec<[u32; 4]> = changed.into_iter().filter_map(|r| clamp_rect(r, width, height)).collect();
    rects.sort_unstable();
    rects.dedup();
    let overlapping = (0..rects.len()).any(|i| (i + 1..rects.len()).any(|j| overlaps(&rects[i], &rects[j])));
    if overlapping {
        rects = tile_union(&rects, width, height);
    }
    let area: u64 = rects.iter().map(|r| r[2] as u64 * r[3] as u64).sum();
    if area >= width as u64 * height as u64 {
        return whole;
    }
    rects
}

const UNION_TILE: u32 = 32;

fn overlaps(a: &[u32; 4], b: &[u32; 4]) -> bool {
    a[0] < b[0] + b[2] && b[0] < a[0] + a[2] && a[1] < b[1] + b[3] && b[1] < a[1] + a[3]
}

fn tile_union(rects: &[[u32; 4]], width: u32, height: u32) -> Vec<[u32; 4]> {
    let (cols, rows) = (width.div_ceil(UNION_TILE), height.div_ceil(UNION_TILE));
    let mut grid = vec![false; (cols * rows) as usize];
    for r in rects {
        for ty in r[1] / UNION_TILE..=(r[1] + r[3] - 1) / UNION_TILE {
            for tx in r[0] / UNION_TILE..=(r[0] + r[2] - 1) / UNION_TILE {
                grid[(ty * cols + tx) as usize] = true;
            }
        }
    }
    let mut out: Vec<[u32; 4]> = Vec::new();
    let mut above: Vec<(u32, u32, usize)> = Vec::new();
    for ty in 0..rows {
        let mut here = Vec::new();
        let mut tx = 0;
        while tx < cols {
            if !grid[(ty * cols + tx) as usize] {
                tx += 1;
                continue;
            }
            let start = tx;
            while tx < cols && grid[(ty * cols + tx) as usize] {
                tx += 1;
            }
            match above.iter().find(|(s, e, _)| *s == start && *e == tx) {
                Some(&(_, _, i)) => {
                    out[i][3] += UNION_TILE;
                    here.push((start, tx, i));
                }
                None => {
                    out.push([start * UNION_TILE, ty * UNION_TILE, (tx - start) * UNION_TILE, UNION_TILE]);
                    here.push((start, tx, out.len() - 1));
                }
            }
        }
        above = here;
    }
    out.into_iter().filter_map(|r| clamp_rect(r, width, height)).collect()
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
    fn plan_upload_clamps_and_dedups_the_changed_rects() {
        let changed = vec![[10, 10, 20, 20], [90, 90, 20, 20], [1000, 1000, 5, 5], [10, 10, 20, 20]];
        assert_eq!(plan_upload(Some(changed), 100, 100), vec![[10, 10, 20, 20], [90, 90, 10, 10]]);
    }

    #[test]
    fn plan_upload_takes_the_whole_screen_without_a_history_or_when_it_is_cheaper() {
        assert_eq!(plan_upload(None, 100, 60), vec![[0, 0, 100, 60]]);
        assert_eq!(plan_upload(Some(vec![[0, 0, 100, 40], [0, 20, 100, 40]]), 100, 60), vec![[0, 0, 100, 60]]);
    }

    #[test]
    fn overlapping_paints_upload_their_union_once() {
        let a = [0, 0, 64, 64];
        let b = [32, 32, 64, 64];
        let plan = plan_upload(Some(vec![a, b]), 200, 200);
        let area: u32 = plan.iter().map(|r| r[2] * r[3]).sum();
        assert!(area < 64 * 64 * 2, "union {area} is less than the two rects summed");
        let covered = |x: u32, y: u32| plan.iter().any(|r| x >= r[0] && x < r[0] + r[2] && y >= r[1] && y < r[1] + r[3]);
        for r in [a, b] {
            for y in r[1]..r[1] + r[3] {
                for x in r[0]..r[0] + r[2] {
                    assert!(covered(x, y), "({x},{y}) left out");
                }
            }
        }
        let rows_overlap = (0..plan.len()).any(|i| (i + 1..plan.len()).any(|j| overlaps(&plan[i], &plan[j])));
        assert!(!rows_overlap, "no pixel is sent twice");
    }

    #[test]
    fn plan_upload_ignores_a_zero_sized_screen() {
        assert_eq!(plan_upload(None, 0, 60), Vec::<[u32; 4]>::new());
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

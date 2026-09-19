//! Polar-to-image: a set of geo-referenced returns, rasterised into the
//! straight RGBA buffer opcode 60 NATIVE_IMAGE draws (docs/display-stream.md).
//! Each return is painted as a small filled square around its screen pixel,
//! worse levels last so a severe cell is never hidden under a milder
//! neighbour it overlaps.

use super::geometry::screen_offset;
use super::levels::Level;

/// One classified, geo-referenced return.
#[derive(Clone, Copy, Debug)]
pub struct Return {
    pub lat: f64,
    pub lon: f64,
    pub level: Level,
}

/// What the image is drawn for: the aircraft's current position and
/// heading, the ND's selected range, and the canvas the image fills.
pub struct View {
    pub own_lat: f64,
    pub own_lon: f64,
    pub heading_true_deg: f64,
    pub range_nm: f64,
    pub half_sector_deg: f64,
    pub width: u32,
    pub height: u32,
    /// Ownship's pixel, within the canvas (ARC mode has it low and centred;
    /// ROSE modes have it in the middle -- `wxr/mod.rs`'s `ARC_CENTER_PX`
    /// / `ROSE_CENTER_PX`).
    pub center_px: (f64, f64),
    /// Range at the edge of the visible fan, in pixels.
    pub max_radius_px: f64,
    /// Half the side of each return's painted square, in pixels; at least
    /// big enough that adjoining cells in the sampled grid touch.
    pub cell_half_px: f64,
}

impl View {
    fn px_per_nm(&self) -> f64 {
        if self.range_nm > 0. {
            self.max_radius_px / self.range_nm
        } else {
            0.
        }
    }
}

/// Straight RGBA, row 0 at the top, `width * height * 4` bytes, transparent
/// where nothing was painted.
pub fn rasterize(returns: &[Return], view: &View) -> Vec<u8> {
    let (width, height) = (view.width as usize, view.height as usize);
    let mut rgba = vec![0u8; width * height * 4];
    let px_per_nm = view.px_per_nm();
    // Worse levels paint last, on top of anything milder they overlap.
    let mut order: Vec<&Return> = returns.iter().filter(|r| !r.level.is_none()).collect();
    order.sort_by_key(|r| r.level);
    for r in order {
        let Some((dx, dy)) = screen_offset(r.lat, r.lon, view.own_lat, view.own_lon, view.heading_true_deg, view.range_nm, px_per_nm, view.half_sector_deg) else {
            continue;
        };
        let (cx, cy) = (view.center_px.0 + dx, view.center_px.1 + dy);
        paint_square(&mut rgba, width, height, cx, cy, view.cell_half_px, r.level.rgba());
    }
    rgba
}

/// Fills the square around `(cx, cy)` with `colour`, clipped to the buffer.
fn paint_square(rgba: &mut [u8], width: usize, height: usize, cx: f64, cy: f64, half: f64, colour: [u8; 4]) {
    let x0 = (cx - half).floor().max(0.) as usize;
    let x1 = ((cx + half).ceil() as isize).clamp(0, width as isize) as usize;
    let y0 = (cy - half).floor().max(0.) as usize;
    let y1 = ((cy + half).ceil() as isize).clamp(0, height as isize) as usize;
    for y in y0..y1.min(height) {
        for x in x0..x1.min(width) {
            let i = (y * width + x) * 4;
            rgba[i..i + 4].copy_from_slice(&colour);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn view() -> View {
        View {
            own_lat: 0.,
            own_lon: 0.,
            heading_true_deg: 0.,
            range_nm: 40.,
            half_sector_deg: 60.,
            width: 100,
            height: 100,
            center_px: (50., 80.),
            max_radius_px: 70.,
            cell_half_px: 4.,
        }
    }

    fn pixel(rgba: &[u8], width: u32, x: i32, y: i32) -> [u8; 4] {
        let i = ((y as u32 * width + x as u32) * 4) as usize;
        [rgba[i], rgba[i + 1], rgba[i + 2], rgba[i + 3]]
    }

    #[test]
    fn an_empty_grid_is_fully_transparent() {
        let v = view();
        let rgba = rasterize(&[], &v);
        assert!(rgba.iter().all(|&b| b == 0));
    }

    #[test]
    fn a_return_dead_ahead_paints_above_ownship() {
        let v = view();
        // 10 nm north of an aircraft at the origin heading north: dead ahead.
        let returns = [Return { lat: 10. / 60., lon: 0., level: Level::Green }];
        let rgba = rasterize(&returns, &v);
        // Ownship is at (50, 80); 10 nm at 70 px / 40 nm is 17.5 px up.
        let p = pixel(&rgba, v.width, 50, 80 - 18);
        assert_eq!(p, Level::Green.rgba());
    }

    #[test]
    fn a_worse_level_paints_over_a_milder_one_at_the_same_spot() {
        let mut v = view();
        v.cell_half_px = 20.; // force the two squares to overlap fully
        let returns = [
            Return { lat: 10. / 60., lon: 0., level: Level::Green },
            Return { lat: 10. / 60., lon: 0.01, level: Level::Magenta },
        ];
        let rgba = rasterize(&returns, &v);
        let p = pixel(&rgba, v.width, 50, 80 - 18);
        assert_eq!(p, Level::Magenta.rgba());
    }

    #[test]
    fn behind_the_aircraft_is_never_painted() {
        let v = view();
        let returns = [Return { lat: -10. / 60., lon: 0., level: Level::Red }];
        let rgba = rasterize(&returns, &v);
        assert!(rgba.iter().all(|&b| b == 0));
    }
}

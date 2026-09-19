//! The display command stream (docs/display-stream.md), read into commands.
//!
//! A stream is a flat list of numbers: an opcode, then its operands. It is
//! read whole before anything is drawn, so a stream that is cut short or
//! carries an opcode this renderer does not know is refused as a whole and
//! the screen keeps showing the last good one.

/// Red, green, blue, alpha, each 0..1, not premultiplied.
pub type Colour = [f32; 4];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FillRule {
    NonZero,
    EvenOdd,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cap {
    Butt,
    Round,
    Square,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Join {
    Miter,
    Round,
    Bevel,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Stroke {
    pub colour: Colour,
    pub width: f64,
    pub cap: Cap,
    pub join: Join,
    pub miter_limit: f64,
    pub dashes: Vec<f64>,
    pub dash_offset: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Align {
    Left,
    Center,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Baseline {
    Alphabetic,
    Middle,
    Top,
    Bottom,
    Hanging,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Text {
    pub text: u32,
    pub font: u32,
    pub size: f64,
    pub weight: u16,
    pub italic: bool,
    pub x: f64,
    pub y: f64,
    pub align: Align,
    pub baseline: Baseline,
    pub fill: Colour,
    pub stroke_width: f64,
    pub stroke: Colour,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Image {
    pub url: u32,
    /// Source rectangle in the image's pixels: x, y, w, h.
    pub src: [f64; 4],
    /// Destination rectangle in CSS pixels: x, y, w, h.
    pub dst: [f64; 4],
}

/// An image the plugin makes itself (NATIVE_IMAGE), by id.
#[derive(Clone, Debug, PartialEq)]
pub struct NativeImage {
    pub id: u32,
    /// Destination rectangle in CSS pixels: x, y, w, h.
    pub dst: [f64; 4],
}

#[derive(Clone, Debug, PartialEq)]
pub struct Gradient {
    pub from: [f64; 2],
    pub to: [f64; 2],
    /// Offset and colour, in the order given.
    pub stops: Vec<(f64, Colour)>,
    pub rule: FillRule,
}

/// One drawing command. Strings are indices into the stream's strings.
#[derive(Clone, Debug, PartialEq)]
pub enum Op {
    Save,
    Restore,
    Transform([f64; 6]),
    SetTransform([f64; 6]),
    GlobalAlpha(f64),
    ClipRect([f64; 4]),
    BeginPath,
    MoveTo(f64, f64),
    LineTo(f64, f64),
    QuadTo([f64; 4]),
    CubicTo([f64; 6]),
    Arc { cx: f64, cy: f64, r: f64, start: f64, end: f64, ccw: bool },
    Ellipse { cx: f64, cy: f64, rx: f64, ry: f64, rotation: f64, start: f64, end: f64, ccw: bool },
    Rect([f64; 4]),
    ClosePath,
    Fill(Colour, FillRule),
    Stroke(Stroke),
    ClipPath(FillRule),
    Text(Text),
    Image(Image),
    LinearGradientFill(Gradient),
    NativeImage(NativeImage),
}

pub mod opcode {
    pub const SAVE: u32 = 1;
    pub const RESTORE: u32 = 2;
    pub const TRANSFORM: u32 = 3;
    pub const SET_TRANSFORM: u32 = 4;
    pub const GLOBAL_ALPHA: u32 = 5;
    pub const CLIP_RECT: u32 = 6;
    pub const BEGIN_PATH: u32 = 10;
    pub const MOVE_TO: u32 = 11;
    pub const LINE_TO: u32 = 12;
    pub const QUAD_TO: u32 = 13;
    pub const CUBIC_TO: u32 = 14;
    pub const ARC: u32 = 15;
    pub const ELLIPSE: u32 = 16;
    pub const RECT: u32 = 17;
    pub const CLOSE_PATH: u32 = 18;
    pub const FILL: u32 = 20;
    pub const STROKE: u32 = 21;
    pub const CLIP_PATH: u32 = 22;
    pub const TEXT: u32 = 30;
    pub const IMAGE: u32 = 40;
    pub const LINEAR_GRADIENT_FILL: u32 = 50;
    pub const NATIVE_IMAGE: u32 = 60;
}

/// Why a stream was refused: where reading stopped, and what was wrong there.
#[derive(Debug, PartialEq)]
pub struct StreamError {
    pub at: usize,
    pub reason: String,
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "at number {}: {}", self.at, self.reason)
    }
}

/// A dash list longer than this is not a dash pattern anyone draws.
const MAX_DASHES: usize = 64;
/// Nor is a gradient with more stops than this.
const MAX_STOPS: usize = 256;

struct Reader<'a> {
    ops: &'a [f64],
    at: usize,
    strings: usize,
}

impl Reader<'_> {
    fn error(&self, reason: impl Into<String>) -> StreamError {
        StreamError { at: self.at, reason: reason.into() }
    }

    fn num(&mut self) -> Result<f64, StreamError> {
        let v = *self.ops.get(self.at).ok_or_else(|| self.error("the stream ends inside an operation"))?;
        self.at += 1;
        Ok(v)
    }

    fn nums<const N: usize>(&mut self) -> Result<[f64; N], StreamError> {
        let mut out = [0.; N];
        for v in &mut out {
            *v = self.num()?;
        }
        Ok(out)
    }

    fn count(&mut self, limit: usize, what: &str) -> Result<usize, StreamError> {
        let v = self.num()?;
        if !(v >= 0. && v.fract() == 0. && v <= limit as f64) {
            return Err(self.error(format!("{what} count {v} is not a whole number up to {limit}")));
        }
        Ok(v as usize)
    }

    fn string(&mut self) -> Result<u32, StreamError> {
        let v = self.num()?;
        if !(v >= 0. && v.fract() == 0. && (v as usize) < self.strings) {
            return Err(self.error(format!("string index {v} is not one of the {} strings", self.strings)));
        }
        Ok(v as u32)
    }

    fn colour(&mut self) -> Result<Colour, StreamError> {
        let [r, g, b, a] = self.nums::<4>()?;
        let c = |v: f64| if v.is_finite() { v.clamp(0., 1.) as f32 } else { 0. };
        Ok([c(r), c(g), c(b), c(a)])
    }

    fn flag(&mut self) -> Result<bool, StreamError> {
        Ok(self.num()? != 0.)
    }

    fn rule(&mut self) -> Result<FillRule, StreamError> {
        Ok(if self.num()? == 1. { FillRule::EvenOdd } else { FillRule::NonZero })
    }
}

/// Read a whole stream. `strings` is how many strings came with it.
pub fn parse(ops: &[f64], strings: usize) -> Result<Vec<Op>, StreamError> {
    use opcode::*;
    let mut r = Reader { ops, at: 0, strings };
    let mut out = Vec::with_capacity(ops.len() / 4);
    while r.at < ops.len() {
        let code = r.num()?;
        if !(code >= 0. && code.fract() == 0. && code < 1000.) {
            r.at -= 1;
            return Err(r.error(format!("{code} is not an opcode")));
        }
        let op = match code as u32 {
            SAVE => Op::Save,
            RESTORE => Op::Restore,
            TRANSFORM => Op::Transform(r.nums()?),
            SET_TRANSFORM => Op::SetTransform(r.nums()?),
            GLOBAL_ALPHA => Op::GlobalAlpha(r.num()?),
            CLIP_RECT => Op::ClipRect(r.nums()?),
            BEGIN_PATH => Op::BeginPath,
            MOVE_TO => {
                let [x, y] = r.nums()?;
                Op::MoveTo(x, y)
            }
            LINE_TO => {
                let [x, y] = r.nums()?;
                Op::LineTo(x, y)
            }
            QUAD_TO => Op::QuadTo(r.nums()?),
            CUBIC_TO => Op::CubicTo(r.nums()?),
            ARC => {
                let [cx, cy, radius, start, end] = r.nums()?;
                Op::Arc { cx, cy, r: radius, start, end, ccw: r.flag()? }
            }
            ELLIPSE => {
                let [cx, cy, rx, ry, rotation, start, end] = r.nums()?;
                Op::Ellipse { cx, cy, rx, ry, rotation, start, end, ccw: r.flag()? }
            }
            RECT => Op::Rect(r.nums()?),
            CLOSE_PATH => Op::ClosePath,
            FILL => {
                let colour = r.colour()?;
                Op::Fill(colour, r.rule()?)
            }
            STROKE => {
                let colour = r.colour()?;
                let width = r.num()?;
                let cap = match r.num()? as i64 {
                    1 => Cap::Round,
                    2 => Cap::Square,
                    _ => Cap::Butt,
                };
                let join = match r.num()? as i64 {
                    1 => Join::Round,
                    2 => Join::Bevel,
                    _ => Join::Miter,
                };
                let miter_limit = r.num()?;
                let n = r.count(MAX_DASHES, "dash")?;
                let mut dashes = Vec::with_capacity(n);
                for _ in 0..n {
                    dashes.push(r.num()?);
                }
                let dash_offset = r.num()?;
                Op::Stroke(Stroke { colour, width, cap, join, miter_limit, dashes, dash_offset })
            }
            CLIP_PATH => Op::ClipPath(r.rule()?),
            TEXT => {
                let text = r.string()?;
                let font = r.string()?;
                let [size, weight] = r.nums()?;
                let italic = r.flag()?;
                let [x, y] = r.nums()?;
                let align = match r.num()? as i64 {
                    1 => Align::Center,
                    2 => Align::Right,
                    _ => Align::Left,
                };
                let baseline = match r.num()? as i64 {
                    1 => Baseline::Middle,
                    2 => Baseline::Top,
                    3 => Baseline::Bottom,
                    4 => Baseline::Hanging,
                    _ => Baseline::Alphabetic,
                };
                let fill = r.colour()?;
                let stroke_width = r.num()?;
                let stroke = r.colour()?;
                let weight = if weight.is_finite() { weight.clamp(1., 1000.) as u16 } else { 400 };
                Op::Text(Text { text, font, size, weight, italic, x, y, align, baseline, fill, stroke_width, stroke })
            }
            IMAGE => {
                let url = r.string()?;
                Op::Image(Image { url, src: r.nums()?, dst: r.nums()? })
            }
            LINEAR_GRADIENT_FILL => {
                let [x0, y0, x1, y1] = r.nums()?;
                let n = r.count(MAX_STOPS, "gradient stop")?;
                let mut stops = Vec::with_capacity(n);
                for _ in 0..n {
                    let offset = r.num()?;
                    stops.push((offset, r.colour()?));
                }
                Op::LinearGradientFill(Gradient { from: [x0, y0], to: [x1, y1], stops, rule: r.rule()? })
            }
            NATIVE_IMAGE => {
                let id = r.string()?;
                Op::NativeImage(NativeImage { id, dst: r.nums()? })
            }
            other => {
                r.at -= 1;
                return Err(r.error(format!("opcode {other} is not in the contract")));
            }
        };
        out.push(op);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stream_reads_into_its_commands() {
        let ops = [
            1., 3., 1., 0., 0., 1., 10., 20., 10., 11., 0., 0., 12., 5., 5., 18., //
            21., 1., 0., 0., 1., 2., 1., 1., 4., 2., 3., 1., 0.5, //
            30., 0., 1., 20., 700., 0., 5., 6., 1., 2., 0., 1., 0., 1., 0., 0., 0., 0., 0., 2.,
        ];
        let parsed = parse(&ops, 2).unwrap();
        assert_eq!(parsed.len(), 9);
        assert_eq!(parsed[1], Op::Transform([1., 0., 0., 1., 10., 20.]));
        let Op::Stroke(s) = &parsed[6] else { panic!("{:?}", parsed[6]) };
        assert_eq!((s.width, s.cap, s.join, s.miter_limit), (2., Cap::Round, Join::Round, 4.));
        assert_eq!((s.dashes.clone(), s.dash_offset), (vec![3., 1.], 0.5));
        let Op::Text(t) = &parsed[7] else { panic!() };
        assert_eq!((t.text, t.font, t.weight, t.align, t.baseline), (0, 1, 700, Align::Center, Baseline::Top));
        assert_eq!(parsed[8], Op::Restore);
        let native = parse(&[60., 1., 0., 0., 768., 1024.], 2).unwrap();
        assert_eq!(native, vec![Op::NativeImage(NativeImage { id: 1, dst: [0., 0., 768., 1024.] })]);
    }

    #[test]
    fn a_broken_stream_is_refused_with_where_it_broke() {
        // Cut short inside LINE_TO.
        assert_eq!(parse(&[10., 12., 1.], 0).unwrap_err().at, 3);
        // An opcode not in the contract.
        let e = parse(&[10., 99., 1.], 0).unwrap_err();
        assert_eq!(e.at, 1);
        assert!(e.reason.contains("99"), "{e}");
        // A string index past the strings sent.
        assert!(parse(&[40., 3., 0., 0., 1., 1., 0., 0., 1., 1.], 3).is_err());
        // A dash count that is not a count.
        assert!(parse(&[21., 1., 1., 1., 1., 1., 0., 0., 10., -1., 0.], 0).is_err());
    }
}

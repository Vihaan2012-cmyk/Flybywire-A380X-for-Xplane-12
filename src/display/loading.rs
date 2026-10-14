use std::collections::HashMap;
use std::ffi::c_int;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{channel, Receiver};
use std::time::Instant;

use super::gl::Renderer;
use super::image::{self, Picture, Source};

const SLIDE_S: f32 = 8.;
const CROSSFADE_S: f32 = 1.2;
const FADE_IN_S: f32 = 0.6;
const HOLD_S: f32 = 2.;
const FADE_OUT_S: f32 = 1.5;
const TIMEOUT_S: f32 = 240.;
const CYAN: [f32; 3] = [0., 200. / 255., 1.];

pub struct Progress {
    pub systems: bool,
    pub screens_up: usize,
    pub screens: usize,
}

impl Progress {
    fn complete(&self) -> bool {
        self.systems && self.screens_up >= self.screens
    }

    fn percent(&self) -> f32 {
        let screens = if self.screens == 0 { 1. } else { self.screens_up as f32 / self.screens as f32 };
        f32::from(u8::from(self.systems)) * 10. + screens * 80. + if self.complete() { 10. } else { 0. }
    }
}

struct Manifest {
    design_width: f32,
    layer_height: f32,
    sprites: HashMap<String, [f32; 4]>,
    backgrounds: Vec<String>,
    tips: Vec<String>,
    digits_advance: f32,
}

fn read_manifest(dir: &Path) -> Option<Manifest> {
    let text = std::fs::read_to_string(dir.join("manifest.json")).ok()?;
    let v: serde_json::Value = serde_json::from_str(&text).ok()?;
    let num = |v: &serde_json::Value| v.as_f64().map(|x| x as f32);
    let sprites = v["sprites"]
        .as_object()?
        .iter()
        .filter_map(|(k, r)| {
            let r = r.as_array()?;
            Some((k.clone(), [num(r.first()?)?, num(r.get(1)?)?, num(r.get(2)?)?, num(r.get(3)?)?]))
        })
        .collect();
    let strings = |key: &str| v[key].as_array().map(|a| a.iter().filter_map(|s| s.as_str().map(str::to_owned)).collect::<Vec<_>>()).unwrap_or_default();
    Some(Manifest {
        design_width: num(&v["design_width"])?,
        layer_height: num(&v["layer_height"])?,
        sprites,
        backgrounds: strings("backgrounds"),
        tips: strings("tips"),
        digits_advance: num(&v["digits"]["advance"])?,
    })
}

pub struct Loading {
    manifest: Manifest,
    incoming: Receiver<(String, Picture)>,
    textures: HashMap<String, (c_int, u32, u32)>,
    started: Instant,
    complete_at: Option<Instant>,
    closing_at: Option<Instant>,
    first_bg: usize,
    first_tip: usize,
    shown_percent: f32,
    last_frame: Instant,
    pub finished: bool,
}

impl Loading {
    pub fn new(dir: PathBuf) -> Option<Self> {
        let manifest = read_manifest(&dir)?;
        let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.subsec_nanos() as usize);
        let first_bg = seed % manifest.backgrounds.len().max(1);
        let first_tip = (seed / 7) % manifest.tips.len().max(1);
        let mut names: Vec<String> = manifest.sprites.keys().filter(|k| k.as_str() != "band").cloned().collect();
        names.extend(["band", "white", "bar_glow"].map(String::from));
        let n = manifest.backgrounds.len();
        names.extend((0..n).map(|i| manifest.backgrounds[(first_bg + i) % n].trim_end_matches(".png").to_owned()));
        let (send, incoming) = channel();
        std::thread::spawn(move || {
            for name in names {
                if let Ok(picture) = image::load(&Source::File(dir.join(format!("{name}.png")))) {
                    if send.send((name, picture)).is_err() {
                        return;
                    }
                }
            }
        });
        let now = Instant::now();
        Some(Self {
            manifest,
            incoming,
            textures: HashMap::new(),
            started: now,
            complete_at: None,
            closing_at: None,
            first_bg,
            first_tip,
            shown_percent: 0.,
            last_frame: now,
            finished: false,
        })
    }

    pub fn dismiss(&mut self) {
        self.closing_at.get_or_insert_with(Instant::now);
    }

    pub fn release(&mut self, renderer: &mut Renderer) {
        let ids: Vec<c_int> = self.textures.values().map(|t| t.0).collect();
        renderer.overlay_release(&ids);
        self.textures.clear();
    }

    fn texture(&self, name: &str) -> Option<(c_int, u32, u32)> {
        self.textures.get(name).copied()
    }

    pub fn draw(&mut self, renderer: &mut Renderer, window: (c_int, c_int, c_int, c_int), progress: &Progress) {
        let mut backgrounds_uploaded = 0;
        while backgrounds_uploaded == 0 {
            let Ok((name, picture)) = self.incoming.try_recv() else { break };
            if name.starts_with("bg_") {
                backgrounds_uploaded += 1;
            }
            let texture = renderer.overlay_texture(&picture);
            self.textures.insert(name, (texture, picture.width, picture.height));
        }

        let now = Instant::now();
        let dt = (now - self.last_frame).as_secs_f32().min(0.25);
        self.last_frame = now;
        let t = (now - self.started).as_secs_f32();
        if progress.complete() {
            let done = *self.complete_at.get_or_insert(now);
            if (now - done).as_secs_f32() > HOLD_S {
                self.closing_at.get_or_insert(now);
            }
        }
        if t > TIMEOUT_S {
            self.closing_at.get_or_insert(now);
        }
        let closing = self.closing_at.map_or(0., |c| (now - c).as_secs_f32() / FADE_OUT_S);
        if closing >= 1. {
            self.finished = true;
            return;
        }
        let alpha = (t / FADE_IN_S).min(1.) * (1. - closing);
        self.shown_percent += (progress.percent() - self.shown_percent) * (dt * 4.).min(1.);

        let (l, top, r, b) = window;
        let (l, top, r, b) = (l as f32, top as f32, r as f32, b as f32);
        let (w, h) = (r - l, top - b);
        let slide = (t / SLIDE_S) as usize;
        let into = t - slide as f32 * SLIDE_S;
        let blend = ((into - (SLIDE_S - CROSSFADE_S)) / CROSSFADE_S).clamp(0., 1.);

        if let Some((white, _, _)) = self.texture("white") {
            renderer.overlay_quad(white, [l, b, r, top], [0., 0., 1., 1.], [0.02, 0.03, 0.05, alpha]);
        }
        let nbg = self.manifest.backgrounds.len();
        if nbg > 0 {
            let bg = |k: usize| self.manifest.backgrounds[(self.first_bg + k) % nbg].trim_end_matches(".png").to_owned();
            let current = self.texture(&bg(slide));
            let next = self.texture(&bg(slide + 1));
            let next_alpha = if next.is_some() { blend } else { 0. };
            for (tex, a) in [(current, 1.), (next, next_alpha)] {
                let Some((tex, tw, th)) = tex else { continue };
                if a <= 0. {
                    continue;
                }
                let (tw, th) = (tw as f32, th as f32);
                let scale = (w / tw).max(h / th);
                let (cw, ch) = (w / scale / tw, h / scale / th);
                let (u0, v0) = ((1. - cw) / 2., (1. - ch) / 2.);
                renderer.overlay_quad(tex, [l, b, r, top], [u0, v0, u0 + cw, v0 + ch], [1., 1., 1., alpha * a]);
            }
        }

        let s = w / self.manifest.design_width;
        let lh = self.manifest.layer_height;
        let place = |rect: [f32; 4], dx: f32| [l + (rect[0] + dx) * s, b + (lh - rect[1] - rect[3]) * s, l + (rect[0] + dx + rect[2]) * s, b + (lh - rect[1]) * s];
        let sprite = |renderer: &mut Renderer, name: &str, dx: f32, a: f32| {
            let (Some(rect), Some((tex, _, _))) = (self.manifest.sprites.get(name).copied(), self.textures.get(name).copied()) else { return };
            renderer.overlay_quad(tex, place(rect, dx), [0., 0., 1., 1.], [1., 1., 1., alpha * a]);
        };

        if let Some((band, _, _)) = self.texture("band") {
            renderer.overlay_quad(band, [l, b, r, b + lh * s], [0., 0., 1., 1.], [1., 1., 1., alpha]);
        }
        sprite(renderer, "wordmark", 0., 1.);
        let ntips = self.manifest.tips.len();
        if ntips > 0 {
            let tip = |k: usize| self.manifest.tips[(self.first_tip + k) % ntips].clone();
            let (now_tip, next_tip) = (tip(slide), tip(slide + 1));
            sprite(renderer, &now_tip, 0., 1. - blend);
            sprite(renderer, &next_tip, 0., blend);
        }
        sprite(renderer, "loading", 0., 1.);
        sprite(renderer, "percent", 0., 1.);
        let digits = format!("{}", self.shown_percent.round().clamp(0., 100.) as u32);
        let advance = self.manifest.digits_advance;
        for (i, ch) in digits.chars().rev().enumerate() {
            sprite(renderer, &format!("digit_{ch}"), -(i as f32) * advance, 1.);
        }
        let complete = progress.complete();
        let states = [
            if progress.systems { "done" } else { "active" },
            if progress.screens_up >= progress.screens { "done" } else if progress.systems { "active" } else { "waiting" },
            if complete { "done" } else { "waiting" },
        ];
        for (row, state) in states.iter().enumerate() {
            sprite(renderer, &format!("item_{row}_{state}"), 0., 1.);
        }

        if let Some((white, _, _)) = self.texture("white") {
            let bar_h = 6. * s;
            renderer.overlay_quad(white, [l, b, r, b + bar_h], [0., 0., 1., 1.], [1., 1., 1., 0.12 * alpha]);
            let fill = l + w * (self.shown_percent / 100.).clamp(0., 1.);
            if let Some((glow, _, _)) = self.texture("bar_glow") {
                renderer.overlay_quad(glow, [l, b, fill, b + 32. * s], [0., 0., 1., 1.], [1., 1., 1., 0.8 * alpha]);
            }
            renderer.overlay_quad(white, [l, b, fill, b + bar_h], [0., 0., 1., 1.], [CYAN[0], CYAN[1], CYAN[2], alpha]);
        }
    }
}

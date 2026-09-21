//! The deep systems layer's MSFS module. Stage 0 of `docs/msfs-port.md`:
//! a fifth WASM module that loads beside FlyByWire's four, reads the host
//! inputs `Truth` needs from the simulator, and publishes them back as
//! LVars so the dev console can show that every input arrives in the unit
//! the model expects.
//!
//! **No physics runs here yet.** That is deliberate and it is the point of
//! a stage 0: the module has to load, tick and talk before there is any
//! value in linking a hundred thousand lines of systems behind it. What
//! this stage settles, and prints to the console so it can be checked:
//!
//! 1. The module instantiates in MSFS's runtime and receives frames.
//! 2. Each of the ten inputs in `msfs_inputs::INPUTS` either resolves as a
//!    simulator variable with the unit the table names, or is reported as
//!    failing to. Three rows in that table are marked *unverified*; this is
//!    where they stop being so. A row whose name or unit the simulator
//!    rejects is printed as such rather than read as zero.
//! 3. The converted values reach LVars (`DEEP_IN_*`) through the same
//!    `lvar_bridge` the systems will publish through, so the bridge's
//!    resolve-once, write-every-frame shape is exercised end to end.
//!
//! What crosses the boundary later -- the ~49 authority couplings and the
//! EFB's live state -- is hundreds of variables, and `msfs/lvar-bench`
//! measured that at 11 ns a write. Nothing here needs to be clever.
//!
//! The bridge and the input table are the plugin crate's own modules, used
//! through it as a library, so this crate cannot drift from what the X-Plane
//! side documents. The crate is linked whole, with none of its optional
//! features; stage 1 of the port is that link succeeding.

use msfs::legacy::{AircraftVariable, NamedVariable};
use msfs::MSFSEvent;
use std::collections::BTreeMap;
use std::error::Error;

use fbw_a380_systems::deep::lvar_bridge::{Handle, Published, VarStore};
use fbw_a380_systems::deep::msfs_inputs::{Confidence, Convert, INPUTS};

/// How often the status line is printed, in frames: about ten seconds.
const STATUS_EVERY: u64 = 600;

/// The host's LVar table, behind the bridge's trait. `resolve` registers the
/// name once; `set`/`get` are the 11-16 ns operations the benchmark measured.
struct Lvars {
    names: Vec<String>,
    vars: Vec<NamedVariable>,
}

impl Lvars {
    fn new() -> Self {
        Self { names: Vec::new(), vars: Vec::new() }
    }
}

impl VarStore for Lvars {
    fn resolve(&mut self, name: &str) -> Handle {
        if let Some(i) = self.names.iter().position(|n| n == name) {
            return Handle(i);
        }
        self.vars.push(NamedVariable::from(name));
        self.names.push(name.to_owned());
        Handle(self.vars.len() - 1)
    }
    fn set(&mut self, handle: Handle, value: f64) {
        self.vars[handle.0].set_value(value);
    }
    fn get(&mut self, handle: Handle) -> f64 {
        self.vars[handle.0].get_value::<f64>()
    }
}

/// One `Truth` input the simulator agreed to provide.
struct HostInput {
    field: &'static str,
    lvar: String,
    var: AircraftVariable,
    convert: Convert,
    confidence: Confidence,
}

/// The LVar a converted input is published to: `DEEP_IN_PITCH_DEG` for the
/// `pitch_deg` field, so the dev toolbar's variable watch shows it by name.
fn lvar_name(field: &str) -> String {
    format!("DEEP_IN_{}", field.to_ascii_uppercase())
}

/// Ask the simulator for every row of the table. Nothing is assumed: a row
/// the simulator rejects is returned in `failed` with its reason, and the
/// one row with no source at all is returned in `unsourced`.
fn resolve_inputs() -> (Vec<HostInput>, Vec<&'static str>, Vec<(&'static str, String)>) {
    let mut ok = Vec::new();
    let mut unsourced = Vec::new();
    let mut failed = Vec::new();
    for input in INPUTS {
        let (name, unit) = input.msfs;
        if name.is_empty() {
            unsourced.push(input.field);
            continue;
        }
        match AircraftVariable::from(name, unit, 0) {
            Ok(var) => ok.push(HostInput {
                field: input.field,
                lvar: lvar_name(input.field),
                var,
                convert: input.convert,
                confidence: input.confidence,
            }),
            Err(e) => failed.push((input.field, format!("{name} [{unit}]: {e}"))),
        }
    }
    (ok, unsourced, failed)
}

struct Module {
    store: Lvars,
    inputs: Vec<HostInput>,
    /// Handles for the `DEEP_IN_*` values plus the module's own three.
    published: Published,
    frame: BTreeMap<String, f64>,
    frames: u64,
}

impl Module {
    fn new() -> Self {
        let (inputs, unsourced, failed) = resolve_inputs();

        println!("DEEP: {} of {} host inputs resolved", inputs.len(), INPUTS.len());
        for i in &inputs {
            let tag = match i.confidence {
                Confidence::Verified => "verified",
                Confidence::Unverified => "UNVERIFIED in the table; the simulator accepted the name and unit",
            };
            println!("DEEP:   {:<18} -> {:<28} ({tag})", i.field, i.lvar);
        }
        for f in &unsourced {
            println!("DEEP:   {f:<18} -> (no source: see msfs_inputs.rs; not published)");
        }
        for (f, why) in &failed {
            println!("DEEP:   {f:<18} -> REJECTED by the simulator: {why}");
        }

        let mut store = Lvars::new();
        let mut names: Vec<String> = inputs.iter().map(|i| i.lvar.clone()).collect();
        names.extend(["DEEP_ALIVE", "DEEP_FRAME", "DEEP_DT_MS"].map(String::from));
        let published = Published::resolve(&mut store, &names);
        let frame = names.iter().map(|n| (n.clone(), 0.0)).collect();

        Self { store, inputs, published, frame, frames: 0 }
    }

    fn tick(&mut self, dt_s: f64) {
        self.frames += 1;
        for i in &self.inputs {
            let raw: f64 = i.var.get();
            let value = i.convert.apply(raw);
            if let Some(slot) = self.frame.get_mut(&i.lvar) {
                *slot = value;
            }
        }
        self.frame.insert("DEEP_ALIVE".into(), 1.0);
        self.frame.insert("DEEP_FRAME".into(), self.frames as f64);
        self.frame.insert("DEEP_DT_MS".into(), dt_s * 1000.0);

        let written = self.published.write(&mut self.store, &self.frame);
        let missing = self.published.missing(&self.frame);
        if !missing.is_empty() {
            println!("DEEP: {} published names went unwritten this frame: {missing:?}", missing.len());
        }

        if self.frames % STATUS_EVERY == 1 {
            let mut line = format!("DEEP: frame {} dt {:.1} ms, {written} LVars written;", self.frames, dt_s * 1000.0);
            for i in &self.inputs {
                if let Some(v) = self.frame.get(&i.lvar) {
                    line.push_str(&format!(" {}={:.3}", i.field, v));
                }
            }
            println!("{line}");
        }
    }
}

#[msfs::gauge(name = deep)]
async fn deep(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    // The default panic output is lost when the WASM instance aborts, so
    // print it to the MSFS console first -- the same pattern FlyByWire's own
    // module uses (a380_systems_wasm/src/lib.rs).
    std::panic::set_hook(Box::new(|panic_info| {
        println!("DEEP PANIC: {panic_info}");
    }));

    println!("DEEP: module loaded (stage 0: inputs and bridge, no physics yet)");
    let mut module: Option<Module> = None;

    while let Some(event) = gauge.next_event().await {
        match event {
            MSFSEvent::PostInstall => println!("DEEP: post-install"),
            MSFSEvent::PreInitialize => {
                // Variables are resolved here rather than at load: the
                // simulator's variable tables are not guaranteed to be ready
                // before the panel initialises.
                module = Some(Module::new());
                // Stage 1's first honest question: does the deep layer --
                // all seventeen areas, every registry -- instantiate in this
                // runtime at all? Nothing is ticked yet; the count printed
                // is the same one the X-Plane plugin reports.
                let deep = fbw_a380_systems::deep::live::all_areas();
                println!("DEEP: the deep layer instantiated; it publishes {} names", deep.published_names().len());
            }
            MSFSEvent::PreDraw(data) => {
                if let Some(m) = module.as_mut() {
                    m.tick(data.delta_time().as_secs_f64());
                }
            }
            MSFSEvent::PreKill => println!("DEEP: pre-kill"),
            _ => {}
        }
    }
    Ok(())
}

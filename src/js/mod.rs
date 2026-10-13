//! A JavaScript and TypeScript engine for the plugin.
//!
//! QuickJS (through `rquickjs`) runs ES2023 JavaScript; Oxc turns TypeScript
//! and JSX into JavaScript as modules load. Modules resolve the way Node and
//! bundlers resolve them (`resolve.rs`). The environment scripts get
//! (`prelude.js`) is the part of MSFS's they rely on first: `console`,
//! timers and animation frames on the simulator's clock, and `SimVar`, which
//! reaches the plugin's variables through the [`Host`] the plugin passes in
//! each tick. It also shims a bare-bones `process.env` (still `prelude.js`)
//! for the two bundles that read it unguarded (the OIT views).
//!
//! Scripts run on the simulator's thread, inside a time budget per call:
//! a script that runs past it is interrupted rather than stalling X-Plane.
//! Memory is capped too.
//!
//! This module has no dependency on the rest of the plugin, so it can be
//! tested alone; `js_bridge.rs` connects it to the plugin's variables.

pub mod msfs;
// [dom] The DOM, SVG, CSS and Canvas2D the instruments draw with (src/js/dom).
pub mod dom;
pub mod resolve;
pub mod transpile;
pub mod units;

use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::ptr::NonNull;
use std::rc::Rc;
use std::time::{Duration, Instant, SystemTime};

use rquickjs::loader::{ImportAttributes, Loader, Resolver};
use rquickjs::module::Declared;
use rquickjs::promise::PromiseState;
use rquickjs::{CatchResultExt, Context, Ctx, Function, Module, Object, Promise, Runtime, Value};

pub use resolve::ImportMap;
pub use transpile::JsxConfig;

const PRELUDE: &str = include_str!("prelude.js");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogLevel {
    Info,
    Warn,
    Error,
}

/// What scripts reach outside the engine: the simulator's variables and its
/// log. Names are as MSFS instruments write them: `L:NAME` for aircraft
/// variables, `A:NAME:index` (or no prefix) for simulator variables,
/// `E:NAME` for environment variables, `K:EVENT` and `H:EVENT` for events.
///
/// Only the first three are required. The rest are what MSFS's runtime
/// (`msfs/`) builds on; a host without them answers as a simulator with
/// nothing behind that part would.
pub trait Host {
    fn get_var(&mut self, name: &str, unit: &str) -> f64;
    fn set_var(&mut self, name: &str, unit: &str, value: f64);
    fn log(&mut self, level: LogLevel, message: &str);
    /// A variable registered with the engine (`__host.registerVar`): `id`
    /// is the engine's number for this name and unit, so a host can keep
    /// what it resolved them to.
    fn get_var_reg(&mut self, _id: usize, name: &str, unit: &str) -> f64 {
        self.get_var(name, unit)
    }
    fn set_var_reg(&mut self, _id: usize, name: &str, unit: &str, value: f64) {
        self.set_var(name, unit, value)
    }
    /// A variable read or written in the `string` unit.
    fn get_string(&mut self, _name: &str) -> String {
        String::new()
    }
    fn set_string(&mut self, _name: &str, _value: &str) {}
    /// A Coherent call the scripts could not answer themselves; the
    /// arguments and a resolved value are JSON.
    fn call(&mut self, name: &str, _args_json: &str) -> CallReply {
        CallReply::Rejected(format!("Coherent.call('{name}'): nothing here answers this call"))
    }
    /// A Coherent trigger or flow event for the simulator side.
    fn trigger(&mut self, _name: &str, _args_json: &str) {}
    /// An event for the simulator: `name` as written (`K:NAME`, `K:2:NAME`,
    /// `H:NAME`) with its values (msfs-sdk `triggerKey` has three).
    fn send_event(&mut self, name: &str, values: &[f64]) {
        self.set_var(name, "number", values.first().copied().unwrap_or(0.));
    }
    /// A file under the view's `coui://html_ui/` root, by its absolute path
    /// there (`/Pages/VCockpit/...`).
    fn read_file(&mut self, path: &str) -> Result<String, String> {
        Err(format!("{path}: this host has no files"))
    }
    /// MSFS's stored data: `op` is `get`, `set`, `delete` or `search`;
    /// `get` answers the value (empty if none), `search` a JSON array of
    /// `{key, data}`.
    fn stored_data(&mut self, _op: &str, _key: &str, _value: &str) -> String {
        String::new()
    }
    /// The magnetic variation at a point, degrees east (`Facilities.getMagVar`,
    /// msfs.d.ts; MagVar.ts and navdata's Mapping.ts call it for waypoints and
    /// airports, not just the aircraft's own position). Without one, 0.
    fn get_magvar(&mut self, _lat: f64, _lon: f64) -> f64 {
        0.
    }
}

/// How the host answers a Coherent call.
#[derive(Clone, Debug, PartialEq)]
pub enum CallReply {
    /// The promise resolves with this JSON value (empty for `undefined`).
    Resolved(String),
    /// The promise rejects with this message.
    Rejected(String),
    /// The answer comes later, delivered to the scripts under this id.
    Pending(u64),
}

thread_local! {
    static CURRENT_HOST: Cell<Option<NonNull<dyn Host>>> = const { Cell::new(None) };
    static UNHOSTED_LOG: RefCell<Vec<(LogLevel, String)>> = const { RefCell::new(Vec::new()) };
}

/// Makes `host` the one scripts reach for the duration of `f`.
fn with_host<R>(host: &mut dyn Host, f: impl FnOnce() -> R) -> R {
    struct Restore(Option<NonNull<dyn Host>>);
    impl Drop for Restore {
        fn drop(&mut self) {
            CURRENT_HOST.with(|c| c.set(self.0));
        }
    }
    // The pointer only lives while `host` is borrowed here: it is cleared
    // before this function returns, even on a panic.
    let ptr: NonNull<dyn Host + '_> = NonNull::from(host);
    let ptr: NonNull<dyn Host> = unsafe { std::mem::transmute(ptr) };
    let _restore = Restore(CURRENT_HOST.with(|c| c.replace(Some(ptr))));
    f()
}

fn host_call<R>(f: impl FnOnce(&mut dyn Host) -> R) -> Option<R> {
    CURRENT_HOST.with(|c| c.get()).map(|mut ptr| f(unsafe { ptr.as_mut() }))
}

fn host_log(level: i32, message: String) {
    let level = match level {
        1 => LogLevel::Warn,
        2 => LogLevel::Error,
        _ => LogLevel::Info,
    };
    if host_call(|h| h.log(level, &message)).is_none() {
        UNHOSTED_LOG.with(|l| l.borrow_mut().push((level, message)));
    }
}

/// How the engine is set up.
#[derive(Clone, Debug)]
pub struct EngineOptions {
    /// Longest a single call into scripts may run.
    pub budget: Duration,
    /// Longest a script file may take to load and run its top level:
    /// FlyByWire's bundles are megabytes of JavaScript.
    pub load_budget: Duration,
    /// Heap limit in bytes.
    pub memory_limit: usize,
    /// Where bare specifiers and evaluated code resolve from.
    pub root: PathBuf,
    pub import_map: ImportMap,
    pub jsx: JsxConfig,
}

impl Default for EngineOptions {
    fn default() -> Self {
        Self {
            budget: Duration::from_millis(8),
            load_budget: Duration::from_secs(30),
            memory_limit: 256 * 1024 * 1024,
            root: std::env::current_dir().unwrap_or_default(),
            import_map: ImportMap::new(),
            jsx: JsxConfig::default(),
        }
    }
}

/// Transpiled sources, kept until their file changes.
#[derive(Default)]
struct Cache {
    entries: HashMap<PathBuf, (SystemTime, String)>,
}

struct FileResolver {
    map: ImportMap,
    root: PathBuf,
}

impl Resolver for FileResolver {
    fn resolve<'js>(&mut self, _ctx: &Ctx<'js>, base: &str, name: &str, _attributes: Option<ImportAttributes<'js>>) -> rquickjs::Result<String> {
        resolve::resolve(base, name, &self.map, &self.root)
            .map_err(|message| rquickjs::Error::new_resolving_message(base, name, message))
    }
}

struct FileLoader {
    jsx: JsxConfig,
    cache: Rc<RefCell<Cache>>,
}

impl Loader for FileLoader {
    fn load<'js>(&mut self, ctx: &Ctx<'js>, name: &str, _attributes: Option<ImportAttributes<'js>>) -> rquickjs::Result<Module<'js, Declared>> {
        let code = load_source(Path::new(name), &self.jsx, &self.cache)
            .map_err(|message| rquickjs::Error::new_loading_message(name, message))?;
        Module::declare(ctx.clone(), name, code)
    }
}

/// A file's JavaScript: read, transpiled if it needs it, cached by mtime.
fn load_source(path: &Path, jsx: &JsxConfig, cache: &Rc<RefCell<Cache>>) -> Result<String, String> {
    let modified = std::fs::metadata(path).and_then(|m| m.modified()).map_err(|e| format!("{}: {e}", path.display()))?;
    if let Some((stamp, code)) = cache.borrow().entries.get(path) {
        if *stamp == modified {
            return Ok(code.clone());
        }
    }
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    let code = if ext == "json" {
        format!("export default {text};")
    } else if transpile::needs_transpile(path) {
        transpile::transpile(path, &text, jsx)?
    } else {
        text
    };
    cache.borrow_mut().entries.insert(path.to_path_buf(), (modified, code.clone()));
    Ok(code)
}

/// The engine: one QuickJS runtime and context.
pub struct Engine {
    // Owns the context and its memory; read only by garbage collection.
    #[allow(dead_code)]
    runtime: Runtime,
    context: Context,
    deadline: Rc<Cell<Option<Instant>>>,
    budget: Duration,
    load_budget: Duration,
    jsx: JsxConfig,
}

impl Engine {
    pub fn new(options: EngineOptions) -> Result<Self, String> {
        let runtime = Runtime::new().map_err(|e| e.to_string())?;
        runtime.set_memory_limit(options.memory_limit);
        let deadline: Rc<Cell<Option<Instant>>> = Rc::new(Cell::new(None));
        let watch = deadline.clone();
        runtime.set_interrupt_handler(Some(Box::new(move || watch.get().is_some_and(|d| Instant::now() > d))));
        let cache = Rc::new(RefCell::new(Cache::default()));
        runtime.set_loader(
            FileResolver { map: options.import_map.clone(), root: options.root.clone() },
            FileLoader { jsx: options.jsx.clone(), cache },
        );
        let context = Context::full(&runtime).map_err(|e| e.to_string())?;

        let registry: Rc<RefCell<Registry>> = Rc::default();
        context.with(|ctx| -> Result<(), String> {
            let host = host_object(&ctx, &registry, &deadline, options.load_budget).map_err(|e| e.to_string())?;
            ctx.globals().set("__host", host).map_err(|e| e.to_string())?;
            ctx.eval::<(), _>(PRELUDE).catch(&ctx).map_err(|e| e.to_string())
        })?;

        Ok(Self { runtime, context, deadline, budget: options.budget, load_budget: options.load_budget, jsx: options.jsx })
    }

    /// Run `f` with the script deadline armed.
    fn budgeted<R>(&self, f: impl FnOnce() -> R) -> R {
        self.deadline.set(Some(Instant::now() + self.budget));
        let r = f();
        self.deadline.set(None);
        r
    }

    // [display] Added for the screens (src/display, registered in
    // js_bridge.rs): lets the plugin put more host functions on `__host`.
    /// Run `f` with the engine's context and its `__host` object.
    pub fn with_host_object(&self, f: impl for<'js> FnOnce(&Ctx<'js>, &Object<'js>) -> rquickjs::Result<()>) -> Result<(), String> {
        self.context.with(|ctx| {
            let host: Object = ctx.globals().get("__host").map_err(|e| e.to_string())?;
            f(&ctx, &host).catch(&ctx).map_err(|e| e.to_string())
        })
    }

    /// Evaluate a script and return its completion value as a string.
    pub fn eval(&self, name: &str, source: &str) -> Result<String, String> {
        self.budgeted(|| {
            self.context.with(|ctx| {
                let mut options = rquickjs::context::EvalOptions::default();
                options.global = true;
                options.strict = false;
                options.filename = Some(name.to_string());
                let value: Value = ctx.eval_with_options(source, options).catch(&ctx).map_err(|e| e.to_string())?;
                stringify(&ctx, value)
            })
        })
    }

    /// Evaluate TypeScript (or TSX, by the name's extension) as a script.
    pub fn eval_typescript(&self, name: &str, source: &str) -> Result<String, String> {
        let path = if Path::new(name).extension().is_some() { PathBuf::from(name) } else { PathBuf::from(format!("{name}.ts")) };
        let js = transpile::transpile(&path, source, &self.jsx)?;
        self.eval(name, &js)
    }

    /// Import a module file (and everything it imports), running its top
    /// level. A module still awaiting something (a timer, say) finishes on
    /// later ticks.
    pub fn load_module(&self, host: &mut dyn Host, path: &Path) -> Result<(), String> {
        let name = resolve::name_of(&std::path::absolute(path).map_err(|e| e.to_string())?);
        with_host(host, || {
            self.budgeted(|| {
                self.context.with(|ctx| -> Result<(), String> {
                    let promise: Promise = Module::import(&ctx, name.clone()).catch(&ctx).map_err(|e| e.to_string())?;
                    self.drain_jobs(&ctx);
                    match promise.state() {
                        PromiseState::Rejected => {
                            let err = promise.result::<()>().unwrap_or(Ok(())).catch(&ctx);
                            Err(err.err().map_or_else(|| format!("{name} failed to load"), |e| e.to_string()))
                        }
                        _ => Ok(()),
                    }
                })
            })
        })
    }

    /// Advance the scripts' clock to `now_ms`: due timers and animation
    /// frames run, then pending promise jobs.
    pub fn tick(&self, host: &mut dyn Host, now_ms: f64) -> Result<(), String> {
        with_host(host, || {
            self.budgeted(|| {
                self.context.with(|ctx| -> Result<(), String> {
                    let tick: Function = ctx.globals().get("__tick").map_err(|e| e.to_string())?;
                    tick.call::<_, ()>((now_ms,)).catch(&ctx).map_err(|e| e.to_string())?;
                    self.drain_jobs(&ctx);
                    Ok(())
                })
            })
        })
    }

    /// Run pending promise jobs within the budget. A job that throws is
    /// reported through the log rather than stopping the others.
    fn drain_jobs(&self, ctx: &Ctx<'_>) {
        while ctx.execute_pending_job() {
            if self.deadline.get().is_some_and(|d| Instant::now() > d) {
                break;
            }
        }
    }

    /// Run a classic script (not a module) in the global scope, as a
    /// `<script>` element runs it, within the load budget.
    pub fn run_script(&self, host: &mut dyn Host, name: &str, source: &str) -> Result<(), String> {
        with_host(host, || {
            self.deadline.set(Some(Instant::now() + self.load_budget));
            let r = self.context.with(|ctx| -> Result<(), String> {
                eval_global(&ctx, name, source)?;
                self.drain_jobs(&ctx);
                Ok(())
            });
            self.deadline.set(None);
            r
        })
    }

    /// Call the global function `name` with string arguments, then run the
    /// promise jobs it queued.
    pub fn invoke(&self, host: &mut dyn Host, name: &str, args: &[&str]) -> Result<(), String> {
        with_host(host, || {
            self.budgeted(|| {
                self.context.with(|ctx| -> Result<(), String> {
                    let f: Function = ctx.globals().get(name).map_err(|e| format!("{name}: {e}"))?;
                    let args: Vec<String> = args.iter().map(|a| a.to_string()).collect();
                    let r = f.call::<_, ()>((rquickjs::function::Rest(args),)).catch(&ctx).map_err(|e| e.to_string());
                    self.drain_jobs(&ctx);
                    r
                })
            })
        })
    }

    /// Bytes the engine's heap holds.
    pub fn memory_used(&self) -> usize {
        self.runtime.memory_usage().memory_used_size.max(0) as usize
    }

    /// Log lines written while no host was set.
    #[allow(dead_code)]
    pub fn take_unhosted_log(&self) -> Vec<(LogLevel, String)> {
        UNHOSTED_LOG.with(|l| std::mem::take(&mut *l.borrow_mut()))
    }

    /// Run a garbage collection.
    #[allow(dead_code)]
    pub fn collect_garbage(&self) {
        self.runtime.run_gc();
    }
}

/// The engine's registered variables: MSFS's `simvar.registerSimVarWatcher`
/// numbers a name and unit once, and reads by number after that.
#[derive(Default)]
struct Registry {
    ids: HashMap<(String, String), usize>,
    entries: Vec<(String, String)>,
}

impl Registry {
    fn register(&mut self, name: String, unit: String) -> usize {
        let key = (name, unit);
        if let Some(&id) = self.ids.get(&key) {
            return id;
        }
        let id = self.entries.len();
        self.entries.push(key.clone());
        self.ids.insert(key, id);
        id
    }

    fn entry(registry: &RefCell<Self>, id: usize) -> Option<(String, String)> {
        registry.borrow().entries.get(id).cloned()
    }
}

fn eval_global(ctx: &Ctx<'_>, name: &str, source: &str) -> Result<(), String> {
    let mut options = rquickjs::context::EvalOptions::default();
    options.global = true;
    options.strict = false;
    options.filename = Some(name.to_string());
    ctx.eval_with_options::<(), _>(source, options).catch(ctx).map_err(|e| e.to_string())
}

fn throw(ctx: &Ctx<'_>, message: &str) -> rquickjs::Error {
    rquickjs::Exception::throw_message(ctx, message)
}

// The functions on `__host`, as plain functions so their lifetimes are
// their own.

fn js_get_var(name: String, unit: String) -> f64 {
    host_call(|h| h.get_var(&name, &unit)).unwrap_or(0.)
}

fn js_set_var(name: String, unit: String, value: f64) {
    host_call(|h| h.set_var(&name, &unit, value));
}

fn js_get_string(name: String) -> String {
    host_call(|h| h.get_string(&name)).unwrap_or_default()
}

fn js_set_string(name: String, value: String) {
    host_call(|h| h.set_string(&name, &value));
}

/// The status digit, then the payload: `0` and the resolved JSON, `1` and
/// the rejection message, `2` and the id the answer will come under.
fn js_call(name: String, args: String) -> String {
    match host_call(|h| h.call(&name, &args)) {
        Some(CallReply::Resolved(json)) => format!("0{json}"),
        Some(CallReply::Rejected(message)) => format!("1{message}"),
        Some(CallReply::Pending(id)) => format!("2{id}"),
        None => format!("1Coherent.call('{name}'): no host"),
    }
}

fn js_trigger(name: String, args: String) {
    host_call(|h| h.trigger(&name, &args));
}

fn js_stored_data(op: String, key: String, value: rquickjs::function::Opt<String>) -> String {
    host_call(|h| h.stored_data(&op, &key, value.0.as_deref().unwrap_or(""))).unwrap_or_default()
}

fn js_get_magvar(lat: f64, lon: f64) -> f64 {
    host_call(|h| h.get_magvar(lat, lon)).unwrap_or(0.)
}

fn js_read_file<'js>(ctx: Ctx<'js>, path: String) -> rquickjs::Result<String> {
    match host_call(|h| h.read_file(&path)) {
        Some(Ok(text)) => Ok(text),
        Some(Err(e)) => Err(throw(&ctx, &e)),
        None => Err(throw(&ctx, &format!("{path}: no host"))),
    }
}

/// `__host`: what the scripts reach of the plugin.
fn host_object<'js>(
    ctx: &Ctx<'js>,
    registry: &Rc<RefCell<Registry>>,
    deadline: &Rc<Cell<Option<Instant>>>,
    load_budget: Duration,
) -> rquickjs::Result<Object<'js>> {
    let host = Object::new(ctx.clone())?;
    host.set("log", Function::new(ctx.clone(), host_log)?)?;
    host.set("getVar", Function::new(ctx.clone(), js_get_var)?)?;
    host.set("setVar", Function::new(ctx.clone(), js_set_var)?)?;
    host.set("getString", Function::new(ctx.clone(), js_get_string)?)?;
    host.set("setString", Function::new(ctx.clone(), js_set_string)?)?;
    host.set("call", Function::new(ctx.clone(), js_call)?)?;
    host.set("trigger", Function::new(ctx.clone(), js_trigger)?)?;
    host.set("storedData", Function::new(ctx.clone(), js_stored_data)?)?;
    host.set("readFile", Function::new(ctx.clone(), js_read_file)?)?;
    host.set("getMagVar", Function::new(ctx.clone(), js_get_magvar)?)?;

    let r = registry.clone();
    let register = move |name: String, unit: String| r.borrow_mut().register(name, unit) as f64;
    host.set("registerVar", Function::new(ctx.clone(), register)?)?;
    let r = registry.clone();
    let get_reg = move |id: usize| -> f64 {
        Registry::entry(&r, id).and_then(|(name, unit)| host_call(|h| h.get_var_reg(id, &name, &unit))).unwrap_or(0.)
    };
    host.set("getReg", Function::new(ctx.clone(), get_reg)?)?;
    let r = registry.clone();
    let set_reg = move |id: usize, value: f64| {
        if let Some((name, unit)) = Registry::entry(&r, id) {
            host_call(|h| h.set_var_reg(id, &name, &unit, value));
        }
    };
    host.set("setReg", Function::new(ctx.clone(), set_reg)?)?;
    let r = registry.clone();
    let get_reg_string =
        move |id: usize| -> String { Registry::entry(&r, id).and_then(|(name, _)| host_call(|h| h.get_string(&name))).unwrap_or_default() };
    host.set("getRegString", Function::new(ctx.clone(), get_reg_string)?)?;
    let r = registry.clone();
    let set_reg_string = move |id: usize, value: String| {
        if let Some((name, _)) = Registry::entry(&r, id) {
            host_call(|h| h.set_string(&name, &value));
        }
    };
    host.set("setRegString", Function::new(ctx.clone(), set_reg_string)?)?;

    // Run a script file in the global scope, as <script src> does. A load
    // is allowed the load budget wherever it happens, and the time it took
    // is not held against the call around it.
    let watch = deadline.clone();
    host.set(
        "runScript",
        Function::new(ctx.clone(), move |ctx: Ctx<'js>, path: String| -> rquickjs::Result<()> {
            let source = js_read_file(ctx.clone(), path.clone())?;
            let outer = watch.get();
            let started = Instant::now();
            watch.set(Some(started + load_budget));
            let mut options = rquickjs::context::EvalOptions::default();
            options.global = true;
            options.strict = false;
            options.filename = Some(path);
            let r = ctx.eval_with_options::<(), _>(source, options);
            watch.set(outer.map(|d| d + started.elapsed()));
            r
        })?,
    )?;
    Ok(host)
}

fn stringify<'js>(ctx: &Ctx<'js>, value: Value<'js>) -> Result<String, String> {
    if value.is_undefined() {
        return Ok("undefined".into());
    }
    let string: Function = ctx.globals().get("String").map_err(|e| e.to_string())?;
    string.call::<_, String>((value,)).catch(ctx).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct FakeHost {
        vars: HashMap<String, f64>,
        events: Vec<(String, f64)>,
        log: Vec<(LogLevel, String)>,
    }

    impl Host for FakeHost {
        fn get_var(&mut self, name: &str, _unit: &str) -> f64 {
            self.vars.get(name).copied().unwrap_or(0.)
        }
        fn set_var(&mut self, name: &str, _unit: &str, value: f64) {
            if name.starts_with("K:") || name.starts_with("H:") {
                self.events.push((name.to_string(), value));
            } else {
                self.vars.insert(name.to_string(), value);
            }
        }
        fn log(&mut self, level: LogLevel, message: &str) {
            self.log.push((level, message.to_string()));
        }
    }

    fn engine_in(root: &Path) -> Engine {
        Engine::new(EngineOptions { root: root.to_path_buf(), ..Default::default() }).unwrap()
    }

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("fbw-js-engine-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn javascript_runs() {
        let e = engine_in(&std::env::temp_dir());
        assert_eq!(e.eval("t", "[1,2,3].map(x => x * 2).join(',')").unwrap(), "2,4,6");
    }

    /// The OIT views' bundle (A380X/OIT/oit.js, compiled from
    /// fbw-common/src/systems/instruments/src/navigraph.ts) reads
    /// `process.env.CLIENT_ID`/`process.env.CLIENT_SECRET` with no
    /// `typeof process !== 'undefined'` guard, unlike everything else that
    /// touches `process` in that bundle; with no `process` global at all
    /// that used to throw `ReferenceError: process is not defined` and take
    /// the whole view down before it rendered anything (views 15/16 in
    /// X-Plane's log). `prelude.js`'s shim only needs to make `process`
    /// exist with an `env` object so those two reads fall through to
    /// `undefined`, same as real Node would with the variables unset.
    #[test]
    fn process_env_reads_are_undefined_not_a_referenceerror() {
        let e = engine_in(&std::env::temp_dir());
        assert_eq!(e.eval("t", "typeof process").unwrap(), "object");
        assert_eq!(e.eval("t", "typeof process.env").unwrap(), "object");
        assert_eq!(e.eval("t", "String(process.env.CLIENT_ID)").unwrap(), "undefined");
        assert_eq!(e.eval("t", "String(process.env.CLIENT_SECRET)").unwrap(), "undefined");
        // The shim must stay minimal: nothing beyond `env` should appear.
        assert_eq!(e.eval("t", "Object.keys(process).join(',')").unwrap(), "env");
    }

    #[test]
    fn typescript_runs_with_classes_enums_and_generics() {
        let e = engine_in(&std::env::temp_dir());
        let out = e
            .eval_typescript(
                "t.ts",
                "enum Phase { Preflight, Takeoff = 5 }\n\
                 class Box<T> { constructor(private readonly v: T) {} get(): T { return this.v; } }\n\
                 const b: Box<number> = new Box(Phase.Takeoff);\n\
                 `${b.get()}-${Phase[0]}`;",
            )
            .unwrap();
        assert_eq!(out, "5-Preflight");
    }

    #[test]
    fn errors_come_back_with_their_message() {
        let e = engine_in(&std::env::temp_dir());
        let err = e.eval("t", "throw new Error('boom')").unwrap_err();
        assert!(err.contains("boom"), "{err}");
    }

    #[test]
    fn a_runaway_script_is_interrupted() {
        let e = Engine::new(EngineOptions { budget: Duration::from_millis(30), ..Default::default() }).unwrap();
        let started = Instant::now();
        let err = e.eval("t", "while (true) {}").unwrap_err();
        assert!(started.elapsed() < Duration::from_secs(2));
        assert!(err.to_lowercase().contains("interrupt"), "{err}");
        // The engine still works afterwards.
        assert_eq!(e.eval("t", "1+1").unwrap(), "2");
    }

    #[test]
    fn simvars_and_events_reach_the_host() {
        let e = engine_in(&std::env::temp_dir());
        let mut host = FakeHost::default();
        host.vars.insert("L:A32NX_ENGINE_N1:1".into(), 42.5);
        let dir = temp("simvar");
        std::fs::write(
            dir.join("main.ts"),
            "const n1: number = SimVar.GetSimVarValue('L:A32NX_ENGINE_N1:1', 'number');\n\
             SimVar.SetSimVarValue('L:A32NX_OUT', 'number', n1 * 2);\n\
             SimVar.SetSimVarValue('H:A32NX_FCU_SPD_PUSH', 'number', 1);\n\
             console.warn('n1 is', n1);\n",
        )
        .unwrap();
        e.load_module(&mut host, &dir.join("main.ts")).unwrap();
        assert_eq!(host.vars.get("L:A32NX_OUT"), Some(&85.0));
        assert_eq!(host.events, vec![("H:A32NX_FCU_SPD_PUSH".to_string(), 1.0)]);
        assert_eq!(host.log, vec![(LogLevel::Warn, "n1 is 42.5".to_string())]);
    }

    #[test]
    fn modules_import_typescript_json_and_packages() {
        let dir = temp("modules");
        std::fs::create_dir_all(dir.join("lib")).unwrap();
        std::fs::create_dir_all(dir.join("node_modules/units")).unwrap();
        std::fs::write(dir.join("lib/math.ts"), "export const twice = (x: number): number => x * 2;").unwrap();
        std::fs::write(dir.join("config.json"), r#"{ "gain": 21 }"#).unwrap();
        std::fs::write(dir.join("node_modules/units/package.json"), r#"{ "main": "index.js" }"#).unwrap();
        std::fs::write(dir.join("node_modules/units/index.js"), "export const kt = 'kt';").unwrap();
        std::fs::write(
            dir.join("main.ts"),
            "import { twice } from './lib/math';\nimport cfg from './config.json';\nimport { kt } from 'units';\n\
             (globalThis as any).result = `${twice(cfg.gain)} ${kt}`;",
        )
        .unwrap();
        let e = engine_in(&dir);
        e.load_module(&mut FakeHost::default(), &dir.join("main.ts")).unwrap();
        assert_eq!(e.eval("t", "globalThis.result").unwrap(), "42 kt");
    }

    #[test]
    fn tsx_components_build_through_the_framework_factory() {
        let dir = temp("tsx");
        std::fs::write(
            dir.join("main.tsx"),
            "const FSComponent = { buildComponent: (tag: string, props: any, ...children: any[]) => ({ tag, props, children }), Fragment: 'frag' };\n\
             const node: any = <g id=\"n1\"><text>N1</text></g>;\n\
             (globalThis as any).tree = JSON.stringify(node);",
        )
        .unwrap();
        let e = engine_in(&dir);
        e.load_module(&mut FakeHost::default(), &dir.join("main.tsx")).unwrap();
        let tree = e.eval("t", "globalThis.tree").unwrap();
        assert!(tree.contains("\"tag\":\"g\"") && tree.contains("\"id\":\"n1\""), "{tree}");
    }

    #[test]
    fn timers_and_promises_follow_the_simulator_clock() {
        let e = engine_in(&std::env::temp_dir());
        let mut host = FakeHost::default();
        e.eval(
            "t",
            "globalThis.log = [];\n\
             setTimeout(() => log.push('once'), 100);\n\
             const id = setInterval(() => { log.push('tick'); if (log.filter(x => x === 'tick').length === 3) clearInterval(id); }, 50);\n\
             (async () => { await new Promise(r => setTimeout(r, 120)); log.push('awaited'); })();\n\
             requestAnimationFrame(t => log.push('frame ' + t));",
        )
        .unwrap();
        for ms in [16., 60., 110., 130., 200., 400.] {
            e.tick(&mut host, ms).unwrap();
        }
        let log = e.eval("t", "log.join(',')").unwrap();
        // Two timers due at the same time fire in the order they were set, as in a browser.
        assert_eq!(log, "frame 16,tick,once,tick,awaited,tick");
    }

    #[test]
    fn a_failing_module_reports_why() {
        let dir = temp("failing");
        std::fs::write(dir.join("main.ts"), "import './missing';").unwrap();
        let e = engine_in(&dir);
        let err = e.load_module(&mut FakeHost::default(), &dir.join("main.ts")).unwrap_err();
        assert!(err.contains("missing"), "{err}");
    }
}

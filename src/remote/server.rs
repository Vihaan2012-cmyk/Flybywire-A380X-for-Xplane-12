//! The systems process: FlyByWire's `Simulation<A380>`, ticked on the
//! plugin's request over the shared block (wire.rs).

use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use a380_systems::A380;
use rustc_hash::FxHashSet;
use systems::simulation::{Simulation, SimulatorReaderWriter, StartState, VariableIdentifier, VariableRegistry};

use super::win::{self, Event, Process, Shared};
use super::wire::{self, Block};

/// What this process's `serve` loop has done so far: for XPHFBW.exe's own
/// settings page (app/src/web.rs `status`), which runs `serve` on a thread of
/// itself and has no other way to see inside it (docs/briefs/xphfbw-js-bridge.md
/// agent A). The wire protocol (wire.rs) is unchanged; these are process-local
/// counters, not part of the shared block.
pub static TICKS_SERVED: AtomicU64 = AtomicU64::new(0);
/// The most recent tick's `Simulation::tick` duration, microseconds (the same
/// figure written to the shared block's `tick_micros`).
pub static LAST_TICK_MICROS: AtomicU64 = AtomicU64::new(0);
/// How many variables the aircraft registered, once built.
pub static VARIABLE_COUNT: AtomicU32 = AtomicU32::new(0);

/// Names in the order the systems register them, each an index into the
/// shared values.
#[derive(Default)]
struct Recorder {
    names: Vec<(bool, String)>,
    index: HashMap<(bool, String), usize>,
}

impl Recorder {
    fn add(&mut self, unprefixed: bool, name: String) -> VariableIdentifier {
        let key = (unprefixed, name);
        let n = self.names.len();
        let i = *self.index.entry(key.clone()).or_insert_with(|| {
            self.names.push(key);
            n
        });
        identifier(i)
    }
}

fn identifier(i: usize) -> VariableIdentifier {
    let mut id = VariableIdentifier::new::<usize>(0);
    for _ in 0..i {
        id = id.next();
    }
    id
}

impl VariableRegistry for Recorder {
    fn get(&mut self, name: String) -> VariableIdentifier {
        self.add(false, name)
    }
    fn get_unprefixed(&mut self, name: String) -> VariableIdentifier {
        self.add(true, name)
    }
}

/// Reads and writes straight into the shared values, marking what was written.
pub(super) struct SharedValues<'a> {
    pub values: &'a mut [f64],
    pub written: &'a mut [u64],
}

impl SimulatorReaderWriter for SharedValues<'_> {
    fn read(&mut self, id: &VariableIdentifier) -> f64 {
        self.values.get(id.identifier_index()).copied().unwrap_or(0.)
    }
    fn write(&mut self, id: &VariableIdentifier, value: f64) {
        let i = id.identifier_index();
        if let Some(v) = self.values.get_mut(i) {
            *v = value;
            self.written[i / 64] |= 1 << (i % 64);
        }
    }
}

/// Registration for construction: names recorded, writes marked in the block.
struct Construction<'a> {
    recorder: &'a mut Recorder,
    shared: SharedValues<'a>,
}

impl VariableRegistry for Construction<'_> {
    fn get(&mut self, name: String) -> VariableIdentifier {
        self.recorder.get(name)
    }
    fn get_unprefixed(&mut self, name: String) -> VariableIdentifier {
        self.recorder.get_unprefixed(name)
    }
}

impl SimulatorReaderWriter for Construction<'_> {
    fn read(&mut self, id: &VariableIdentifier) -> f64 {
        self.shared.read(id)
    }
    fn write(&mut self, id: &VariableIdentifier, value: f64) {
        self.shared.write(id, value)
    }
}

pub fn start_state_from_number(n: u32) -> StartState {
    StartState::from(n as f64)
}

/// Serve one plugin until it quits or its process goes away.
pub fn serve(tag: &str, parent: Option<u32>) -> Result<(), String> {
    let (shm_name, req_name, rep_name) = wire::names(tag);
    let shared = Shared::open(&shm_name, wire::TOTAL_BYTES).ok_or("cannot open the shared block")?;
    let request = Event::open(&req_name).ok_or("cannot open the request event")?;
    let reply = Event::open(&rep_name).ok_or("cannot open the reply event")?;
    let parent = parent.and_then(Process::open);
    let mut block = unsafe { Block::new(shared.ptr()) };
    if block.header().magic.load(Ordering::Relaxed) != wire::MAGIC || block.header().version.load(Ordering::Relaxed) != wire::VERSION {
        return Err("the shared block is not this version's".into());
    }

    // Build the aircraft, recording every variable it registers.
    let start = start_state_from_number(block.header().start_state.load(Ordering::Relaxed));
    let mut recorder = Recorder::default();
    let built = {
        let values = block.values();
        let written = block.written();
        let mut construction = Construction { recorder: &mut recorder, shared: SharedValues { values, written } };
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| Simulation::new(start, A380::new, &mut construction)))
    };
    let mut simulation = match built {
        Ok(s) => s,
        Err(_) => {
            block.set_message("the systems panicked while being built");
            block.header().state.store(wire::STATE_FAILED, Ordering::Relaxed);
            reply.set();
            return Err("build panicked".into());
        }
    };
    if recorder.names.len() > wire::CAPACITY {
        block.set_message(&format!("{} variables, more than the {} the block holds", recorder.names.len(), wire::CAPACITY));
        block.header().state.store(wire::STATE_FAILED, Ordering::Relaxed);
        reply.set();
        return Err("too many variables".into());
    }
    let encoded = wire::encode_names(&recorder.names);
    if encoded.len() > wire::NAMES_BYTES {
        block.set_message("the variable names do not fit the block");
        block.header().state.store(wire::STATE_FAILED, Ordering::Relaxed);
        reply.set();
        return Err("names too long".into());
    }
    block.names_mut()[..encoded.len()].copy_from_slice(&encoded);
    let h = block.header();
    h.names_len.store(encoded.len() as u32, Ordering::Relaxed);
    h.count.store(recorder.names.len() as u32, Ordering::Relaxed);
    h.state.store(wire::STATE_READY, Ordering::Relaxed);
    VARIABLE_COUNT.store(recorder.names.len() as u32, Ordering::Relaxed);
    reply.set();

    let catalogue = crate::failures::catalogue_types();
    let mut failed = false;
    loop {
        let objects: Vec<win::Handle> = match &parent {
            Some(p) => vec![request.handle(), p.handle()],
            None => vec![request.handle()],
        };
        match win::wait_any(&objects, 1000) {
            Some(0) => {}
            Some(_) => return Ok(()), // the plugin's process ended
            None => continue,
        }
        let h = block.header();
        if h.state.load(Ordering::Relaxed) == wire::STATE_QUIT {
            return Ok(());
        }
        let seq = h.request.load(Ordering::Relaxed);
        if !failed {
            if h.failures_dirty.swap(0, Ordering::Relaxed) != 0 {
                let n = (h.failures_len.load(Ordering::Relaxed) as usize).min(wire::MAX_FAILURES);
                let set: FxHashSet<_> =
                    h.failures[..n].iter().filter_map(|i| catalogue.get(i.load(Ordering::Relaxed) as usize).copied()).collect();
                simulation.update_active_failures(set);
            }
            let delta = wire::bits_to_f64(h.delta.load(Ordering::Relaxed));
            let time = wire::bits_to_f64(h.time.load(Ordering::Relaxed));
            let written = block.written();
            written.iter_mut().for_each(|w| *w = 0);
            let started = Instant::now();
            let ticked = {
                let mut rw = SharedValues { values: block.values(), written: block.written() };
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    simulation.tick(Duration::from_secs_f64(delta.max(0.)), time, &mut rw)
                }))
            };
            let tick_micros = started.elapsed().as_micros() as u64;
            block.header().tick_micros.store(tick_micros, Ordering::Relaxed);
            LAST_TICK_MICROS.store(tick_micros, Ordering::Relaxed);
            TICKS_SERVED.fetch_add(1, Ordering::Relaxed);
            if ticked.is_err() {
                failed = true;
                block.set_message("the systems panicked during a tick; they are stopped");
                block.header().state.store(wire::STATE_FAILED, Ordering::Relaxed);
            }
        }
        block.header().reply.store(seq, Ordering::Relaxed);
        reply.set();
    }
}

/// Wait for a request with no process to watch (tests).
#[allow(dead_code)]
pub fn wait_request(event: &Event) -> bool {
    event.wait(win::INFINITE) == win::WAIT_OBJECT_0
}

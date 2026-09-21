//! The plugin's side of the bridge: starts the systems process, mirrors the
//! variables it registered, and ticks it in lockstep with the frame.

use std::process::{Child, Command};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use rustc_hash::FxHashSet;
use systems::failures::FailureType;
use systems::simulation::{SimulatorReaderWriter, StartState, VariableIdentifier, VariableRegistry};

use super::win::{self, Event, Shared};
use super::wire::{self, Block};

/// How long a frame waits for the systems before going on with last frame's
/// values (a stalled or dying process must not freeze X-Plane).
const TICK_TIMEOUT_MS: u32 = 100;
/// How long the systems may take to build the aircraft.
const START_TIMEOUT: Duration = Duration::from_secs(60);

pub fn start_state_number(state: StartState) -> u32 {
    match state {
        StartState::Hangar => 1,
        StartState::Apron => 2,
        StartState::Taxi => 3,
        StartState::Runway => 4,
        StartState::Climb => 5,
        StartState::Cruise => 6,
        StartState::Approach => 7,
        StartState::Final => 8,
    }
}

/// Round-trip timing, reported now and then.
#[derive(Default)]
pub struct Stats {
    pub ticks: u64,
    pub late: u64,
    pub round_trip_us: u64,
    pub max_round_trip_us: u64,
    pub systems_us: u64,
}

pub struct RemoteSystems {
    // Field order: the views go before the mapping they point into.
    block: Block,
    request: Event,
    reply: Event,
    _shared: Shared,
    child: Option<Child>,
    map: Vec<VariableIdentifier>,
    seq: u64,
    pending: bool,
    failed_logged: bool,
    catalogue: Vec<FailureType>,
    pub stats: Stats,
}

impl RemoteSystems {
    /// Start `exe` and connect to it.
    pub fn start<V: VariableRegistry + SimulatorReaderWriter>(exe: &std::path::Path, state: StartState, vars: &mut V) -> Result<Self, String> {
        let exe = exe.to_path_buf();
        Self::connect(state, vars, move |tag| {
            let mut cmd = Command::new(&exe);
            cmd.arg(tag).arg(std::process::id().to_string());
            // Keep the systems process off the taskbar and without a
            // console. The flag is Windows' own; on any other target (the
            // MSFS wasm build type-checks this file) there is no window to
            // suppress and no process to spawn, and `spawn` says so.
            #[cfg(windows)]
            {
                use std::os::windows::process::CommandExt;
                const CREATE_NO_WINDOW: u32 = 0x0800_0000;
                cmd.creation_flags(CREATE_NO_WINDOW);
            }
            cmd.spawn()
                .map(Some)
                .map_err(|e| format!("cannot start {}: {e}", exe.display()))
        })
    }

    /// Create the shared objects, let `launch` start the systems for `tag`,
    /// wait for them to register their variables, and mirror those.
    pub fn connect<V: VariableRegistry + SimulatorReaderWriter>(
        state: StartState,
        vars: &mut V,
        launch: impl FnOnce(&str) -> Result<Option<Child>, String>,
    ) -> Result<Self, String> {
        let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.subsec_nanos());
        let tag = format!("{}_{nanos}", std::process::id());
        let (shm_name, req_name, rep_name) = wire::names(&tag);
        let shared = Shared::create(&shm_name, wire::TOTAL_BYTES).ok_or("cannot create the shared block")?;
        let request = Event::create(&req_name).ok_or("cannot create the request event")?;
        let reply = Event::create(&rep_name).ok_or("cannot create the reply event")?;
        let block = unsafe { Block::new(shared.ptr()) };
        let h = block.header();
        h.magic.store(wire::MAGIC, Ordering::Relaxed);
        h.version.store(wire::VERSION, Ordering::Relaxed);
        h.start_state.store(start_state_number(state), Ordering::Relaxed);
        h.state.store(wire::STATE_STARTING, Ordering::Relaxed);

        let mut child = launch(&tag)?;
        let started = Instant::now();
        loop {
            if reply.wait(100) == win::WAIT_OBJECT_0 {
                break;
            }
            if let Some(c) = child.as_mut() {
                if let Ok(Some(status)) = c.try_wait() {
                    return Err(format!("the systems process exited while starting ({status})"));
                }
            }
            if started.elapsed() > START_TIMEOUT {
                if let Some(c) = child.as_mut() {
                    let _ = c.kill();
                }
                return Err("the systems process did not start in time".into());
            }
        }
        if block.header().state.load(Ordering::Relaxed) != wire::STATE_READY {
            return Err(format!("the systems process could not build the aircraft: {}", block.message()));
        }

        // Mirror every variable the systems registered, in their order.
        let names = wire::decode_names(block.names());
        let map: Vec<VariableIdentifier> = names
            .into_iter()
            .map(|(unprefixed, name)| if unprefixed { vars.get_unprefixed(name) } else { vars.get(name) })
            .collect();
        // What the systems wrote while being built.
        {
            let values = block.values();
            let written = block.written();
            for (i, id) in map.iter().enumerate() {
                if written[i / 64] & (1 << (i % 64)) != 0 {
                    vars.write(id, values[i]);
                }
            }
        }
        Ok(Self {
            block,
            request,
            reply,
            _shared: shared,
            child,
            map,
            seq: 0,
            pending: false,
            failed_logged: false,
            catalogue: crate::failures::catalogue_types(),
            stats: Stats::default(),
        })
    }

    /// The process serving the systems (XPHFBW.exe or the server), when this
    /// plugin started it.
    pub fn process_id(&self) -> Option<u32> {
        self.child.as_ref().map(|c| c.id())
    }

    pub fn variable_count(&self) -> usize {
        self.map.len()
    }

    /// One systems tick, in lockstep: this frame's variables out, the systems'
    /// writes back before the frame goes on.
    pub fn tick<V: SimulatorReaderWriter>(&mut self, delta: Duration, time: f64, vars: &mut V) -> Option<String> {
        if self.pending {
            // Last frame's tick came back late: its results are stale, and
            // the block is the systems' until they reply.
            if self.reply.wait(0) != win::WAIT_OBJECT_0 {
                self.stats.late += 1;
                return None;
            }
            self.pending = false;
        }
        let started = Instant::now();
        {
            let values = self.block.values();
            for (i, id) in self.map.iter().enumerate() {
                values[i] = vars.read(id);
            }
        }
        let h = self.block.header();
        h.delta.store(wire::f64_to_bits(delta.as_secs_f64()), Ordering::Relaxed);
        h.time.store(wire::f64_to_bits(time), Ordering::Relaxed);
        self.seq += 1;
        h.request.store(self.seq, Ordering::Relaxed);
        self.request.set();
        if self.reply.wait(TICK_TIMEOUT_MS) != win::WAIT_OBJECT_0 {
            self.pending = true;
            self.stats.late += 1;
            return None;
        }
        let h = self.block.header();
        let mut message = None;
        if h.state.load(Ordering::Relaxed) == wire::STATE_FAILED && !self.failed_logged {
            self.failed_logged = true;
            message = Some(self.block.message());
        }
        if h.reply.load(Ordering::Relaxed) == self.seq {
            let values = self.block.values();
            let written = self.block.written();
            for (i, id) in self.map.iter().enumerate() {
                if written[i / 64] & (1 << (i % 64)) != 0 {
                    vars.write(id, values[i]);
                }
            }
        }
        let rt = started.elapsed().as_micros() as u64;
        self.stats.ticks += 1;
        self.stats.round_trip_us += rt;
        self.stats.max_round_trip_us = self.stats.max_round_trip_us.max(rt);
        self.stats.systems_us += self.block.header().tick_micros.load(Ordering::Relaxed);
        message
    }

    /// The failures the systems should have active, sent with the next tick.
    pub fn update_active_failures(&mut self, set: FxHashSet<FailureType>) {
        if self.pending {
            // The block is the systems' until they reply; try next frame.
            if self.reply.wait(0) != win::WAIT_OBJECT_0 {
                return;
            }
            self.pending = false;
        }
        let h = self.block.header();
        let mut n = 0;
        for t in set {
            if let Some(i) = self.catalogue.iter().position(|c| *c == t) {
                if n < wire::MAX_FAILURES {
                    h.failures[n].store(i as u32, Ordering::Relaxed);
                    n += 1;
                }
            }
        }
        h.failures_len.store(n as u32, Ordering::Relaxed);
        h.failures_dirty.store(1, Ordering::Relaxed);
    }

    /// A one-line timing summary, resetting the counters.
    pub fn take_report(&mut self) -> Option<String> {
        let s = std::mem::take(&mut self.stats);
        (s.ticks > 0).then(|| {
            format!(
                "systems process: {} ticks, round trip mean {:.2} ms max {:.2} ms, systems {:.2} ms, {} late",
                s.ticks,
                s.round_trip_us as f64 / s.ticks as f64 / 1000.,
                s.max_round_trip_us as f64 / 1000.,
                s.systems_us as f64 / s.ticks as f64 / 1000.,
                s.late
            )
        })
    }
}

impl Drop for RemoteSystems {
    fn drop(&mut self) {
        self.block.header().state.store(wire::STATE_QUIT, Ordering::Relaxed);
        self.request.set();
        if let Some(c) = self.child.as_mut() {
            let until = Instant::now() + Duration::from_secs(2);
            while Instant::now() < until {
                if let Ok(Some(_)) = c.try_wait() {
                    return;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
            let _ = c.kill();
        }
    }
}

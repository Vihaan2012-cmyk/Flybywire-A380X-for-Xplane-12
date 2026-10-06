use std::collections::{BTreeSet, HashMap, VecDeque};
use std::sync::OnceLock;

use systems::simulation::test::SimulationTestBed;
use systems::simulation::VariableIdentifier;
use wasmtime::{Caller, Engine, ExternType, Func, Instance, Linker, Memory, Module, Store, Val, ValType};

use crate::A380;

const WASM_DIR: &str = "D:/A380/msfs-a380/install/out";
const MODULES: [(&str, &str); 3] = [
    ("fbw.wasm", "fbw_gauge_callback"),
    ("fadec-a380x.wasm", "Gauge_Fadec_gauge_callback"),
    ("extra-backend-a380x.wasm", "Gauge_Extra_Backend_gauge_callback"),
];
const PANEL_SERVICE_PRE_INSTALL: i32 = 2;
const PANEL_SERVICE_POST_INSTALL: i32 = 3;
const PANEL_SERVICE_PRE_DRAW: i32 = 10;
const MSG_CAPACITY: i32 = 1 << 20;
const EPOCH_TICKS_PER_CALL: u64 = 100;
const DRAW_DATA_SIZE: i32 = 48;
const S_OK: i32 = 0;
const E_FAIL: i32 = 0x8000_4005_u32 as i32;
const RECV_ID_EVENT: u32 = 4;
const RECV_ID_EVENT_EX1: u32 = 27;
const RECV_ID_SIMOBJECT_DATA: u32 = 8;
const RECV_ID_CLIENT_DATA: u32 = 16;
const PERIOD_NEVER: u32 = 0;
const PERIOD_ONCE: u32 = 1;
const CLIENT_PERIOD_ON_SET: u32 = 3;
const WASI_EBADF: i32 = 8;
const WASI_ENOENT: i32 = 44;

struct Compiled {
    engine: Engine,
    modules: Vec<(String, Module, String)>,
    pres: Vec<wasmtime::InstancePre<HostState>>,
}

pub(super) static LEGACY_LINKER: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

fn linker_for(engine: &Engine, module: &Module) -> Linker<HostState> {
    let mut linker: Linker<HostState> = Linker::new(engine);
    for import in module.imports() {
        let ExternType::Func(ty) = import.ty() else { continue };
        let (m, n) = (import.module().to_string(), import.name().to_string());
        let results: Vec<ValType> = ty.results().collect();
        linker
            .func_new(&m.clone(), &n.clone(), ty.clone(), move |mut caller, params, out| {
                let v = host_call(&mut caller, &m, &n, params)?;
                if let (Some(slot), Some(t)) = (out.first_mut(), results.first()) {
                    *slot = result_val(t, v);
                }
                Ok(())
            })
            .expect("define import");
    }
    linker
}

fn compiled() -> &'static Compiled {
    static COMPILED: OnceLock<Compiled> = OnceLock::new();
    COMPILED.get_or_init(|| {
        let mut config = wasmtime::Config::new();
        config.epoch_interruption(true);
        let engine = Engine::new(&config).expect("wasmtime engine");
        let ticker = engine.clone();
        std::thread::spawn(move || loop {
            std::thread::sleep(std::time::Duration::from_millis(100));
            ticker.increment_epoch();
        });
        let modules: Vec<(String, Module, String)> = MODULES
            .iter()
            .map(|(file, callback)| {
                let path = format!("{WASM_DIR}/{file}");
                let module = Module::from_file(&engine, &path).unwrap_or_else(|e| panic!("compile {path}: {e:#}"));
                (file.to_string(), module, callback.to_string())
            })
            .collect();
        let pres = modules
            .iter()
            .map(|(file, module, _): &(String, Module, String)| linker_for(&engine, module).instantiate_pre(module).unwrap_or_else(|e| panic!("link {file}: {e:#}")))
            .collect();
        Compiled { engine, modules, pres }
    })
}

#[derive(Clone, Copy, PartialEq)]
pub(super) enum Family {
    Plain,
    Length,
    Speed,
    Pressure,
    Temperature,
    Angle,
    Mass,
    Density,
    VerticalSpeed,
}

fn canonical_default(name: &str) -> Option<(Family, f64)> {
    let base = name.split(':').next().unwrap_or(name).trim();
    Some(match base {
        "SIM ON GROUND" | "GEAR HANDLE POSITION" | "SIMULATION RATE" | "G FORCE" => (Family::Plain, 1.),
        "PLANE ALTITUDE" | "PLANE ALT ABOVE GROUND" | "INDICATED ALTITUDE" | "PRESSURE ALTITUDE" | "RADIO HEIGHT" | "PLANE ALT ABOVE GROUND MINUS CG" => (Family::Length, 0.),
        "AIRSPEED INDICATED" | "AIRSPEED TRUE" | "GROUND VELOCITY" | "VERTICAL SPEED" | "VELOCITY BODY X" | "VELOCITY BODY Y" | "VELOCITY BODY Z" | "VELOCITY WORLD Y" => (Family::Speed, 0.),
        "AMBIENT TEMPERATURE" | "TOTAL AIR TEMPERATURE" | "STANDARD ATM TEMPERATURE" => (Family::Temperature, 15.),
        "AMBIENT PRESSURE" | "SEA LEVEL PRESSURE" | "BAROMETER PRESSURE" | "KOHLSMAN SETTING MB" => (Family::Pressure, 1013.25),
        "AMBIENT DENSITY" => (Family::Density, 1.225),
        "PLANE PITCH DEGREES" | "PLANE BANK DEGREES" | "PLANE HEADING DEGREES TRUE" | "PLANE HEADING DEGREES MAGNETIC" | "INCIDENCE ALPHA" | "INCIDENCE BETA" => (Family::Angle, 0.),
        "TOTAL WEIGHT" => (Family::Mass, 400_000.),
        "GEAR ANIMATION POSITION" | "GEAR CENTER POSITION" | "GEAR LEFT POSITION" | "GEAR RIGHT POSITION" => (Family::Plain, 1.),
        "AMBIENT WIND VELOCITY" | "AMBIENT WIND DIRECTION" | "MACH" | "AIRSPEED MACH" => (Family::Plain, 0.),
        _ => return None,
    })
}

fn convert(family: Family, canonical: f64, unit: &str) -> f64 {
    let u = unit.trim().to_lowercase();
    match family {
        Family::Length => match u.as_str() {
            "meter" | "meters" => canonical * 0.3048,
            _ => canonical,
        },
        Family::Speed => match u.as_str() {
            "meter per second" | "meters per second" => canonical * 0.514444,
            "feet per second" | "foot per second" => canonical * 1.68781,
            "feet per minute" | "foot per minute" => canonical * 101.269,
            _ => canonical,
        },
        Family::Pressure => match u.as_str() {
            "inches of mercury" | "inhg" => canonical * 0.0295300,
            "psi" | "pound-force per square inch" | "pounds per square inch" => canonical * 0.0145038,
            "pascal" | "pascals" => canonical * 100.,
            _ => canonical,
        },
        Family::Temperature => match u.as_str() {
            "kelvin" => canonical + 273.15,
            "fahrenheit" => canonical * 1.8 + 32.,
            "rankine" => (canonical + 273.15) * 1.8,
            _ => canonical,
        },
        Family::Angle => canonical,
        Family::Mass => match u.as_str() {
            "pound" | "pounds" | "lbs" => canonical * 2.20462,
            _ => canonical,
        },
        Family::Density => match u.as_str() {
            "slug per cubic feet" | "slugs per cubic feet" | "slug/ft3" => canonical * 0.00194032,
            _ => canonical,
        },
        Family::VerticalSpeed => match u.as_str() {
            "feet per second" | "foot per second" => canonical / 60.,
            "meter per second" | "meters per second" => canonical * 0.00508,
            _ => canonical,
        },
        Family::Plain => canonical,
    }
}

struct Datum {
    lvar: Option<usize>,
    avar: usize,
    unit: String,
    dtype: u32,
}

fn datum_size(dtype: u32) -> usize {
    match dtype {
        1 | 3 => 4,
        2 | 4 => 8,
        5 => 8,
        6 => 32,
        7 => 64,
        8 => 128,
        9 => 256,
        10 => 260,
        12 => 56,
        15 | 16 => 24,
        _ => 0,
    }
}

struct SimRequest {
    request_id: u32,
    define_id: u32,
    period: u32,
    sent: bool,
}

struct ClientRequest {
    area: u32,
    request_id: u32,
    define_id: u32,
    period: u32,
    sent: bool,
}

#[derive(Default)]
pub(super) struct HostState {
    module: String,
    memory: Option<Memory>,
    lvar_names: Vec<String>,
    lvar_values: Vec<f64>,
    lvar_dirty: Vec<bool>,
    lvar_ids: Vec<Option<VariableIdentifier>>,
    lvar_index: HashMap<String, usize>,
    avar_names: Vec<String>,
    avar_values: Vec<f64>,
    avar_present: Vec<bool>,
    avar_dirty: Vec<bool>,
    avar_ids: Vec<Option<VariableIdentifier>>,
    avar_index: HashMap<String, usize>,
    profile: HashMap<String, (Family, f64)>,
    pub avars_written: BTreeSet<String>,
    commbus: Vec<(String, u32, i32)>,
    aenum_names: Vec<String>,
    units: Vec<String>,
    data_defs: HashMap<u32, Vec<Datum>>,
    sim_requests: Vec<SimRequest>,
    client_names: HashMap<u32, String>,
    client_areas: HashMap<u32, Vec<u8>>,
    client_defs: HashMap<u32, Vec<(u32, u32)>>,
    client_requests: Vec<ClientRequest>,
    client_set: BTreeSet<u32>,
    queue: VecDeque<Vec<u8>>,
    msg_ptr: i32,
    event_names: HashMap<u32, String>,
    pub events: Vec<String>,
    pub unsupported: BTreeSet<String>,
    pub log: String,
    sim_time: f64,
}

impl HostState {
    fn lvar_slot(&mut self, raw: &str) -> usize {
        let name = raw.trim().trim_start_matches("L:").split(',').next().unwrap_or("").trim().to_string();
        if let Some(&i) = self.lvar_index.get(&name) {
            return i;
        }
        let i = self.lvar_names.len();
        self.lvar_index.insert(name.clone(), i);
        self.lvar_names.push(name);
        self.lvar_values.push(0.);
        self.lvar_dirty.push(false);
        self.lvar_ids.push(None);
        i
    }

    fn avar_slot(&mut self, raw: &str) -> usize {
        let name = raw.trim().trim_start_matches("A:").to_string();
        if let Some(&i) = self.avar_index.get(&name) {
            return i;
        }
        let i = self.avar_names.len();
        self.avar_index.insert(name.clone(), i);
        self.avar_names.push(name);
        self.avar_values.push(0.);
        self.avar_present.push(false);
        self.avar_dirty.push(false);
        self.avar_ids.push(None);
        i
    }

    fn avar_value(&self, i: usize, unit: &str) -> f64 {
        if self.avar_names[i] == "SIMULATION TIME" {
            return self.sim_time;
        }
        if self.avar_present[i] {
            return self.avar_values[i];
        }
        let base = self.avar_names[i].split(':').next().unwrap_or("").trim();
        if let Some((family, v)) = self.profile.get(base) {
            return convert(*family, *v, unit);
        }
        canonical_default(&self.avar_names[i]).map_or(0., |(family, v)| convert(family, v, unit))
    }

    fn pack_definition(&self, define_id: u32) -> (u32, Vec<u8>) {
        let mut data = Vec::new();
        let mut count = 0;
        if let Some(defs) = self.data_defs.get(&define_id) {
            for d in defs {
                count += 1;
                let v = match d.lvar {
                    Some(l) => self.lvar_values[l],
                    None => self.avar_value(d.avar, &d.unit),
                };
                match d.dtype {
                    1 => data.extend_from_slice(&(v as i32).to_le_bytes()),
                    2 => data.extend_from_slice(&(v as i64).to_le_bytes()),
                    3 => data.extend_from_slice(&(v as f32).to_le_bytes()),
                    4 => data.extend_from_slice(&v.to_le_bytes()),
                    other => data.extend(std::iter::repeat_n(0u8, datum_size(other))),
                }
            }
        }
        (count, data)
    }

    fn message(id: u32, fields: &[u32], payload: &[u8]) -> Vec<u8> {
        let size = 12 + fields.len() * 4 + payload.len();
        let mut m = Vec::with_capacity(size);
        for v in [size as u32, 6, id].iter().chain(fields) {
            m.extend_from_slice(&v.to_le_bytes());
        }
        m.extend_from_slice(payload);
        m
    }

    fn enqueue_sim_request(&mut self, r: usize) {
        let (request_id, define_id) = (self.sim_requests[r].request_id, self.sim_requests[r].define_id);
        let (count, data) = self.pack_definition(define_id);
        let msg = Self::message(RECV_ID_SIMOBJECT_DATA, &[request_id, 0, define_id, 0, 1, 1, count], &data);
        self.queue.push_back(msg);
        self.sim_requests[r].sent = true;
    }

    fn client_payload(&self, area: u32, define_id: u32) -> (u32, Vec<u8>) {
        let bytes = self.client_areas.get(&area).cloned().unwrap_or_default();
        match self.client_defs.get(&define_id) {
            Some(defs) if !defs.is_empty() => {
                let mut out = Vec::new();
                for (offset, size) in defs {
                    let (o, s) = (*offset as usize, *size as usize);
                    let mut chunk = vec![0u8; s];
                    if o < bytes.len() {
                        let end = (o + s).min(bytes.len());
                        chunk[..end - o].copy_from_slice(&bytes[o..end]);
                    }
                    out.extend_from_slice(&chunk);
                }
                (defs.len() as u32, out)
            }
            _ => (1, bytes),
        }
    }

    fn enqueue_client_request(&mut self, r: usize) {
        let (area, request_id, define_id) = (self.client_requests[r].area, self.client_requests[r].request_id, self.client_requests[r].define_id);
        let (count, data) = self.client_payload(area, define_id);
        let msg = Self::message(RECV_ID_CLIENT_DATA, &[request_id, 0, define_id, 0, 1, 1, count], &data);
        self.queue.push_back(msg);
        self.client_requests[r].sent = true;
    }

    fn enqueue_periodic(&mut self) {
        for r in 0..self.sim_requests.len() {
            let p = self.sim_requests[r].period;
            if p > PERIOD_ONCE || (p == PERIOD_ONCE && !self.sim_requests[r].sent) {
                self.enqueue_sim_request(r);
            }
        }
        let set = std::mem::take(&mut self.client_set);
        for r in 0..self.client_requests.len() {
            let c = &self.client_requests[r];
            let due = match c.period {
                PERIOD_NEVER => false,
                PERIOD_ONCE => !c.sent,
                CLIENT_PERIOD_ON_SET => set.contains(&c.area),
                _ => true,
            };
            if due {
                self.enqueue_client_request(r);
            }
        }
    }

    fn calculator(&mut self, code: &str) -> f64 {
        let mut tokens = Vec::new();
        let chars: Vec<char> = code.chars().collect();
        let mut i = 0;
        while i < chars.len() {
            let c = chars[i];
            if c.is_whitespace() {
                i += 1;
            } else if c == '(' {
                let start = i + 1;
                while i < chars.len() && chars[i] != ')' {
                    i += 1;
                }
                tokens.push(chars[start..i.min(chars.len())].iter().collect::<String>());
                i += 1;
            } else {
                let start = i;
                while i < chars.len() && !chars[i].is_whitespace() && chars[i] != '(' {
                    i += 1;
                }
                tokens.push(chars[start..i].iter().collect::<String>());
            }
        }
        let mut stack: Vec<f64> = Vec::new();
        for t in tokens {
            let t = t.trim();
            if let Ok(v) = t.parse::<f64>() {
                stack.push(v);
            } else if let Some(rest) = t.strip_prefix(">L:") {
                let slot = self.lvar_slot(rest);
                self.lvar_values[slot] = stack.pop().unwrap_or(0.);
                self.lvar_dirty[slot] = true;
            } else if let Some(rest) = t.strip_prefix("L:") {
                let slot = self.lvar_slot(rest);
                stack.push(self.lvar_values[slot]);
            } else if let Some(rest) = t.strip_prefix("A:") {
                let mut parts = rest.splitn(2, ',');
                let name = parts.next().unwrap_or("").trim().to_string();
                let unit = parts.next().unwrap_or("").trim().to_string();
                let slot = self.avar_slot(&name);
                stack.push(self.avar_value(slot, &unit));
            } else if let Some(ev) = t.strip_prefix(">K:").or_else(|| t.strip_prefix(">H:")).or_else(|| t.strip_prefix(">B:")) {
                self.events.push(ev.to_string());
            } else if t == "+" || t == "-" || t == "*" || t == "/" {
                let b = stack.pop().unwrap_or(0.);
                let a = stack.pop().unwrap_or(0.);
                stack.push(match t {
                    "+" => a + b,
                    "-" => a - b,
                    "*" => a * b,
                    _ => if b != 0. { a / b } else { 0. },
                });
            } else if !t.is_empty() {
                self.unsupported.insert(format!("calc `{t}`"));
            }
        }
        stack.pop().unwrap_or(0.)
    }
}

fn memory(caller: &mut Caller<'_, HostState>) -> Memory {
    if let Some(m) = caller.data().memory {
        return m;
    }
    let m = caller.get_export("memory").and_then(|e| e.into_memory()).expect("module exports memory");
    caller.data_mut().memory = Some(m);
    m
}

fn cstr(caller: &mut Caller<'_, HostState>, ptr: i32) -> String {
    if ptr == 0 {
        return String::new();
    }
    let m = memory(caller);
    let data = m.data(&*caller);
    let start = ptr as usize;
    if start >= data.len() {
        return String::new();
    }
    let end = data[start..].iter().position(|&b| b == 0).map_or(data.len(), |p| start + p);
    String::from_utf8_lossy(&data[start..end]).into_owned()
}

fn write_bytes(caller: &mut Caller<'_, HostState>, ptr: i32, bytes: &[u8]) {
    if ptr == 0 {
        return;
    }
    let m = memory(caller);
    let _ = m.write(&mut *caller, ptr as usize, bytes);
}

fn read_bytes(caller: &mut Caller<'_, HostState>, ptr: i32, len: usize) -> Vec<u8> {
    let m = memory(caller);
    let mut buf = vec![0u8; len];
    let _ = m.read(&*caller, ptr as usize, &mut buf);
    buf
}

fn read_u32(caller: &mut Caller<'_, HostState>, ptr: i32) -> u32 {
    let b = read_bytes(caller, ptr, 4);
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn int(v: &Val) -> i64 {
    match v {
        Val::I32(x) => *x as i64,
        Val::I64(x) => *x,
        Val::F32(x) => f32::from_bits(*x) as i64,
        Val::F64(x) => f64::from_bits(*x) as i64,
        _ => 0,
    }
}

fn float(v: &Val) -> f64 {
    match v {
        Val::F32(x) => f32::from_bits(*x) as f64,
        Val::F64(x) => f64::from_bits(*x),
        Val::I32(x) => *x as f64,
        Val::I64(x) => *x as f64,
        _ => 0.,
    }
}

fn host_call(caller: &mut Caller<'_, HostState>, module: &str, name: &str, p: &[Val]) -> wasmtime::Result<f64> {
    let a = |i: usize| p.get(i).map(int).unwrap_or(0) as i32;
    Ok(match (module, name) {
        ("env", "SimConnect_Open") => {
            write_bytes(caller, a(0), &1u32.to_le_bytes());
            S_OK as f64
        }
        ("env", "SimConnect_Close") => S_OK as f64,
        ("env", "SimConnect_AddToDataDefinition") => {
            let (define_id, datum, unit, dtype) = (a(1) as u32, cstr(caller, a(2)), cstr(caller, a(3)), a(4) as u32);
            let st = caller.data_mut();
            let lvar = datum.trim().starts_with("L:").then(|| st.lvar_slot(&datum));
            let avar = st.avar_slot(&datum);
            st.data_defs.entry(define_id).or_default().push(Datum { lvar, avar, unit, dtype });
            S_OK as f64
        }
        ("env", "SimConnect_ClearDataDefinition") => {
            caller.data_mut().data_defs.remove(&(a(1) as u32));
            S_OK as f64
        }
        ("env", "SimConnect_RequestDataOnSimObject") => {
            let (request_id, define_id, period) = (a(1) as u32, a(2) as u32, a(4) as u32);
            let st = caller.data_mut();
            st.sim_requests.retain(|r| r.request_id != request_id);
            st.sim_requests.push(SimRequest { request_id, define_id, period, sent: false });
            if period == PERIOD_ONCE {
                let r = st.sim_requests.len() - 1;
                st.enqueue_sim_request(r);
            }
            S_OK as f64
        }
        ("env", "SimConnect_SetDataOnSimObject") => {
            let (define_id, count, unit_size, data_ptr) = (a(1) as u32, a(4).max(1) as usize, a(5) as usize, a(6));
            let total: usize = caller.data().data_defs.get(&define_id).map_or(0, |d| d.iter().map(|x| datum_size(x.dtype)).sum());
            let len = if unit_size > 0 { unit_size * count } else { total };
            let bytes = read_bytes(caller, data_ptr, len.max(total));
            let st = caller.data_mut();
            let Some(defs) = st.data_defs.get(&define_id) else { return Ok(S_OK as f64) };
            let mut off = 0usize;
            let mut writes = Vec::new();
            for d in defs {
                let size = datum_size(d.dtype);
                if off + size > bytes.len() {
                    break;
                }
                let b = &bytes[off..off + size];
                let v = match d.dtype {
                    1 => i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
                    2 => i64::from_le_bytes(b.try_into().unwrap_or([0; 8])) as f64,
                    3 => f32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
                    4 => f64::from_le_bytes(b.try_into().unwrap_or([0; 8])),
                    _ => f64::NAN,
                };
                off += size;
                if !v.is_nan() {
                    writes.push((d.lvar, d.avar, v));
                }
            }
            for (lvar, avar, v) in writes {
                match lvar {
                    Some(l) => {
                        st.lvar_values[l] = v;
                        st.lvar_dirty[l] = true;
                    }
                    None => {
                        st.avar_values[avar] = v;
                        st.avar_present[avar] = true;
                        st.avar_dirty[avar] = true;
                    }
                }
            }
            S_OK as f64
        }
        ("env", "SimConnect_GetNextDispatch") => {
            let (pp_data, pcb_data) = (a(1), a(2));
            let Some(msg) = caller.data_mut().queue.pop_front() else { return Ok(E_FAIL as f64) };
            let ptr = caller.data().msg_ptr;
            if ptr == 0 || msg.len() > MSG_CAPACITY as usize {
                caller.data_mut().unsupported.insert(format!("message of {} bytes", msg.len()));
                return Ok(E_FAIL as f64);
            }
            write_bytes(caller, ptr, &msg);
            write_bytes(caller, pp_data, &(ptr as u32).to_le_bytes());
            write_bytes(caller, pcb_data, &(msg.len() as u32).to_le_bytes());
            S_OK as f64
        }
        ("env", "SimConnect_MapClientEventToSimEvent") => {
            let (event_id, name) = (a(1) as u32, cstr(caller, a(2)));
            caller.data_mut().event_names.insert(event_id, name);
            S_OK as f64
        }
        ("env", "SimConnect_TransmitClientEvent") | ("env", "SimConnect_TransmitClientEvent_EX1") => {
            let event_id = a(2) as u32;
            let st = caller.data_mut();
            let name = st.event_names.get(&event_id).cloned().unwrap_or_else(|| format!("#{event_id}"));
            st.events.push(name);
            S_OK as f64
        }
        ("env", "SimConnect_AddClientEventToNotificationGroup")
        | ("env", "SimConnect_SetNotificationGroupPriority")
        | ("env", "SimConnect_SubscribeToSystemEvent")
        | ("env", "SimConnect_UnsubscribeFromSystemEvent") => S_OK as f64,
        ("env", "SimConnect_MapClientDataNameToID") => {
            let (name, id) = (cstr(caller, a(1)), a(2) as u32);
            caller.data_mut().client_names.insert(id, name);
            S_OK as f64
        }
        ("env", "SimConnect_CreateClientData") => {
            let (id, size) = (a(1) as u32, a(2).max(0) as usize);
            caller.data_mut().client_areas.entry(id).or_insert_with(|| vec![0; size]);
            S_OK as f64
        }
        ("env", "SimConnect_AddToClientDataDefinition") => {
            let (define_id, offset, size_or_type) = (a(1) as u32, a(2) as u32, a(3));
            let size = match size_or_type {
                -1 => 1,
                -2 => 2,
                -3 | -5 => 4,
                -4 | -6 => 8,
                s => s.max(0) as u32,
            };
            let defs = caller.data_mut().client_defs.entry(define_id).or_default();
            let offset = if offset == u32::MAX { defs.last().map_or(0, |(o, s)| o + s) } else { offset };
            defs.push((offset, size));
            S_OK as f64
        }
        ("env", "SimConnect_ClearClientDataDefinition") => {
            caller.data_mut().client_defs.remove(&(a(1) as u32));
            S_OK as f64
        }
        ("env", "SimConnect_RequestClientData") => {
            let (area, request_id, define_id, period) = (a(1) as u32, a(2) as u32, a(3) as u32, a(4) as u32);
            let st = caller.data_mut();
            st.client_requests.retain(|r| r.request_id != request_id);
            st.client_requests.push(ClientRequest { area, request_id, define_id, period, sent: false });
            if period == PERIOD_ONCE {
                let r = st.client_requests.len() - 1;
                st.enqueue_client_request(r);
            }
            S_OK as f64
        }
        ("env", "SimConnect_SetClientData") => {
            let (area, define_id, unit_size, data_ptr) = (a(1) as u32, a(2) as u32, a(5).max(0) as usize, a(6));
            let bytes = read_bytes(caller, data_ptr, unit_size);
            let st = caller.data_mut();
            let defs = st.client_defs.get(&define_id).cloned().unwrap_or_default();
            let buf = st.client_areas.entry(area).or_default();
            let mut src = 0usize;
            if defs.is_empty() {
                if buf.len() < bytes.len() {
                    buf.resize(bytes.len(), 0);
                }
                buf[..bytes.len()].copy_from_slice(&bytes);
            } else {
                for (offset, size) in defs {
                    let (o, s) = (offset as usize, size as usize);
                    if src + s > bytes.len() {
                        break;
                    }
                    if buf.len() < o + s {
                        buf.resize(o + s, 0);
                    }
                    buf[o..o + s].copy_from_slice(&bytes[src..src + s]);
                    src += s;
                }
            }
            st.client_set.insert(area);
            S_OK as f64
        }
        ("env", "register_named_variable") => {
            let name = cstr(caller, a(0));
            caller.data_mut().lvar_slot(&name) as f64
        }
        ("env", "get_named_variable_value") | ("env", "get_named_variable_typed_value") => {
            let st = caller.data();
            st.lvar_values.get(a(0) as usize).copied().unwrap_or(0.)
        }
        ("env", "set_named_variable_value") | ("env", "set_named_variable_typed_value") => {
            let (id, v) = (a(0) as usize, p.get(1).map(float).unwrap_or(0.));
            let st = caller.data_mut();
            if id < st.lvar_values.len() {
                st.lvar_values[id] = v;
                st.lvar_dirty[id] = true;
            }
            0.
        }
        ("env", "unregister_all_named_vars") => 0.,
        ("env", "get_units_enum") => {
            let name = cstr(caller, a(0));
            let st = caller.data_mut();
            st.units.push(name);
            (st.units.len() - 1) as f64
        }
        ("env", "get_aircraft_var_enum") => {
            let name = cstr(caller, a(0));
            let st = caller.data_mut();
            st.aenum_names.push(name);
            (st.aenum_names.len() - 1) as f64
        }
        ("env", "aircraft_varget") => {
            let (var, unit, index) = (a(0) as usize, a(1) as usize, a(2));
            let st = caller.data_mut();
            let base = st.aenum_names.get(var).cloned().unwrap_or_default();
            let unit = st.units.get(unit).cloned().unwrap_or_default();
            let full = if index > 0 { format!("{base}:{index}") } else { base };
            let slot = st.avar_slot(&full);
            st.avar_value(slot, &unit)
        }
        ("env", "execute_calculator_code") => {
            let (code, fptr, iptr) = (cstr(caller, a(0)), a(1), a(2));
            let v = caller.data_mut().calculator(&code);
            if fptr != 0 {
                write_bytes(caller, fptr, &v.to_le_bytes());
            }
            if iptr != 0 {
                write_bytes(caller, iptr, &(v as i32).to_le_bytes());
            }
            1.
        }
        ("env", "register_key_event_handler_EX1") | ("env", "unregister_key_event_handler_EX1") => 0.,
        ("env", "fsCommBusRegister") => {
            let (name, callback, ctx) = (cstr(caller, a(0)), a(1) as u32, a(2));
            caller.data_mut().commbus.push((name, callback, ctx));
            1.
        }
        ("env", "fsCommBusUnregisterAll") => {
            caller.data_mut().commbus.clear();
            1.
        }
        ("env", "fsCommBusCall") => {
            let name = cstr(caller, a(0));
            caller.data_mut().events.push(format!("commbus {name}"));
            1.
        }
        ("wasi_snapshot_preview1", "fd_write") => {
            let (iovs, n, nwritten) = (a(1), a(2), a(3));
            let mut total = 0u32;
            let mut text = Vec::new();
            for k in 0..n {
                let base = read_u32(caller, iovs + 8 * k);
                let len = read_u32(caller, iovs + 8 * k + 4);
                text.extend(read_bytes(caller, base as i32, len as usize));
                total += len;
            }
            let st = caller.data_mut();
            if st.log.len() < 1 << 16 {
                st.log.push_str(&String::from_utf8_lossy(&text));
            }
            write_bytes(caller, nwritten, &total.to_le_bytes());
            0.
        }
        ("wasi_snapshot_preview1", "fd_read") => {
            write_bytes(caller, a(3), &0u32.to_le_bytes());
            0.
        }
        ("wasi_snapshot_preview1", "fd_fdstat_get") => {
            if a(0) > 2 {
                return Ok(WASI_EBADF as f64);
            }
            let mut stat = [0u8; 24];
            stat[0] = 2;
            write_bytes(caller, a(1), &stat);
            0.
        }
        ("wasi_snapshot_preview1", "environ_sizes_get") => {
            write_bytes(caller, a(0), &0u32.to_le_bytes());
            write_bytes(caller, a(1), &0u32.to_le_bytes());
            0.
        }
        ("wasi_snapshot_preview1", "environ_get") | ("wasi_snapshot_preview1", "fd_close") | ("wasi_snapshot_preview1", "fd_fdstat_set_flags") => 0.,
        ("wasi_snapshot_preview1", "clock_time_get") => {
            let ns = (caller.data().sim_time * 1e9) as u64;
            write_bytes(caller, a(2), &ns.to_le_bytes());
            0.
        }
        ("wasi_snapshot_preview1", "fd_prestat_get") | ("wasi_snapshot_preview1", "fd_prestat_dir_name") | ("wasi_snapshot_preview1", "fd_seek") | ("wasi_snapshot_preview1", "fd_tell") | ("wasi_snapshot_preview1", "fd_readdir") => {
            WASI_EBADF as f64
        }
        ("wasi_snapshot_preview1", "path_open")
        | ("wasi_snapshot_preview1", "path_filestat_get")
        | ("wasi_snapshot_preview1", "path_create_directory")
        | ("wasi_snapshot_preview1", "path_remove_directory")
        | ("wasi_snapshot_preview1", "path_unlink_file") => WASI_ENOENT as f64,
        ("wasi_snapshot_preview1", "commit_pages") => 0.,
        ("wasi_snapshot_preview1", "proc_exit") => return Err(wasmtime::Error::msg(format!("{} called proc_exit({})", caller.data().module, a(0)))),
        _ => {
            caller.data_mut().unsupported.insert(format!("{module}.{name}"));
            0.
        }
    })
}

fn result_val(ty: &ValType, v: f64) -> Val {
    match ty {
        ValType::I32 => Val::I32(v as i64 as i32),
        ValType::I64 => Val::I64(v as i64),
        ValType::F32 => Val::F32((v as f32).to_bits()),
        ValType::F64 => Val::F64(v.to_bits()),
        _ => Val::I32(0),
    }
}

struct Gauge {
    store: Store<HostState>,
    instance: Instance,
    callback: Func,
    params: Vec<ValType>,
    n_results: usize,
    draw_ptr: i32,
    failed: Option<String>,
}

pub(super) struct CppHost {
    gauges: Vec<Gauge>,
    t: f64,
}

impl CppHost {
    pub(super) fn new(bench: &mut SimulationTestBed<A380>) -> Self {
        let c = compiled();
        let ready = bench.variable_identifier("A32NX_IS_READY");
        bench.write_identifier(&ready, 1.);
        let rust_ready = bench.variable_identifier("IS_READY");
        bench.write_identifier(&rust_ready, 1.);
        let mut gauges = Vec::new();
        for (k, (file, module, callback)) in c.modules.iter().enumerate() {
            let mut store = Store::new(&c.engine, HostState { module: file.clone(), ..Default::default() });
            store.set_epoch_deadline(EPOCH_TICKS_PER_CALL * 10);
            let instance: Instance = if LEGACY_LINKER.load(std::sync::atomic::Ordering::Relaxed) {
                linker_for(&c.engine, module).instantiate(&mut store, module).unwrap_or_else(|e| panic!("instantiate {file}: {e:#}"))
            } else {
                c.pres[k].instantiate(&mut store).unwrap_or_else(|e| panic!("instantiate {file}: {e:#}"))
            };
            store.data_mut().memory = instance.get_memory(&mut store, "memory");
            if let Some(init) = instance.get_func(&mut store, "_initialize").or_else(|| instance.get_func(&mut store, "__wasm_call_ctors")) {
                init.call(&mut store, &[], &mut []).unwrap_or_else(|e| panic!("{file} constructors: {e:#}"));
            }
            let malloc = instance.get_typed_func::<i32, i32>(&mut store, "malloc").expect("malloc export");
            let msg_ptr = malloc.call(&mut store, MSG_CAPACITY).expect("message buffer");
            let draw_ptr = malloc.call(&mut store, DRAW_DATA_SIZE).expect("draw data");
            store.data_mut().msg_ptr = msg_ptr;
            let callback = instance.get_func(&mut store, callback).unwrap_or_else(|| panic!("{file} exports {callback}"));
            let ty = callback.ty(&store);
            let params: Vec<ValType> = ty.params().collect();
            let n_results = ty.results().len();
            let mut gauge = Gauge { store, instance, callback, params, n_results, draw_ptr, failed: None };
            Self::sync_in(&mut gauge, bench);
            for service in [PANEL_SERVICE_PRE_INSTALL, PANEL_SERVICE_POST_INSTALL] {
                gauge.call(service);
            }
            Self::sync_out(&mut gauge, bench);
            gauges.push(gauge);
        }
        Self { gauges, t: 0. }
    }

    fn sync_in(g: &mut Gauge, bench: &mut SimulationTestBed<A380>) {
        let st = g.store.data_mut();
        for i in 0..st.lvar_names.len() {
            if st.lvar_ids[i].is_none() {
                let name = &st.lvar_names[i];
                let id = bench
                    .known_variable_identifier(name)
                    .or_else(|| name.strip_prefix("A32NX_").and_then(|s| bench.known_variable_identifier(s)))
                    .unwrap_or_else(|| bench.variable_identifier(name));
                st.lvar_ids[i] = Some(id);
            }
            if let Some(v) = st.lvar_ids[i].as_ref().and_then(|id| bench.read_identifier(id)) {
                st.lvar_values[i] = v;
            }
            st.lvar_dirty[i] = false;
        }
        for i in 0..st.avar_names.len() {
            if st.avar_ids[i].is_none() {
                st.avar_ids[i] = bench.known_variable_identifier(&st.avar_names[i]);
            }
            if let Some(v) = st.avar_ids[i].as_ref().and_then(|id| bench.read_identifier(id)) {
                st.avar_values[i] = v;
                st.avar_present[i] = true;
            }
            st.avar_dirty[i] = false;
        }
    }

    fn sync_out(g: &mut Gauge, bench: &mut SimulationTestBed<A380>) {
        let st = g.store.data_mut();
        for i in 0..st.lvar_names.len() {
            if st.lvar_dirty[i] {
                let id = match st.lvar_ids[i] {
                    Some(id) => id,
                    None => {
                        let name = &st.lvar_names[i];
                        let id = bench
                            .known_variable_identifier(name)
                            .or_else(|| name.strip_prefix("A32NX_").and_then(|s| bench.known_variable_identifier(s)))
                            .unwrap_or_else(|| bench.variable_identifier(name));
                        st.lvar_ids[i] = Some(id);
                        id
                    }
                };
                bench.write_identifier(&id, st.lvar_values[i]);
            }
        }
        for i in 0..st.avar_names.len() {
            if st.avar_dirty[i] {
                st.avars_written.insert(st.avar_names[i].clone());
                let id = st.avar_ids[i].unwrap_or_else(|| bench.variable_identifier(&st.avar_names[i]));
                st.avar_ids[i] = Some(id);
                bench.write_identifier(&id, st.avar_values[i]);
            }
        }
    }

    pub(super) fn frame(&mut self, bench: &mut SimulationTestBed<A380>, dt: f64) {
        self.t += dt;
        for g in &mut self.gauges {
            if g.failed.is_some() {
                continue;
            }
            Self::sync_in(g, bench);
            g.store.data_mut().sim_time = self.t;
            g.store.data_mut().enqueue_periodic();
            let mut draw = [0u8; DRAW_DATA_SIZE as usize];
            draw[16..24].copy_from_slice(&self.t.to_le_bytes());
            draw[24..32].copy_from_slice(&dt.to_le_bytes());
            let mem = g.store.data().memory;
            if let Some(m) = mem {
                let _ = m.write(&mut g.store, g.draw_ptr as usize, &draw);
            }
            g.call(PANEL_SERVICE_PRE_DRAW);
            g.store.data_mut().queue.clear();
            Self::sync_out(g, bench);
        }
        Self::sim_turbines(bench);
    }

    fn sim_turbines(bench: &mut SimulationTestBed<A380>) {
        let read = |bench: &SimulationTestBed<A380>, name: &str| bench.known_variable_identifier(name).and_then(|id| bench.read_identifier(&id));
        let sat_c = read(bench, "AMBIENT TEMPERATURE").unwrap_or(15.);
        let sqrt_theta = ((sat_c + 273.15) / 288.15).max(1e-6).sqrt();
        for n in 1..=4 {
            for spool in ["N1", "N2"] {
                if let Some(corrected) = read(bench, &format!("TURB ENG CORRECTED {spool}:{n}")) {
                    let id = bench.variable_identifier(&format!("TURB ENG {spool}:{n}"));
                    bench.write_identifier(&id, corrected * sqrt_theta);
                }
            }
        }
    }

    pub(super) fn report(&self) -> Vec<(String, Option<String>, BTreeSet<String>, String)> {
        self.gauges
            .iter()
            .map(|g| {
                let st = g.store.data();
                (st.module.clone(), g.failed.clone(), st.unsupported.clone(), st.log.chars().take(2000).collect())
            })
            .collect()
    }

    pub(super) fn inject_key_event(&mut self, event: &str, data0: i32) {
        for g in &mut self.gauges {
            let st = g.store.data_mut();
            let ids: Vec<u32> = st.event_names.iter().filter(|(_, n)| n.as_str() == event).map(|(id, _)| *id).collect();
            for id in ids {
                st.queue.push_back(HostState::message(RECV_ID_EVENT_EX1, &[0, id, data0 as u32, 0, 0, 0, 0], &[]));
            }
        }
    }

    pub(super) fn set_flight_profile(&mut self, profile: &[(&str, Family, f64)]) {
        for g in &mut self.gauges {
            let st = g.store.data_mut();
            st.profile = profile.iter().map(|(n, f, v)| (n.to_string(), (*f, *v))).collect();
        }
    }

    pub(super) fn send_commbus(&mut self, event: &str, payload: &str) {
        for g in &mut self.gauges {
            if g.failed.is_some() {
                continue;
            }
            let handlers: Vec<(u32, i32)> = g.store.data().commbus.iter().filter(|(n, _, _)| n == event).map(|(_, f, c)| (*f, *c)).collect();
            if handlers.is_empty() {
                continue;
            }
            let mut bytes = payload.as_bytes().to_vec();
            bytes.push(0);
            let Ok(malloc) = g.instance.get_typed_func::<i32, i32>(&mut g.store, "malloc") else { continue };
            let Ok(ptr) = malloc.call(&mut g.store, bytes.len() as i32) else { continue };
            if let Some(m) = g.store.data().memory {
                let _ = m.write(&mut g.store, ptr as usize, &bytes);
            }
            let Some(table) = g.instance.get_table(&mut g.store, "__indirect_function_table") else { continue };
            for (index, ctx) in handlers {
                let Some(func) = table.get(&mut g.store, index as u64).and_then(|r| r.as_func().flatten().copied()) else { continue };
                let ty = func.ty(&g.store);
                let args = [Val::I32(ptr), Val::I32(bytes.len() as i32), Val::I32(ctx)];
                let params: Vec<Val> = args.iter().take(ty.params().len()).cloned().collect();
                let mut out: Vec<Val> = ty.results().map(|_| Val::I32(0)).collect();
                g.store.set_epoch_deadline(EPOCH_TICKS_PER_CALL);
                if let Err(e) = func.call(&mut g.store, &params, &mut out) {
                    g.failed = Some(format!("commbus {event}: {e:#}"));
                }
            }
            if let Ok(free) = g.instance.get_typed_func::<i32, ()>(&mut g.store, "free") {
                let _ = free.call(&mut g.store, ptr);
            }
        }
    }

    pub(super) fn avars_written(&self) -> Vec<(String, Vec<String>)> {
        self.gauges.iter().map(|g| (g.store.data().module.clone(), g.store.data().avars_written.iter().cloned().collect())).collect()
    }

    pub(super) fn failures(&self) -> Vec<String> {
        self.gauges.iter().filter_map(|g| g.failed.as_ref().map(|f| format!("{}: {f}", g.store.data().module))).collect()
    }
}

impl Gauge {
    fn call(&mut self, service: i32) {
        if self.failed.is_some() {
            return;
        }
        let params: Vec<Val> = self
            .params
            .iter()
            .enumerate()
            .map(|(i, t)| {
                let v: i64 = match i {
                    0 => 0,
                    1 => service as i64,
                    _ => self.draw_ptr as i64,
                };
                match t {
                    ValType::I64 => Val::I64(v),
                    _ => Val::I32(v as i32),
                }
            })
            .collect();
        let mut out: Vec<Val> = vec![Val::I32(0); self.n_results];
        self.store.set_epoch_deadline(EPOCH_TICKS_PER_CALL);
        if let Err(e) = self.callback.call(&mut self.store, &params, &mut out) {
            self.failed = Some(format!("service {service}: {e:#}"));
        }
    }
}

#[test]
#[ignore]
fn the_cpp_modules_run_on_the_bench() {
    use super::tests::run;
    use super::tests_flight_state::apply_flight_state;
    use systems::simulation::test::{ReadByName, TestBed};

    deep_systems::set_log_quiet(true);
    let started = std::time::Instant::now();
    let mut bench = SimulationTestBed::new(A380::new);
    let applied = apply_flight_state(&mut bench, "runway.FLT");
    let mut host = CppHost::new(&mut bench);
    println!("runway.FLT: {applied} values; instantiated in {:.2} s", started.elapsed().as_secs_f64());
    let t = std::time::Instant::now();
    let watch = [
        "PRIM_1_HEALTHY",
        "PRIM_2_HEALTHY",
        "PRIM_3_HEALTHY",
        "SEC_1_HEALTHY",
        "SEC_3_HEALTHY",
        "A32NX_ENG_1_PHYS_VALID",
        "A32NX_ENG_1_PHYS_LIT",
        "ENGINE_STATE:1",
        "ENGINE_N1:1",
        "ELEC_DC_ESS_BUS_IS_POWERED",
        "ELEC_108PH_BUS_IS_POWERED",
        "ELEC_AC_1_BUS_IS_POWERED",
        "HYD_GREEN_SYSTEM_1_SECTION_PRESSURE",
        "TURB ENG CORRECTED N2:1",
        "TURB ENG N2:1",
        "ENGINE_N2:1",
        "ENGINE_N3:1",
        "GENERAL ENG STARTER:1",
        "AIRCRAFT_PRESET_QUICK_MODE",
    ];
    for second in 1..=60 {
        for _ in 0..10 {
            run(&mut bench, 1);
            host.frame(&mut bench, 0.1);
        }
        if second <= 3 || second % 20 == 0 {
            let line: Vec<String> = watch.iter().map(|n| format!("{n}={:.1}", ReadByName::<SimulationTestBed<A380>, f64>::read_by_name(&mut bench, n))).collect();
            println!("t={second}s {}", line.join(" "));
        }
    }
    println!("600 frames in {:.2} s", t.elapsed().as_secs_f64());
    let snap = bench.query(|a| a.deep_systems.snapshot());
    let phys: Vec<String> = snap.iter().filter(|(k, _)| k.contains("ENG_1_PHYS")).map(|(k, v)| format!("{k}={v:.2}")).collect();
    println!("deep snapshot: {phys:?}");
    let ids: Vec<String> = bench.variable_identifiers().into_iter().map(|(n, _)| n).filter(|n| n.contains("ENG_1_PHYS")).collect();
    println!("bench names: {ids:?}");
    for (module, written) in host.avars_written() {
        println!("{module} writes sim vars: {written:?}");
    }
    for (module, failed, unsupported, _log) in host.report() {
        println!("== {module}: failed {failed:?} unsupported {unsupported:?}");
    }
    assert!(host.failures().is_empty(), "{:?}", host.failures());
}

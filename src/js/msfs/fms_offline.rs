use std::collections::BTreeMap;
use std::path::Path;

use super::*;
use crate::navdata::{NavData, NOT_READY};

pub const MFD_SCREEN: &str = "SCREEN_DU_MFD";
pub const SIMBRIEF_USER_KEY: &str = "A380X_CONFIG_OVERRIDE_SIMBRIEF_USERID";
pub const DUMMY_SIMBRIEF_USER: &str = "100001";
pub const EGLL_LAT: f64 = 51.4706;
pub const EGLL_LON: f64 = -0.4619;
pub const FIXED_EPOCH_MS: u64 = 1_760_000_000_000;

const STEP_MS: f64 = 50.;

const BUSES: [&str; 14] =
    ["AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "DC_1", "DC_2", "DC_ESS", "DC_HOT_1", "DC_HOT_2", "AC_ESS_SHED", "DC_ESS_SHED", "247PP", "108PH"];

const NO_OP_CALLS: [&str; 16] = [
    "PLAY_INSTRUMENT_SOUND",
    "OPEN_WEB_BROWSER",
    "TOOLBAR_SET_ACTIVE_PAUSE",
    "COMM_BUS_WASM_CALLBACK",
    "SHOW_MAP",
    "SHOW_MAP_WEATHER",
    "SHOW_MAP_ISOLINES",
    "SET_MAP_RESOLUTION",
    "SET_MAP_CLEAR_COLOR",
    "SET_MAP_WEATHER_RADAR_COLORS",
    "SET_MAP_HEIGHT_COLORS",
    "SET_MAP_ALTITUDE_RANGE",
    "SET_MAP_PARAMS",
    "SET_3D_MAP_CAMERA_TRANSFORM",
    "SET_3D_MAP_CAMERA_FOV",
    "SET_LOAD_LATLON",
];

const QUERY_JS: &str = r#"(() => {
  const squash = (s) => String(s == null ? '' : s).replace(/\s+/g, '');
  const docOf = (screen) => [...globalThis.__dom.documents.values()].find((d) => d.__screen === screen);
  const shown = (el) => {
    for (let n = el; n && n.nodeType === 1; n = n.parentNode) {
      let cs = null;
      try { cs = getComputedStyle(n); } catch (e) { return false; }
      if (!cs || cs.display === 'none' || cs.visibility === 'hidden') return false;
    }
    return true;
  };
  const visible = (el) => {
    if (!shown(el)) return false;
    const r = el.getBoundingClientRect();
    return r.width > 0 && r.height > 0;
  };
  const pick = (d, text, exact, scope) => {
    const want = squash(text);
    let best = null;
    for (const el of candidates(d, scope)) {
      if (el.localName === 'style' || el.localName === 'script') continue;
      const t = squash(el.textContent);
      if (!(exact ? t === want : t.includes(want))) continue;
      if (!visible(el)) continue;
      const r = el.getBoundingClientRect();
      const area = r.width * r.height;
      if (!best || area < best.area - 0.5 || (Math.abs(area - best.area) <= 0.5 && (r.left < best.r.left - 0.5 || (Math.abs(r.left - best.r.left) <= 0.5 && r.top < best.r.top)))) {
        best = { el, r, area };
      }
    }
    return best;
  };
  const candidates = (d, scope) => (scope ? d.querySelectorAll(scope + ', ' + scope + ' *') : d.querySelectorAll('*'));
  globalThis.__fmsq = {
    find(screen, text, exact, scope) {
      const d = docOf(screen);
      if (!d) return null;
      const best = pick(d, text, exact, scope);
      if (!best) return null;
      const r = best.r;
      const x = r.left + r.width / 2;
      const y = r.top + r.height / 2;
      const t = __dom.paint.hitTest(d, x, y);
      const reachable = !!t && (t === best.el || best.el.contains(t) || t.contains(best.el));
      return { x, y, w: r.width, h: r.height, tag: best.el.localName, cls: String(best.el.className || ''), text: String(best.el.textContent || '').replace(/\s+/g, ' ').trim(), reachable };
    },
    clickDirect(screen, text, exact, scope) {
      const d = docOf(screen);
      if (!d) return false;
      const best = pick(d, text, exact, scope);
      if (!best) return false;
      const x = best.r.left + best.r.width / 2;
      const y = best.r.top + best.r.height / 2;
      const init = { bubbles: true, cancelable: true, composed: true, clientX: x, clientY: y, screenX: x, screenY: y, button: 0, buttons: 1, view: globalThis };
      const el = best.el;
      el.dispatchEvent(new __dom.PointerEvent('pointerdown', init));
      el.dispatchEvent(new __dom.MouseEvent('mousedown', init));
      el.dispatchEvent(new __dom.PointerEvent('pointerup', { ...init, buttons: 0 }));
      el.dispatchEvent(new __dom.MouseEvent('mouseup', { ...init, buttons: 0 }));
      el.dispatchEvent(new __dom.PointerEvent('click', { ...init, buttons: 0, detail: 1 }));
      return true;
    },
    leaves(screen, scope) {
      const d = docOf(screen);
      if (!d) return null;
      const out = [];
      for (const el of candidates(d, scope)) {
        if (el.localName === 'style' || el.localName === 'script') continue;
        if (el.children && el.children.length > 0) continue;
        const t = String(el.textContent || '').replace(/\s+/g, ' ').trim();
        if (!t) continue;
        if (!shown(el)) continue;
        const r = el.getBoundingClientRect();
        out.push([t, Math.round(r.left), Math.round(r.top)]);
      }
      return out;
    },
    strings(screen) {
      const d = docOf(screen);
      return d ? d._strings.slice() : null;
    },
    present(screen) {
      return !!docOf(screen);
    },
  };
  return 'ok';
})()"#;

pub struct VarStore {
    pub vars: HashMap<String, f64>,
    pub strings: HashMap<String, String>,
    pub errors: BTreeMap<String, usize>,
    pub unset_reads: BTreeMap<String, usize>,
    pub warnings: BTreeMap<String, usize>,
    pub trace_reads: bool,
}

impl VarStore {
    pub fn new() -> Self {
        Self { vars: HashMap::new(), strings: HashMap::new(), errors: BTreeMap::new(), unset_reads: BTreeMap::new(), warnings: BTreeMap::new(), trace_reads: false }
    }
}

impl Default for VarStore {
    fn default() -> Self {
        Self::new()
    }
}

impl Host for VarStore {
    fn get_var(&mut self, name: &str, _unit: &str) -> f64 {
        match self.vars.get(name) {
            Some(v) => *v,
            None => {
                if self.trace_reads {
                    *self.unset_reads.entry(name.to_string()).or_default() += 1;
                }
                0.
            }
        }
    }
    fn set_var(&mut self, name: &str, _unit: &str, value: f64) {
        self.vars.insert(name.to_string(), value);
    }
    fn get_string(&mut self, name: &str) -> String {
        self.strings.get(name).cloned().unwrap_or_default()
    }
    fn set_string(&mut self, name: &str, value: &str) {
        self.strings.insert(name.to_string(), value.to_string());
    }
    fn log(&mut self, level: LogLevel, message: &str) {
        if level == LogLevel::Warn && (message.contains("SimBrief") || message.contains("airway") || message.contains("Airway")) {
            let first: String = message.lines().next().unwrap_or("").chars().take(400).collect();
            *self.warnings.entry(first).or_default() += 1;
        }
        if level == LogLevel::Error {
            let first: String = message.lines().take(4).collect::<Vec<_>>().join(" | ").chars().take(500).collect();
            *self.errors.entry(first).or_default() += 1;
        }
    }
}

pub struct FmsHost<H: Host> {
    pub inner: H,
    pub nav: NavData,
    pub now_ms: f64,
    pub ofp: String,
    pub fetches: Vec<String>,
    pub nav_events: BTreeMap<String, usize>,
    pub unanswered_calls: BTreeMap<String, usize>,
    pub direct_clicks: Vec<String>,
}

pub fn attach_navdata<H: Host>(inner: H, xplane_root: &Path) -> Result<FmsHost<H>, String> {
    let mut nav = NavData::load(xplane_root)?;
    nav.wait_ready()?;
    Ok(FmsHost {
        inner,
        nav,
        now_ms: 0.,
        ofp: String::new(),
        fetches: Vec::new(),
        nav_events: BTreeMap::new(),
        unanswered_calls: BTreeMap::new(),
        direct_clicks: Vec::new(),
    })
}

impl<H: Host> FmsHost<H> {
    pub fn pump_nav(&mut self, cockpit: &mut Cockpit) {
        for (name, args) in self.nav.take_events_blocking(Duration::from_secs(5)) {
            *self.nav_events.entry(name.clone()).or_default() += 1;
            cockpit.broadcast(&name, &args);
        }
    }

    pub fn step(&mut self, cockpit: &mut Cockpit) {
        self.pump_nav(cockpit);
        let now = self.now_ms;
        cockpit.tick(self, now);
        self.now_ms += STEP_MS;
    }

    pub fn steps(&mut self, cockpit: &mut Cockpit, n: usize) {
        for _ in 0..n {
            self.step(cockpit);
        }
    }

    fn answer_fetch(&mut self, args_json: &str) -> CallReply {
        let args: Vec<serde_json::Value> = serde_json::from_str(args_json).unwrap_or_default();
        let url = args.get(1).and_then(|v| v.as_str()).unwrap_or("").to_string();
        self.fetches.push(url.clone());
        if url.contains("simbrief.com/api/xml.fetcher.php") && !self.ofp.is_empty() {
            return CallReply::Resolved(serde_json::json!({ "status": 200, "body": self.ofp }).to_string());
        }
        CallReply::Resolved(serde_json::json!({ "status": 404, "body": "" }).to_string())
    }
}

impl<H: Host> Host for FmsHost<H> {
    fn get_var(&mut self, name: &str, unit: &str) -> f64 {
        self.inner.get_var(name, unit)
    }

    fn set_var(&mut self, name: &str, unit: &str, value: f64) {
        self.inner.set_var(name, unit, value)
    }

    fn log(&mut self, level: LogLevel, message: &str) {
        self.inner.log(level, message)
    }

    fn get_var_reg(&mut self, id: usize, name: &str, unit: &str) -> f64 {
        self.inner.get_var_reg(id, name, unit)
    }

    fn set_var_reg(&mut self, id: usize, name: &str, unit: &str, value: f64) {
        self.inner.set_var_reg(id, name, unit, value)
    }

    fn get_string(&mut self, name: &str) -> String {
        if name == "GAME:FLIGHT NAVDATA DATE RANGE" {
            if let Some(range) = self.nav.date_range() {
                return range;
            }
        }
        self.inner.get_string(name)
    }

    fn set_string(&mut self, name: &str, value: &str) {
        self.inner.set_string(name, value)
    }

    fn call(&mut self, name: &str, args_json: &str) -> CallReply {
        if name == "fetch" {
            return self.answer_fetch(args_json);
        }
        if NavData::handles(name) {
            return match self.nav.call(name, args_json) {
                Ok(value) => CallReply::Resolved(value),
                Err(e) if e == NOT_READY => CallReply::Pending(0),
                Err(e) => CallReply::Rejected(e),
            };
        }
        if NO_OP_CALLS.contains(&name) {
            return CallReply::Resolved("null".to_string());
        }
        let reply = self.inner.call(name, args_json);
        if matches!(reply, CallReply::Rejected(_)) {
            *self.unanswered_calls.entry(name.to_string()).or_default() += 1;
        }
        reply
    }

    fn trigger(&mut self, name: &str, args_json: &str) {
        self.inner.trigger(name, args_json)
    }

    fn send_event(&mut self, name: &str, values: &[f64]) {
        self.inner.send_event(name, values)
    }

    fn get_magvar(&mut self, lat: f64, lon: f64) -> f64 {
        self.inner.get_magvar(lat, lon)
    }
}

pub fn setup_fixed_date(engine: &Engine) -> Result<(), String> {
    engine.eval(
        "sim-date",
        &format!(
            "(() => {{ const Real = Date; const now = () => {FIXED_EPOCH_MS} + (globalThis.performance ? globalThis.performance.now() : 0); \
             class SimDate extends Real {{ constructor(...a) {{ if (a.length === 0) super(now()); else super(...a); }} static now() {{ return now(); }} }} \
             globalThis.Date = SimDate; }})()"
        ),
    )?;
    Ok(())
}

pub fn setup_seeded_crypto(engine: &Engine) -> Result<(), String> {
    engine.eval(
        "sim-crypto",
        "(() => { let s = 0x2545F491; const next = () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s; };          globalThis.crypto = { getRandomValues(a) { for (let i = 0; i < a.length; i++) a[i] = next() & (a.BYTES_PER_ELEMENT === 1 ? 0xff : 0xffffffff); return a; },          randomUUID() { const h = () => (next() >>> 0).toString(16).padStart(8, '0'); const a = h() + h() + h() + h(); return a.slice(0, 8) + '-' + a.slice(8, 12) + '-4' + a.slice(13, 16) + '-a' + a.slice(17, 20) + '-' + a.slice(20, 32); } }; })()",
    )?;
    Ok(())
}

pub fn new_cockpit(package_dir: &Path) -> Result<Cockpit, String> {
    let html_ui = package_dir.join("html_ui");
    let panel_cfg = std::fs::read_to_string(package_dir.join("panel.cfg")).map_err(|e| format!("panel.cfg: {e}"))?;
    let mut options = CockpitOptions::new(html_ui, panel_cfg);
    options.panel_xml = std::fs::read_to_string(package_dir.join("panel.xml")).unwrap_or_default();
    options.budget = Duration::from_secs(20);
    Cockpit::new(options, &|engine: &Engine| {
        setup_fixed_date(engine)?;
        setup_seeded_crypto(engine)
    })
}

pub fn arinc_word(value: f32, ssm: u8) -> f64 {
    value.to_bits() as f64 + ssm as f64 * 4_294_967_296.
}

fn set_both<H: Host>(host: &mut H, name: &str, unit: &str, value: f64) {
    host.set_var(name, unit, value);
    host.set_var(&format!("A:{name}"), unit, value);
}

pub fn power_up_on_ground<H: Host>(host: &mut H, lat: f64, lon: f64) {
    host.set_var("L:A32NX_FMS_SWITCHING_KNOB", "number", 1.);
    host.set_var("L:A32NX_ELEC_247XP_BUS_IS_POWERED", "bool", 1.);
    for bus in BUSES {
        host.set_var(&format!("L:A32NX_ELEC_{bus}_BUS_IS_POWERED"), "bool", 1.);
    }
    host.set_var("L:A32NX_COLD_AND_DARK_SPAWN", "bool", 0.);
    for fmc in ["A", "B", "C"] {
        host.set_var(&format!("L:A32NX_FMC_{fmc}_IS_HEALTHY"), "bool", 1.);
    }
    for potentiometer in [80, 81, 82, 87, 88, 89, 90, 91, 92, 93, 98, 99] {
        host.set_var(&format!("LIGHT POTENTIOMETER:{potentiometer}"), "number", 1.);
    }
    set_both(host, "PLANE LATITUDE", "degrees", lat);
    set_both(host, "PLANE LONGITUDE", "degrees", lon);
    set_both(host, "PLANE ALTITUDE", "feet", 83.);
    set_both(host, "PLANE ALT ABOVE GROUND", "feet", 0.);
    set_both(host, "SIM ON GROUND", "bool", 1.);
    set_both(host, "GEAR IS ON GROUND", "bool", 1.);
    set_both(host, "FLAPS HANDLE INDEX", "number", 0.);
    for lgciu in [1, 2] {
        host.set_var(&format!("L:A32NX_LGCIU_{lgciu}_NOSE_GEAR_COMPRESSED"), "bool", 1.);
        host.set_var(&format!("L:A32NX_LGCIU_{lgciu}_LEFT_GEAR_COMPRESSED"), "bool", 1.);
        host.set_var(&format!("L:A32NX_LGCIU_{lgciu}_RIGHT_GEAR_COMPRESSED"), "bool", 1.);
    }
    set_both(host, "GROUND VELOCITY", "knots", 0.);
    set_both(host, "AMBIENT PRESSURE", "inHg", 29.92);
    set_both(host, "AMBIENT TEMPERATURE", "celsius", 15.);
    set_both(host, "SEA LEVEL PRESSURE", "millibars", 1013.25);
    set_both(host, "PLANE HEADING DEGREES MAGNETIC", "degrees", 270.);
    set_both(host, "PLANE HEADING DEGREES TRUE", "degrees", 270.);
    let zulu = 8. * 3600. + 53. * 60. + 20.;
    for name in ["ZULU TIME", "LOCAL TIME"] {
        host.set_var(&format!("E:{name}"), "seconds", zulu);
        host.set_var(name, "seconds", zulu);
    }
    for (name, v) in [("ZULU YEAR", 2025.), ("ZULU MONTH OF YEAR", 10.), ("ZULU DAY OF MONTH", 9.), ("ZULU DAY OF WEEK", 3.), ("ABSOLUTE TIME", 63_895_000_000.)] {
        host.set_var(&format!("E:{name}"), "number", v);
        host.set_var(name, "number", v);
    }
    host.set_var("E:SIMULATION RATE", "number", 1.);
    host.set_var("E:SIMULATION TIME", "seconds", 100.);
    for ir in 1..=3 {
        let pre = format!("L:A32NX_ADIRS_IR_{ir}");
        host.set_var(&format!("{pre}_LATITUDE"), "number", arinc_word(lat as f32, 3));
        host.set_var(&format!("{pre}_LONGITUDE"), "number", arinc_word(lon as f32, 3));
        host.set_var(&format!("{pre}_HEADING"), "number", arinc_word(270., 3));
        host.set_var(&format!("{pre}_TRUE_HEADING"), "number", arinc_word(270., 3));
        host.set_var(&format!("{pre}_TRUE_TRACK"), "number", arinc_word(270., 3));
        host.set_var(&format!("{pre}_TRACK"), "number", arinc_word(270., 3));
        host.set_var(&format!("{pre}_GROUND_SPEED"), "number", arinc_word(0., 3));
        host.set_var(&format!("{pre}_ALIGNED"), "bool", 1.);
        host.set_var(&format!("{pre}_MAINT_WORD"), "number", arinc_word(4., 3));
        host.set_var(&format!("L:A32NX_OVHD_ADIRS_IR_{ir}_MODE_SELECTOR_KNOB"), "number", 1.);
        host.set_var(&format!("L:A32NX_ADIRS_ADR_{ir}_ALTITUDE"), "number", arinc_word(83., 3));
        host.set_var(&format!("L:A32NX_ADIRS_ADR_{ir}_BAROMETRIC_CORRECTION_1"), "number", arinc_word(1013.25, 3));
    }
    host.set_var("L:A32NX_ADIRS_USES_GPS_AS_PRIMARY", "bool", 1.);
    host.set_var("L:A32NX_ADIRS_PFD_ALIGNED_FIRST", "bool", 1.);
    host.set_var("L:A32NX_ADIRS_PFD_ALIGNED_ATT", "bool", 1.);
    for side in ["L", "R"] {
        host.set_var(&format!("L:A32NX_EFIS_{side}_ND_MODE"), "number", 3.);
        host.set_var(&format!("L:A32NX_EFIS_{side}_ND_RANGE"), "number", 2.);
    }
}

struct Fix {
    ident: &'static str,
    kind: &'static str,
    lat: f64,
    lon: f64,
    via: &'static str,
    sid_star: bool,
}

const fn fx(ident: &'static str, kind: &'static str, lat: f64, lon: f64, via: &'static str, sid_star: bool) -> Fix {
    Fix { ident, kind, lat, lon, via, sid_star }
}

fn navlog_fix(f: &Fix) -> serde_json::Value {
    serde_json::json!({
        "ident": f.ident,
        "name": f.ident,
        "type": f.kind,
        "frequency": "0",
        "pos_lat": format!("{:.6}", f.lat),
        "pos_long": format!("{:.6}", f.lon),
        "stage": "CLB",
        "via_airway": f.via,
        "is_sid_star": if f.sid_star { "1" } else { "0" },
        "distance": "10",
        "track_true": "270",
        "track_mag": "270",
        "heading_true": "270",
        "heading_mag": "270",
        "altitude_feet": "10000",
        "ind_airsspeed": "250",
        "true_airspeed": "300",
        "mach": "0.50",
        "mach_thousandths": "500",
        "wind_component": "0",
        "groundspeed": "300",
        "time_leg": "120",
        "time_total": "120",
        "fuel_flow": "9000",
        "fuel_leg": "300",
        "fuel_totalused": "300",
        "fuel_min_onboard": "9000",
        "fuel_plan_onboard": "100000",
        "oat": "-10",
        "oat_isa_dev": "0",
        "wind_dir": "270",
        "wind_spd": "20",
        "shear": "0",
        "tropopause_feet": "36000",
        "ground_height": "0",
        "mora": "3000",
        "fir": "EGTT",
        "fir_units": "FL",
        "fir_valid_levels": "",
        "wind_data": { "level": [] },
        "fir_crossing": []
    })
}

pub const EGLL_KJFK_IDENTS: [&str; 13] =
    ["EGLL", "CPT", "IPNOX", "OLPOP", "SETEL", "N55W010", "N56W030", "N52W050", "GONZZ", "PUPPY", "PAYGE", "ENE", "KJFK"];

pub fn canned_ofp_json() -> String {
    let fixes = [
        fx("EGLL", "apt", 51.4706, -0.4619, "DCT", false),
        fx("D255G", "wpt", 51.458917, -0.647523, "CPT3G", true),
        fx("D259K", "wpt", 51.455411, -0.755266, "CPT3G", true),
        fx("WOD", "ndb", 51.452783, -0.8788, "CPT3G", true),
        fx("D100H", "wpt", 51.467817, -1.00984, "CPT3G", true),
        fx("CPT", "vor", 51.491572, -1.219692, "CPT3G", false),
        fx("IPNOX", "wpt", 54.227778, -2.706111, "DCT", false),
        fx("OLPOP", "wpt", 54.092222, -2.535556, "UN57", false),
        fx("SETEL", "wpt", 54.0124, -2.435753, "UN57", false),
        fx("N55W010", "ltlg", 55., -10., "DCT", false),
        fx("N56W030", "ltlg", 56., -30., "DCT", false),
        fx("N52W050", "ltlg", 52., -50., "DCT", false),
        fx("GONZZ", "wpt", 43.089444, -76.686667, "DCT", false),
        fx("PUPPY", "wpt", 43.05735, -75.29425, "Q822", false),
        fx("PAYGE", "wpt", 43.014022, -74.253544, "Q822", false),
        fx("ENE", "vor", 43.425672, -70.613525, "DCT", false),
        fx("ASPEN", "wpt", 42.815986, -70.911497, "ROBER2", true),
        fx("PVD", "vor", 41.724342, -71.42964, "ROBER2", true),
        fx("TRAIT", "wpt", 41.284653, -71.917597, "ROBER2", true),
        fx("PARCH", "wpt", 41.099228, -72.120739, "ROBER2", true),
        fx("KJFK", "apt", 40.639801, -73.778900, "DCT", false),
    ];
    let navlog: Vec<serde_json::Value> = fixes.iter().map(navlog_fix).collect();
    let airport = |icao: &str, iata: &str, name: &str, lat: f64, lon: f64, rwy: &str, alt: &str, level: &str| {
        serde_json::json!({
            "icao_code": icao, "iata_code": iata, "name": name, "pos_lat": format!("{lat:.6}"), "pos_long": format!("{lon:.6}"),
            "plan_rwy": rwy, "trans_alt": alt, "trans_level": level, "avg_wind_dir": "270", "avg_wind_spd": "15", "cruise_altitude": "10000"
        })
    };
    let ofp = serde_json::json!({
        "fetch": { "userid": DUMMY_SIMBRIEF_USER, "status": "Success" },
        "general": {
            "icao_airline": "FBW", "flight_number": "1", "initial_altitude": "36000", "costindex": "30",
            "route": "CPT3G CPT IPNOX UN57 SETEL N55W010 N56W030 N52W050 GONZZ Q822 PAYGE ENE ROBER2",
            "air_distance": "3000", "avg_tropopause": "36000", "avg_wind_dir": "270", "avg_wind_spd": "30"
        },
        "atc": { "callsign": "FBW1" },
        "aircraft": { "reg": "GFBWA", "icaocode": "A388" },
        "navlog": { "fix": navlog },
        "files": { "directory": "https://example.invalid/", "pdf": { "link": "" } },
        "origin": airport("EGLL", "LHR", "London Heathrow", 51.4706, -0.4619, "27L", "6000", "60"),
        "destination": airport("KJFK", "JFK", "John F Kennedy Intl", 40.639801, -73.7789, "22L", "18000", "180"),
        "alternate": airport("KEWR", "EWR", "Newark Liberty Intl", 40.6925, -74.168667, "22L", "18000", "180"),
        "times": {
            "contfuel_time": "600", "dest_timezone": "-5", "endurance": "50000", "est_block": "28000", "est_in": "1760030000",
            "est_off": "1760001000", "est_on": "1760029000", "est_out": "1760000000", "est_time_enroute": "27000",
            "etopsfuel_time": "0", "extrafuel_time": "0", "orig_timezone": "0", "reserve_time": "2700", "sched_block": "28000",
            "sched_in": "1760030000", "sched_off": "1760001000", "sched_on": "1760029000", "sched_out": "1760000000",
            "sched_time_enroute": "27000", "taxi_in": "300", "taxi_out": "900"
        },
        "weights": {
            "cargo": "5000", "est_ldw": "400000", "est_tow": "480000", "est_zfw": "380000", "max_ldw": "391000", "max_tow": "560000",
            "max_zfw": "373000", "pax_count_actual": "300", "bag_count_actual": "300", "pax_weight": "84", "bag_weight": "20",
            "payload": "35000", "freight_added": "5000"
        },
        "fuel": {
            "avg_fuel_flow": "9000", "contingency": "2000", "enroute_burn": "80000", "etops": "0", "extra": "0", "max_tanks": "254000",
            "min_takeoff": "95000", "plan_landing": "20000", "plan_ramp": "105000", "plan_takeoff": "103000", "reserve": "3000", "taxi": "2000"
        },
        "params": { "units": "kgs" },
        "weather": { "orig_metar": "EGLL 090850Z 27010KT 9999 FEW030 15/08 Q1013", "dest_metar": "KJFK 090851Z 22008KT 10SM FEW040 18/10 A2992" },
        "text": { "plan_html": "<div class=\"plan\">canned</div>" }
    });
    ofp.to_string()
}

#[derive(Clone, Debug)]
pub struct Hit {
    pub x: f64,
    pub y: f64,
    pub w: f64,
    pub h: f64,
    pub text: String,
    pub reachable: bool,
}

pub struct Mfd {
    pub view: String,
    pub screen: String,
}

pub fn view_with_gauge(cockpit: &Cockpit, needle: &str) -> Option<String> {
    cockpit.stats().into_iter().find(|s| s.gauges.iter().any(|g| g.contains(needle))).map(|s| s.name)
}

fn js_str(s: &str) -> String {
    json_string(s)
}

impl Mfd {
    pub fn locate(cockpit: &Cockpit) -> Result<Self, String> {
        let view = view_with_gauge(cockpit, "MFD/mfd.html").ok_or("panel.cfg has no MFD view")?;
        Ok(Self { view, screen: MFD_SCREEN.to_string() })
    }

    pub fn install_queries(&self, cockpit: &Cockpit) -> Result<(), String> {
        cockpit.eval_in(&self.view, QUERY_JS).map(|_| ())
    }

    pub fn find(&self, cockpit: &Cockpit, text: &str, exact: bool, scope: &str) -> Option<Hit> {
        let js = format!(
            "(() => {{ if (!globalThis.__fmsq) return 'null'; return JSON.stringify(__fmsq.find({}, {}, {}, {})); }})()",
            js_str(&self.screen),
            js_str(text),
            exact,
            js_str(scope)
        );
        let out = cockpit.eval_in(&self.view, &js).ok()?;
        let v: serde_json::Value = serde_json::from_str(&out).ok()?;
        if v.is_null() {
            return None;
        }
        Some(Hit {
            x: v["x"].as_f64()?,
            y: v["y"].as_f64()?,
            w: v["w"].as_f64().unwrap_or(0.),
            h: v["h"].as_f64().unwrap_or(0.),
            text: v["text"].as_str().unwrap_or("").to_string(),
            reachable: v["reachable"].as_bool().unwrap_or(false),
        })
    }

    pub fn texts(&self, cockpit: &Cockpit, scope: &str) -> Vec<String> {
        let js = format!(
            "(() => {{ if (!globalThis.__fmsq) return '[]'; return JSON.stringify(__fmsq.leaves({}, {}) || []); }})()",
            js_str(&self.screen),
            js_str(scope)
        );
        let out = cockpit.eval_in(&self.view, &js).unwrap_or_default();
        let rows: Vec<serde_json::Value> = serde_json::from_str(&out).unwrap_or_default();
        rows.iter().filter_map(|r| r[0].as_str().map(str::to_string)).collect()
    }

    pub fn positioned_texts(&self, cockpit: &Cockpit) -> Vec<String> {
        let js = format!(
            "(() => {{ if (!globalThis.__fmsq) return '[]'; return JSON.stringify(__fmsq.leaves({}, '') || []); }})()",
            js_str(&self.screen)
        );
        let out = cockpit.eval_in(&self.view, &js).unwrap_or_default();
        let rows: Vec<serde_json::Value> = serde_json::from_str(&out).unwrap_or_default();
        rows.iter().map(|r| format!("{}@{},{}", r[0].as_str().unwrap_or(""), r[1], r[2])).collect()
    }

    pub fn active_page(&self, cockpit: &Cockpit) -> String {
        cockpit.eval_in(&self.view, "String(SimVar.GetSimVarValue('L:A380X_MFD_L_ACTIVE_PAGE', 'string'))").unwrap_or_default()
    }

    pub fn click_at<H: Host>(&self, cockpit: &mut Cockpit, host: &mut FmsHost<H>, x: f64, y: f64) {
        cockpit.screen_event(&self.screen, "down", x, y, 0, 0.);
        host.steps(cockpit, 2);
        cockpit.screen_event(&self.screen, "up", x, y, 0, 0.);
        host.steps(cockpit, 4);
    }

    pub fn click_text<H: Host>(&self, cockpit: &mut Cockpit, host: &mut FmsHost<H>, text: &str, exact: bool, scope: &str) -> Result<Hit, String> {
        let hit = self.find(cockpit, text, exact, scope).ok_or_else(|| format!("no visible {text:?} on the MFD"))?;
        if std::env::var("FMS_VERBOSE").is_ok() {
            let js = format!(
                "(() => {{ const d = [...__dom.documents.values()].find((x) => x.__screen === {}); const t = __dom.paint.hitTest(d, {}, {}); return t ? t.localName + '|' + String(t.className) + '|' + String(t.textContent).slice(0, 40) : 'none'; }})()",
                js_str(&self.screen),
                hit.x,
                hit.y
            );
            println!("click {text:?} at ({:.0},{:.0}) hits {:?}", hit.x, hit.y, cockpit.eval_in(&self.view, &js));
        }
        if hit.reachable {
            self.click_at(cockpit, host, hit.x, hit.y);
        } else {
            let js = format!(
                "(() => String(__fmsq.clickDirect({}, {}, {}, {})))()",
                js_str(&self.screen),
                js_str(text),
                exact,
                js_str(scope)
            );
            cockpit.eval_in(&self.view, &js)?;
            host.direct_clicks.push(text.to_string());
            host.steps(cockpit, 6);
        }
        Ok(hit)
    }

    pub fn wait_for<H: Host>(&self, cockpit: &mut Cockpit, host: &mut FmsHost<H>, text: &str, exact: bool, scope: &str, max_steps: usize) -> Option<Hit> {
        for i in 0..max_steps {
            if i % 100 == 0 && std::env::var("FMS_VERBOSE").is_ok() {
                println!("waiting for {text:?} step {i}: {:?}", self.positioned_texts(cockpit).iter().take(14).collect::<Vec<_>>());
            }
            if i % 5 == 0 {
                if let Some(hit) = self.find(cockpit, text, exact, scope) {
                    return Some(hit);
                }
            }
            host.step(cockpit);
        }
        self.find(cockpit, text, exact, scope)
    }

    pub fn select_page<H: Host>(&self, cockpit: &mut Cockpit, host: &mut FmsHost<H>, selector: &str, item: &str) -> Result<(), String> {
        self.click_text(cockpit, host, selector, true, ".mfd-page-selector-label")?;
        host.steps(cockpit, 4);
        if std::env::var("FMS_VERBOSE").is_ok() {
            let js = format!("(() => {{ const d = [...__dom.documents.values()].find((x) => x.__screen === {}); return JSON.stringify([...d.querySelectorAll('.mfd-dropdown-menu')].map((m) => {{ const r = m.getBoundingClientRect(); return [getComputedStyle(m).display, r.left, r.top, r.width, r.height, [...m.querySelectorAll('*')].filter((e) => !e.children.length).map((e) => {{ const q = e.getBoundingClientRect(); return [e.textContent, q.left, q.top, q.width, q.height]; }})]; }})); }})()", js_str(&self.screen));
            println!("menus after opening {selector}: {:?}", cockpit.eval_in(&self.view, &js).map(|s| s.chars().take(600).collect::<String>()));
        }
        if let Ok(path) = std::env::var("FMS_EVAL_MENU") {
            let js = std::fs::read_to_string(path).unwrap_or_default();
            println!("FMS_EVAL_MENU: {:?}", cockpit.eval_in(&self.view, &js));
        }
        self.click_text(cockpit, host, item, true, ".mfd-dropdown-menu")
            .map_err(|e| format!("{selector} > {item}: {e}; menu shows {:?}", self.texts(cockpit, ".mfd-dropdown-menu")))?;
        host.steps(cockpit, 10);
        if std::env::var("FMS_VERBOSE").is_ok() {
            println!("after {selector} > {item}: active page {}", self.active_page(cockpit));
        }
        Ok(())
    }
}

#[derive(Debug, Default)]
pub struct RouteReport {
    pub fetches: Vec<String>,
    pub nav_events: BTreeMap<String, usize>,
    pub steps_to_ready: usize,
}

pub fn load_route<H: Host>(cockpit: &mut Cockpit, host: &mut FmsHost<H>, ofp_json: &str) -> Result<RouteReport, String> {
    host.ofp = ofp_json.to_string();
    cockpit.shared.store.op("set", SIMBRIEF_USER_KEY, DUMMY_SIMBRIEF_USER);
    power_up_on_ground(host, EGLL_LAT, EGLL_LON);

    let mfd = Mfd::locate(cockpit)?;
    let mut waited = 0;
    loop {
        host.steps(cockpit, 20);
        waited += 20;
        if cockpit.all_loaded() && mfd.install_queries(cockpit).is_ok() && mfd.find(cockpit, "ACTIVE", true, ".mfd-page-selector-label").is_some() {
            break;
        }
        if waited > 2400 {
            return Err(format!("the MFD never showed its page selectors (loaded: {}, texts: {:?})", cockpit.all_loaded(), mfd.texts(cockpit, "")));
        }
    }
    let steps_to_ready = waited;

    mfd.select_page(cockpit, host, "ACTIVE", "INIT")?;
    let request = "CPNY F-PLN REQUEST";
    if mfd.wait_for(cockpit, host, request, false, "", 800).is_none() {
        return Err(format!("the INIT page never offered {request:?}; it shows {:?}", mfd.texts(cockpit, "")));
    }
    mfd.click_text(cockpit, host, request, false, "")?;
    if std::env::var("FMS_VERBOSE").is_ok() {
        println!("requested; active page {}", mfd.active_page(cockpit));
    }
    let received = "RECEIVED CPNY F-PLN";
    if mfd.wait_for(cockpit, host, received, false, "", 1200).is_none() {
        return Err(format!(
            "the uplink never arrived; fetches {:?}; page shows {:?}; calls nothing answered {:?}",
            host.fetches,
            mfd.texts(cockpit, ""),
            host.unanswered_calls
        ));
    }
    mfd.click_text(cockpit, host, received, false, "")?;
    host.steps(cockpit, 6);
    if mfd.click_text(cockpit, host, "INSERT*", true, "").is_err() {
        mfd.click_text(cockpit, host, "INSERT", false, "")?;
    }
    host.steps(cockpit, 40);
    Ok(RouteReport { fetches: host.fetches.clone(), nav_events: host.nav_events.clone(), steps_to_ready })
}

pub fn show_fpln<H: Host>(cockpit: &mut Cockpit, host: &mut FmsHost<H>) -> Result<Vec<String>, String> {
    let mfd = Mfd::locate(cockpit)?;
    mfd.select_page(cockpit, host, "ACTIVE", "F-PLN")?;
    host.steps(cockpit, 60);
    let mut texts = mfd.texts(cockpit, "");
    if mfd.click_text(cockpit, host, "DEST", true, "").is_ok() {
        host.steps(cockpit, 40);
        texts.extend(mfd.texts(cockpit, ""));
    }
    Ok(texts)
}

pub fn nd_strings(cockpit: &Cockpit, gauge_needle: &str, screen: &str) -> Vec<String> {
    let Some(view) = view_with_gauge(cockpit, gauge_needle) else { return Vec::new() };
    let js = format!("(() => {{ if (!globalThis.__dom) return '[]'; const d = [...__dom.documents.values()].find((x) => x.__screen === {}); return JSON.stringify(d ? d._strings.slice() : []); }})()", js_str(screen));
    let out = cockpit.eval_in(&view, &js).unwrap_or_default();
    serde_json::from_str::<Vec<String>>(&out).unwrap_or_default()
}

#[test]
#[ignore]
fn fms_offline_flight_plan() {
    let package = std::path::PathBuf::from(std::env::var("FMS_GOLDEN").unwrap_or_else(|_| r"D:\A380\fbw-build\golden\2026-10-13".into()));
    let xplane = std::path::PathBuf::from(std::env::var("FMS_XPLANE_ROOT").unwrap_or_else(|_| r"D:\Steam Games\steamapps\common\X-Plane 12".into()));
    let mut cockpit = new_cockpit(&package).expect("cockpit");
    let mut store = VarStore::new();
    store.trace_reads = std::env::var("FMS_TRACE_READS").is_ok();
    let mut host = attach_navdata(store, &xplane).expect("navdata");

    let loaded = load_route(&mut cockpit, &mut host, &canned_ofp_json());
    let report = match loaded {
        Ok(report) => report,
        Err(e) => {
            if let Ok(mfd) = Mfd::locate(&cockpit) {
                println!("MFD positioned texts: {:?}", mfd.positioned_texts(&cockpit));
                if let Ok(path) = std::env::var("FMS_EVAL") {
                    let js = std::fs::read_to_string(path).unwrap_or_default();
                    println!("FMS_EVAL: {:?}", cockpit.eval_in(&mfd.view, &js));
                }
            }
            dump_state(&host);
            panic!("load_route: {e}");
        }
    };
    println!("fetches: {:?}", report.fetches);
    println!("navdata events: {:?}", report.nav_events);
    println!("MFD ready after {} steps", report.steps_to_ready);

    let texts = show_fpln(&mut cockpit, &mut host).expect("F-PLN page");
    println!("F-PLN page text: {texts:?}");
    let missing: Vec<&str> = EGLL_KJFK_IDENTS.iter().copied().filter(|id| !texts.iter().any(|t| t.contains(id))).collect();
    println!("route idents not on the F-PLN page: {missing:?}");

    for (needle, screen) in [("ND/nd.html?Index=1", "SCREEN_DU_NDL"), ("ND/nd.html?Index=2", "SCREEN_DU_NDR")] {
        host.steps(&mut cockpit, 60);
        let strings = nd_strings(&cockpit, needle, screen);
        let shown: Vec<&str> = EGLL_KJFK_IDENTS.iter().copied().filter(|id| strings.iter().any(|s| s.contains(id))).collect();
        println!("{screen}: {} strings, route idents drawn: {shown:?}", strings.len());
        println!("{screen} strings: {strings:?}");
    }
    dump_state(&host);
    for must in ["EGLL", "KJFK", "OLPOP", "PUPPY"] {
        assert!(texts.iter().any(|t| t.contains(must)), "{must} is not on the F-PLN page: {texts:?}");
    }
    assert!(missing.is_empty() || missing.len() < EGLL_KJFK_IDENTS.len() - 2, "most of the route is missing from the F-PLN page: {missing:?}");
}

fn dump_state(host: &FmsHost<VarStore>) {
    println!("fetches: {:?}", host.fetches);
    println!("calls nothing answered: {:?}", host.unanswered_calls);
    println!("navdata events: {:?}", host.nav_events);
    for (k, v) in &host.inner.strings {
        if k.contains("MFD") || k.contains("FMS") {
            println!("string {k} = {v}");
        }
    }
    let mut watched: Vec<_> = host.inner.vars.iter().filter(|(k, _)| k.contains("FLIGHT_PHASE") || k.contains("FMGC") || k.contains("FM1_") || k.contains("FMC")).collect();
    watched.sort_by(|a, b| a.0.cmp(b.0));
    println!("flight management variables: {watched:?}");
    println!("uplink warnings: {:#?}", host.inner.warnings);
    println!("script errors:");
    for (e, n) in &host.inner.errors {
        println!("{n:>5}x {e}");
    }
    let mut unset: Vec<(&String, &usize)> = host.inner.unset_reads.iter().collect();
    unset.sort_by(|a, b| b.1.cmp(a.1));
    if !unset.is_empty() {
        println!("variables read but never set ({})", unset.len());
        if let Ok(path) = std::env::var("FMS_TRACE_FILE") {
            let mut all: Vec<_> = unset.iter().map(|(name, n)| format!("{n:>7} {name}")).collect();
            all.sort_by(|a, b| a[8..].cmp(&b[8..]));
            let _ = std::fs::write(path, all.join("
"));
        }
    }
}

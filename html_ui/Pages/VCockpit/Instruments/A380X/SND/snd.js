(() => {
  const SVG = 'http://www.w3.org/2000/svg';
  const RAD = Math.PI / 180;
  const NM = 3440.065;
  const CX = 256;
  const CY = 262;
  const R = 166;
  const CLIP_Y = 346;
  const POWER_UP_S = 90;
  const STORE = 'A380X_SND_STATE';
  const MAX_WPTS = 20;
  const MENU = ['INSERT WPT', 'INSERT FIX', 'EDIT WPT/FIX', 'CLEAR WPT/FIX'];
  const FIELD_SIZES = [2, 90, 60, 2, 180, 60];
  const FIELD_CHARS = [[0, 1], [2, 2], [5, 2], [10, 1], [12, 3], [16, 2]];

  const words = new Uint32Array(1);
  const floats = new Float32Array(words.buffer);
  const arinc = (name) => {
    const raw = SimVar.GetSimVarValue(name, 'number') || 0;
    words[0] = (raw & 0xffffffff) >>> 0;
    return (Math.trunc(raw / 4294967296) & 3) === 3 ? floats[0] : null;
  };

  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
  const wrap = (v, n) => ((v % n) + n) % n;
  const norm = (a) => wrap(a, 360);
  const pad = (n, w) => String(n).padStart(w, '0');

  const bearing = (a, b) => {
    const p1 = a.lat * RAD;
    const p2 = b.lat * RAD;
    const dl = (b.lon - a.lon) * RAD;
    const y = Math.sin(dl) * Math.cos(p2);
    const x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
    return norm(Math.atan2(y, x) / RAD);
  };
  const arc = (a, b) => {
    const dp = (b.lat - a.lat) * RAD;
    const dl = (b.lon - a.lon) * RAD;
    const h = Math.sin(dp / 2) ** 2 + Math.cos(a.lat * RAD) * Math.cos(b.lat * RAD) * Math.sin(dl / 2) ** 2;
    return 2 * Math.asin(Math.min(1, Math.sqrt(h)));
  };
  const travel = (a, brg, d) => {
    const p1 = a.lat * RAD;
    const t = brg * RAD;
    const p2 = Math.asin(Math.sin(p1) * Math.cos(d) + Math.cos(p1) * Math.sin(d) * Math.cos(t));
    const l2 = a.lon * RAD + Math.atan2(Math.sin(t) * Math.sin(d) * Math.cos(p1), Math.cos(d) - Math.sin(p1) * Math.sin(p2));
    return { lat: p2 / RAD, lon: wrap(l2 / RAD + 180, 360) - 180 };
  };

  const degMin = (v) => {
    const a = Math.abs(v);
    const d = Math.floor(a);
    return [d, Math.min(59, Math.floor((a - d) * 60 + 1e-6))];
  };
  const shortPos = (p) => {
    const [la, lam] = degMin(p.lat);
    const [lo, lom] = degMin(p.lon);
    return `${p.lat < 0 ? 'S' : 'N'}${pad(la, 2)}${pad(lam, 2)} / ${p.lon < 0 ? 'W' : 'E'}${pad(lo, 3)}${pad(lom, 2)}`;
  };
  const longPos = (p) => {
    const part = (v, w, h) => {
      const a = Math.abs(v);
      const d = Math.floor(a);
      const m = Math.min(599, Math.floor((a - d) * 600 + 1e-4)) / 10;
      return `${h} ${pad(d, w)} ${m.toFixed(1).padStart(4, '0')}`;
    };
    return `${part(p.lat, 2, p.lat < 0 ? 'S' : 'N')} / ${part(p.lon, 3, p.lon < 0 ? 'W' : 'E')}`;
  };
  const distText = (nm) => (nm >= 20 ? String(Math.round(nm)) : nm.toFixed(1));

  const make = (tag, attrs, parent, text) => {
    const e = document.createElementNS(SVG, tag);
    for (const k in attrs) {
      e.setAttribute(k, attrs[k]);
    }
    if (text !== undefined) {
      e.textContent = text;
    }
    if (parent) {
      parent.appendChild(e);
    }
    return e;
  };
  const attr = (e, k, v) => {
    const s = String(v);
    if (e.getAttribute(k) !== s) {
      e.setAttribute(k, s);
    }
  };
  const text = (e, s) => {
    if (e.textContent !== s) {
      e.textContent = s;
    }
  };
  const show = (e, v) => attr(e, 'visibility', v ? 'inherit' : 'hidden');
  const rot = (e, a, x = CX, y = CY) => attr(e, 'transform', `rotate(${a.toFixed(2)} ${x} ${y})`);
  const tri = (x, y, w, h) => `M ${x} ${y} L ${x - w} ${y - h} L ${x + w} ${y - h} Z`;

  class SND extends BaseInstrument {
    get templateID() {
      return 'A380X_SND';
    }

    get isInteractive() {
      return true;
    }

    connectedCallback() {
      super.connectedCallback();
      this.state = this.load();
      this.mode = 'nav';
      this.menuItem = 0;
      this.pick = null;
      this.editor = null;
      this.off = false;
      this.power = null;
      this.timer = null;
      this.held = 0;
      this.heldAt = 0;
      this.lastTurn = 0;
      this.next = 0;
      this.prevTime = 0;
      this.pos = null;
      this.build();
    }

    Init() {
      super.Init();
      this.power = SimVar.GetSimVarValue('L:A32NX_COLD_AND_DARK_SPAWN', 'bool') ? 'off' : 'standby';
      this.offset = SimVar.GetSimVarValue('L:A380X_SND_MANUAL_BRIGHTNESS_OFFSET', 'number') || 0;
    }

    load() {
      try {
        const s = JSON.parse(GetStoredData(STORE) || 'null');
        if (s && Array.isArray(s.wpts)) {
          return { wpts: s.wpts, fix: s.fix || null, nav: s.nav || null };
        }
      } catch (e) {}
      return { wpts: [], fix: null, nav: null };
    }

    save() {
      try {
        SetStoredData(STORE, JSON.stringify(this.state));
      } catch (e) {}
    }

    build() {
      const root = this.querySelector('#SND_ROOT');
      const svg = make('svg', { viewBox: '0 0 512 512' }, root);
      const defs = make('defs', {}, svg);
      const wedge = (id, deg) => {
        const r = R + 30;
        const sx = r * Math.sin(deg * RAD);
        const sy = -r * Math.cos(deg * RAD);
        make('path', { d: `M ${CX} ${CY} L ${CX - sx} ${CY + sy} A ${r} ${r} 0 1 1 ${CX + sx} ${CY + sy} Z` }, make('clipPath', { id }, defs));
      };
      wedge('snd-sector', 122);
      wedge('snd-labels', 136);
      const low = make('clipPath', { id: 'snd-low' }, defs);
      make('rect', { x: 0, y: 0, width: 512, height: CLIP_Y }, low);

      this.main = make('g', {}, svg);
      const m = this.main;
      make('text', { x: 16, y: 46, 'font-size': 24, class: 'w' }, m, 'GS');
      this.gs = make('text', { x: 56, y: 46, 'font-size': 30, class: 'g' }, m);
      this.src = make('g', {}, m);
      make('rect', { x: 220, y: 19, width: 72, height: 34, class: 'sw', 'stroke-width': 2 }, this.src);
      this.srcText = make('text', { x: CX, y: 46, 'font-size': 26, class: 'w mid' }, this.src);
      this.trueTag = make('text', { x: CX, y: 76, 'font-size': 20, class: 'c mid' }, m, 'TRUE');

      this.toTop = make('g', {}, m);
      make('text', { x: 300, y: 46, 'font-size': 22, class: 'w' }, this.toTop, 'TO WPT');
      this.toBrg = make('text', { x: 496, y: 46, 'font-size': 30, class: 'g end' }, this.toTop);
      this.toDist = make('text', { x: 458, y: 82, 'font-size': 30, class: 'g end' }, this.toTop);
      make('text', { x: 462, y: 82, 'font-size': 20, class: 'c' }, this.toTop, 'NM');

      this.fixTop = make('g', {}, m);
      make('path', { d: `M 31 78 L 22 94 L 40 94 Z M 16 94 L 46 94`, class: 'sm', 'stroke-width': 3 }, this.fixTop);
      make('text', { x: 52, y: 98, 'font-size': 26, class: 'm' }, this.fixTop, 'FIX');
      this.fixDist = make('text', { x: 18, y: 134, 'font-size': 30, class: 'm' }, this.fixTop);
      make('text', { x: 18, y: 160, 'font-size': 20, class: 'c' }, this.fixTop, 'NM');

      this.rose = make('g', {}, m);
      this.card = make('g', {}, make('g', { 'clip-path': 'url(#snd-sector)' }, this.rose));
      this.labels = make('g', {}, make('g', { 'clip-path': 'url(#snd-labels)' }, this.rose));
      for (let a = 0; a < 360; a += 5) {
        const long = a % 10 === 0;
        rot(make('line', { x1: CX, y1: CY - R, x2: CX, y2: CY - R + (long ? 18 : 10), class: 'sw', 'stroke-width': long ? 3 : 2 }, this.card), a);
        if (a % 30 === 0) {
          rot(make('text', { x: CX, y: CY - R + 46, 'font-size': 28, class: 'w mid' }, this.labels, String(a / 10)), a);
        }
      }
      make('path', { d: `M ${CX - 12} ${CY - R - 22} L ${CX + 12} ${CY - R - 22} L ${CX} ${CY - R - 3} Z`, class: 'y' }, this.rose);
      for (const a of [-90, -45, 45, 90]) {
        rot(make('path', { d: tri(CX, CY - R - 4, 8, 14), class: 'w' }, this.rose), a);
      }
      this.track = make('path', { d: `M ${CX} ${CY - R + 6} L ${CX + 7} ${CY - R + 15} L ${CX} ${CY - R + 24} L ${CX - 7} ${CY - R + 15} Z`, class: 'sg', 'stroke-width': 3 }, this.rose);

      const pointers = make('g', { 'clip-path': 'url(#snd-low)' }, this.rose);
      this.course = make('g', {}, pointers);
      make('line', { x1: CX, y1: CY - R + 24, x2: CX, y2: CY - R * 0.5, class: 'sw', 'stroke-width': 4 }, this.course);
      make('path', { d: `M ${CX} ${CY - R + 8} L ${CX - 10} ${CY - R + 28} L ${CX + 10} ${CY - R + 28} Z`, class: 'w' }, this.course);
      make('line', { x1: CX, y1: CY + R * 0.5, x2: CX, y2: CY + R - 20, class: 'sw', 'stroke-width': 4 }, this.course);
      for (const d of [-80, -60, -40, -20, 20, 40, 60, 80]) {
        const h = Math.abs(d) % 40 === 0 ? 15 : 8;
        make('line', { x1: CX + d, y1: CY - h, x2: CX + d, y2: CY + h, class: 'sw', 'stroke-width': 3 }, this.course);
      }
      this.devBar = make('g', {}, this.course);
      make('line', { x1: CX, y1: CY - R * 0.45, x2: CX, y2: CY + R * 0.45, class: 'sw', 'stroke-width': 4 }, this.devBar);
      make('path', { d: `M ${CX} ${CY - R * 0.3 - 14} L ${CX - 9} ${CY - R * 0.3 + 2} L ${CX + 9} ${CY - R * 0.3 + 2} Z`, class: 'w' }, this.devBar);

      this.fixPtr = make('g', {}, pointers);
      make('line', { x1: CX, y1: CY - R + 26, x2: CX, y2: CY + R - 26, class: 'sm', 'stroke-width': 3 }, this.fixPtr);
      const fy = CY - R * 0.55;
      make('path', { d: `M ${CX} ${fy - 16} L ${CX - 11} ${fy + 4} L ${CX + 11} ${fy + 4} Z M ${CX - 19} ${fy + 4} L ${CX + 19} ${fy + 4}`, class: 'sm', 'stroke-width': 3 }, this.fixPtr);

      make('path', { d: `M ${CX} ${CY - 20} L ${CX} ${CY + 24} M ${CX - 26} ${CY - 3} L ${CX + 26} ${CY - 3} M ${CX - 10} ${CY + 20} L ${CX + 10} ${CY + 20}`, class: 'sy', 'stroke-width': 5, 'stroke-linecap': 'round' }, m);

      this.hdgFlag = make('g', {}, m);
      make('rect', { x: CX - 46, y: CY - 70, width: 92, height: 42, class: 'sr', 'stroke-width': 3 }, this.hdgFlag);
      make('text', { x: CX, y: CY - 38, 'font-size': 30, class: 'r mid' }, this.hdgFlag, 'HDG');

      this.posBox = make('g', {}, m);
      make('rect', { x: 10, y: 353, width: 84, height: 32, class: 'sw', 'stroke-width': 2 }, this.posBox);
      this.posSrc = make('text', { x: 52, y: 378, 'font-size': 22, class: 'w mid' }, this.posBox);
      this.posText = make('text', { x: 102, y: 379, 'font-size': 26, class: 'g' }, m);
      make('line', { x1: 8, y1: 394, x2: 504, y2: 394, class: 'sw', 'stroke-width': 2 }, m);

      this.list = make('g', {}, m);
      this.rows = [0, 1, 2, 3].map((i) => ({
        label: make('text', { x: 12, y: 424 + 29 * i, 'font-size': 22, class: 'w' }, this.list),
        value: make('text', { x: 96, y: 424 + 29 * i, 'font-size': 22, class: 'g' }, this.list),
      }));
      this.leg = make('g', {}, this.list);
      make('path', { d: 'M 312 446 H 326 V 475 H 312', class: 'sg', 'stroke-width': 2 }, this.leg);
      this.legBrg = make('text', { x: 336, y: 455, 'font-size': 20, class: 'g' }, this.leg);
      this.legDist = make('text', { x: 336, y: 480, 'font-size': 20, class: 'g' }, this.leg);

      this.ui = make('g', {}, m);
      this.uiTitle = make('text', { x: 12, y: 424, 'font-size': 22, class: 'c' }, this.ui);
      this.uiBox = make('rect', { x: 8, y: 447, width: 496, height: 36, class: 'sc', 'stroke-width': 2 }, this.ui);
      this.uiItem = make('text', { x: CX, y: 474, 'font-size': 26, class: 'c mid' }, this.ui);
      this.uiLabel = make('text', { x: 14, y: 474, 'font-size': 24, class: 'w' }, this.ui);
      this.uiValue = make('text', { x: 100, y: 474, 'font-size': 24, class: 'g' }, this.ui);
      this.fieldBox = make('rect', { y: 448, height: 34, class: 'sc', 'stroke-width': 2 }, this.ui);
      this.coord = make('text', { x: 24, y: 476, 'font-size': 30, class: 'w' }, this.ui);

      this.test = make('g', {}, svg);
      const box = (x, y, w, label) => {
        make('rect', { x, y, width: w, height: 40, class: 'y' }, this.test);
        return make('text', { x: x + w / 2, y: y + 31, 'font-size': 30, class: 'k mid' }, this.test, label);
      };
      box(14, 26, 84, 'GS');
      box(CX - 50, CY - 70, 100, 'HDG');
      box(10, 350, 104, 'POS');
      box(CX - 90, CY + 40, 180, 'INIT');
      this.testTimer = make('text', { x: CX + 84, y: CY + 71, 'font-size': 30, class: 'k end' }, this.test);

      this.dim = make('rect', { x: 0, y: 0, width: 512, height: 512, fill: '#000000', opacity: 0 }, svg);
    }

    Update() {
      super.Update();
      if (this.power === null) {
        return;
      }
      const now = Date.now();
      const dt = this.prevTime ? (now - this.prevTime) / 1000 : 0;
      this.prevTime = now;
      this.updatePower(dt);
      this.updateBrightness(now);
      if (now < this.next) {
        return;
      }
      this.next = now + 50;
      this.draw();
    }

    updatePower(dt) {
      const ias = SimVar.GetSimVarValue('AIRSPEED INDICATED', 'knots') || 0;
      const live =
        !!SimVar.GetSimVarValue('L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED', 'bool') ||
        (ias > 50 && !!SimVar.GetSimVarValue('L:A32NX_ELEC_DC_HOT_1_BUS_IS_POWERED', 'bool'));
      if (this.timer !== null) {
        this.timer -= dt;
        if (this.timer <= 0) {
          this.timer = null;
          if (this.power === 'standby') {
            this.power = 'off';
          } else if (this.power === 'test') {
            this.power = 'on';
          }
        }
      }
      if (this.power === 'on' && !live) {
        this.power = 'standby';
        this.timer = 10;
      } else if (this.power === 'standby' && live) {
        this.power = 'on';
        this.timer = null;
      } else if (this.power === 'off' && live) {
        this.power = 'test';
        this.timer = POWER_UP_S;
      } else if (this.power === 'test' && !live) {
        this.power = 'off';
        this.timer = null;
      }
    }

    updateBrightness(now) {
      const raw = (SimVar.GetSimVarValue('GLASSCOCKPIT AUTOMATIC BRIGHTNESS', 'percent') || 0) / 100;
      const auto = 0.2 + 0.79 * clamp(raw, 0, 1);
      if (this.held && now - this.heldAt >= 150) {
        this.heldAt = now;
        this.offset = clamp(this.offset + 0.025 * this.held, 0.03 - auto, 0.99 - auto);
        SimVar.SetSimVarValue('L:A380X_SND_MANUAL_BRIGHTNESS_OFFSET', 'number', this.offset);
      }
      attr(this.dim, 'opacity', (1 - clamp(auto + this.offset, 0.03, 0.99)).toFixed(3));
    }

    draw() {
      const on = !this.off && (this.power === 'on' || this.power === 'standby');
      show(this.main, on);
      show(this.test, !this.off && this.power === 'test');
      if (this.power === 'test') {
        text(this.testTimer, `${Math.max(0, Math.ceil(this.timer || 0))}s`);
      }
      if (!on) {
        return;
      }

      const trueRef = !!SimVar.GetSimVarValue('L:A32NX_PUSH_TRUE_REF', 'bool');
      const ir = [3, 1].find((n) => arinc(`L:A32NX_ADIRS_IR_${n}_HEADING`) !== null);
      let hdg = null;
      let trk = null;
      let magvar = null;
      let gs = null;
      if (ir) {
        const mag = arinc(`L:A32NX_ADIRS_IR_${ir}_HEADING`);
        const tru = arinc(`L:A32NX_ADIRS_IR_${ir}_TRUE_HEADING`);
        if (tru !== null) {
          magvar = tru - mag;
        }
        hdg = trueRef ? tru : mag;
        trk = arinc(`L:A32NX_ADIRS_IR_${ir}_${trueRef ? 'TRUE_TRACK' : 'TRACK'}`);
        gs = arinc(`L:A32NX_ADIRS_IR_${ir}_GROUND_SPEED`);
      }
      if (magvar === null) {
        magvar = SimVar.GetSimVarValue('MAGVAR', 'degrees') || 0;
      }
      const ref = (t) => norm(trueRef ? t : t - magvar);

      let pos = null;
      let posSrc = '';
      for (const n of [3, 1]) {
        const lat = arinc(`L:A32NX_ADIRS_IR_${n}_LATITUDE`);
        const lon = arinc(`L:A32NX_ADIRS_IR_${n}_LONGITUDE`);
        if (lat !== null && lon !== null) {
          pos = { lat, lon };
          posSrc = `GPIR${n}`;
          break;
        }
      }
      if (!pos) {
        const lat = SimVar.GetSimVarValue('GPS POSITION LAT', 'degrees');
        const lon = SimVar.GetSimVarValue('GPS POSITION LON', 'degrees');
        if (lat || lon) {
          pos = { lat, lon };
          posSrc = 'GPS';
        }
      }
      if (gs === null && pos) {
        gs = SimVar.GetSimVarValue('GPS GROUND SPEED', 'knots');
      }
      this.pos = pos;

      text(this.gs, gs === null ? '---' : String(Math.round(gs)));
      show(this.src, !!ir);
      text(this.srcText, ir ? `IR${ir}` : '');
      show(this.trueTag, trueRef);

      const s = this.state;
      let to = null;
      let dtk = null;
      let xtk = 0;
      if (s.nav && pos && s.wpts[s.nav.to]) {
        to = s.wpts[s.nav.to];
        const from = s.nav.from;
        const leg = arc(from, to);
        const t12 = bearing(from, to);
        const d13 = arc(from, pos);
        const diff = (bearing(from, pos) - t12) * RAD;
        const x = Math.asin(Math.sin(d13) * Math.sin(diff));
        const along = Math.acos(clamp(Math.cos(d13) / Math.cos(x), -1, 1)) * (Math.cos(diff) < 0 ? -1 : 1);
        if (along >= leg - 0.2 / NM && s.nav.to + 1 < s.wpts.length) {
          s.nav = { from: { lat: to.lat, lon: to.lon }, to: s.nav.to + 1 };
          this.save();
          this.next = 0;
          return;
        }
        xtk = x * NM;
        dtk = leg - along > 0.05 / NM ? bearing(travel(from, t12, clamp(along, 0, leg)), to) : t12;
      }

      const hdgOk = hdg !== null;
      show(this.hdgFlag, !hdgOk);
      show(this.rose, hdgOk);
      if (hdgOk) {
        rot(this.card, -hdg);
        rot(this.labels, -hdg);
        show(this.track, trk !== null);
        if (trk !== null) {
          rot(this.track, trk - hdg);
        }
        show(this.course, dtk !== null);
        if (dtk !== null) {
          rot(this.course, ref(dtk) - hdg);
          attr(this.devBar, 'transform', `translate(${clamp(-xtk * 8, -88, 88).toFixed(1)} 0)`);
        }
      }

      show(this.toTop, !!to);
      if (to) {
        text(this.toBrg, `${pad(Math.round(ref(bearing(pos, to))) % 360, 3)}${trueRef ? ' T' : '°'}`);
        text(this.toDist, distText(arc(pos, to) * NM));
      }

      const fixOk = !!(s.fix && pos);
      show(this.fixTop, fixOk);
      show(this.fixPtr, fixOk && hdgOk);
      if (fixOk) {
        text(this.fixDist, distText(arc(pos, s.fix) * NM));
        if (hdgOk) {
          rot(this.fixPtr, ref(bearing(pos, s.fix)) - hdg);
        }
      }

      show(this.posBox, !!pos);
      text(this.posSrc, posSrc);
      text(this.posText, pos ? longPos(pos) : 'POS');
      attr(this.posText, 'class', pos ? 'g' : 'r');

      show(this.list, this.mode === 'nav');
      show(this.ui, this.mode !== 'nav');
      if (this.mode === 'nav') {
        this.drawList(to);
      } else {
        this.drawUi();
      }
    }

    drawList(to) {
      const s = this.state;
      const rows = [];
      if (to) {
        rows.push(['FROM', s.nav.from, 'g', 'w']);
        rows.push(['TO', to, 'w', 'w']);
        const next = s.wpts[s.nav.to + 1];
        if (next) {
          rows.push(['NEXT', next, 'g', 'w']);
        }
        show(this.leg, !!next);
        if (next) {
          text(this.legBrg, `${pad(Math.round(bearing(to, next)) % 360, 3)} T`);
          text(this.legDist, `${distText(arc(to, next) * NM)} NM`);
        }
      } else {
        show(this.leg, false);
        s.wpts.slice(0, 3).forEach((w, i) => rows.push([`WPT${i + 1}`, w, 'g', 'w']));
      }
      if (s.fix) {
        rows.push(['FIX', s.fix, 'm', 'm']);
      }
      this.rows.forEach((row, i) => {
        const r = rows[i];
        text(row.label, r ? r[0] : '');
        text(row.value, r ? shortPos(r[1]) : '');
        if (r) {
          attr(row.value, 'class', r[2]);
          attr(row.label, 'class', r[3]);
        }
      });
    }

    drawUi() {
      const menu = this.mode === 'menu';
      const pick = this.mode === 'pick';
      const edit = this.mode === 'edit';
      show(this.uiBox, !edit);
      show(this.uiItem, menu);
      show(this.uiLabel, pick);
      show(this.uiValue, pick);
      show(this.fieldBox, edit);
      show(this.coord, edit);
      if (menu) {
        text(this.uiTitle, 'MENU');
        text(this.uiItem, MENU[this.menuItem]);
      } else if (pick) {
        const item = this.pick.items[this.pick.index];
        text(this.uiTitle, this.pick.title);
        text(this.uiLabel, item.label);
        text(this.uiValue, shortPos(item.pos));
        attr(this.uiValue, 'class', item.kind === 'fix' ? 'm' : 'g');
      } else if (edit) {
        const e = this.editor;
        text(this.uiTitle, e.title);
        const v = e.v;
        text(this.coord, `${v[0] ? 'S' : 'N'} ${pad(v[1], 2)} ${pad(v[2], 2)} / ${v[3] ? 'W' : 'E'} ${pad(v[4], 3)} ${pad(v[5], 2)}`);
        const [first, count] = FIELD_CHARS[e.field];
        try {
          const x0 = this.coord.getStartPositionOfChar(first).x;
          const x1 = this.coord.getEndPositionOfChar(first + count - 1).x;
          attr(this.fieldBox, 'x', (x0 - 4).toFixed(1));
          attr(this.fieldBox, 'width', (x1 - x0 + 8).toFixed(1));
        } catch (err) {}
      }
    }

    onInteractionEvent(args) {
      const name = String(args && args[0]);
      if (!name.startsWith('A380X_SND_')) {
        return;
      }
      const ev = name.slice('A380X_SND_'.length);
      if (ev === 'PLUS_PRESSED' || ev === 'MINUS_PRESSED') {
        this.held = ev === 'PLUS_PRESSED' ? 1 : -1;
        this.heldAt = 0;
      } else if (ev === 'PLUS_RELEASED' || ev === 'MINUS_RELEASED') {
        this.held = 0;
      } else if (ev === 'MODE_PRESSED') {
        this.off = !this.off;
      } else if (this.power === 'on' || this.power === 'standby') {
        this.input(ev);
      }
      this.next = 0;
    }

    input(ev) {
      if (ev === 'MENU_PRESSED') {
        this.mode = this.mode === 'nav' ? 'menu' : 'nav';
        this.menuItem = 0;
        return;
      }
      if (ev === 'DIR_TO_PRESSED') {
        this.dirTo();
        return;
      }
      const turn = ev === 'KNOB_CLOCKWISE' ? 1 : ev === 'KNOB_ANTI_CLOCKWISE' ? -1 : 0;
      const push = ev === 'KNOB_PRESSED';
      if (this.mode === 'menu') {
        if (turn) {
          this.menuItem = wrap(this.menuItem + turn, MENU.length);
        } else if (push) {
          this.menuSelect();
        }
      } else if (this.mode === 'pick') {
        if (turn) {
          this.pick.index = wrap(this.pick.index + turn, this.pick.items.length);
        } else if (push) {
          const item = this.pick.items[this.pick.index];
          this.mode = 'nav';
          this.pick.select(item);
          this.save();
        }
      } else if (this.mode === 'edit') {
        if (turn) {
          const now = Date.now();
          const fast = now - this.lastTurn < 120;
          this.lastTurn = now;
          const f = this.editor.field;
          const step = fast && (f === 1 || f === 4) ? 5 : 1;
          this.editor.v[f] = wrap(this.editor.v[f] + turn * step, FIELD_SIZES[f]);
        } else if (push) {
          this.editor.field += 1;
          if (this.editor.field === FIELD_SIZES.length) {
            const v = this.editor.v;
            const p = { lat: (v[0] ? -1 : 1) * (v[1] + v[2] / 60), lon: (v[3] ? -1 : 1) * (v[4] + v[5] / 60) };
            this.mode = 'nav';
            this.editor.done(p);
            this.save();
          }
        }
      }
    }

    candidates(withFix) {
      const s = this.state;
      const items = s.wpts.map((w, i) => ({ kind: 'wpt', i, label: `WPT${i + 1}`, pos: w }));
      if (withFix && s.fix) {
        items.push({ kind: 'fix', label: 'FIX', pos: s.fix });
      }
      return items;
    }

    openEditor(title, p, done) {
      const [latD, latM] = degMin(p.lat);
      const [lonD, lonM] = degMin(p.lon);
      this.editor = { title, done, field: 0, v: [p.lat < 0 ? 1 : 0, latD, latM, p.lon < 0 ? 1 : 0, lonD, lonM] };
      this.mode = 'edit';
    }

    openPick(title, items, index, select) {
      if (!items.length) {
        return false;
      }
      this.pick = { title, items, index: clamp(index, 0, items.length - 1), select };
      this.mode = 'pick';
      return true;
    }

    menuSelect() {
      const s = this.state;
      const here = this.pos || { lat: 0, lon: 0 };
      const item = MENU[this.menuItem];
      if (item === 'INSERT WPT') {
        if (s.wpts.length < MAX_WPTS) {
          this.openEditor(`INSERT WPT${s.wpts.length + 1}`, s.wpts[s.wpts.length - 1] || here, (p) => s.wpts.push(p));
        }
      } else if (item === 'INSERT FIX') {
        this.openEditor('INSERT FIX', s.fix || here, (p) => {
          s.fix = p;
        });
      } else if (item === 'EDIT WPT/FIX') {
        this.openPick('EDIT WPT/FIX', this.candidates(true), 0, (c) =>
          this.openEditor(`EDIT ${c.label}`, c.pos, (p) => {
            if (c.kind === 'fix') {
              s.fix = p;
            } else {
              s.wpts[c.i] = p;
            }
          }),
        );
      } else if (item === 'CLEAR WPT/FIX') {
        this.openPick('CLEAR WPT/FIX', this.candidates(true), 0, (c) => this.clear(c));
      }
    }

    clear(c) {
      const s = this.state;
      if (c.kind === 'fix') {
        s.fix = null;
        return;
      }
      s.wpts.splice(c.i, 1);
      if (!s.nav) {
        return;
      }
      if (c.i < s.nav.to) {
        s.nav.to -= 1;
      } else if (c.i === s.nav.to && !s.wpts[s.nav.to]) {
        s.nav = null;
      }
    }

    dirTo() {
      const s = this.state;
      if (!this.pos) {
        return;
      }
      const go = (i) => {
        const here = this.pos || s.wpts[i];
        s.nav = { from: { lat: here.lat, lon: here.lon }, to: i };
      };
      if (s.wpts.length) {
        this.openPick('DIR TO', this.candidates(false), s.nav ? s.nav.to : 0, (c) => go(c.i));
      } else {
        this.openEditor('DIR TO', this.pos, (p) => {
          s.wpts.push(p);
          go(s.wpts.length - 1);
        });
      }
    }
  }

  registerInstrument('a380x-snd', SND);
})();

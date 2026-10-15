(() => {
  const W = 512;
  const H = 256;
  const RAD = Math.PI / 180;
  const NM = 3440.065;
  const BASE = '/Pages/VCockpit/Instruments/A380X/IFE/';
  const PAGE_S = 12;
  const DRAW_EVERY_MS = 1000;
  const PAGES = ['welcome', 'map', 'info', 'map'];
  const DEFAULT_THEME = {
    airline: '',
    welcome: 'Welcome aboard',
    background: '#0b1626',
    accent: '#c9a45c',
    text: '#ffffff',
    land: '#1f3b2c',
    sea: '#0d2340',
  };

  const distanceNm = (a, b) => {
    const dp = (b.lat - a.lat) * RAD;
    const dl = (b.lon - a.lon) * RAD;
    const h = Math.sin(dp / 2) ** 2 + Math.cos(a.lat * RAD) * Math.cos(b.lat * RAD) * Math.sin(dl / 2) ** 2;
    return 2 * NM * Math.asin(Math.min(1, Math.sqrt(h)));
  };
  const wrapLon = (lon, centre) => ((((lon - centre) % 360) + 540) % 360) - 180 + centre;
  const fmtTime = (h) => {
    if (!isFinite(h) || h <= 0) return '--:--';
    const m = Math.round(h * 60);
    return `${Math.floor(m / 60)}h ${String(m % 60).padStart(2, '0')}m`;
  };

  class IFE extends BaseInstrument {
    get templateID() {
      return 'A380X_IFE';
    }

    get isInteractive() {
      return false;
    }

    connectedCallback() {
      super.connectedCallback();
      this.canvas = this.querySelector('#IFE_CANVAS');
      this.ctx = this.canvas.getContext('2d');
      this.theme = Object.assign({}, DEFAULT_THEME);
      this.images = {};
      this.land = [];
      this.themeRev = -1;
      this.lastDraw = 0;
      this.started = Date.now();
      this.origin = null;
      this.wasOnGround = true;
      this.land = window.IFE_LAND || [];
    }

    loadTheme(rev) {
      this.themeRev = rev;
      const apply = () => {
        this.theme = Object.assign({}, DEFAULT_THEME, window.IFE_THEME || {});
        this.images = {};
        for (const key of ['ife_logo', 'ife_background']) {
          if (this.theme[key]) {
            const img = new Image();
            img.src = `${BASE}theme/${this.theme[key]}?r=${rev}`;
            this.images[key] = img;
          }
        }
      };
      window.IFE_THEME = null;
      const tag = document.createElement('script');
      tag.src = `${BASE}theme/theme.js?r=${rev}`;
      tag.onload = apply;
      tag.onerror = apply;
      document.head.appendChild(tag);
    }

    read() {
      const lat = SimVar.GetSimVarValue('GPS POSITION LAT', 'degrees') || 0;
      const lon = SimVar.GetSimVarValue('GPS POSITION LON', 'degrees') || 0;
      const destLat = SimVar.GetSimVarValue('L:A32NX_EGPWC_DEST_LAT', 'number') || 0;
      const destLon = SimVar.GetSimVarValue('L:A32NX_EGPWC_DEST_LONG', 'number') || 0;
      const onGround = (SimVar.GetSimVarValue('PLANE ALT ABOVE GROUND MINUS CG', 'feet') || 0) < 5;
      const s = {
        pos: { lat, lon },
        dest: destLat || destLon ? { lat: destLat, lon: destLon } : null,
        gs: SimVar.GetSimVarValue('GPS GROUND SPEED', 'knots') || 0,
        alt: SimVar.GetSimVarValue('INDICATED ALTITUDE', 'feet') || 0,
        hdg: SimVar.GetSimVarValue('PLANE HEADING DEGREES TRUE', 'degrees') || 0,
        oat: SimVar.GetSimVarValue('AMBIENT TEMPERATURE', 'celsius'),
        onGround,
      };
      if (!this.origin || (this.wasOnGround && !onGround)) {
        this.origin = { lat, lon };
      }
      this.wasOnGround = onGround;
      return s;
    }

    Update() {
      super.Update();
      const rev = SimVar.GetSimVarValue('L:A380X_IFE_THEME', 'number') || 0;
      if (rev !== this.themeRev) {
        this.loadTheme(rev);
      }
      const now = Date.now();
      if (now - this.lastDraw < DRAW_EVERY_MS) {
        return;
      }
      this.lastDraw = now;
      const s = this.read();
      const page = PAGES[Math.floor((now - this.started) / 1000 / PAGE_S) % PAGES.length];
      const ctx = this.ctx;
      ctx.save();
      ctx.fillStyle = this.theme.background;
      ctx.fillRect(0, 0, W, H);
      if (page === 'welcome') this.drawWelcome(ctx);
      else if (page === 'map') this.drawMap(ctx, s);
      else this.drawInfo(ctx, s);
      ctx.restore();
    }

    drawImage(ctx, key, x, y, w, h, cover) {
      const img = this.images[key];
      if (!img || !img.complete || !img.naturalWidth) return false;
      let dw = w;
      let dh = h;
      const r = img.naturalWidth / img.naturalHeight;
      if (cover ? w / h < r : w / h > r) dw = h * r;
      else dh = w / r;
      ctx.drawImage(img, x + (w - dw) / 2, y + (h - dh) / 2, dw, dh);
      return true;
    }

    drawWelcome(ctx) {
      const t = this.theme;
      this.drawImage(ctx, 'ife_background', 0, 0, W, H, true);
      const hasLogo = this.drawImage(ctx, 'ife_logo', W / 2 - 110, 30, 220, 110, false);
      ctx.fillStyle = t.text;
      ctx.textAlign = 'center';
      ctx.font = 'bold 26px sans-serif';
      ctx.fillText(t.welcome, W / 2, hasLogo ? 180 : 120);
      if (t.airline) {
        ctx.fillStyle = t.accent;
        ctx.font = '18px sans-serif';
        ctx.fillText(t.airline, W / 2, hasLogo ? 210 : 155);
      }
    }

    drawMap(ctx, s) {
      const t = this.theme;
      const pts = [s.pos, this.origin, s.dest].filter(Boolean);
      const centreLon = s.pos.lon;
      const lons = pts.map((p) => wrapLon(p.lon, centreLon));
      const lats = pts.map((p) => p.lat);
      let span = Math.max(Math.max(...lons) - Math.min(...lons), (Math.max(...lats) - Math.min(...lats)) * 2, 12) * 1.3;
      span = Math.min(span, 360);
      const cx = (Math.max(...lons) + Math.min(...lons)) / 2;
      const cy = (Math.max(...lats) + Math.min(...lats)) / 2;
      const k = W / span;
      const px = (lon) => W / 2 + (wrapLon(lon, cx) - cx) * k;
      const py = (lat) => H / 2 - (lat - cy) * k;
      ctx.fillStyle = t.sea;
      ctx.fillRect(0, 0, W, H);
      ctx.fillStyle = t.land;
      for (const ring of this.land) {
        ctx.beginPath();
        let prev = null;
        for (let i = 0; i < ring.length; i += 2) {
          const x = px(ring[i]);
          const y = py(ring[i + 1]);
          if (prev === null || Math.abs(x - prev) > W / 2) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
          prev = x;
        }
        ctx.fill();
      }
      if (this.origin && s.dest) {
        ctx.strokeStyle = t.accent;
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.moveTo(px(this.origin.lon), py(this.origin.lat));
        ctx.lineTo(px(s.pos.lon), py(s.pos.lat));
        ctx.lineTo(px(s.dest.lon), py(s.dest.lat));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = t.accent;
        ctx.beginPath();
        ctx.arc(px(s.dest.lon), py(s.dest.lat), 4, 0, 2 * Math.PI);
        ctx.fill();
      }
      ctx.save();
      ctx.translate(px(s.pos.lon), py(s.pos.lat));
      ctx.rotate(s.hdg * RAD);
      ctx.fillStyle = t.text;
      ctx.beginPath();
      ctx.moveTo(0, -11);
      ctx.lineTo(3, -3);
      ctx.lineTo(11, 3);
      ctx.lineTo(3, 2);
      ctx.lineTo(2, 8);
      ctx.lineTo(5, 11);
      ctx.lineTo(-5, 11);
      ctx.lineTo(-2, 8);
      ctx.lineTo(-3, 2);
      ctx.lineTo(-11, 3);
      ctx.lineTo(-3, -3);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
      this.drawStrip(ctx, s);
    }

    drawStrip(ctx, s) {
      const t = this.theme;
      ctx.fillStyle = 'rgba(0,0,0,0.55)';
      ctx.fillRect(0, H - 30, W, 30);
      ctx.fillStyle = t.text;
      ctx.font = '14px sans-serif';
      ctx.textAlign = 'left';
      const togo = s.dest ? distanceNm(s.pos, s.dest) : null;
      const eta = togo !== null && s.gs > 50 ? fmtTime(togo / s.gs) : '--:--';
      ctx.fillText(`${Math.round(s.alt).toLocaleString()} ft`, 10, H - 10);
      ctx.fillText(`${Math.round(s.gs)} kt`, 140, H - 10);
      ctx.fillText(togo !== null ? `${Math.round(togo).toLocaleString()} nm to go` : '', 240, H - 10);
      ctx.textAlign = 'right';
      ctx.fillText(eta, W - 10, H - 10);
    }

    drawInfo(ctx, s) {
      const t = this.theme;
      const togo = s.dest ? distanceNm(s.pos, s.dest) : null;
      const rows = [
        ['Altitude', `${Math.round(s.alt).toLocaleString()} ft  /  ${Math.round(s.alt * 0.3048).toLocaleString()} m`],
        ['Ground speed', `${Math.round(s.gs)} kt  /  ${Math.round(s.gs * 1.852)} km/h`],
        ['Outside air', typeof s.oat === 'number' ? `${Math.round(s.oat)} °C` : '--'],
        ['Distance to go', togo !== null ? `${Math.round(togo).toLocaleString()} nm` : '--'],
        ['Time to destination', togo !== null && s.gs > 50 ? fmtTime(togo / s.gs) : '--:--'],
      ];
      ctx.fillStyle = t.accent;
      ctx.fillRect(0, 0, W, 34);
      ctx.fillStyle = t.background;
      ctx.font = 'bold 18px sans-serif';
      ctx.textAlign = 'left';
      ctx.fillText(t.airline ? `${t.airline}  ·  Flight information` : 'Flight information', 14, 23);
      ctx.font = '16px sans-serif';
      rows.forEach(([label, value], i) => {
        const y = 66 + i * 38;
        ctx.fillStyle = t.accent;
        ctx.textAlign = 'left';
        ctx.fillText(label, 20, y);
        ctx.fillStyle = t.text;
        ctx.textAlign = 'right';
        ctx.fillText(value, W - 20, y);
      });
    }
  }

  registerInstrument('a380x-ife', IFE);
})();

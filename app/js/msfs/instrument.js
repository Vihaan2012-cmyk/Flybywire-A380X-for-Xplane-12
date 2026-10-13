// Instruments and the VCockpit lifecycle, as MSFS runs them, ported from
// src/js/msfs/instrument.js for a real Chromium page.
//
// What stayed the same: `BaseInstrument`/`TemplateElement` themselves — they
// already only used `document`/`HTMLElement`/`customElements`, which are
// real here, so they port close to verbatim.
//
// What changed, and why: the QuickJS port's `VCockpitPanel`/`__vcockpit.load`
// loaded a whole panel.cfg section's `daInstruments` list (built on the
// plugin side from panel.cfg) into *one* shared document, importing each
// gauge's HTML fragment by fetching it. Agent G's browsers are one per view
// already (docs/briefs/xphfbw-js-bridge.md's view numbering: one browser per
// `[VCockpitNN]` section's `htmlgauge00`), navigated directly to that
// gauge's own HTML file with this runtime injected as its first <head>
// script — so the gauge's fragment (`<script type="text/html" id="...">`
// template, `import-script` entries, `<link rel="stylesheet">`) is already
// sitting in *this* document by the time the parser reaches it, not
// somewhere this script has to go fetch. There is also no external
// `daInstruments`/rect data reaching this page at all (the `__xphfbw`
// contract doesn't carry panel.cfg), so:
//  - `registerInstrument` no longer waits for an externally-supplied `#panel`
//    element with position data; it defines the element and creates a
//    single instance sized to fill the view (agent G already sizes the
//    browser itself to the gauge's own pixels), after the same ~1s delay
//    real MSFS uses (kept for fidelity with what the QuickJS shim already
//    verified).
//  - the bootstrap below (not part of real MSFS's JS surface, purely this
//    runtime's glue) scans this document once for `import-script` entries
//    and loads them in order through `Include.addScript`, replacing the old
//    `__vcockpit.load(data)` call the plugin used to make.
//  - native WASM gauge compositing (`composeNatives`/`nativeGauges`, our
//    custom "60 NATIVE_IMAGE" op-stream for terronnd/WXR images layered
//    into a view's screen) is dropped entirely: that was specific to our
//    own custom display-stream protocol over `submitDisplay`, which doesn't
//    exist here — a real Chromium page paints normally and agent G/D
//    capture the rendered pixels directly (CEF's on_paint). If FlyByWire's
//    native WASM gauges (terronnd, WXR) still need compositing on top of
//    this page's own rendering, that has to happen outside the browser
//    layer (D's screen upload) — flagged for agent H/D to confirm.
//  - `Guid`/`Url` are synthesized (`<view>_1`, `location.href`) rather than
//    coming from panel.cfg's `iGUId`/`sUrl`; `location.href` still carries
//    whatever query string (`?index=2`, etc.) agent G navigated this view
//    with, so `loadURLAttributes()` below (unchanged) still sees it.
(() => {
  const host = globalThis.__host;

  // Query parameters as URLSearchParams reads them.
  const searchParams = (url) => {
    const params = new Map();
    const q = url.indexOf('?');
    if (q < 0) {
      return params;
    }
    const hash = url.indexOf('#', q);
    const query = url.slice(q + 1, hash < 0 ? undefined : hash);
    for (const pair of query.split('&')) {
      if (pair === '') {
        continue;
      }
      const eq = pair.indexOf('=');
      const key = decodeURIComponent((eq < 0 ? pair : pair.slice(0, eq)).replace(/\+/g, ' '));
      const value = eq < 0 ? '' : decodeURIComponent(pair.slice(eq + 1).replace(/\+/g, ' '));
      if (!params.has(key)) {
        params.set(key, value);
      }
    }
    return params;
  };

  const diffAndSetAttribute = (element, name, value) => {
    if (element && element.getAttribute(name) !== String(value)) {
      element.setAttribute(name, value);
    }
  };
  globalThis.diffAndSetAttribute = diffAndSetAttribute;
  globalThis.diffAndSetStyle = (element, property, value) => {
    if (element && element.style[property] !== value) {
      element.style[property] = value;
    }
  };
  globalThis.diffAndSetText = (element, text) => {
    if (element && element.textContent !== String(text)) {
      element.textContent = text;
    }
  };
  globalThis.fastToFixed = (value, digits) => Number(value).toFixed(digits);

  class UIElement extends HTMLElement {
    connectedCallback() {}
    disconnectedCallback() {}
  }
  globalThis.UIElement = UIElement;

  const templateText = (id) => {
    const holder = document.getElementById(id);
    return holder === null ? null : holder.textContent;
  };

  class TemplateElement extends UIElement {
    constructor() {
      super();
      this.created = false;
    }
    get templateID() {
      return '';
    }
    Instanciate(forcedTemplate = '') {
      const id = forcedTemplate || this.templateID;
      if (!id) {
        return null;
      }
      const text = templateText(id);
      if (text === null) {
        console.error(`Template ${id} not found`);
        return null;
      }
      const holder = document.createElement('div');
      holder.innerHTML = text;
      const source = document.getElementById(id);
      for (const attribute of source.attributes) {
        if (attribute.name !== 'id' && attribute.name !== 'type' && !this.hasAttribute(attribute.name)) {
          this.setAttribute(attribute.name, attribute.value);
        }
      }
      // `holder.children` is a live HTMLCollection in a real DOM (unlike
      // our old DOM stand-in, where this `.slice()` apparently worked);
      // snapshot it first since `appendChild` below mutates it as we go.
      for (const child of Array.from(holder.children)) {
        this.appendChild(child);
      }
      return this;
    }
    connectedCallback() {
      if (this.created) {
        return;
      }
      this.Instanciate();
      this.setAttribute('created', 'true');
      this.created = true;
      super.connectedCallback();
      setTimeout(() => this.dispatchEvent(new Event('created')), 0);
    }
  }
  TemplateElement.call = (obj, fn, ...args) => {
    if (obj) {
      if (obj.hasAttribute('created')) {
        fn.call(fn, ...args);
      } else {
        obj.addEventListener('created', fn.bind(fn, ...args), { once: true });
      }
    }
  };
  TemplateElement.callNoBinding = (obj, callback) => {
    if (obj.hasAttribute('created')) {
      callback();
    } else {
      obj.addEventListener('created', callback, { once: true });
    }
  };
  globalThis.TemplateElement = TemplateElement;

  const ScreenState = { OFF: 0, INIT: 1, WAITING_VALIDATION: 2, ON: 3, REVERSIONARY: 4 };
  Object.keys(ScreenState).forEach((k) => (ScreenState[ScreenState[k]] = k));
  globalThis.ScreenState = ScreenState;

  class URLConfig {}
  globalThis.URLConfig = URLConfig;

  // panel.xml's <Electric> logic. XPHFBW does not currently pass panel.xml
  // text to this page (the `__xphfbw` contract carries no panel.cfg/xml
  // data), so `xmlConfig` is always empty here and every instrument falls
  // back to `CIRCUIT AVIONICS ON` for its electrical logic below — matching
  // what the QuickJS port already did whenever no panel.xml was supplied.
  const parseXml = (text) => {
    const root = { name: '#root', attrs: {}, children: [], text: '' };
    const stack = [root];
    const re = /<!--[\s\S]*?-->|<\?[\s\S]*?\?>|<\/([\w:.-]+)\s*>|<([\w:.-]+)((?:\s+[\w:.-]+\s*=\s*(?:"[^"]*"|'[^']*'))*)\s*(\/?)>|([^<]+)/g;
    let m;
    while ((m = re.exec(String(text))) !== null) {
      const top = stack[stack.length - 1];
      if (m[1] !== undefined) {
        if (stack.length > 1) stack.pop();
      } else if (m[2] !== undefined) {
        const node = { name: m[2], attrs: {}, children: [], text: '' };
        const attrRe = /([\w:.-]+)\s*=\s*(?:"([^"]*)"|'([^']*)')/g;
        let a;
        while ((a = attrRe.exec(m[3])) !== null) node.attrs[a[1]] = a[2] ?? a[3];
        top.children.push(node);
        if (m[4] !== '/') stack.push(node);
      } else if (m[5] !== undefined) {
        top.text += m[5];
      }
    }
    return root;
  };
  const childrenNamed = (node, name) => node.children.filter((c) => c.name.toLowerCase() === name.toLowerCase());
  const evaluate = (node) => {
    switch (node.name.toLowerCase()) {
      case 'simvar':
        return Number(SimVar.GetSimVarValue(node.attrs.name, node.attrs.unit || 'number')) || 0;
      case 'and':
        return node.children.every((n) => evaluate(n) != 0) ? 1 : 0;
      case 'or':
        return node.children.some((n) => evaluate(n) != 0) ? 1 : 0;
      case 'not':
        return node.children.length > 0 && evaluate(node.children[0]) != 0 ? 0 : 1;
      case 'constant':
        return Number(node.text.trim()) || 0;
      default:
        return node.children.length > 0 ? evaluate(node.children[0]) : 0;
    }
  };

  class BaseInstrument extends TemplateElement {
    constructor() {
      super();
      this.urlConfig = new URLConfig();
      this._frameCount = 0;
      this.electricityAvailable = false;
      this.initDuration = 0;
      this.hasBeenOff = false;
      this.isStarted = false;
      this.needValidationAfterInit = false;
      this.initAcknowledged = false;
      this.screenState = ScreenState.OFF;
      this.reversionaryMode = false;
      this._lastTime = 0;
      this._deltaTime = 0;
      this._frameLastTime = 0;
      this._frameDeltaTime = 0;
      this._isConnected = false;
      this._isInitialized = false;
      this._quality = Quality.high;
      this._gameState = GameState.ingame;
      this._alwaysUpdate = false;
      this._alwaysUpdateList = [];
      this._pendingCalls = [];
      this._pendingCallUId = 0;
    }
    get initialized() {
      return this._isInitialized;
    }
    get instrumentIdentifier() {
      return this._instrumentId;
    }
    get instrumentIndex() {
      return this.urlConfig.index != null ? this.urlConfig.index : 1;
    }
    get isInteractive() {
      return false;
    }
    get IsGlassCockpit() {
      return false;
    }
    get isPrimary() {
      return this.urlConfig.index == null || this.urlConfig.index == 1;
    }
    get deltaTime() {
      return this._deltaTime;
    }
    get frameCount() {
      return this._frameCount;
    }
    get flightPlanManager() {
      return null;
    }
    get instrumentAlias() {
      return null;
    }
    connectedCallback() {
      super.connectedCallback();
      this.electricity = this.getChildById('Electricity');
      this.highlightSvg = this.getChildById('highlight');
      this.loadDocumentAttributes();
      this.loadURLAttributes();
      this.loadXMLConfig();
      document.addEventListener('OnVCockpitPanelAttributesChanged', this.loadDocumentAttributes.bind(this));
      this.startTime = Date.now();
      if (this.getGameState() != GameState.mainmenu) {
        this.createMainLoop();
      }
    }
    disconnectedCallback() {
      super.disconnectedCallback();
      this._isConnected = false;
    }
    Init() {
      this._isInitialized = true;
      if (this.xmlConfig) {
        this.parseXMLConfig();
      }
    }
    setInstrumentIdentifier(identifier) {
      if (identifier && identifier != '' && identifier != this.instrumentIdentifier) {
        this._instrumentId = identifier;
        const guid = this.getAttribute('Guid');
        if (guid != undefined) {
          LaunchFlowEvent('ON_VCOCKPIT_INSTRUMENT_INITIALIZED', guid, this.instrumentIdentifier, this.isInteractive, this.IsGlassCockpit);
        }
      }
    }
    setConfigFile(file) {
      this._xmlConfigFile = file;
    }
    triggerEventToAllInstruments(event, ...args) {
      LaunchFlowEvent('ON_HTMLEVENT_TO_ALL_VIEWS', event, ...args);
    }
    getChildById(selector) {
      if (selector == '') {
        return null;
      }
      if (!selector.startsWith('#') && !selector.startsWith('.')) {
        selector = '#' + selector;
      }
      return this.querySelector(selector);
    }
    getChildrenById(selector) {
      if (selector == '') {
        return null;
      }
      if (!selector.startsWith('#') && !selector.startsWith('.')) {
        selector = '#' + selector;
      }
      return this.querySelectorAll(selector);
    }
    getChildrenByClassName(selector) {
      return this.getElementsByClassName(selector);
    }
    onInteractionEvent(_args) {}
    DecomposeEventFromPrefix(args) {
      let search = this.instrumentIdentifier + '_';
      if (args[0].startsWith(search)) {
        return args[0].slice(search.length);
      }
      search = this.instrumentAlias;
      if (search != null && search != '') {
        if (this.urlConfig.index) {
          search += '_' + this.urlConfig.index;
        }
        search += '_';
        if (args[0].startsWith(search)) {
          return args[0].slice(search.length);
        }
      }
      search = this.templateID + '_';
      if (args[0].startsWith(search)) {
        const evt = args[0].slice(search.length);
        const separator = evt.search('_');
        if (separator >= 0) {
          if (!isFinite(parseInt(evt.substring(0, separator)))) {
            return evt;
          }
        } else if (!isFinite(parseInt(evt))) {
          return evt;
        }
      }
      search = 'Generic_';
      if (args[0].startsWith(search)) {
        return args[0].slice(search.length);
      }
      return null;
    }
    onSoundEnd(_event) {}
    getQuality() {
      if (this._alwaysUpdate && this._quality != Quality.disabled) {
        return Quality.high;
      }
      return this._quality;
    }
    getGameState() {
      return this._gameState;
    }
    reboot() {
      console.log('Rebooting Instrument...');
      this.startTime = Date.now();
      this._frameCount = 0;
      this.hasBeenOff = false;
      this.isStarted = false;
      this.initAcknowledged = false;
      this.dispatchEvent(new Event('Reboot'));
    }
    onFlightStart() {
      console.log('Flight Starting...');
      SimVar.SetSimVarValue('L:HUD_AP_SELECTED_SPEED', 'Number', 0);
      SimVar.SetSimVarValue('L:HUD_AP_SELECTED_ALTITUDE', 'Number', 0);
      this.dispatchEvent(new Event('FlightStart'));
    }
    onQualityChanged(quality) {
      this._quality = quality;
    }
    onGameStateChanged(oldState, newState) {
      if (newState != GameState.mainmenu) {
        this.createMainLoop();
        if (oldState == GameState.loading && (newState == GameState.ingame || newState == GameState.briefing)) {
          this.reboot();
        } else if (oldState == GameState.briefing && newState == GameState.ingame) {
          this.onFlightStart();
        }
      } else {
        this.killMainLoop();
      }
      this._gameState = newState;
    }
    loadDocumentAttributes() {
      let attr;
      if (document.body.hasAttribute('quality')) {
        attr = document.body.getAttribute('quality');
      }
      if (attr != undefined) {
        const quality = Quality[attr];
        if (quality != undefined && this._quality != quality) {
          this.onQualityChanged(quality);
        }
      }
      attr = undefined;
      if (document.body.hasAttribute('gamestate')) {
        attr = document.body.getAttribute('gamestate');
      }
      if (attr != undefined) {
        const state = GameState[attr];
        if (state != undefined && this._gameState != state) {
          this.onGameStateChanged(this._gameState, state);
        }
      }
    }
    // What BaseInstrument reads from its <Instrument> entry.
    parseXMLConfig() {
      const config = this.instrumentXmlConfig;
      if (!config) {
        return;
      }
      const electric = childrenNamed(config, 'Electric');
      if (electric.length > 0) {
        const tree = electric[0];
        this.electricalLogic = { getValue: () => evaluate(tree) };
      }
      const always = childrenNamed(config, 'AlwaysUpdate');
      if (always.length > 0 && always[0].text.trim().toLowerCase() == 'true') {
        this._alwaysUpdate = true;
      }
      const skip = childrenNamed(config, 'SkipValidationAfterInit');
      if (skip.length > 0 && this.needValidationAfterInit) {
        this.needValidationAfterInit = skip[0].text.trim() != 'True';
      }
    }
    parseURLAttributes() {
      let instrumentID = this.templateID;
      if (this.urlConfig.index) {
        instrumentID += '_' + this.urlConfig.index;
      }
      this.setInstrumentIdentifier(instrumentID);
      if (this.urlConfig.style) {
        diffAndSetAttribute(this, 'instrumentstyle', this.urlConfig.style);
      }
    }
    beforeUpdate() {
      const now = Date.now();
      this._frameDeltaTime = now - this._frameLastTime;
      this._frameLastTime = now;
    }
    Update() {
      this.updateElectricity();
    }
    afterUpdate() {
      this._frameCount++;
      if (this._frameCount >= Number.MAX_SAFE_INTEGER) {
        this._frameCount = 0;
      }
    }
    doUpdate() {
      this.beforeUpdate();
      if (this.canUpdate()) {
        const now = Date.now();
        this._deltaTime = now - this._lastTime;
        this._lastTime = now;
        this.updatePendingCalls();
        this.Update();
      } else {
        this.updateAlwaysList();
      }
      this.afterUpdate();
    }
    canUpdate() {
      return this.getQuality() != Quality.disabled;
    }
    updateElectricity() {
      if (this.isElectricityAvailable()) {
        if (!this.isStarted) {
          this.onPowerOn();
        }
        if (this.isBootProcedureComplete()) {
          const state = this.reversionaryMode ? ScreenState.REVERSIONARY : ScreenState.ON;
          if (this.screenState != state) {
            this.screenState = state;
            if (this.electricity) {
              diffAndSetAttribute(this.electricity, 'state', this.reversionaryMode ? 'Backup' : 'on');
            }
            SimVar.SetSimVarValue('L:' + this.instrumentIdentifier + '_ScreenLuminosity', 'number', 1);
            SimVar.SetSimVarValue('L:' + this.instrumentIdentifier + '_State', 'number', this.reversionaryMode ? 3 : 2);
          }
        } else if (Date.now() - this.startTime > this.initDuration) {
          if (this.screenState != ScreenState.WAITING_VALIDATION) {
            this.screenState = ScreenState.WAITING_VALIDATION;
            if (this.electricity) {
              diffAndSetAttribute(this.electricity, 'state', 'initWaitingValidation');
            }
            SimVar.SetSimVarValue('L:' + this.instrumentIdentifier + '_ScreenLuminosity', 'number', 0.2);
            SimVar.SetSimVarValue('L:' + this.instrumentIdentifier + '_State', 'number', 1);
          }
        } else if (this.screenState != ScreenState.INIT) {
          this.screenState = ScreenState.INIT;
          if (this.electricity) {
            diffAndSetAttribute(this.electricity, 'state', 'init');
          }
          SimVar.SetSimVarValue('L:' + this.instrumentIdentifier + '_ScreenLuminosity', 'number', 0.2);
          SimVar.SetSimVarValue('L:' + this.instrumentIdentifier + '_State', 'number', 1);
        }
      } else {
        this.hasBeenOff = true;
        if (this.isStarted) {
          this.onShutDown();
        }
        if (this.screenState != ScreenState.OFF) {
          this.screenState = ScreenState.OFF;
          if (this.electricity) {
            diffAndSetAttribute(this.electricity, 'state', 'off');
          }
          SimVar.SetSimVarValue('L:' + this.instrumentIdentifier + '_ScreenLuminosity', 'number', 0);
          SimVar.SetSimVarValue('L:' + this.instrumentIdentifier + '_State', 'number', 0);
        }
      }
    }
    isElectricityAvailable() {
      if (this.electricalLogic) {
        return this.electricalLogic.getValue() != 0;
      }
      return SimVar.GetSimVarValue('CIRCUIT AVIONICS ON', 'Bool');
    }
    onShutDown() {
      console.log('System Turned Off');
      this.hasBeenOff = true;
      this.isStarted = false;
      this.initAcknowledged = false;
      this._alwaysUpdateList.length = 0;
      this._pendingCalls.length = 0;
    }
    onPowerOn() {
      console.log('System Turned ON');
      this.startTime = Date.now();
      this.isStarted = true;
    }
    isBootProcedureComplete() {
      if (!this.hasBeenOff) {
        return true;
      }
      return Date.now() - this.startTime > this.initDuration && (this.initAcknowledged || !this.needValidationAfterInit);
    }
    acknowledgeInit() {
      this.initAcknowledged = true;
    }
    isInReversionaryMode() {
      return this.reversionaryMode;
    }
    wasTurnedOff() {
      return this.hasBeenOff;
    }
    playInstrumentSound(soundId) {
      if (this.isElectricityAvailable() && this.getGameState() == GameState.ingame) {
        Coherent.call('PLAY_INSTRUMENT_SOUND', soundId);
        return true;
      }
      return false;
    }
    createMainLoop() {
      if (this._isConnected) {
        return;
      }
      this._lastTime = Date.now();
      this._isConnected = true;
      this._mainLoopFuncInstance = this.mainLoop.bind(this);
      requestAnimationFrame(this._mainLoopFuncInstance);
    }
    mainLoop() {
      if (!this._isConnected) {
        return;
      }
      try {
        if (BaseInstrument.allInstrumentsLoaded && SimVar.IsReady()) {
          if (!this._isInitialized) {
            this.Init();
          }
          this.doUpdate();
        }
      } catch (error) {
        console.error(this.instrumentIdentifier + ' : ' + error, error && error.stack);
      }
      requestAnimationFrame(this._mainLoopFuncInstance);
    }
    killMainLoop() {
      this._isConnected = false;
    }
    loadXMLConfig() {
      this.xmlConfig = parseXml(this._xmlConfigFile || '');
      const find = (node) => {
        for (const child of node.children) {
          if (child.name === 'Instrument') {
            const name = childrenNamed(child, 'Name')[0];
            if (name && name.text.trim() == this.instrumentIdentifier) {
              this.instrumentXmlConfig = child;
            }
          } else {
            find(child);
          }
        }
      };
      find(this.xmlConfig);
    }
    loadURLAttributes() {
      const url = String(this.getAttribute('Url') || '').toLowerCase();
      const params = searchParams(url);
      this.urlConfig.style = params.has('style') ? params.get('style') : null;
      this.urlConfig.index = params.has('index') ? parseInt(params.get('index')) : null;
      this.urlConfig.wasmModule = params.has('wasm_module') ? params.get('wasm_module') : null;
      this.urlConfig.wasmGauge = params.has('wasm_gauge') ? params.get('wasm_gauge') : null;
      this.parseURLAttributes();
    }
    getTimeSinceStart() {
      return Date.now() - this.startTime;
    }
    requestCall(func, timeout = 0) {
      const uid = ++this._pendingCallUId;
      this._pendingCalls.push({ func, timeout, uid });
      return uid;
    }
    removeCall(uid) {
      const i = this._pendingCalls.findIndex((c) => c.uid == uid);
      if (i >= 0) {
        this._pendingCalls.splice(i, 1);
      }
    }
    updatePendingCalls() {
      for (let i = 0; i < this._pendingCalls.length; ) {
        const call = this._pendingCalls[i];
        call.timeout -= this.deltaTime;
        if (call.timeout <= 0) {
          this._pendingCalls.splice(i, 1);
          call.func();
          continue;
        }
        i++;
      }
    }
    alwaysUpdate(element, value) {
      const i = this._alwaysUpdateList.indexOf(element);
      if (i >= 0) {
        if (!value) {
          this._alwaysUpdateList.splice(i, 1);
        }
      } else if (value) {
        this._alwaysUpdateList.push(element);
      }
    }
    updateAlwaysList() {
      for (const element of this._alwaysUpdateList) {
        element.onUpdate(this._frameDeltaTime);
      }
    }
  }
  BaseInstrument.allInstrumentsLoaded = false;
  BaseInstrument.useSvgImages = false;
  globalThis.BaseInstrument = BaseInstrument;

  // An HTML import (Include.addImport, for a gauge that pulls in *another*
  // page at runtime, rather than the one this browser was navigated to):
  // <script type="text/html" id> templates and <link> styles go into the
  // document head; import-script scripts run in order. Ported from the
  // QuickJS shim's `importPage`, which already used plain DOM APIs, so it
  // is unchanged beyond the name (this document's own fragment, already
  // parsed by the browser, is handled by `bootstrapGauge` below instead of
  // going through this function).
  const importFragment = (path) => {
    const html = host.readFile(path);
    const holder = document.createElement('div');
    holder.innerHTML = html;
    const scripts = [];
    const collect = (node) => {
      for (const child of Array.from(node.childNodes)) {
        if (child.nodeType !== 1) {
          continue;
        }
        const tag = child.localName.toLowerCase();
        if (tag === 'script' && child.hasAttribute('import-script')) {
          scripts.push(child.getAttribute('import-script'));
        } else if (tag === 'script' && child.hasAttribute('id')) {
          document.head.appendChild(child);
        } else if (tag === 'link' && String(child.getAttribute('rel')).toLowerCase() === 'stylesheet') {
          // A real `<link>` loads its own stylesheet once appended; no need
          // to read+inline the CSS by hand the way the QuickJS DOM stand-in
          // required.
          child.setAttribute('href', Include.absolutePath(path, child.getAttribute('href')));
          document.head.appendChild(child);
        } else if (tag === 'style') {
          document.head.appendChild(child);
        } else {
          collect(child);
        }
      }
    };
    collect(holder);
    for (const script of scripts) {
      Include.addScript(Include.absolutePath(path, script));
    }
  };
  globalThis.__vcockpit = { importPage: importFragment };

  // ------------------------------------------------------------------
  // The VCockpit bootstrap: one instrument per view (see file header).
  // ------------------------------------------------------------------
  let instrumentElement = null;

  const attachInstrument = (name, cls) => {
    if (instrumentElement) {
      return; // Real MSFS only hosts one instrument per htmlgauge entry too.
    }
    if (customElements.get(name) === undefined) {
      customElements.define(name, cls);
    }
    const element = document.createElement(name);
    element.setConfigFile('');
    diffAndSetAttribute(element, 'Guid', `${__xphfbw.view}_1`);
    diffAndSetAttribute(element, 'Url', location.href);
    // MSFS's VCockpit page holds each gauge in a <vcockpit-panel> with the
    // panel.cfg gauge path as its `url`; FlyByWire reads which display unit
    // it is from that (PFD.tsx getDisplayIndex: the last character).
    diffAndSetAttribute(element, 'url', `${location.pathname}${location.search}`);
    element.style.position = 'absolute';
    element.style.left = '0';
    element.style.top = '0';
    element.style.width = '100%';
    element.style.height = '100%';
    if (customElements.get('vcockpit-panel') === undefined) {
      customElements.define('vcockpit-panel', class extends HTMLElement {});
    }
    const panel = document.createElement('vcockpit-panel');
    panel.style.position = 'absolute';
    panel.style.inset = '0';
    panel.appendChild(element);
    document.body.appendChild(panel);
    instrumentElement = element;
    document.title = `${document.title} - ${element.instrumentIdentifier || name}`;
    // Same event the QuickJS port's `loadNextInstrument()` fired once every
    // daInstruments entry was in; there is only ever one here.
    Coherent.trigger('ALL_INSTRUMENTS_LOADED');
    host.loaded(true, element.instrumentIdentifier || name);
  };

  // registerInstrument: called by the gauge's own bundle
  // (e.g. pfd.js's `registerInstrument('a380x-pfd', PFDComponent)`), after
  // the same ~1s delay real MSFS's VCockpit.js uses before defining and
  // creating the custom element.
  globalThis.registerInstrument = (name, cls) => {
    setTimeout(() => attachInstrument(name, cls), 1000);
  };

  // Scan this document for the gauge fragment already sitting in it
  // (`<script type="text/html" id>` templates need no action —
  // `document.getElementById` already finds them; `<link rel=stylesheet>`
  // is already loading natively) and run its `import-script` entries in
  // order, exactly like the QuickJS port's `importPage` did for an
  // externally-fetched page, just without the fetch. Once `registerInstrument`
  // eventually creates the instrument (or bootstrapping fails outright),
  // report the result through `__xphfbw.loaded` (rule 7,
  // docs/briefs/xphfbw-js-bridge.md: the plugin waits for every screened
  // view to report `Loaded{ok:true}` before switching engines).
  const bootstrapGauge = () => {
    try {
      // Matches VCockpitPanel.setAttributes's effect in the QuickJS port:
      // XPHFBW's views are always inside a running flight, so this is
      // "ingame"/"high" from the start, not something that transitions.
      document.body.setAttribute('quality', 'high');
      document.body.setAttribute('gamestate', 'ingame');
      document.dispatchEvent(new Event('OnVCockpitPanelAttributesChanged'));

      const scripts = Array.from(document.querySelectorAll('script[type="text/html"][import-script]'));
      for (const el of scripts) {
        Include.addScript(el.getAttribute('import-script'));
      }
      // If nothing called registerInstrument, this gauge has nothing to
      // show; still leave `loaded` unreported rather than lying about it —
      // rule 7 exists precisely so a broken view never gets treated as
      // ready.
    } catch (e) {
      host.loaded(false, e && e.stack ? e.stack : String(e));
      throw e;
    }
  };
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', bootstrapGauge, { once: true });
  } else {
    bootstrapGauge();
  }

  // Events for this view's instrument (there is at most one now; `target`
  // is still checked against its Guid, matching real MSFS's addressing, for
  // the HTML_EVENT_TO/TO_ALL_SUBSCRIBERS case).
  const forInstrument = (target, fn) => {
    if (!instrumentElement || (target && instrumentElement.getAttribute('Guid') != target)) {
      return;
    }
    try {
      fn(instrumentElement);
    } catch (e) {
      console.error(e);
    }
  };
  Coherent.on('OnInteractionEvent', (target, args) => forInstrument(target, (i) => i.onInteractionEvent(args)));
  Coherent.on('OnSoundEnd', (target, eventId) => forInstrument(target, (i) => i.onSoundEnd(eventId)));
  Coherent.on('OnAllInstrumentsLoaded', () => {
    BaseInstrument.allInstrumentsLoaded = true;
  });
})();

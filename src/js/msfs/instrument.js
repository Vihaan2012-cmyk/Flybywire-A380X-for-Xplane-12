// Instruments and the VCockpit panel that hosts them, as MSFS runs them.
//
// A view is one panel.cfg [VCockpitNN] section. The plugin describes it to
// __vcockpit.load() and imports each gauge's HTML; the gauge's script calls
// registerInstrument(), and the panel then (a second later, as MSFS does)
// defines the element, creates it with its Guid and Url attributes and
// appends it, which connects it: TemplateElement instantiates the gauge's
// template, and BaseInstrument starts its main loop, which calls Update()
// every animation frame once every instrument in the panel is loaded.
// Interaction events (H: events, flow events) reach every instrument's
// onInteractionEvent.
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
      for (const child of holder.children.slice()) {
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

  // panel.xml. MSFS parses it with DOMParser and evaluates <Electric> with
  // XMLLogic.js; this reads the same elements with a small XML reader, into
  // { name, attrs, children, text } nodes, and evaluates the logic elements
  // panel.xml files use: Simvar, And, Or, Not, Constant.
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


  // The panel, as asobo-vcockpits-core VCockpit.js runs it: instruments are
  // imported one after the other; a gauge's script registers its element,
  // which is defined and created a second later, and then the next gauge
  // is imported.
  const INSTRUMENT_ROOT = '../Instruments/';

  class VCockpitPanel extends HTMLElement {
    constructor() {
      super();
      this.data = null;
      this.curInstrumentIndex = -1;
      this.curAttributes = null;
    }
    load(data) {
      this.data = data;
      this.curInstrumentIndex = -1;
      document.title = data.sName;
      this.setAttributes(data.daAttributes);
      this.loadNextInstrument();
    }
    hasData() {
      return this.data != null;
    }
    setAttributes(attributes) {
      if (this.curAttributes) {
        for (const a of this.curAttributes) {
          document.body.removeAttribute(a.name);
        }
      }
      this.curAttributes = attributes;
      for (const a of attributes) {
        diffAndSetAttribute(document.body, a.name, a.value);
        if (a.name == 'quality') {
          diffAndSetStyle(this, 'display', a.value == 'hidden' || a.value == 'disabled' ? 'none' : 'block');
        }
      }
      document.dispatchEvent(new Event('OnVCockpitPanelAttributesChanged'));
    }
    registerInstrument(name, cls) {
      customElements.define(name, cls);
      this.createInstrument(name, cls);
    }
    createInstrument(name, cls) {
      let element;
      try {
        element = document.createElement(name);
      } catch (e) {
        console.error(`Error while creating instrument ${name}: ${e}`, e && e.stack);
      }
      if (element) {
        try {
          this.setupInstrument(element);
        } catch (e) {
          console.error(`Error while setting up instrument ${name}: ${e}`, e && e.stack);
        }
        this.data.daInstruments[this.curInstrumentIndex].templateName = name;
        this.data.daInstruments[this.curInstrumentIndex].templateClass = cls;
        document.title += ' - ' + element.instrumentIdentifier;
      }
      this.loadNextInstrument();
    }
    loadNextInstrument() {
      this.curInstrumentIndex++;
      if (this.curInstrumentIndex < this.data.daInstruments.length) {
        const instrument = this.data.daInstruments[this.curInstrumentIndex];
        const index = this.urlAlreadyImported(instrument.sUrl);
        if (index >= 0) {
          this.createInstrument(this.data.daInstruments[index].templateName, this.data.daInstruments[index].templateClass);
        } else {
          try {
            Include.addImport(INSTRUMENT_ROOT + instrument.sUrl);
          } catch (e) {
            console.error(`Instrument ${instrument.sUrl} did not import: ${e}`, e && e.stack);
            this.loadNextInstrument();
          }
        }
      } else if (this.curInstrumentIndex === this.data.daInstruments.length) {
        host.trigger('ALL_INSTRUMENTS_LOADED', '[]');
      }
    }
    setupInstrument(element) {
      const instrument = this.data.daInstruments[this.curInstrumentIndex];
      const url = Include.absoluteURL(location.pathname, INSTRUMENT_ROOT + instrument.sUrl);
      diffAndSetAttribute(element, 'Guid', instrument.iGUId + '');
      diffAndSetAttribute(element, 'Url', url);
      // A headless instrument (systems-host, extras-host: no VCockpit
      // texture, so vLogicalSize is 0,0) would otherwise divide by zero and
      // position its child instrument element at NaN,NaN; 1 leaves it where
      // vPosAndSize already puts it, which is unused since nothing paints it.
      const ratioX = this.data.vLogicalSize.x ? this.data.vDisplaySize.x / this.data.vLogicalSize.x : 1;
      const ratioY = this.data.vLogicalSize.y ? this.data.vDisplaySize.y / this.data.vLogicalSize.y : 1;
      let w = Math.round(instrument.vPosAndSize.z * ratioX);
      let h = Math.round(instrument.vPosAndSize.w * ratioY);
      if (!(w > 0)) w = 10;
      if (!(h > 0)) h = 10;
      element.style.position = 'absolute';
      element.style.left = Math.round(instrument.vPosAndSize.x * ratioX) + 'px';
      element.style.top = Math.round(instrument.vPosAndSize.y * ratioY) + 'px';
      element.style.width = w + 'px';
      element.style.height = h + 'px';
      element.setConfigFile(this.data.sConfigFile);
      this.appendChild(element);
    }
    urlAlreadyImported(url) {
      const real = url.split('?')[0];
      for (let i = 0; i < this.curInstrumentIndex; i++) {
        if (this.data.daInstruments[i].sUrl.split('?')[0] === real) {
          return i;
        }
      }
      return -1;
    }
  }
  customElements.define('vcockpit-panel', VCockpitPanel);
  globalThis.VCockpitPanel = VCockpitPanel;

  globalThis.registerInstrument = (name, cls) => {
    const panel = document.getElementById('panel');
    if (panel) {
      setTimeout(() => panel.registerInstrument(name, cls), 1000);
    }
  };

  // An HTML import: <script type="text/html" id> templates and <link>
  // styles go into the document head; import-script scripts run in order.
  const importPage = (path) => {
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
          const href = Include.absolutePath(path, child.getAttribute('href'));
          let css = '';
          try {
            css = host.readFile(href);
          } catch (e) {
            console.warn(`${href}: ${e}`);
          }
          const style = document.createElement('style');
          style.setAttribute('data-href', href);
          style.textContent = css;
          document.head.appendChild(style);
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

  // Native gauges in this view ([id, over, x, y, w, h], from panel.cfg):
  // their images are part of the screen's stream, as 60 NATIVE_IMAGE ops
  // (docs/display-stream.md) before this document's ops, or after them for
  // a native gauge listed after an HTML gauge.
  let nativeGauges = [];
  const composeNatives = (data) => {
    if (nativeGauges.length === 0 || typeof host.submitDisplay !== 'function') {
      return;
    }
    const rx = data.vLogicalSize.x > 0 ? data.vDisplaySize.x / data.vLogicalSize.x : 1;
    const ry = data.vLogicalSize.y > 0 ? data.vDisplaySize.y / data.vLogicalSize.y : 1;
    const submit = host.submitDisplay;
    host.submitDisplay = (screen, ops, strings) => {
      const list = Array.from(strings);
      const op = (n) => {
        list.push(n[0]);
        return [60, list.length - 1, Math.round(n[2] * rx), Math.round(n[3] * ry), Math.round(n[4] * rx), Math.round(n[5] * ry)];
      };
      const under = nativeGauges.filter((n) => !n[1]).flatMap(op);
      const over = nativeGauges.filter((n) => n[1]).flatMap(op);
      const out = new Float64Array(under.length + ops.length + over.length);
      out.set(under, 0);
      out.set(ops, under.length);
      out.set(over, under.length + ops.length);
      return submit(screen, out, list);
    };
  };

  // What the plugin calls.
  globalThis.__vcockpit = {
    natives(list) {
      nativeGauges = list;
    },
    // The simulator's ShowVCockpitPanel.
    load(data) {
      composeNatives(data);
      let panel = document.getElementById('panel');
      if (!panel) {
        panel = document.createElement('vcockpit-panel');
        panel.id = 'panel';
        document.body.appendChild(panel);
      }
      panel.load(data);
    },
    importPage,
  };

  // Events for the panel's instruments.
  const forInstruments = (target, fn) => {
    const panel = document.getElementById('panel');
    if (closed || !panel) {
      return;
    }
    for (const instrument of Array.from(panel.children)) {
      if (target && instrument.getAttribute('Guid') != target) {
        continue;
      }
      try {
        fn(instrument);
      } catch (e) {
        console.error(e);
      }
    }
  };
  Coherent.on('RefreshVCockpitPanel', (data) => {
    const panel = document.getElementById('panel');
    if (panel) panel.setAttributes(data.daAttributes);
  });
  Coherent.on('OnInteractionEvent', (target, args) => forInstruments(target, (i) => i.onInteractionEvent(args)));
  Coherent.on('OnSoundEnd', (target, eventId) => forInstruments(target, (i) => i.onSoundEnd(eventId)));
  Coherent.on('OnAllInstrumentsLoaded', () => {
    if (!closed) {
      BaseInstrument.allInstrumentsLoaded = true;
    }
  });
})();

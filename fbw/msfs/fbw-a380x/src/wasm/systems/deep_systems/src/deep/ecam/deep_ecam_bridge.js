(function (global) {
  'use strict';

  function readVar(name, unit) {
    try {
      var v = global.SimVar.GetSimVarValue(name, unit);
      return typeof v === 'number' ? v : Number(v) || 0;
    } catch (e) {
      return 0;
    }
  }

  var CMP = {
    lt: function (a, b) { return a < b; },
    le: function (a, b) { return a <= b; },
    gt: function (a, b) { return a > b; },
    ge: function (a, b) { return a >= b; },
    eq: function (a, b) { return Math.abs(a - b) < 1e-9; },
    ne: function (a, b) { return Math.abs(a - b) >= 1e-9; },
  };

  function evalCond(c) {
    switch (c[0]) {
      case 'always':
        return true;
      case 'var':
        return CMP[c[3]](readVar(c[1], c[2]), c[4]);
      case 'varvar':
        return CMP[c[3]](readVar(c[1], c[2]), readVar(c[4], c[5]));
      case 'and':
        for (var i = 0; i < c[1].length; i++) {
          if (!evalCond(c[1][i])) return false;
        }
        return true;
      case 'or':
        for (var j = 0; j < c[1].length; j++) {
          if (evalCond(c[1][j])) return true;
        }
        return false;
      case 'not':
        return !evalCond(c[1][0]);
      default:
        return false;
    }
  }
  global.__deepEcamEvalCond = evalCond;

  function makeFlag(initial) {
    var v = !!initial;
    return {
      get: function () { return v; },
      set: function (x) { v = !!x; },
      sub: function () { /* never called: see file doc comment */ },
    };
  }

  function makeAlertRuntime(def) {
    var heldS = 0;
    var shownS = 0;
    var flag = makeFlag(false);
    return {
      item: {
        flightPhaseInhib: def.flightPhaseInhib,
        simVarIsActive: flag,
        notActiveWhenItemActive: def.notActiveWhenItemActive || [],
        monitorConfirmTime: 0,
        whichItemsToShow: function () {
          return def.items.map(function (it) {
            return evalCond(it.appliesIf) && shownS >= it.afterS;
          });
        },
        whichItemsChecked: function () {
          return def.items.map(function (it) {
            return it.doneWhen ? evalCond(it.doneWhen) : false;
          });
        },
        failure: def.failure,
        sysPage: def.sysPage,
        inopSysAllPhases: def.inopIds.length ? function () { return def.inopIds; } : undefined,
        info: def.infoIds.length ? function () { return def.infoIds; } : undefined,
      },
      step: function (dtS) {
        var triggered = evalCond(def.trigger);
        heldS = triggered ? heldS + Math.max(dtS, 0) : 0;
        var active = triggered && heldS >= def.confirmS;
        shownS = active ? shownS + Math.max(dtS, 0) : 0;
        flag.set(active);
      },
    };
  }

  global.installDeepEcamFbw = function (fws, defs, procs) {
    if (!fws || !fws.ewdAbnormal || !fws.allSuppressableItems || !procs) {
      return function () {};
    }
    var sensed = fws.abnormalSensed && fws.abnormalSensed.ewdAbnormalSensed;
    var runtimes = [];
    for (var d = 0; d < defs.length; d++) {
      var def = defs[d];
      var proc = procs[def.id];
      if (!proc || !proc.items) continue;
      if (fws.ewdAbnormal[def.id]) continue;
      runtimes.push(makeFbwRuntime(def, proc.items.length, fws, sensed));
    }
    var lastMs = Date.now();
    return function stepAllFbw() {
      var nowMs = Date.now();
      var dtS = Math.min(Math.max((nowMs - lastMs) / 1000, 0), 1);
      lastMs = nowMs;
      for (var i = 0; i < runtimes.length; i++) {
        runtimes[i].step(dtS);
      }
    };
  };

  function makeFbwRuntime(def, n, fws, sensed) {
    var heldS = 0;
    var flag = makeFlag(false);
    var show = new Array(n);
    var checked = new Array(n);
    for (var i = 0; i < n; i++) {
      show[i] = null;
      checked[i] = null;
    }
    for (var k = 0; k < def.items.length; k++) {
      var it = def.items[k];
      if (it.index < 0 || it.index >= n) continue;
      if (it.show) show[it.index] = it.show;
      if (it.checked) checked[it.index] = it.checked;
    }
    var item = {
      flightPhaseInhib: def.flightPhaseInhib,
      simVarIsActive: flag,
      notActiveWhenItemActive: def.notActiveWhenItemActive || [],
      monitorConfirmTime: 0,
      whichItemsToShow: function () {
        var out = new Array(n);
        for (var i = 0; i < n; i++) {
          out[i] = show[i] === null ? true : evalCond(show[i]);
        }
        return out;
      },
      whichItemsChecked: function () {
        var out = new Array(n);
        for (var i = 0; i < n; i++) {
          out[i] = checked[i] === null ? false : evalCond(checked[i]);
        }
        return out;
      },
      failure: def.failure,
      sysPage: def.sysPage,
    };
    if (sensed) {
      sensed[def.id] = item;
    }
    fws.ewdAbnormal[def.id] = item;
    fws.allSuppressableItems[def.id] = item;
    return {
      item: item,
      step: function (dtS) {
        var triggered = evalCond(def.trigger);
        heldS = triggered ? heldS + Math.max(dtS, 0) : 0;
        flag.set(triggered && heldS >= def.confirmS);
      },
    };
  }

  global.installDeepEcam = function (fws, defs) {
    if (!fws || !fws.ewdAbnormal || !fws.allSuppressableItems) {
      return function () {};
    }
    var sensed = fws.abnormalSensed && fws.abnormalSensed.ewdAbnormalSensed;
    var runtimes = defs.map(function (def) {
      var rt = makeAlertRuntime(def);
      if (sensed) {
        sensed[def.id] = rt.item;
      }
      fws.ewdAbnormal[def.id] = rt.item;
      fws.allSuppressableItems[def.id] = rt.item;
      return rt;
    });
    var lastMs = Date.now();
    return function stepAll() {
      var nowMs = Date.now();
      var dtS = Math.min(Math.max((nowMs - lastMs) / 1000, 0), 1);
      lastMs = nowMs;
      for (var i = 0; i < runtimes.length; i++) {
        runtimes[i].step(dtS);
      }
    };
  };
})(globalThis);

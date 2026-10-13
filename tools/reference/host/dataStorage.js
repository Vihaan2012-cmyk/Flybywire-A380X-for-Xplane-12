// MSFS's persistent key/value store (served at /JS/dataStorage.js, which
// gauge html imports). Values come from the state file's "storage" section.
// A missing key returns '' as on MSFS 2020.
(function () {
  'use strict';
  if (window.GetStoredData) return;
  const cfg = (window.__REFERENCE__ && window.__REFERENCE__.state && window.__REFERENCE__.state.storage) || {};
  const data = new Map(Object.entries(cfg).map(([k, v]) => [k, String(v)]));
  const log = window.__refLog;
  const bump = (map, key, found) => {
    if (!log) return;
    let e = map.get(key);
    if (!e) map.set(key, (e = { key, count: 0, found }));
    e.count++;
  };
  window.GetDataStorage = () => ({
    getData: (k) => window.GetStoredData(k),
    setData: (k, v) => window.SetStoredData(k, v),
    searchData: (k) => window.SearchStoredData(k),
    deleteData: (k) => window.DeleteStoredData(k),
  });
  window.GetStoredData = (key) => {
    bump(log && log.storage, key, data.has(key));
    return data.has(key) ? data.get(key) : '';
  };
  window.SetStoredData = (key, value) => {
    bump(log && log.storageWrites, key, true);
    data.set(key, String(value));
    return String(value);
  };
  window.SearchStoredData = (key) => [...data.entries()].filter(([k]) => k.startsWith(key)).map(([k, v]) => ({ key: k, data: v }));
  window.DeleteStoredData = (key) => data.delete(key);
  window.OnDataStorageReady = () => {};
})();

export default {
  window: {
    minimize: 'window/minimize',
    maximize: 'window/maximize',
    close: 'window/close',
    isMaximized: 'window/isMaximized',
    reload: 'window/reload',
  },
  update: {
    error: 'update/error',
    available: 'update/available',
    downloaded: 'update/downloaded',
  },
  checkForInstallerUpdate: 'checkForInstallerUpdate',
  installManager: {
    fragmenterEvent: 'installManager/fragmenterEvent',
    installFromUrl: 'installManager/installFromUrl',
    cancelInstall: 'installManager/cancelInstall',
    uninstall: 'installManager/uninstall',
  },
  xplaneEngine: {
    query: 'xplaneEngine/query',
    install: 'xplaneEngine/install',
    event: 'xplaneEngine/event',
    cancel: 'xplaneEngine/cancel',
  },
  sentry: {
    requestSessionID: 'sentry/requestSessionID',
    provideSessionID: 'sentry/provideSessionID',
  },
  openPath: 'openPath',
};

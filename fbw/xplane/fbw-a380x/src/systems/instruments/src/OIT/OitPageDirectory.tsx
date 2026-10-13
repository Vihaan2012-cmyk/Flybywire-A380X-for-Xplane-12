//  Copyright (c) 2025 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { EventBus, FSComponent, VNode } from '@microsoft/msfs-sdk';
import { OitFltOpsEfbOverlay } from './Pages/FltOps/OitFltOpsEfbOverlay';
import { OitFltOpsMenuPage } from './Pages/FltOps/OitFltOpsMenuPage';
import { OitFltOpsTakeoffPerformance } from './Pages/FltOps/OitFltOpsTakeoffPerformance';
import { OitFltOpsLandingPerformance } from './Pages/FltOps/OitFltOpsLandingPerformance';
import { OitFltOpsStatus } from './Pages/FltOps/OitFltOpsStatus';
import { OitFltOpsAoc } from './Pages/FltOps/OitFltOpsAoc';
import { OitFltOpsLoadsheet } from './Pages/FltOps/OitFltOpsLoadsheet';
import { OitNotFound } from './Pages/OitNotFound';
import { OitUiService } from './OitUiService';
import { OitFltOpsContainer } from './OitFltOpsContainer';
import { OitAvncsContainer } from './OitAvncsContainer';
import { OitAvncsCompanyCom } from './Pages/NssAvncs/CompanyCom/OitAvncsCompanyCom';
import { OitAvncsMenu } from './Pages/NssAvncs/OitAvncsMenu';
import { OitAvncsCompanyComFlightLog } from './Pages/NssAvncs/CompanyCom/OitAvncsCompanyComFlightLog';
import { OitAvncsCompanyComInbox } from './Pages/NssAvncs/CompanyCom/OitAvncsCompanyComInbox';
import { OitAvncsFbwSystems } from './Pages/NssAvncs/FbwSystems/OitAvncsFbwSystems';
import { OitAvncsFbwSystemsGenericDebug } from './Pages/NssAvncs/FbwSystems/OitAvncsFbwSystemsGenericDebug';
import { OitAvncsFbwSystemsAppLdgCap } from './Pages/NssAvncs/FbwSystems/OitAvncsFbwSystemsAppLdgCap';
import { OitAvncsMaintenance } from './Pages/NssAvncs/Maintenance/OitAvncsMaintenance';
import { OitAvncsMaintenanceReport } from './Pages/NssAvncs/Maintenance/OitAvncsMaintenanceReport';
import { OitAvncsMaintenanceSystemStatus } from './Pages/NssAvncs/Maintenance/OitAvncsMaintenanceSystemStatus';
import { OitAvncsMaintenanceLogbook } from './Pages/NssAvncs/Maintenance/OitAvncsMaintenanceLogbook';
import { OitAvncsMaintenanceRemoteCb } from './Pages/NssAvncs/Maintenance/OitAvncsMaintenanceRemoteCb';

// Page imports
// eslint-disable-next-line jsdoc/require-jsdoc
export function fltOpsPageForUrl(
  url: string,
  bus: EventBus,
  uiService: OitUiService,
  container: OitFltOpsContainer,
): VNode {
  switch (url) {
    case 'flt-ops':
      return <OitFltOpsMenuPage bus={bus} uiService={uiService} container={container} />;
    case 'flt-ops/sts':
      return <OitFltOpsStatus bus={bus} uiService={uiService} container={container} />;
    case 'flt-ops/to-perf':
      return <OitFltOpsTakeoffPerformance bus={bus} uiService={uiService} container={container} />;
    case 'flt-ops/ldg-perf':
      return <OitFltOpsLandingPerformance bus={bus} uiService={uiService} container={container} />;
    case 'flt-ops/charts':
      return <OitFltOpsEfbOverlay bus={bus} uiService={uiService} container={container} />;
    case 'flt-ops/flt-folder':
      return <OitFltOpsEfbOverlay bus={bus} uiService={uiService} container={container} />;
    case 'flt-ops/aoc':
      return <OitFltOpsAoc bus={bus} uiService={uiService} container={container} />;
    case 'flt-ops/loadsheet':
      return <OitFltOpsLoadsheet bus={bus} uiService={uiService} container={container} />;

    default:
      return <OitNotFound uiService={uiService} />;
  }
}

// eslint-disable-next-line jsdoc/require-jsdoc
export function avncsPageForUrl(
  url: string,
  bus: EventBus,
  uiService: OitUiService,
  container: OitAvncsContainer,
): VNode {
  switch (url) {
    case 'nss-avncs':
      return <OitAvncsMenu bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/company-com':
      return <OitAvncsCompanyCom bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/a380x-systems':
      return <OitAvncsFbwSystems bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/maintenance':
      return <OitAvncsMaintenance bus={bus} uiService={uiService} container={container} />;

    default:
      return <OitNotFound uiService={uiService} />;
  }
}

// eslint-disable-next-line jsdoc/require-jsdoc
export function avncsMaintenancePageForUrl(
  url: string,
  bus: EventBus,
  uiService: OitUiService,
  container: OitAvncsContainer,
): VNode {
  const ataMatch = url.match(/^nss-avncs\/maintenance\/system-status\/(\d+)$/);
  if (ataMatch) {
    return (
      <OitAvncsMaintenanceReport
        bus={bus}
        uiService={uiService}
        container={container}
        mode="current"
        ata={Number(ataMatch[1])}
      />
    );
  }
  switch (url) {
    case 'nss-avncs/maintenance':
    case 'nss-avncs/maintenance/current-flight-report':
      return <OitAvncsMaintenanceReport bus={bus} uiService={uiService} container={container} mode="current" />;
    case 'nss-avncs/maintenance/post-flight-report':
      return <OitAvncsMaintenanceReport bus={bus} uiService={uiService} container={container} mode="post" />;
    case 'nss-avncs/maintenance/system-status':
      return <OitAvncsMaintenanceSystemStatus bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/maintenance/logbook':
      return <OitAvncsMaintenanceLogbook bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/maintenance/remote-cb':
      return <OitAvncsMaintenanceRemoteCb bus={bus} uiService={uiService} container={container} />;

    default:
      return <OitNotFound uiService={uiService} />;
  }
}

// eslint-disable-next-line jsdoc/require-jsdoc
export function avncsCompanyComPageForUrl(
  url: string,
  bus: EventBus,
  uiService: OitUiService,
  container: OitAvncsContainer,
): VNode {
  switch (url) {
    case 'nss-avncs/company-com/inbox':
      return <OitAvncsCompanyComInbox bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/company-com/pre-flight/flight-log':
      return <OitAvncsCompanyComFlightLog bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/company-com/in-flight/flight-log':
      return <OitAvncsCompanyComFlightLog bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/company-com/post-flight/flight-log':
      return <OitAvncsCompanyComFlightLog bus={bus} uiService={uiService} container={container} />;

    default:
      return <OitNotFound uiService={uiService} />;
  }
}

// eslint-disable-next-line jsdoc/require-jsdoc
export function avncsFbwSystemsPageForUrl(
  url: string,
  bus: EventBus,
  uiService: OitUiService,
  container: OitAvncsContainer,
): VNode {
  switch (url) {
    case 'nss-avncs/a380x-systems':
      return <OitAvncsFbwSystems bus={bus} uiService={uiService} container={container} />;
    case 'nss-avncs/a380x-systems/debug-data':
      return (
        <OitAvncsFbwSystemsGenericDebug
          bus={bus}
          uiService={uiService}
          container={container}
          title={'Flight Warning System Debug'}
          controlEventName="a380x_ois_fws_debug_data_enabled"
          dataEventName="a380x_ois_fws_debug_data"
        />
      );
    case 'nss-avncs/a380x-systems/app-ldg-cap':
      return (
        <OitAvncsFbwSystemsAppLdgCap
          bus={bus}
          uiService={uiService}
          container={container}
          title={'Approach & Landing Capability'}
        />
      );

    default:
      return <OitNotFound uiService={uiService} />;
  }
}

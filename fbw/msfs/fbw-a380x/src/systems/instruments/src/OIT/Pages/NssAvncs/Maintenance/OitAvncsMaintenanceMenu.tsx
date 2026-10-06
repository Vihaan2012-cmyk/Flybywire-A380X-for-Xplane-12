//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { EventBus, FSComponent, VNode } from '@microsoft/msfs-sdk';
import { DestroyableComponent } from '../../../../MsfsAvionicsCommon/DestroyableComponent';
import { OitFile, OitFolder } from '../OitAvncsFolderNavigator';
import { OitUiService } from '../../../OitUiService';

interface OitAvncsMaintenanceMenuProps {
  readonly bus: EventBus;
  readonly uiService: OitUiService;
}

export class OitAvncsMaintenanceMenu extends DestroyableComponent<OitAvncsMaintenanceMenuProps> {
  render(): VNode {
    return (
      <>
        <OitFolder name={'Reports'} initExpanded={true} hideFolderOpener={true}>
          <OitFile
            name={'Current Flight Report'}
            uiService={this.props.uiService}
            navigationTarget="nss-avncs/maintenance/current-flight-report"
          />
          <OitFile
            name={'Post Flight Report'}
            uiService={this.props.uiService}
            navigationTarget="nss-avncs/maintenance/post-flight-report"
          />
        </OitFolder>
        <OitFolder name={'System Status'} initExpanded={true} hideFolderOpener={true}>
          <OitFile
            name={'By ATA Chapter'}
            uiService={this.props.uiService}
            navigationTarget="nss-avncs/maintenance/system-status"
          />
        </OitFolder>
        <OitFolder name={'Circuit Breakers'} initExpanded={true} hideFolderOpener={true}>
          <OitFile
            name={'Remote C/B Control'}
            uiService={this.props.uiService}
            navigationTarget="nss-avncs/maintenance/remote-cb"
          />
        </OitFolder>
        <OitFolder name={'Logbook'} initExpanded={true} hideFolderOpener={true}>
          <OitFile
            name={'MEL Deferred Items'}
            uiService={this.props.uiService}
            navigationTarget="nss-avncs/maintenance/logbook"
          />
        </OitFolder>
      </>
    );
  }
}

//  Copyright (c) 2026 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { DisplayComponent, FSComponent, VNode } from '@microsoft/msfs-sdk';
import { DestroyableComponent } from '../../../../MsfsAvionicsCommon/DestroyableComponent';
import { AbstractOitAvncsPageProps } from '../../../OIT';
import { OitAvncsSubHeader } from '../OitAvncsSubHeader';
import { OitUriInformation } from '../../../OitUiService';
import { avncsMaintenancePageForUrl } from '../../../OitPageDirectory';
import { OitAvncsMaintenanceMenu } from './OitAvncsMaintenanceMenu';
import { MaintenanceData } from '../../../System/MaintenanceData';

interface OitAvncsMaintenanceProps extends AbstractOitAvncsPageProps {}

export class OitAvncsMaintenance extends DestroyableComponent<OitAvncsMaintenanceProps> {
  private readonly activePageRef = FSComponent.createRef<HTMLDivElement>();

  private activePage: VNode = (<></>) as VNode;

  private activeUriChanged(uri: OitUriInformation) {
    if (!uri.uri.match(/nss-avncs\/maintenance(\/\S*)?$/gm)) {
      return;
    }

    if (this.activePageRef.getOrDefault()) {
      while (this.activePageRef.instance.firstChild) {
        this.activePageRef.instance.removeChild(this.activePageRef.instance.firstChild);
      }
    }
    if (this.activePage && this.activePage.instance instanceof DisplayComponent) {
      this.activePage.instance.destroy();
    }

    const url = uri.extra ? `${uri.sys}/${uri.page}/${uri.extra}` : `${uri.sys}/${uri.page}`;
    this.activePage = avncsMaintenancePageForUrl(url, this.props.bus, this.props.uiService, this.props.container);
    FSComponent.render(this.activePage, this.activePageRef?.getOrDefault());
  }

  public onAfterRender(node: VNode): void {
    super.onAfterRender(node);

    MaintenanceData.instance().acquire();
    this.subscriptions.push(
      this.props.uiService.activeUri.sub((uri) => {
        this.activeUriChanged(uri);
      }, true),
    );
  }

  destroy(): void {
    MaintenanceData.instance().release();
    if (this.activePage && this.activePage.instance instanceof DisplayComponent) {
      this.activePage.instance.destroy();
    }
    super.destroy();
  }

  render(): VNode {
    return (
      <>
        <OitAvncsSubHeader title={'MAINTENANCE'} uiService={this.props.uiService} />
        <div class="oit-page-container">
          <div class="oit-avncs-navigator-container">
            <div class="oit-avncs-navigator-left">
              <OitAvncsMaintenanceMenu bus={this.props.bus} uiService={this.props.uiService} />
            </div>
            <div class="oit-avncs-navigator-right oit-centered" ref={this.activePageRef} />
          </div>
        </div>
      </>
    );
  }
}

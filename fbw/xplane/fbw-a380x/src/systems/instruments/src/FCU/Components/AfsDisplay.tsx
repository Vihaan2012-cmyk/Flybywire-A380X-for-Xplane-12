import {
  ComponentProps,
  ConsumerSubject,
  DisplayComponent,
  EventBus,
  FSComponent,
  MappedSubject,
  VNode,
} from '@microsoft/msfs-sdk';
import { HdgDisplay } from './HdgDisplay';
import { SpdDisplay } from './SpdDisplay';
import { VsDisplay } from './VsDisplay';
import { AltDisplay } from './AltDisplay';
import { FcuSimvars } from '../Publishers/FcuSimvarPublisher';

interface AfsDisplayProps extends ComponentProps {
  bus: EventBus;
}

export class AfsDisplay extends DisplayComponent<AfsDisplayProps> {
  private sub = this.props.bus.getSubscriber<FcuSimvars>();

  private afsCpActive = MappedSubject.create(
    ([active, backup]) => active && !backup,
    ConsumerSubject.create(this.sub.on('afsDisplayActive'), false),
    ConsumerSubject.create(this.sub.on('afsBackupActive'), false),
  );

  public render(): VNode {
    return (
      <div id="LargeScreen" style={{ visibility: this.afsCpActive.map((active) => (active ? 'inherit' : 'hidden')) }}>
        <SpdDisplay bus={this.props.bus} />
        <HdgDisplay bus={this.props.bus} />

        <div id="AltVS">
          <AltDisplay bus={this.props.bus} />
          <VsDisplay bus={this.props.bus} />
        </div>
      </div>
    );
  }
}

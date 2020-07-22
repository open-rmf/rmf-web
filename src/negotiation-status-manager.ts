import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export default class NegotiationStatusManager extends EventEmitter<Events> {
  negotiationStatus(): RomiCore.NegotiationStatus | undefined {
    return this._negotiationStatus;
  }

  startSubscription(transport: RomiCore.Transport) {
    transport.subscribe(RomiCore.negotiationStatus, negotiationStatus => {
      this._negotiationStatus = negotiationStatus;
      this.emit('updated');
    });
  }

  private _negotiationStatus ?: RomiCore.NegotiationStatus;
}
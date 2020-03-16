import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
}

export default class FleetManager extends EventEmitter<Events> {
  fleets(): RomiCore.FleetState[] {
    return Array.from(Object.values(this._fleets));
  }

  startSubscription(transport: RomiCore.Transport) {
    transport.subscribe(RomiCore.fleetStates, fleetState => {
      this._fleets[fleetState.name] = fleetState;
      this.emit('updated');
    });
  }

  private _fleets: Record<string, RomiCore.FleetState> = {};
}

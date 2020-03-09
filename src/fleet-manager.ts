import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  data: [];
}

export default class FleetManager extends EventEmitter<Events> {
  fleets(): readonly RomiCore.FleetState[] {
    return Array.from(Object.values(this._fleets));
  }

  startSubscription(transport: RomiCore.Transport) {
    transport.subscribe(RomiCore.fleetStates, fleetState => {
      this._fleets[fleetState.name] = fleetState;
      this.emit('data');
    });
  }

  private _fleets: Record<string, RomiCore.FleetState> = {};
}

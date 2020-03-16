import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export default class DoorStateManager extends EventEmitter<Events> {
  doorStates(): Record<string, RomiCore.DoorState> {
    return { ...this._doorStates };
  }

  startSubscription(transport: RomiCore.Transport) {
    transport.subscribe(RomiCore.doorStates, doorState => {
      this._doorStates[doorState.door_name] = doorState;
      this.emit('updated');
    });
  }

  private _doorStates: Record<string, RomiCore.DoorState> = {};
}

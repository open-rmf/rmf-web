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
    this._subscriptions.push(
      transport.subscribe(RomiCore.doorStates, doorState => {
        this._doorStates[doorState.door_name] = doorState;
        this.emit('updated');
      }),
    );
  }

  stopAllSubscriptions(): void {
    this._subscriptions.forEach(sub => sub.unsubscribe());
  }

  private _doorStates: Record<string, RomiCore.DoorState> = {};
  private _subscriptions: RomiCore.Subscription[] = [];
}

import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export default class LiftStateManager extends EventEmitter<Events> {
  liftStates(): Record<string, RomiCore.LiftState> {
    return { ...this._liftStates };
  }

  startSubscription(transport: RomiCore.Transport) {
    // TODO remove the skip validation when the type problem of available_floors is solved
    transport.subscribe(RomiCore.skipValidation(RomiCore.liftStates), liftState => {
      this._liftStates[liftState.lift_name] = liftState;
      this.emit('updated');
    });
  }

  private _liftStates: Record<string, RomiCore.LiftState> = {};
}

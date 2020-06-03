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

  static liftModeToString(liftMode: number): string {
    switch (liftMode) {
      case RomiCore.LiftState.MODE_AGV:
        return 'AGV';
      case RomiCore.LiftState.MODE_EMERGENCY:
        return 'Emergency';
      case RomiCore.LiftState.MODE_FIRE:
        return 'Fire';
      case RomiCore.LiftState.MODE_HUMAN:
        return 'Human';
      case RomiCore.LiftState.MODE_OFFLINE:
        return 'Offline';
      default:
        return `Unknown (${liftMode})`;
    }
  }

  static doorStateToString(doorState: number): string {
    switch (doorState) {
      case RomiCore.LiftState.DOOR_OPEN:
        return 'Open';
      case RomiCore.LiftState.DOOR_CLOSED:
        return 'Closed';
      case RomiCore.LiftState.DOOR_MOVING:
        return 'Moving';
      default:
        return `Unknown (${doorState})`;
    }
  }

  static motionStateToString(motionState: number): string {
    switch (motionState) {
      case RomiCore.LiftState.MOTION_DOWN:
        return 'Down';
      case RomiCore.LiftState.MOTION_STOPPED:
        return 'Stopped';
      case RomiCore.LiftState.MOTION_UP:
        return 'Up';
      default:
        return `Unknown (${motionState})`;
    }
  }

  private _liftStates: Record<string, RomiCore.LiftState> = {};
}

export class LiftRequestManager {

  static getLiftRequestModes() {
    const liftModes = [
      RomiCore.LiftRequest.REQUEST_END_SESSION,
      RomiCore.LiftRequest.REQUEST_AGV_MODE,
      RomiCore.LiftRequest.REQUEST_HUMAN_MODE,
    ]
    let listOfModes: Array<any> = [];
    liftModes.forEach(element => {
      const key = this.liftModeToString(element);
      const value = element;
      listOfModes.push({ key, value })
    });
    return listOfModes
  }

  static getAllDoorModes() {
    const doorModes = [
      RomiCore.LiftRequest.DOOR_CLOSED,
      RomiCore.LiftRequest.DOOR_OPEN,
    ]
    let listOfModes: Array<any> = [];
    doorModes.forEach(element => {
      const key = this.doorStateToString(element);
      const value = element;
      listOfModes.push({ key, value })
    });
    return listOfModes
  }

  static liftModeToString(liftMode: number) {
    switch (liftMode) {
      case RomiCore.LiftRequest.REQUEST_END_SESSION:
        return 'End Session';
      case RomiCore.LiftRequest.REQUEST_AGV_MODE:
        return 'AVG';
      case RomiCore.LiftRequest.REQUEST_HUMAN_MODE:
        return 'Human';
      default:
        return `Unknown (${liftMode})`;
    }
  }

  static doorStateToString(doorState: number) {
    switch (doorState) {
      case RomiCore.LiftState.DOOR_OPEN:
        return 'Open';
      case RomiCore.LiftState.DOOR_CLOSED:
        return 'Closed';
      default:
        return `Unknown (${doorState})`;
    }
  }
}


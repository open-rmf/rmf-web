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

  static doorTypeToString(doorType: number): string {
    switch (doorType) {
      case RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING:
        return 'Double Sliding';
      case RomiCore.Door.DOOR_TYPE_DOUBLE_SWING:
        return 'Double Swing';
      case RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE:
        return 'Double Telescope';
      case RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING:
        return 'Sliding';
      case RomiCore.Door.DOOR_TYPE_SINGLE_TELESCOPE:
        return 'Telescope';
      default:
        return `Unknown (${doorType})`;
    }
  }

  static doorModeToString(doorState?: RomiCore.DoorState): string {
    if (!doorState) {
      return 'UNKNOWN';
    }
    switch (doorState.current_mode.value) {
      case RomiCore.DoorMode.MODE_OPEN:
        return 'OPEN';
      case RomiCore.DoorMode.MODE_CLOSED:
        return 'CLOSED';
      case RomiCore.DoorMode.MODE_MOVING:
        return 'MOVING';
      default:
        return 'UNKNOWN';
    }
  }

  static motionDirectionToString(motionDirection: number): string {
    switch (motionDirection) {
      case 1:
        return 'Clockwise';
      case -1:
        return 'Anti-Clockwise';
      default:
        return `Unknown (${motionDirection})`;
    }
  }

  private _doorStates: Record<string, RomiCore.DoorState> = {};
}

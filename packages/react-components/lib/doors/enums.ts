import * as RomiCore from '@osrf/romi-js-core-interfaces';

export enum DoorType {
  SingleSwing = RomiCore.Door.DOOR_TYPE_SINGLE_SWING,
  SingleSliding = RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
  SingleTelescope = RomiCore.Door.DOOR_TYPE_SINGLE_TELESCOPE,
  DoubleSwing = RomiCore.Door.DOOR_TYPE_DOUBLE_SWING,
  DoubleSliding = RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING,
  DoubleTelescope = RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
}

export enum DoorMotion {
  Clockwise = 1,
  AntiClockwise = -1,
}

export enum DoorMode {
  Open = RomiCore.DoorMode.MODE_OPEN,
  Closed = RomiCore.DoorMode.MODE_CLOSED,
  Moving = RomiCore.DoorMode.MODE_MOVING,
}

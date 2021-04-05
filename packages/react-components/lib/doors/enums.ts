import * as RmfModels from 'rmf-models';

export enum DoorType {
  SingleSwing = RmfModels.Door.DOOR_TYPE_SINGLE_SWING,
  SingleSliding = RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING,
  SingleTelescope = RmfModels.Door.DOOR_TYPE_SINGLE_TELESCOPE,
  DoubleSwing = RmfModels.Door.DOOR_TYPE_DOUBLE_SWING,
  DoubleSliding = RmfModels.Door.DOOR_TYPE_DOUBLE_SLIDING,
  DoubleTelescope = RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
}

export enum DoorMotion {
  Clockwise = 1,
  AntiClockwise = -1,
}

export enum DoorMode {
  Open = RmfModels.DoorMode.MODE_OPEN,
  Closed = RmfModels.DoorMode.MODE_CLOSED,
  Moving = RmfModels.DoorMode.MODE_MOVING,
}

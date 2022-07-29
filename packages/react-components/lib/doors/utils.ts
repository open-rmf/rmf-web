import { Door as RmfDoor, DoorMode as RmfDoorMode } from 'rmf-models';

export enum DoorType {
  SingleSwing = RmfDoor.DOOR_TYPE_SINGLE_SWING,
  SingleSliding = RmfDoor.DOOR_TYPE_SINGLE_SLIDING,
  SingleTelescope = RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE,
  DoubleSwing = RmfDoor.DOOR_TYPE_DOUBLE_SWING,
  DoubleSliding = RmfDoor.DOOR_TYPE_DOUBLE_SLIDING,
  DoubleTelescope = RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE,
}

export enum DoorMotion {
  Clockwise = 1,
  AntiClockwise = -1,
}

export enum DoorMode {
  Open = RmfDoorMode.MODE_OPEN,
  Closed = RmfDoorMode.MODE_CLOSED,
  Moving = RmfDoorMode.MODE_MOVING,
}

export interface DoorData {
  level: string;
  door: RmfDoor;
}

export function doorModeToString(doorMode?: number): string {
  if (doorMode === undefined) {
    return 'N/A';
  }
  switch (doorMode) {
    case RmfDoorMode.MODE_OPEN:
      return 'OPEN';
    case RmfDoorMode.MODE_CLOSED:
      return 'CLOSED';
    case RmfDoorMode.MODE_MOVING:
      return 'MOVING';
    default:
      return 'UNKNOWN';
  }
}

export function doorTypeToString(doorType: number): string {
  switch (doorType) {
    case RmfDoor.DOOR_TYPE_DOUBLE_SLIDING:
      return 'Double Sliding';
    case RmfDoor.DOOR_TYPE_DOUBLE_SWING:
      return 'Double Swing';
    case RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE:
      return 'Double Telescope';
    case RmfDoor.DOOR_TYPE_SINGLE_SLIDING:
      return 'Single Sliding';
    case RmfDoor.DOOR_TYPE_SINGLE_SWING:
      return 'Single Swing';
    case RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE:
      return 'Single Telescope';
    default:
      return `Unknown (${doorType})`;
  }
}

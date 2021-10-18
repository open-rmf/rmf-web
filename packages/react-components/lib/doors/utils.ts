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

export interface DoorData {
  level: string;
  door: RmfModels.Door;
}

export function doorModeToString(doorMode?: number): string {
  if (doorMode === undefined) {
    return 'N/A';
  }
  switch (doorMode) {
    case RmfModels.DoorMode.MODE_OPEN:
      return 'OPEN';
    case RmfModels.DoorMode.MODE_CLOSED:
      return 'CLOSED';
    case RmfModels.DoorMode.MODE_MOVING:
      return 'MOVING';
    default:
      return 'N/A';
  }
}

export function doorTypeToString(doorType: number): string {
  switch (doorType) {
    case RmfModels.Door.DOOR_TYPE_DOUBLE_SLIDING:
      return 'Double Sliding';
    case RmfModels.Door.DOOR_TYPE_DOUBLE_SWING:
      return 'Double Swing';
    case RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE:
      return 'Double Telescope';
    case RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING:
      return 'Single Sliding';
    case RmfModels.Door.DOOR_TYPE_SINGLE_SWING:
      return 'Single Swing';
    case RmfModels.Door.DOOR_TYPE_SINGLE_TELESCOPE:
      return 'Single Telescope';
    default:
      return `Unknown (${doorType})`;
  }
}

// table cell has padding of 16px left and 24px right respectively
// need to deduct 40px away from actual width
export const doorCellConfig = {
  doorName: 105.6,
  doorMode: 69.6,
  doorLevel: 40,
  doorType: 96.8,
  doorState: 84.8,
  doorControl: 128.8,
  rowHeight: 31,
};

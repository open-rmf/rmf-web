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

export interface DetailedDoor extends RmfModels.Door {
  level: string;
}

export interface DoorInfoProps {
  door: DetailedDoor;
  doorState: RmfModels.DoorState;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export interface DoorPanelAndTableProps {
  doors: DetailedDoor[];
  doorStates: Record<string, RmfModels.DoorState>;
  onDoorControlClick?(event: React.MouseEvent, door: RmfModels.Door, mode: number): void;
}

export function doorModeToString(doorState?: RmfModels.DoorState): string {
  if (!doorState) {
    return 'N/A';
  }
  switch (doorState.current_mode.value) {
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

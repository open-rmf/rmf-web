import * as RmfModels from 'rmf-models';
import { DoorData } from './utils';

export function makeDoor(door?: Partial<RmfModels.Door>): RmfModels.Door {
  return {
    name: 'test',
    door_type: RmfModels.Door.DOOR_TYPE_SINGLE_SWING,
    motion_direction: 1,
    motion_range: Math.PI / 2,
    v1_x: 0,
    v1_y: 0,
    v2_x: 1,
    v2_y: 1,
    ...door,
  };
}

export function allDoorTypes(): number[] {
  return [
    RmfModels.Door.DOOR_TYPE_DOUBLE_SLIDING,
    RmfModels.Door.DOOR_TYPE_DOUBLE_SWING,
    RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
    RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING,
    RmfModels.Door.DOOR_TYPE_SINGLE_SWING,
    RmfModels.Door.DOOR_TYPE_SINGLE_TELESCOPE,
    RmfModels.Door.DOOR_TYPE_UNDEFINED,
    -1,
  ];
}

export function makeDoorState(state?: Partial<RmfModels.DoorState>): RmfModels.DoorState {
  return {
    door_name: 'test',
    current_mode: { value: RmfModels.DoorMode.MODE_CLOSED },
    door_time: { sec: 0, nanosec: 0 },
    ...state,
  };
}

export function allDoorModes(): RmfModels.DoorMode[] {
  return [
    { value: RmfModels.DoorMode.MODE_CLOSED },
    { value: RmfModels.DoorMode.MODE_OPEN },
    { value: RmfModels.DoorMode.MODE_MOVING },
    { value: -1 },
  ];
}

export const doors: RmfModels.Door[] = [
  makeDoor({ name: 'main_door' }),
  makeDoor({ name: 'hardware_door' }),
  makeDoor({ name: 'coe_door' }),
  makeDoor({ name: 'exit_door' }),
  makeDoor({ name: 'extra_door' }),
];

export const doorStates: Record<string, RmfModels.DoorState> = {
  main_door: makeDoorState({ door_name: 'main_door', current_mode: { value: -1 } }),
  coe_door: makeDoorState({
    door_name: 'coe_door',
    current_mode: { value: RmfModels.DoorMode.MODE_OPEN },
  }),
  hardware_door: makeDoorState({
    door_name: 'hardware_door',
    current_mode: { value: RmfModels.DoorMode.MODE_CLOSED },
  }),
  exit_door: makeDoorState({
    door_name: 'exit_door',
    current_mode: { value: RmfModels.DoorMode.MODE_MOVING },
  }),
};

export const makeDetailedDoors = (): DoorData[] => {
  return doors.map((door) => ({ door, level: 'L1' }));
};

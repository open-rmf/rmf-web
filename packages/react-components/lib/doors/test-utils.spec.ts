import * as RmfModels from 'rmf-models';
import { DetailedDoor } from './utils';

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
  {
    name: 'main_door',
    v1_x: 8.2,
    v1_y: -5.5,
    v2_x: 7.85,
    v2_y: -6.2,
    door_type: 2,
    motion_range: -1.571,
    motion_direction: 1,
  },
  {
    name: 'hardware_door',
    v1_x: 4.9,
    v1_y: -4,
    v2_x: 4.4,
    v2_y: -5,
    door_type: 1,
    motion_range: 1.571,
    motion_direction: -1,
  },
  {
    name: 'coe_door',
    v1_x: 19.5,
    v1_y: -10.8,
    v2_x: 19.5,
    v2_y: -9.9,
    door_type: 1,
    motion_range: 1.571,
    motion_direction: 1,
  },
  {
    name: 'exit_door',
    v1_x: 12.2,
    v1_y: -2.7,
    v2_x: 14.1,
    v2_y: -2.7,
    door_type: 1,
    motion_range: -1.571,
    motion_direction: 1,
  },
  {
    name: 'extra_door',
    v1_x: 12.2,
    v1_y: -2.7,
    v2_x: 14.1,
    v2_y: -2.7,
    door_type: 1,
    motion_range: -1.571,
    motion_direction: 1,
  },
];

export const doorStates: Record<string, RmfModels.DoorState> = {
  main_door: {
    door_name: 'main_door',
    current_mode: { value: -1 },
    door_time: { sec: 0, nanosec: 0 },
  },
  coe_door: {
    door_name: 'coe_door',
    current_mode: { value: RmfModels.DoorMode.MODE_OPEN },
    door_time: { sec: 0, nanosec: 0 },
  },
  hardware_door: {
    door_name: 'hardware_door',
    current_mode: { value: RmfModels.DoorMode.MODE_CLOSED },
    door_time: { sec: 0, nanosec: 0 },
  },
  exit_door: {
    door_name: 'exit_door',
    current_mode: { value: RmfModels.DoorMode.MODE_MOVING },
    door_time: { sec: 0, nanosec: 0 },
  },
};

export const makeDetailedDoors = (): DetailedDoor[] => {
  return doors.map((door) => ({ ...door, level: 'L1' }));
};

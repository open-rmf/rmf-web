import type { Door, DoorMode, DoorState } from 'api-client';
import { Door as RmfDoor, DoorMode as RmfDoorMode } from 'rmf-models';

export function makeDoor(door?: Partial<Door>): Door {
  return {
    name: 'test',
    door_type: RmfDoor.DOOR_TYPE_SINGLE_SWING,
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
    RmfDoor.DOOR_TYPE_DOUBLE_SLIDING,
    RmfDoor.DOOR_TYPE_DOUBLE_SWING,
    RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE,
    RmfDoor.DOOR_TYPE_SINGLE_SLIDING,
    RmfDoor.DOOR_TYPE_SINGLE_SWING,
    RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE,
    RmfDoor.DOOR_TYPE_UNDEFINED,
    -1,
  ];
}

export function makeDoorState(state?: Partial<DoorState>): DoorState {
  return {
    door_name: 'test',
    current_mode: { value: RmfDoorMode.MODE_CLOSED },
    door_time: { sec: 0, nanosec: 0 },
    ...state,
  };
}

export function allDoorModes(): DoorMode[] {
  return [
    { value: RmfDoorMode.MODE_CLOSED },
    { value: RmfDoorMode.MODE_OPEN },
    { value: RmfDoorMode.MODE_MOVING },
    { value: -1 },
  ];
}

export const doors: Door[] = [
  makeDoor({ name: 'main_door' }),
  makeDoor({ name: 'hardware_door' }),
  makeDoor({ name: 'coe_door' }),
  makeDoor({ name: 'exit_door' }),
  makeDoor({ name: 'extra_door' }),
  makeDoor({ name: 'door_with_super_super_long_name' }),
];

export const doorStates: Record<string, DoorState> = {
  main_door: makeDoorState({ door_name: 'main_door', current_mode: { value: -1 } }),
  coe_door: makeDoorState({
    door_name: 'coe_door',
    current_mode: { value: RmfDoorMode.MODE_OPEN },
  }),
  hardware_door: makeDoorState({
    door_name: 'hardware_door',
    current_mode: { value: RmfDoorMode.MODE_CLOSED },
  }),
  exit_door: makeDoorState({
    door_name: 'exit_door',
    current_mode: { value: RmfDoorMode.MODE_MOVING },
  }),
  door_with_a_really_long_name: makeDoorState({
    door_name: 'door_with_a_really_long_name',
    current_mode: { value: RmfDoorMode.MODE_UNKNOWN },
  }),
};

export const makeDoorsData = () => {
  return doors.map((door) => ({ door, level: 'L1' }));
};

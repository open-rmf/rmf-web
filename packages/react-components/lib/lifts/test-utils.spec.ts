import type { Lift, LiftState } from 'api-client';
import { Door as RmfDoor, LiftState as RmfLiftState } from 'rmf-models';

export function allLiftMotion(): number[] {
  return [
    RmfLiftState.MOTION_UP,
    RmfLiftState.MOTION_DOWN,
    RmfLiftState.MOTION_STOPPED,
    RmfLiftState.MOTION_UNKNOWN,
    -1,
  ];
}

export function allLiftModes(): number[] {
  return [
    RmfLiftState.MODE_AGV,
    RmfLiftState.MODE_EMERGENCY,
    RmfLiftState.MODE_FIRE,
    RmfLiftState.MODE_HUMAN,
    RmfLiftState.MODE_OFFLINE,
    RmfLiftState.MODE_UNKNOWN,
    -1,
  ];
}

export function allDoorStates(): number[] {
  return [RmfLiftState.DOOR_CLOSED, RmfLiftState.DOOR_MOVING, RmfLiftState.DOOR_OPEN, -1];
}

export function makeLift(lift?: Partial<Lift>): Lift {
  return {
    name: 'test',
    doors: [
      {
        name: 'door',
        door_type: RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE,
        motion_direction: 1,
        motion_range: Math.PI / 2,
        v1_x: -1,
        v1_y: -1,
        v2_x: 1,
        v2_y: -1,
      },
    ],
    levels: ['L1', 'L2'],
    ref_x: 0,
    ref_y: 0,
    ref_yaw: 0,
    width: 2,
    depth: 2,
    wall_graph: { name: 'test', edges: [], params: [], vertices: [] },
    ...lift,
  };
}

export function makeLiftState(liftState?: Partial<LiftState>): LiftState {
  return {
    lift_name: 'test',
    available_floors: ['L1', 'L2'],
    available_modes: [],
    current_floor: 'L1',
    current_mode: RmfLiftState.MODE_AGV,
    destination_floor: 'L1',
    door_state: RmfLiftState.DOOR_CLOSED,
    lift_time: { sec: 0, nanosec: 0 },
    motion_state: RmfLiftState.MOTION_STOPPED,
    session_id: 'test_session',
    ...liftState,
  };
}

const defaultLift = makeLift();
export const testLifts = [
  { ...defaultLift },
  { ...defaultLift, name: 'test1' },
  { ...defaultLift, name: 'test2' },
  { ...defaultLift, name: 'test3' },
  { ...defaultLift, name: 'test4' },
  { ...defaultLift, name: 'test5' },
  { ...defaultLift, name: 'test_lift_with_a_really_long_name' },
];
const defaultState = makeLiftState();
export const testLiftStates: Record<string, LiftState> = {
  test: defaultState,
  test1: {
    ...defaultState,
    door_state: RmfLiftState.DOOR_MOVING,
    motion_state: RmfLiftState.MOTION_DOWN,
    current_mode: RmfLiftState.MODE_EMERGENCY,
  },
  test2: {
    ...defaultState,
    door_state: RmfLiftState.DOOR_OPEN,
    motion_state: RmfLiftState.MOTION_UP,
    current_mode: RmfLiftState.MODE_FIRE,
  },
  test3: {
    ...defaultState,
    door_state: -1,
    current_mode: RmfLiftState.MODE_HUMAN,
  },
  test4: {
    ...defaultState,
    current_mode: RmfLiftState.MODE_OFFLINE,
  },
  test5: {
    ...defaultState,
    current_mode: RmfLiftState.MODE_UNKNOWN,
  },
};

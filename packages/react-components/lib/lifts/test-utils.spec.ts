import * as RmfModels from 'rmf-models';

export function allLiftMotion(): number[] {
  return [
    RmfModels.LiftState.MOTION_UP,
    RmfModels.LiftState.MOTION_DOWN,
    RmfModels.LiftState.MOTION_STOPPED,
    RmfModels.LiftState.MOTION_UNKNOWN,
    -1,
  ];
}

export function allLiftModes(): number[] {
  return [
    RmfModels.LiftState.MODE_AGV,
    RmfModels.LiftState.MODE_EMERGENCY,
    RmfModels.LiftState.MODE_FIRE,
    RmfModels.LiftState.MODE_HUMAN,
    RmfModels.LiftState.MODE_OFFLINE,
    RmfModels.LiftState.MODE_UNKNOWN,
    -1,
  ];
}

export function allDoorStates(): number[] {
  return [
    RmfModels.LiftState.DOOR_CLOSED,
    RmfModels.LiftState.DOOR_MOVING,
    RmfModels.LiftState.DOOR_OPEN,
    -1,
  ];
}

export function makeLift(lift?: Partial<RmfModels.Lift>): RmfModels.Lift {
  return {
    name: 'test',
    doors: [
      {
        name: 'door',
        door_type: RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
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

export function makeLiftState(liftState?: Partial<RmfModels.LiftState>): RmfModels.LiftState {
  return {
    lift_name: 'test',
    available_floors: ['L1', 'L2'],
    available_modes: Uint8Array.from([]),
    current_floor: 'L1',
    current_mode: RmfModels.LiftState.MODE_AGV,
    destination_floor: 'L1',
    door_state: RmfModels.LiftState.DOOR_CLOSED,
    lift_time: { sec: 0, nanosec: 0 },
    motion_state: RmfModels.LiftState.MOTION_STOPPED,
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
];
const defaultState = makeLiftState();
export const testLiftStates: Record<string, RmfModels.LiftState> = {
  test: defaultState,
  test1: {
    ...defaultState,
    door_state: RmfModels.LiftState.DOOR_MOVING,
    motion_state: RmfModels.LiftState.MOTION_DOWN,
    current_mode: RmfModels.LiftState.MODE_EMERGENCY,
  },
  test2: {
    ...defaultState,
    door_state: RmfModels.LiftState.DOOR_OPEN,
    motion_state: RmfModels.LiftState.MOTION_UP,
    current_mode: RmfModels.LiftState.MODE_FIRE,
  },
  test3: {
    ...defaultState,
    door_state: -1,
    current_mode: RmfModels.LiftState.MODE_HUMAN,
  },
  test4: {
    ...defaultState,
    current_mode: RmfModels.LiftState.MODE_OFFLINE,
  },
  test5: {
    ...defaultState,
    current_mode: RmfModels.LiftState.MODE_UNKNOWN,
  },
};

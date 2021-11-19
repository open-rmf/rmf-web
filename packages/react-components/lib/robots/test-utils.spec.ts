import type { Location, RobotMode, RobotState } from 'api-client';
import { RobotMode as RmfRobotMode } from 'rmf-models';
import { VerboseRobot } from '.';

export function allRobotModes(): RobotMode[] {
  return [
    { mode: RmfRobotMode.MODE_ADAPTER_ERROR, mode_request_id: 0 },
    { mode: RmfRobotMode.MODE_CHARGING, mode_request_id: 0 },
    { mode: RmfRobotMode.MODE_DOCKING, mode_request_id: 0 },
    { mode: RmfRobotMode.MODE_EMERGENCY, mode_request_id: 0 },
    { mode: RmfRobotMode.MODE_GOING_HOME, mode_request_id: 0 },
    { mode: RmfRobotMode.MODE_IDLE, mode_request_id: 0 },
    { mode: RmfRobotMode.MODE_MOVING, mode_request_id: 0 },
    { mode: RmfRobotMode.MODE_PAUSED, mode_request_id: 0 },
    { mode: RmfRobotMode.MODE_WAITING, mode_request_id: 0 },
    { mode: -1, mode_request_id: 0 },
  ];
}

export function makeRobot(robotState?: Partial<RobotState>): RobotState {
  return {
    name: 'test',
    battery_percent: 1,
    location: { level_name: 'test_level', x: 0, y: 0, yaw: 0, t: { sec: 0, nanosec: 0 }, index: 0 },
    mode: { mode: RmfRobotMode.MODE_PAUSED, mode_request_id: 0 },
    model: 'test_model',
    task_id: 'test_task_id',
    path: [],
    seq: 0,
    ...robotState,
  };
}

function randomNumber(decimal: number): number {
  const r = Math.round(Math.random() * decimal);
  return r;
}

export function createLocation(level_name: string): Location {
  return {
    t: { sec: 0, nanosec: 0 },
    x: randomNumber(10),
    y: randomNumber(10),
    yaw: randomNumber(10),
    level_name: level_name,
    index: randomNumber(10),
  };
}

export function makeRandomRobot(name: string, model: string, mode: number): VerboseRobot {
  const task_id = randomNumber(1000);
  const battery_percent = randomNumber(100);

  return {
    fleet: 'fleet',
    name: name,
    state: {
      name: name,
      model: model,
      task_id: task_id.toString(),
      seq: 1,
      mode: { mode: mode, mode_request_id: 0 },
      battery_percent: battery_percent,
      location: createLocation('Level_1'),
      path: [],
    },
    tasks: [],
  };
}

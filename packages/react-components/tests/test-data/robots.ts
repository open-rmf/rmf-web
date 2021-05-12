import * as RmfModels from 'rmf-models';

interface RobotState {
  name: string;
  model: string;
  task_id: string;
  seq: number;
  mode: number;
  battery_percent: number;
  location: RmfModels.Location;
  path: RmfModels.Location[];
}

function randomNumber(decimal: number): number {
  const r = Math.round(Math.random() * decimal);
  return r;
}

export function createLocation(): RmfModels.Location {
  return {
    t: { sec: 10, nanosec: 200 },
    x: randomNumber(10),
    y: randomNumber(10),
    yaw: randomNumber(10),
    level_name: 'Level 1',
    index: randomNumber(10),
  };
}

export function createRobot(name: string, model: string): RobotState {
  const task_id = randomNumber(1000);
  const battery_percent = randomNumber(100);
  return {
    name: name,
    model: model,
    task_id: task_id.toString(),
    seq: 1,
    mode: RmfModels.RobotMode.MODE_IDLE,
    battery_percent: battery_percent,
    location: createLocation(),
    path: [],
  };
}

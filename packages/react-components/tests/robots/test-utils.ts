import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Trajectory } from '../../lib';

export function makeRobot(robotState?: Partial<RomiCore.RobotState>): RomiCore.RobotState {
  return {
    name: 'test',
    battery_percent: 1,
    location: { level_name: 'test_level', x: 0, y: 0, yaw: 0, t: { sec: 0, nanosec: 0 } },
    mode: { mode: RomiCore.RobotMode.MODE_PAUSED },
    model: 'test_model',
    task_id: 'test_task_id',
    path: [],
    ...robotState,
  };
}

export function makeTrajectory(traj?: Partial<Trajectory>): Trajectory {
  return {
    id: 0,
    dimensions: 1,
    fleet_name: 'test_fleet',
    robot_name: 'test_robot',
    map_name: 'test_map',
    segments: [
      {
        t: 0,
        v: [0, 0, 0.0],
        x: [0, 0, 0],
      },
      {
        t: 1000,
        v: [10, 0, 0],
        x: [10, 0, 0],
      },
    ],
    shape: 'circle',
    ...traj,
  };
}

import * as RomiCore from '@osrf/romi-js-core-interfaces';

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

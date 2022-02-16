import type { RobotState } from 'api-client';

export function makeRobot(robotState?: Partial<RobotState>): Required<RobotState> {
  return {
    name: 'test',
    battery: 1,
    location: { map: 'test_level', x: 0, y: 0, yaw: 0 },
    status: 'idle',
    task_id: 'test_task_id',
    issues: [],
    unix_millis_time: 0,
    ...robotState,
  };
}

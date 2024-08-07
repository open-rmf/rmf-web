import type { RobotState } from 'api-client';

export function makeRobot(robotState?: Partial<RobotState>): RobotState {
  return {
    name: 'test',
    battery: 1,
    location: { map: 'test_level', x: 0, y: 0, yaw: 0 },
    status: 'idle',
    task_id: 'test_task_id',
    issues: [],
    unix_millis_time: 0,
    mutex_groups: { locked: [], requesting: [] },
    commission: { dispatch_tasks: true, direct_tasks: true, idle_behavior: true },
    ...robotState,
  };
}

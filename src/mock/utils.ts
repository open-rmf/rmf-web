import * as RomiCore from '@osrf/romi-js-core-interfaces';

/**
 * Easily create dummy robots for use in tests.
 * @param options
 */
export function makeRobot(options?: Partial<RomiCore.RobotState>): RomiCore.RobotState {
  options = options || {};
  return {
    name: options.name || '',
    battery_percent: options.battery_percent || 100,
    location: options.location || { level_name: '', x: 0, y: 0, t: { sec: 0, nanosec: 0 }, yaw: 0 },
    mode: options.mode || { mode: RomiCore.RobotMode.MODE_IDLE },
    model: options.model || 'BaseModel',
    path: options.path || [],
    task_id: options.task_id || '',
  };
}

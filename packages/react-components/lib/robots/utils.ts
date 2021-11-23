import type { RobotMode, RobotState, Task } from 'api-client';
import { RobotMode as RmfRobotMode } from 'rmf-models';

/**
 * Returns a uniquely identifiable string representing a robot.
 */
export function robotHash(name: string, fleet: string): string {
  return `${name}__${fleet}`;
}

export function robotModeToString(robotMode: RobotMode): string {
  switch (robotMode.mode) {
    case RmfRobotMode.MODE_CHARGING:
      return 'Charging';
    case RmfRobotMode.MODE_DOCKING:
      return 'Docking';
    case RmfRobotMode.MODE_EMERGENCY:
      return 'Emergency';
    case RmfRobotMode.MODE_GOING_HOME:
      return 'Going Home';
    case RmfRobotMode.MODE_IDLE:
      return 'Idle';
    case RmfRobotMode.MODE_MOVING:
      return 'Moving';
    case RmfRobotMode.MODE_PAUSED:
      return 'Paused';
    case RmfRobotMode.MODE_WAITING:
      return 'Waiting';
    default:
      return `Unknown (${robotMode.mode})`;
  }
}

export interface VerboseRobot {
  fleet: string;
  name: string;
  state: RobotState;
  tasks: Task[];
}

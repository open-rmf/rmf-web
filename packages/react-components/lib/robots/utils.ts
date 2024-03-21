import { ApiServerModelsRmfApiRobotStateStatus as RobotStatus } from 'api-client';
import { RobotMode as RmfRobotMode } from 'rmf-models';

/**
 * Returns a uniquely identifiable string representing a robot.
 */
export function robotHash(name: string, fleet: string): string {
  return `${name}__${fleet}`;
}

export function robotStatusToUpperCase(status: RobotStatus): string {
  switch (status) {
    case RobotStatus.Charging:
      return 'CHARGING';
    case RobotStatus.Idle:
      return 'IDLE';
    case RobotStatus.Working:
      return 'WORKING';
    case RobotStatus.Offline:
      return 'OFFLINE';
    case RobotStatus.Uninitialized:
      return 'UNINITIALIZED';
    case RobotStatus.Shutdown:
      return 'SHUTDOWN';
    case RobotStatus.Error:
      return 'ERROR';
    default:
      return `UNKNOWN (${status})`;
  }
}

export function robotModeToString(robotMode: RmfRobotMode): string {
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

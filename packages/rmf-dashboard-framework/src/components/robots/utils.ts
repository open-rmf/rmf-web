import { ApiServerModelsRmfApiRobotStateStatus as Status2 } from 'api-client';
import { RobotMode as RmfRobotMode } from 'rmf-models/ros/rmf_fleet_msgs/msg';

/**
 * Returns a uniquely identifiable string representing a robot.
 */
export function robotHash(name: string, fleet: string): string {
  return `${name}__${fleet}`;
}

export function robotStatusToUpperCase(status: Status2): string {
  switch (status) {
    case Status2.Charging:
      return 'CHARGING';
    case Status2.Idle:
      return 'IDLE';
    case Status2.Working:
      return 'WORKING';
    case Status2.Offline:
      return 'OFFLINE';
    case Status2.Uninitialized:
      return 'UNINITIALIZED';
    case Status2.Shutdown:
      return 'SHUTDOWN';
    case Status2.Error:
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

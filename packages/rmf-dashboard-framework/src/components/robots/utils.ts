import { ApiServerModelsRmfApiRobotStateStatus as Status2 } from 'api-client';

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

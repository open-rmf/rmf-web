/* This is a generated file, do not edit */

import { Location } from '../../rmf_fleet_msgs/msg/Location';

export class DestinationRequest {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/DestinationRequest';

  fleet_name: string;
  robot_name: string;
  destination: Location;
  task_id: string;

  constructor(fields: Partial<DestinationRequest> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.robot_name = fields.robot_name || '';
    this.destination = fields.destination || new Location();
    this.task_id = fields.task_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
    try {
      Location.validate(obj['destination'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "destination":\n  ' + (e as Error).message);
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
  }
}

/*
string fleet_name
string robot_name
Location destination

# task_id must be copied into future RobotState messages
string task_id

*/

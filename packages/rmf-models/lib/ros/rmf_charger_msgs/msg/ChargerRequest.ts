/* This is a generated file, do not edit */

import { Duration } from '../../builtin_interfaces/msg/Duration';

export class ChargerRequest {
  static readonly FullTypeName = 'rmf_charger_msgs/msg/ChargerRequest';

  charger_name: string;
  fleet_name: string;
  robot_name: string;
  start_timeout: Duration;
  request_id: string;

  constructor(fields: Partial<ChargerRequest> = {}) {
    this.charger_name = fields.charger_name || '';
    this.fleet_name = fields.fleet_name || '';
    this.robot_name = fields.robot_name || '';
    this.start_timeout = fields.start_timeout || new Duration();
    this.request_id = fields.request_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['charger_name'] !== 'string') {
      throw new Error('expected "charger_name" to be "string"');
    }
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
    try {
      Duration.validate(obj['start_timeout'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "start_timeout":\n  ' + (e as Error).message);
    }
    if (typeof obj['request_id'] !== 'string') {
      throw new Error('expected "request_id" to be "string"');
    }
  }
}

/*
# The name of the charger that should process this message
string charger_name

# The robot that wishes to charge
string fleet_name
string robot_name

# The maximum amount of time to wait for the charging to start.
# If the robot takes longer than this to arrive and start charging,
# the charge request will be canceled.
builtin_interfaces/Duration start_timeout

# A unique ID for each request. It is advised that you prefix this
# with the sender's node name. This is used for error tracking
# later on
string request_id

*/

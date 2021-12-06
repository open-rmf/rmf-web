/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';
import { Duration } from '../../builtin_interfaces/msg/Duration';

export class ChargerState {
  static readonly FullTypeName = 'rmf_charger_msgs/msg/ChargerState';

  static readonly CHARGER_IDLE = 1;
  static readonly CHARGER_ASSIGNED = 2;
  static readonly CHARGER_CHARGING = 3;
  static readonly CHARGER_RELEASED = 4;
  static readonly CHARGER_ERROR = 200;

  charger_time: Time;
  state: number;
  charger_name: string;
  error_message: string;
  request_id: string;
  robot_fleet: string;
  robot_name: string;
  time_to_fully_charged: Duration;

  constructor(fields: Partial<ChargerState> = {}) {
    this.charger_time = fields.charger_time || new Time();
    this.state = fields.state || 0;
    this.charger_name = fields.charger_name || '';
    this.error_message = fields.error_message || '';
    this.request_id = fields.request_id || '';
    this.robot_fleet = fields.robot_fleet || '';
    this.robot_name = fields.robot_name || '';
    this.time_to_fully_charged = fields.time_to_fully_charged || new Duration();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['charger_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "charger_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['state'] !== 'number') {
      throw new Error('expected "state" to be "number"');
    }
    if (typeof obj['charger_name'] !== 'string') {
      throw new Error('expected "charger_name" to be "string"');
    }
    if (typeof obj['error_message'] !== 'string') {
      throw new Error('expected "error_message" to be "string"');
    }
    if (typeof obj['request_id'] !== 'string') {
      throw new Error('expected "request_id" to be "string"');
    }
    if (typeof obj['robot_fleet'] !== 'string') {
      throw new Error('expected "robot_fleet" to be "string"');
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
    try {
      Duration.validate(obj['time_to_fully_charged'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "time_to_fully_charged":\n  ' + (e as Error).message);
    }
  }
}

/*
# Time when this state message was created
builtin_interfaces/Time charger_time

uint32 CHARGER_IDLE = 1      # Charger is not occupied
uint32 CHARGER_ASSIGNED = 2  # Charger has been assigned a robot
uint32 CHARGER_CHARGING = 3  # Charger is charging
uint32 CHARGER_RELEASED = 4  # Charger has been disconnected from a robot
uint32 CHARGER_ERROR = 200   # Error state, see error_message for info

uint32 state  # One of the previously enumerated states

# The charger name should be unique in the RMF system and
# should match a charger name appearing in the traffic map
string charger_name

# The error_message field should be blank unless state is CHARGER_ERROR
string error_message

# The request_id field will be populated with the ID that started the
# charging cycle if state is anything other than CHARGER_IDLE
string request_id

# The robot that is currently assigned to this charger (if any)
string robot_fleet
string robot_name

# This contains the duration till the robot becomes fully charged.
builtin_interfaces/Duration time_to_fully_charged

*/

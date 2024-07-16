/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class ChargerState {
  static readonly FullTypeName = '';

  static readonly CHARGER_IDLE = 1;
  static readonly CHARGER_ASSIGNED = 2;
  static readonly CHARGER_CHARGING = 3;
  static readonly CHARGER_RELEASED = 4;
  static readonly CHARGER_ERROR = 200;

  charger_time: builtin_interfaces.msg.Time;
  state: number;
  charger_name: string;
  error_message: string;
  request_id: string;
  robot_fleet: string;
  robot_name: string;
  time_to_fully_charged: builtin_interfaces.msg.Duration;

  constructor(fields: Partial<ChargerState> = {}) {
    this.charger_time = fields.charger_time || new builtin_interfaces.msg.Time();
    this.state = fields.state || 0;
    this.charger_name = fields.charger_name || '';
    this.error_message = fields.error_message || '';
    this.request_id = fields.request_id || '';
    this.robot_fleet = fields.robot_fleet || '';
    this.robot_name = fields.robot_name || '';
    this.time_to_fully_charged =
      fields.time_to_fully_charged || new builtin_interfaces.msg.Duration();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      builtin_interfaces.msg.Time.validate(obj['charger_time'] as Record<string, unknown>);
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
      builtin_interfaces.msg.Duration.validate(
        obj['time_to_fully_charged'] as Record<string, unknown>,
      );
    } catch (e) {
      throw new Error('in "time_to_fully_charged":\n  ' + (e as Error).message);
    }
  }
}

export default ChargerState;

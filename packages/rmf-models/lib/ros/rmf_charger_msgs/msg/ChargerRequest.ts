/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class ChargerRequest {
  static readonly FullTypeName = '';

  charger_name: string;
  fleet_name: string;
  robot_name: string;
  start_timeout: builtin_interfaces.msg.Duration;
  request_id: string;

  constructor(fields: Partial<ChargerRequest> = {}) {
    this.charger_name = fields.charger_name || '';
    this.fleet_name = fields.fleet_name || '';
    this.robot_name = fields.robot_name || '';
    this.start_timeout = fields.start_timeout || new builtin_interfaces.msg.Duration();
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
      builtin_interfaces.msg.Duration.validate(obj['start_timeout'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "start_timeout":\n  ' + (e as Error).message);
    }
    if (typeof obj['request_id'] !== 'string') {
      throw new Error('expected "request_id" to be "string"');
    }
  }
}

export default ChargerRequest;

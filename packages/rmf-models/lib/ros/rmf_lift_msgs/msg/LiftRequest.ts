/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class LiftRequest {
  static readonly FullTypeName = '';

  static readonly REQUEST_END_SESSION = 0;
  static readonly REQUEST_AGV_MODE = 1;
  static readonly REQUEST_HUMAN_MODE = 2;
  static readonly DOOR_CLOSED = 0;
  static readonly DOOR_OPEN = 2;

  lift_name: string;
  request_time: builtin_interfaces.msg.Time;
  session_id: string;
  request_type: number;
  destination_floor: string;
  door_state: number;

  constructor(fields: Partial<LiftRequest> = {}) {
    this.lift_name = fields.lift_name || '';
    this.request_time = fields.request_time || new builtin_interfaces.msg.Time();
    this.session_id = fields.session_id || '';
    this.request_type = fields.request_type || 0;
    this.destination_floor = fields.destination_floor || '';
    this.door_state = fields.door_state || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['lift_name'] !== 'string') {
      throw new Error('expected "lift_name" to be "string"');
    }
    try {
      builtin_interfaces.msg.Time.validate(obj['request_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "request_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['session_id'] !== 'string') {
      throw new Error('expected "session_id" to be "string"');
    }
    if (typeof obj['request_type'] !== 'number') {
      throw new Error('expected "request_type" to be "number"');
    }
    if (typeof obj['destination_floor'] !== 'string') {
      throw new Error('expected "destination_floor" to be "string"');
    }
    if (typeof obj['door_state'] !== 'number') {
      throw new Error('expected "door_state" to be "number"');
    }
  }
}

export default LiftRequest;

/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';

export class Session {
  static readonly FullTypeName = 'rmf_door_msgs/msg/Session';

  request_time: Time;
  requester_id: string;

  constructor(fields: Partial<Session> = {}) {
    this.request_time = fields.request_time || new Time();
    this.requester_id = fields.requester_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['request_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "request_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['requester_id'] !== 'string') {
      throw new Error('expected "requester_id" to be "string"');
    }
  }
}

/*

builtin_interfaces/Time request_time
string requester_id

*/

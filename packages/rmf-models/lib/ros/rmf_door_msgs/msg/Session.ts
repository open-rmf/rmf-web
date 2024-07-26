/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class Session {
  static readonly FullTypeName = '';

  request_time: builtin_interfaces.msg.Time;
  requester_id: string;

  constructor(fields: Partial<Session> = {}) {
    this.request_time = fields.request_time || new builtin_interfaces.msg.Time();
    this.requester_id = fields.requester_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      builtin_interfaces.msg.Time.validate(obj['request_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "request_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['requester_id'] !== 'string') {
      throw new Error('expected "requester_id" to be "string"');
    }
  }
}

export default Session;

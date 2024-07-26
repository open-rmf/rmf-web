/* This is a generated file, do not edit */

import * as rmf_door_msgs from '../../rmf_door_msgs';
import * as builtin_interfaces from '../../builtin_interfaces';

export class DoorRequest {
  static readonly FullTypeName = '';

  request_time: builtin_interfaces.msg.Time;
  requester_id: string;
  door_name: string;
  requested_mode: rmf_door_msgs.msg.DoorMode;

  constructor(fields: Partial<DoorRequest> = {}) {
    this.request_time = fields.request_time || new builtin_interfaces.msg.Time();
    this.requester_id = fields.requester_id || '';
    this.door_name = fields.door_name || '';
    this.requested_mode = fields.requested_mode || new rmf_door_msgs.msg.DoorMode();
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
    if (typeof obj['door_name'] !== 'string') {
      throw new Error('expected "door_name" to be "string"');
    }
    try {
      rmf_door_msgs.msg.DoorMode.validate(obj['requested_mode'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "requested_mode":\n  ' + (e as Error).message);
    }
  }
}

export default DoorRequest;

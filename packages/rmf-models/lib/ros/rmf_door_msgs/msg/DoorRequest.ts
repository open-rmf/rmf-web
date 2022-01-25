/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';
import { DoorMode } from '../../rmf_door_msgs/msg/DoorMode';

export class DoorRequest {
  static readonly FullTypeName = 'rmf_door_msgs/msg/DoorRequest';

  request_time: Time;
  requester_id: string;
  door_name: string;
  requested_mode: DoorMode;

  constructor(fields: Partial<DoorRequest> = {}) {
    this.request_time = fields.request_time || new Time();
    this.requester_id = fields.requester_id || '';
    this.door_name = fields.door_name || '';
    this.requested_mode = fields.requested_mode || new DoorMode();
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
    if (typeof obj['door_name'] !== 'string') {
      throw new Error('expected "door_name" to be "string"');
    }
    try {
      DoorMode.validate(obj['requested_mode'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "requested_mode":\n  ' + (e as Error).message);
    }
  }
}

/*
builtin_interfaces/Time request_time
string requester_id
string door_name
DoorMode requested_mode

*/

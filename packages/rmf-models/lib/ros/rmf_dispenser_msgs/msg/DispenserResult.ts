/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';

export class DispenserResult {
  static readonly FullTypeName = 'rmf_dispenser_msgs/msg/DispenserResult';

  static readonly ACKNOWLEDGED = 0;
  static readonly SUCCESS = 1;
  static readonly FAILED = 2;

  time: Time;
  request_guid: string;
  source_guid: string;
  status: number;

  constructor(fields: Partial<DispenserResult> = {}) {
    this.time = fields.time || new Time();
    this.request_guid = fields.request_guid || '';
    this.source_guid = fields.source_guid || '';
    this.status = fields.status || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "time":\n  ' + (e as Error).message);
    }
    if (typeof obj['request_guid'] !== 'string') {
      throw new Error('expected "request_guid" to be "string"');
    }
    if (typeof obj['source_guid'] !== 'string') {
      throw new Error('expected "source_guid" to be "string"');
    }
    if (typeof obj['status'] !== 'number') {
      throw new Error('expected "status" to be "number"');
    }
  }
}

/*
builtin_interfaces/Time time

# A unique ID for the request which this result is for
string request_guid

# The unique ID of the workcell that this result was sent from
string source_guid

# Different basic result statuses
uint8 status
uint8 ACKNOWLEDGED=0
uint8 SUCCESS=1
uint8 FAILED=2

# below are custom workcell message fields

*/

/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';

export class DispenserState {
  static readonly FullTypeName = 'rmf_dispenser_msgs/msg/DispenserState';

  static readonly IDLE = 0;
  static readonly BUSY = 1;
  static readonly OFFLINE = 2;

  time: Time;
  guid: string;
  mode: number;
  request_guid_queue: string[];
  seconds_remaining: number;

  constructor(fields: Partial<DispenserState> = {}) {
    this.time = fields.time || new Time();
    this.guid = fields.guid || '';
    this.mode = fields.mode || 0;
    this.request_guid_queue = fields.request_guid_queue || [];
    this.seconds_remaining = fields.seconds_remaining || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "time":\n  ' + (e as Error).message);
    }
    if (typeof obj['guid'] !== 'string') {
      throw new Error('expected "guid" to be "string"');
    }
    if (typeof obj['mode'] !== 'number') {
      throw new Error('expected "mode" to be "number"');
    }
    if (!Array.isArray(obj['request_guid_queue'])) {
      throw new Error('expected "request_guid_queue" to be an array');
    }
    for (const [i, v] of obj['request_guid_queue'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "request_guid_queue" to be "string"`);
      }
    }
    if (typeof obj['seconds_remaining'] !== 'number') {
      throw new Error('expected "seconds_remaining" to be "number"');
    }
  }
}

/*
builtin_interfaces/Time time

# A unique ID for this workcell
string guid

# Different basic modes that the workcell could be in
int32 mode
int32 IDLE=0
int32 BUSY=1
int32 OFFLINE=2

# Queued up requests that are being handled by this workcell
string[] request_guid_queue

# below are custom workcell message fields
float32 seconds_remaining

*/

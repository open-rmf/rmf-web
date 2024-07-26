/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class IngestorState {
  static readonly FullTypeName = '';

  static readonly IDLE = 0;
  static readonly BUSY = 1;
  static readonly OFFLINE = 2;

  time: builtin_interfaces.msg.Time;
  guid: string;
  mode: number;
  request_guid_queue: Array<string>;
  seconds_remaining: number;

  constructor(fields: Partial<IngestorState> = {}) {
    this.time = fields.time || new builtin_interfaces.msg.Time();
    this.guid = fields.guid || '';
    this.mode = fields.mode || 0;
    this.request_guid_queue = fields.request_guid_queue || [];
    this.seconds_remaining = fields.seconds_remaining || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      builtin_interfaces.msg.Time.validate(obj['time'] as Record<string, unknown>);
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

export default IngestorState;

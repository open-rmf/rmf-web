/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class IngestorResult {
  static readonly FullTypeName = '';

  static readonly ACKNOWLEDGED = 0;
  static readonly SUCCESS = 1;
  static readonly FAILED = 2;

  time: builtin_interfaces.msg.Time;
  request_guid: string;
  source_guid: string;
  status: number;

  constructor(fields: Partial<IngestorResult> = {}) {
    this.time = fields.time || new builtin_interfaces.msg.Time();
    this.request_guid = fields.request_guid || '';
    this.source_guid = fields.source_guid || '';
    this.status = fields.status || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      builtin_interfaces.msg.Time.validate(obj['time'] as Record<string, unknown>);
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

export default IngestorResult;

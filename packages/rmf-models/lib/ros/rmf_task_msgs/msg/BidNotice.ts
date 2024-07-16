/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class BidNotice {
  static readonly FullTypeName = '';

  request: string;
  task_id: string;
  time_window: builtin_interfaces.msg.Duration;

  constructor(fields: Partial<BidNotice> = {}) {
    this.request = fields.request || '';
    this.task_id = fields.task_id || '';
    this.time_window = fields.time_window || new builtin_interfaces.msg.Duration();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['request'] !== 'string') {
      throw new Error('expected "request" to be "string"');
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    try {
      builtin_interfaces.msg.Duration.validate(obj['time_window'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "time_window":\n  ' + (e as Error).message);
    }
  }
}

export default BidNotice;

/* This is a generated file, do not edit */

import { Duration } from '../../builtin_interfaces/msg/Duration';

export class BidNotice {
  static readonly FullTypeName = 'rmf_task_msgs/msg/BidNotice';

  request: string;
  task_id: string;
  time_window: Duration;

  constructor(fields: Partial<BidNotice> = {}) {
    this.request = fields.request || '';
    this.task_id = fields.task_id || '';
    this.time_window = fields.time_window || new Duration();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['request'] !== 'string') {
      throw new Error('expected "request" to be "string"');
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    try {
      Duration.validate(obj['time_window'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "time_window":\n  ' + (e as Error).message);
    }
  }
}

/*
# This message is published by the Task Dispatcher node to notify all
# Fleet Adapters to participate in a bidding process for a new task.
# Fleet Adapters may then submit a BidProposal message with their best proposal
# to accommodate the new task.

# Details of the task request
string request

# The ID for this request
string task_id

# Duration for which the bidding is open
builtin_interfaces/Duration time_window

*/

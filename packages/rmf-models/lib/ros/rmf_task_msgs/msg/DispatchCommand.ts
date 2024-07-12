/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';

export class DispatchCommand {
  static readonly FullTypeName = 'rmf_task_msgs/msg/DispatchCommand';

  static readonly TYPE_AWARD = 1;
  static readonly TYPE_REMOVE = 2;

  fleet_name: string;
  task_id: string;
  dispatch_id: number;
  timestamp: Time;
  type: number;

  constructor(fields: Partial<DispatchCommand> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.task_id = fields.task_id || '';
    this.dispatch_id = fields.dispatch_id || 0;
    this.timestamp = fields.timestamp || new Time();
    this.type = fields.type || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['dispatch_id'] !== 'number') {
      throw new Error('expected "dispatch_id" to be "number"');
    }
    try {
      Time.validate(obj['timestamp'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "timestamp":\n  ' + (e as Error).message);
    }
    if (typeof obj['type'] !== 'number') {
      throw new Error('expected "type" to be "number"');
    }
  }
}

/*
# This message is published by Task Dispatcher Node to either award or cancel a
# task for a Fleet Adapter

# The selected Fleet Adapter to award/cancel the task
string fleet_name

# The task_id of the task that
string task_id

# Unique ID of this request message
uint64 dispatch_id

# The time that this dispatch request was originally made. Dispatch requests may
# expire with an error if they get no response after an extended period of time.
builtin_interfaces/Time timestamp

# Add or Cancel a task
uint8 type
uint8 TYPE_AWARD=1   # to award a task to a fleet
uint8 TYPE_REMOVE=2  # to remove a task from a fleet

*/

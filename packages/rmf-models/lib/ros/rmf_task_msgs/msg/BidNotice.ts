/* This is a generated file, do not edit */

import { TaskProfile } from '../../rmf_task_msgs/msg/TaskProfile';
import { Duration } from '../../builtin_interfaces/msg/Duration';

export class BidNotice {
  static readonly FullTypeName = 'rmf_task_msgs/msg/BidNotice';

  task_profile: TaskProfile;
  time_window: Duration;

  constructor(fields: Partial<BidNotice> = {}) {
    this.task_profile = fields.task_profile || new TaskProfile();
    this.time_window = fields.time_window || new Duration();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      TaskProfile.validate(obj['task_profile'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "task_profile":\n  ' + (e as Error).message);
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

# Details of the new task
TaskProfile task_profile

# Duration for which the bidding is open
builtin_interfaces/Duration time_window

*/

/* This is a generated file, do not edit */

import { TaskProfile } from '../../rmf_task_msgs/msg/TaskProfile';

export class DispatchRequest {
  static readonly FullTypeName = 'rmf_task_msgs/msg/DispatchRequest';

  static readonly ADD = 1;
  static readonly CANCEL = 2;

  fleet_name: string;
  task_profile: TaskProfile;
  method: number;

  constructor(fields: Partial<DispatchRequest> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.task_profile = fields.task_profile || new TaskProfile();
    this.method = fields.method || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    try {
      TaskProfile.validate(obj['task_profile'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "task_profile":\n  ' + (e as Error).message);
    }
    if (typeof obj['method'] !== 'number') {
      throw new Error('expected "method" to be "number"');
    }
  }
}

/*
# This message is published by Task Dispatcher Node to either award or cancel a
# task for a Fleet Adapter

# The selected Fleet Adapter to award/cancel the task
string fleet_name

# The details of the task to be awarded or cancelled. This should match the
# TaskProfile in the corresponding BidNotice message
TaskProfile task_profile

# Add or Cancel a task
uint8 method
uint8 ADD=1     # to add a task
uint8 CANCEL=2  # to cancel and remove a task

*/

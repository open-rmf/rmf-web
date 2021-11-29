/* This is a generated file, do not edit */

import { TaskProfile } from '../../rmf_task_msgs/msg/TaskProfile';
import { Time } from '../../builtin_interfaces/msg/Time';

export class BidProposal {
  static readonly FullTypeName = 'rmf_task_msgs/msg/BidProposal';

  fleet_name: string;
  task_profile: TaskProfile;
  prev_cost: number;
  new_cost: number;
  finish_time: Time;
  robot_name: string;

  constructor(fields: Partial<BidProposal> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.task_profile = fields.task_profile || new TaskProfile();
    this.prev_cost = fields.prev_cost || 0;
    this.new_cost = fields.new_cost || 0;
    this.finish_time = fields.finish_time || new Time();
    this.robot_name = fields.robot_name || '';
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
    if (typeof obj['prev_cost'] !== 'number') {
      throw new Error('expected "prev_cost" to be "number"');
    }
    if (typeof obj['new_cost'] !== 'number') {
      throw new Error('expected "new_cost" to be "number"');
    }
    try {
      Time.validate(obj['finish_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "finish_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
  }
}

/*
# This message is published by a Fleet Adapter in response to a BidNotice
# message.

# The name of the Fleet Adapter publishing this message
string fleet_name

# Details of the task to accommodate. This should math the TaskProfile in the
# BidNotice
TaskProfile task_profile

# The overall cost of task assignments prior to accommodating the new task
float64 prev_cost

# The overall cost of task assignments after accommodating the new task
float64 new_cost

# The estimated finish time of the new task
builtin_interfaces/Time finish_time

# The name of the robot in the fleet which will potentially execute the task
string robot_name

*/

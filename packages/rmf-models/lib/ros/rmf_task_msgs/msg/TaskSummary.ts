/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';
import * as rmf_task_msgs from '../../rmf_task_msgs';

export class TaskSummary {
  static readonly FullTypeName = '';

  static readonly STATE_QUEUED = 0;
  static readonly STATE_ACTIVE = 1;
  static readonly STATE_COMPLETED = 2;
  static readonly STATE_FAILED = 3;
  static readonly STATE_CANCELED = 4;
  static readonly STATE_PENDING = 5;

  fleet_name: string;
  task_id: string;
  task_profile: rmf_task_msgs.msg.TaskProfile;
  state: number;
  status: string;
  submission_time: builtin_interfaces.msg.Time;
  start_time: builtin_interfaces.msg.Time;
  end_time: builtin_interfaces.msg.Time;
  robot_name: string;

  constructor(fields: Partial<TaskSummary> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.task_id = fields.task_id || '';
    this.task_profile = fields.task_profile || new rmf_task_msgs.msg.TaskProfile();
    this.state = fields.state || 0;
    this.status = fields.status || '';
    this.submission_time = fields.submission_time || new builtin_interfaces.msg.Time();
    this.start_time = fields.start_time || new builtin_interfaces.msg.Time();
    this.end_time = fields.end_time || new builtin_interfaces.msg.Time();
    this.robot_name = fields.robot_name || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    try {
      rmf_task_msgs.msg.TaskProfile.validate(obj['task_profile'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "task_profile":\n  ' + (e as Error).message);
    }
    if (typeof obj['state'] !== 'number') {
      throw new Error('expected "state" to be "number"');
    }
    if (typeof obj['status'] !== 'string') {
      throw new Error('expected "status" to be "string"');
    }
    try {
      builtin_interfaces.msg.Time.validate(obj['submission_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "submission_time":\n  ' + (e as Error).message);
    }
    try {
      builtin_interfaces.msg.Time.validate(obj['start_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "start_time":\n  ' + (e as Error).message);
    }
    try {
      builtin_interfaces.msg.Time.validate(obj['end_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "end_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
  }
}

export default TaskSummary;

/* This is a generated file, do not edit */

import { TaskProfile } from '../../rmf_task_msgs/msg/TaskProfile';
import { Time } from '../../builtin_interfaces/msg/Time';

export class TaskSummary {
  static readonly FullTypeName = 'rmf_task_msgs/msg/TaskSummary';

  static readonly STATE_QUEUED = 0;
  static readonly STATE_ACTIVE = 1;
  static readonly STATE_COMPLETED = 2;
  static readonly STATE_FAILED = 3;
  static readonly STATE_CANCELED = 4;
  static readonly STATE_PENDING = 5;

  fleet_name: string;
  task_id: string;
  task_profile: TaskProfile;
  state: number;
  status: string;
  submission_time: Time;
  start_time: Time;
  end_time: Time;
  robot_name: string;

  constructor(fields: Partial<TaskSummary> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.task_id = fields.task_id || '';
    this.task_profile = fields.task_profile || new TaskProfile();
    this.state = fields.state || 0;
    this.status = fields.status || '';
    this.submission_time = fields.submission_time || new Time();
    this.start_time = fields.start_time || new Time();
    this.end_time = fields.end_time || new Time();
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
      TaskProfile.validate(obj['task_profile'] as Record<string, unknown>);
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
      Time.validate(obj['submission_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "submission_time":\n  ' + (e as Error).message);
    }
    try {
      Time.validate(obj['start_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "start_time":\n  ' + (e as Error).message);
    }
    try {
      Time.validate(obj['end_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "end_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
  }
}

/*
# Publish by Fleet Adapter (aka DispatchStatus)

# Fleet Adapter name
string fleet_name

# *optional and duplicated in TaskProfile
string task_id 

TaskProfile task_profile

uint32 state
uint32 STATE_QUEUED=0
uint32 STATE_ACTIVE=1
uint32 STATE_COMPLETED=2  # hooray
uint32 STATE_FAILED=3     # oh no
uint32 STATE_CANCELED=4
uint32 STATE_PENDING=5

# a brief summary of the current status of the task, for UI's
# *optional
string status

# submission_time is when the task was submitted to rmf_core
# *optional and duplicated in TaskProfile
builtin_interfaces/Time submission_time   

# when rmf_core actually began processing the task
builtin_interfaces/Time start_time

# When this message is a summary of an in-process task, the end_time field is
# an estimate. When this message is a summary of a completed or failed task,
# end_time is the actual time.
builtin_interfaces/Time end_time

# Allocated robot name
# *optional
string robot_name

*/

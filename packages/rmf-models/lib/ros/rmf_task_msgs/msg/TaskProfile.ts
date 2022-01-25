/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';
import { TaskDescription } from '../../rmf_task_msgs/msg/TaskDescription';

export class TaskProfile {
  static readonly FullTypeName = 'rmf_task_msgs/msg/TaskProfile';

  task_id: string;
  submission_time: Time;
  description: TaskDescription;

  constructor(fields: Partial<TaskProfile> = {}) {
    this.task_id = fields.task_id || '';
    this.submission_time = fields.submission_time || new Time();
    this.description = fields.description || new TaskDescription();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    try {
      Time.validate(obj['submission_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "submission_time":\n  ' + (e as Error).message);
    }
    try {
      TaskDescription.validate(obj['description'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "description":\n  ' + (e as Error).message);
    }
  }
}

/*
# Unique ID assigned to this task
string task_id

# Task submission time
builtin_interfaces/Time submission_time

# Details of the task
TaskDescription description

*/

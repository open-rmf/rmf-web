/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';
import * as rmf_task_msgs from '../../rmf_task_msgs';

export class TaskProfile {
  static readonly FullTypeName = '';

  task_id: string;
  submission_time: builtin_interfaces.msg.Time;
  description: rmf_task_msgs.msg.TaskDescription;

  constructor(fields: Partial<TaskProfile> = {}) {
    this.task_id = fields.task_id || '';
    this.submission_time = fields.submission_time || new builtin_interfaces.msg.Time();
    this.description = fields.description || new rmf_task_msgs.msg.TaskDescription();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    try {
      builtin_interfaces.msg.Time.validate(obj['submission_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "submission_time":\n  ' + (e as Error).message);
    }
    try {
      rmf_task_msgs.msg.TaskDescription.validate(obj['description'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "description":\n  ' + (e as Error).message);
    }
  }
}

export default TaskProfile;

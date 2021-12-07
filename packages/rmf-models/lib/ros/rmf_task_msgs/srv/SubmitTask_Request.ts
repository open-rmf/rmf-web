/* This is a generated file, do not edit */

import { TaskDescription } from '../../rmf_task_msgs/msg/TaskDescription';

export class SubmitTask_Request {
  static readonly FullTypeName = 'rmf_task_msgs/srv/SubmitTask_Request';

  requester: string;
  description: TaskDescription;

  constructor(fields: Partial<SubmitTask_Request> = {}) {
    this.requester = fields.requester || '';
    this.description = fields.description || new TaskDescription();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['requester'] !== 'string') {
      throw new Error('expected "requester" to be "string"');
    }
    try {
      TaskDescription.validate(obj['description'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "description":\n  ' + (e as Error).message);
    }
  }
}

/*
# Submit Task | POST service call

# Identifier for who is requesting the service
string requester

# desciption of task
TaskDescription description


*/

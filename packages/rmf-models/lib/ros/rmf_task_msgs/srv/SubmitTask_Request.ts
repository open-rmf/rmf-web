/* This is a generated file, do not edit */

import * as rmf_task_msgs from '../../rmf_task_msgs';

export class SubmitTask_Request {
  static readonly FullTypeName = '';

  requester: string;
  description: rmf_task_msgs.msg.TaskDescription;

  constructor(fields: Partial<SubmitTask_Request> = {}) {
    this.requester = fields.requester || '';
    this.description = fields.description || new rmf_task_msgs.msg.TaskDescription();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['requester'] !== 'string') {
      throw new Error('expected "requester" to be "string"');
    }
    try {
      rmf_task_msgs.msg.TaskDescription.validate(obj['description'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "description":\n  ' + (e as Error).message);
    }
  }
}

export default SubmitTask_Request;

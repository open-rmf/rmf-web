/* This is a generated file, do not edit */

import * as rmf_task_msgs from '../../rmf_task_msgs';

export class Tasks {
  static readonly FullTypeName = '';

  tasks: Array<rmf_task_msgs.msg.TaskSummary>;

  constructor(fields: Partial<Tasks> = {}) {
    this.tasks = fields.tasks || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['tasks'])) {
      throw new Error('expected "tasks" to be an array');
    }
    for (const [i, v] of obj['tasks'].entries()) {
      try {
        rmf_task_msgs.msg.TaskSummary.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "tasks":\n  ` + (e as Error).message);
      }
    }
  }
}

export default Tasks;

/* This is a generated file, do not edit */

import { TaskSummary } from '../../rmf_task_msgs/msg/TaskSummary';

export class GetTaskList_Response {
  static readonly FullTypeName = 'rmf_task_msgs/srv/GetTaskList_Response';

  success: boolean;
  active_tasks: TaskSummary[];
  terminated_tasks: TaskSummary[];

  constructor(fields: Partial<GetTaskList_Response> = {}) {
    this.success = fields.success || false;
    this.active_tasks = fields.active_tasks || [];
    this.terminated_tasks = fields.terminated_tasks || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['success'] !== 'boolean') {
      throw new Error('expected "success" to be "boolean"');
    }
    if (!Array.isArray(obj['active_tasks'])) {
      throw new Error('expected "active_tasks" to be an array');
    }
    for (const [i, v] of obj['active_tasks'].entries()) {
      try {
        TaskSummary.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "active_tasks":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['terminated_tasks'])) {
      throw new Error('expected "terminated_tasks" to be an array');
    }
    for (const [i, v] of obj['terminated_tasks'].entries()) {
      try {
        TaskSummary.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "terminated_tasks":\n  ` + (e as Error).message);
      }
    }
  }
}

/*


bool success

TaskSummary[] active_tasks
TaskSummary[] terminated_tasks

*/

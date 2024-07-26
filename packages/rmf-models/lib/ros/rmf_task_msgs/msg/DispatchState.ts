/* This is a generated file, do not edit */

import * as rmf_task_msgs from '../../rmf_task_msgs';

export class DispatchState {
  static readonly FullTypeName = '';

  static readonly STATUS_UNINITIALIZED = 0;
  static readonly STATUS_QUEUED = 1;
  static readonly STATUS_SELECTED = 2;
  static readonly STATUS_DISPATCHED = 3;
  static readonly STATUS_FAILED_TO_ASSIGN = 4;
  static readonly STATUS_CANCELED_IN_FLIGHT = 5;

  task_id: string;
  status: number;
  assignment: rmf_task_msgs.msg.Assignment;
  errors: Array<string>;

  constructor(fields: Partial<DispatchState> = {}) {
    this.task_id = fields.task_id || '';
    this.status = fields.status || 0;
    this.assignment = fields.assignment || new rmf_task_msgs.msg.Assignment();
    this.errors = fields.errors || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['status'] !== 'number') {
      throw new Error('expected "status" to be "number"');
    }
    try {
      rmf_task_msgs.msg.Assignment.validate(obj['assignment'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "assignment":\n  ' + (e as Error).message);
    }
    if (!Array.isArray(obj['errors'])) {
      throw new Error('expected "errors" to be an array');
    }
    for (const [i, v] of obj['errors'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "errors" to be "string"`);
      }
    }
  }
}

export default DispatchState;

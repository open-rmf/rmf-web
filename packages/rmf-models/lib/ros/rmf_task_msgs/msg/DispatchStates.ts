/* This is a generated file, do not edit */

import * as rmf_task_msgs from '../../rmf_task_msgs';

export class DispatchStates {
  static readonly FullTypeName = '';

  active: Array<rmf_task_msgs.msg.DispatchState>;
  finished: Array<rmf_task_msgs.msg.DispatchState>;

  constructor(fields: Partial<DispatchStates> = {}) {
    this.active = fields.active || [];
    this.finished = fields.finished || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['active'])) {
      throw new Error('expected "active" to be an array');
    }
    for (const [i, v] of obj['active'].entries()) {
      try {
        rmf_task_msgs.msg.DispatchState.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "active":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['finished'])) {
      throw new Error('expected "finished" to be an array');
    }
    for (const [i, v] of obj['finished'].entries()) {
      try {
        rmf_task_msgs.msg.DispatchState.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "finished":\n  ` + (e as Error).message);
      }
    }
  }
}

export default DispatchStates;

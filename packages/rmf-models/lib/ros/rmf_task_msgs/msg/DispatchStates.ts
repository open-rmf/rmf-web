/* This is a generated file, do not edit */

import { DispatchState } from '../../rmf_task_msgs/msg/DispatchState';

export class DispatchStates {
  static readonly FullTypeName = 'rmf_task_msgs/msg/DispatchStates';

  active: DispatchState[];
  finished: DispatchState[];

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
        DispatchState.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "active":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['finished'])) {
      throw new Error('expected "finished" to be an array');
    }
    for (const [i, v] of obj['finished'].entries()) {
      try {
        DispatchState.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "finished":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
# States of tasks that are currently in the process of being dispatched
DispatchState[] active

# States of tasks that have recently finished being dispatched. This may mean
# the task was assigned or it may mean it failed to be dispatched or was
# canceled before the dispatch took place.
DispatchState[] finished

*/

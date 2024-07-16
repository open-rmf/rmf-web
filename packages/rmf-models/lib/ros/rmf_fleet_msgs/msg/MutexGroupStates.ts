/* This is a generated file, do not edit */

import * as rmf_fleet_msgs from '../../rmf_fleet_msgs';

export class MutexGroupStates {
  static readonly FullTypeName = '';

  assignments: Array<rmf_fleet_msgs.msg.MutexGroupAssignment>;

  constructor(fields: Partial<MutexGroupStates> = {}) {
    this.assignments = fields.assignments || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['assignments'])) {
      throw new Error('expected "assignments" to be an array');
    }
    for (const [i, v] of obj['assignments'].entries()) {
      try {
        rmf_fleet_msgs.msg.MutexGroupAssignment.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "assignments":\n  ` + (e as Error).message);
      }
    }
  }
}

export default MutexGroupStates;

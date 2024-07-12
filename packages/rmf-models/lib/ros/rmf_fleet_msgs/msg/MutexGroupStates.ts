/* This is a generated file, do not edit */

import { MutexGroupAssignment } from '../../rmf_fleet_msgs/msg/MutexGroupAssignment';

export class MutexGroupStates {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/MutexGroupStates';

  assignments: MutexGroupAssignment[];

  constructor(fields: Partial<MutexGroupStates> = {}) {
    this.assignments = fields.assignments || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['assignments'])) {
      throw new Error('expected "assignments" to be an array');
    }
    for (const [i, v] of obj['assignments'].entries()) {
      try {
        MutexGroupAssignment.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "assignments":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
# A map of all the current mutex group assignments
MutexGroupAssignment[] assignments

*/

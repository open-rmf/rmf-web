/* This is a generated file, do not edit */

import { ChargingAssignment } from '../../rmf_fleet_msgs/msg/ChargingAssignment';

export class ChargingAssignments {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/ChargingAssignments';

  fleet_name: string;
  assignments: ChargingAssignment[];

  constructor(fields: Partial<ChargingAssignments> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.assignments = fields.assignments || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (!Array.isArray(obj['assignments'])) {
      throw new Error('expected "assignments" to be an array');
    }
    for (const [i, v] of obj['assignments'].entries()) {
      try {
        ChargingAssignment.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "assignments":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
string fleet_name
ChargingAssignment[] assignments

*/

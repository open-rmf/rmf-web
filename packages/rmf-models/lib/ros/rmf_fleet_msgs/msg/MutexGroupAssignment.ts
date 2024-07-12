/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';

export class MutexGroupAssignment {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/MutexGroupAssignment';

  group: string;
  claimant: number;
  claim_time: Time;

  constructor(fields: Partial<MutexGroupAssignment> = {}) {
    this.group = fields.group || '';
    this.claimant = fields.claimant || 0;
    this.claim_time = fields.claim_time || new Time();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['group'] !== 'string') {
      throw new Error('expected "group" to be "string"');
    }
    if (typeof obj['claimant'] !== 'number') {
      throw new Error('expected "claimant" to be "number"');
    }
    try {
      Time.validate(obj['claim_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "claim_time":\n  ' + (e as Error).message);
    }
  }
}

/*
# This message maps a mutex group name to the name of an agent that is currently
# holding the claim to that group.

# Name of the mutex group that is being described.
string group

# Traffic Participant ID of the agent that has currently claimed the group.
# If the group is unclaimed, this will be the max uint64 value.
uint64 claimant

# Time stamp of when the claim request began.
builtin_interfaces/Time claim_time

*/

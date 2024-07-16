/* This is a generated file, do not edit */

import * as rmf_door_msgs from '../../rmf_door_msgs';

export class SupervisorHeartbeat {
  static readonly FullTypeName = '';

  all_sessions: Array<rmf_door_msgs.msg.DoorSessions>;

  constructor(fields: Partial<SupervisorHeartbeat> = {}) {
    this.all_sessions = fields.all_sessions || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['all_sessions'])) {
      throw new Error('expected "all_sessions" to be an array');
    }
    for (const [i, v] of obj['all_sessions'].entries()) {
      try {
        rmf_door_msgs.msg.DoorSessions.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "all_sessions":\n  ` + (e as Error).message);
      }
    }
  }
}

export default SupervisorHeartbeat;

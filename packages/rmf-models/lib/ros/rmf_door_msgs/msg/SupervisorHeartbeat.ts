/* This is a generated file, do not edit */

import { DoorSessions } from '../../rmf_door_msgs/msg/DoorSessions';

export class SupervisorHeartbeat {
  static readonly FullTypeName = 'rmf_door_msgs/msg/SupervisorHeartbeat';

  all_sessions: DoorSessions[];

  constructor(fields: Partial<SupervisorHeartbeat> = {}) {
    this.all_sessions = fields.all_sessions || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['all_sessions'])) {
      throw new Error('expected "all_sessions" to be an array');
    }
    for (const [i, v] of obj['all_sessions'].entries()) {
      try {
        DoorSessions.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "all_sessions":\n  ` + (e as Error).message);
      }
    }
  }
}

/*

DoorSessions[] all_sessions

*/

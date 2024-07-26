/* This is a generated file, do not edit */

import * as rmf_door_msgs from '../../rmf_door_msgs';

export class DoorSessions {
  static readonly FullTypeName = '';

  door_name: string;
  sessions: Array<rmf_door_msgs.msg.Session>;

  constructor(fields: Partial<DoorSessions> = {}) {
    this.door_name = fields.door_name || '';
    this.sessions = fields.sessions || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['door_name'] !== 'string') {
      throw new Error('expected "door_name" to be "string"');
    }
    if (!Array.isArray(obj['sessions'])) {
      throw new Error('expected "sessions" to be an array');
    }
    for (const [i, v] of obj['sessions'].entries()) {
      try {
        rmf_door_msgs.msg.Session.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "sessions":\n  ` + (e as Error).message);
      }
    }
  }
}

export default DoorSessions;

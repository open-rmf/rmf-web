/* This is a generated file, do not edit */

import * as rmf_door_msgs from '../../rmf_door_msgs';
import * as builtin_interfaces from '../../builtin_interfaces';

export class DoorState {
  static readonly FullTypeName = '';

  door_time: builtin_interfaces.msg.Time;
  door_name: string;
  current_mode: rmf_door_msgs.msg.DoorMode;

  constructor(fields: Partial<DoorState> = {}) {
    this.door_time = fields.door_time || new builtin_interfaces.msg.Time();
    this.door_name = fields.door_name || '';
    this.current_mode = fields.current_mode || new rmf_door_msgs.msg.DoorMode();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      builtin_interfaces.msg.Time.validate(obj['door_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "door_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['door_name'] !== 'string') {
      throw new Error('expected "door_name" to be "string"');
    }
    try {
      rmf_door_msgs.msg.DoorMode.validate(obj['current_mode'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "current_mode":\n  ' + (e as Error).message);
    }
  }
}

export default DoorState;

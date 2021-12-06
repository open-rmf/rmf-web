/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';
import { DoorMode } from '../../rmf_door_msgs/msg/DoorMode';

export class DoorState {
  static readonly FullTypeName = 'rmf_door_msgs/msg/DoorState';

  door_time: Time;
  door_name: string;
  current_mode: DoorMode;

  constructor(fields: Partial<DoorState> = {}) {
    this.door_time = fields.door_time || new Time();
    this.door_name = fields.door_name || '';
    this.current_mode = fields.current_mode || new DoorMode();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['door_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "door_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['door_name'] !== 'string') {
      throw new Error('expected "door_name" to be "string"');
    }
    try {
      DoorMode.validate(obj['current_mode'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "current_mode":\n  ' + (e as Error).message);
    }
  }
}

/*
builtin_interfaces/Time door_time
string door_name
DoorMode current_mode

*/

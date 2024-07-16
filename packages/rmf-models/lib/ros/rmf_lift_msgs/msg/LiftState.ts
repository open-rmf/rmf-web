/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class LiftState {
  static readonly FullTypeName = '';

  static readonly DOOR_CLOSED = 0;
  static readonly DOOR_MOVING = 1;
  static readonly DOOR_OPEN = 2;
  static readonly MOTION_STOPPED = 0;
  static readonly MOTION_UP = 1;
  static readonly MOTION_DOWN = 2;
  static readonly MOTION_UNKNOWN = 3;
  static readonly MODE_UNKNOWN = 0;
  static readonly MODE_HUMAN = 1;
  static readonly MODE_AGV = 2;
  static readonly MODE_FIRE = 3;
  static readonly MODE_OFFLINE = 4;
  static readonly MODE_EMERGENCY = 5;

  lift_time: builtin_interfaces.msg.Time;
  lift_name: string;
  available_floors: Array<string>;
  current_floor: string;
  destination_floor: string;
  door_state: number;
  motion_state: number;
  available_modes: Uint8Array | number[];
  current_mode: number;
  session_id: string;

  constructor(fields: Partial<LiftState> = {}) {
    this.lift_time = fields.lift_time || new builtin_interfaces.msg.Time();
    this.lift_name = fields.lift_name || '';
    this.available_floors = fields.available_floors || [];
    this.current_floor = fields.current_floor || '';
    this.destination_floor = fields.destination_floor || '';
    this.door_state = fields.door_state || 0;
    this.motion_state = fields.motion_state || 0;
    this.available_modes = fields.available_modes || new Uint8Array(0);
    this.current_mode = fields.current_mode || 0;
    this.session_id = fields.session_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      builtin_interfaces.msg.Time.validate(obj['lift_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "lift_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['lift_name'] !== 'string') {
      throw new Error('expected "lift_name" to be "string"');
    }
    if (!Array.isArray(obj['available_floors'])) {
      throw new Error('expected "available_floors" to be an array');
    }
    for (const [i, v] of obj['available_floors'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "available_floors" to be "string"`);
      }
    }
    if (typeof obj['current_floor'] !== 'string') {
      throw new Error('expected "current_floor" to be "string"');
    }
    if (typeof obj['destination_floor'] !== 'string') {
      throw new Error('expected "destination_floor" to be "string"');
    }
    if (typeof obj['door_state'] !== 'number') {
      throw new Error('expected "door_state" to be "number"');
    }
    if (typeof obj['motion_state'] !== 'number') {
      throw new Error('expected "motion_state" to be "number"');
    }
    if (!(obj['available_modes'] instanceof Uint8Array) && !Array.isArray(obj['available_modes'])) {
      throw new Error('expected "available_modes" to be "Uint8Array" or an array');
    }
    if (Array.isArray(obj['available_modes'])) {
      for (const [i, v] of obj['available_modes'].entries()) {
        if (typeof v !== 'number') {
          throw new Error(`expected index ${i} of "available_modes" to be "number"`);
        }
      }
    } else if (!(obj['available_modes'] instanceof Uint8Array)) {
      throw new Error('"available_modes" must be either an array of number or Uint8Array');
    }
    if (typeof obj['current_mode'] !== 'number') {
      throw new Error('expected "current_mode" to be "number"');
    }
    if (typeof obj['session_id'] !== 'string') {
      throw new Error('expected "session_id" to be "string"');
    }
  }
}

export default LiftState;

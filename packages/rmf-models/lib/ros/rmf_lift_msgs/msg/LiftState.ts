/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';

export class LiftState {
  static readonly FullTypeName = 'rmf_lift_msgs/msg/LiftState';

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

  lift_time: Time;
  lift_name: string;
  available_floors: string[];
  current_floor: string;
  destination_floor: string;
  door_state: number;
  motion_state: number;
  available_modes: Uint8Array | number[];
  current_mode: number;
  session_id: string;

  constructor(fields: Partial<LiftState> = {}) {
    this.lift_time = fields.lift_time || new Time();
    this.lift_name = fields.lift_name || '';
    this.available_floors = fields.available_floors || [];
    this.current_floor = fields.current_floor || '';
    this.destination_floor = fields.destination_floor || '';
    this.door_state = fields.door_state || 0;
    this.motion_state = fields.motion_state || 0;
    this.available_modes = fields.available_modes || [];
    this.current_mode = fields.current_mode || 0;
    this.session_id = fields.session_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['lift_time'] as Record<string, unknown>);
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
    }
    if (typeof obj['current_mode'] !== 'number') {
      throw new Error('expected "current_mode" to be "number"');
    }
    if (typeof obj['session_id'] !== 'string') {
      throw new Error('expected "session_id" to be "string"');
    }
  }
}

/*
# lift_time records when the information in this message was generated
builtin_interfaces/Time lift_time

string lift_name

string[] available_floors
string current_floor
string destination_floor

uint8 door_state
uint8 DOOR_CLOSED=0
uint8 DOOR_MOVING=1
uint8 DOOR_OPEN=2

uint8 motion_state
uint8 MOTION_STOPPED=0
uint8 MOTION_UP=1
uint8 MOTION_DOWN=2
uint8 MOTION_UNKNOWN=3

# We can only set human or agv mode, but we can read other modes: fire, etc.
uint8[] available_modes
uint8 current_mode
uint8 MODE_UNKNOWN=0
uint8 MODE_HUMAN=1
uint8 MODE_AGV=2
uint8 MODE_FIRE=3
uint8 MODE_OFFLINE=4
uint8 MODE_EMERGENCY=5
# we can add more "read-only" modes as we come across more of them.

# this field records the session_id that has been granted control of the lift
# until it sends a request with a request_type of REQUEST_END_SESSION
string session_id

*/

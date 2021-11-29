/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';

export class LiftRequest {
  static readonly FullTypeName = 'rmf_lift_msgs/msg/LiftRequest';

  static readonly REQUEST_END_SESSION = 0;
  static readonly REQUEST_AGV_MODE = 1;
  static readonly REQUEST_HUMAN_MODE = 2;
  static readonly DOOR_CLOSED = 0;
  static readonly DOOR_OPEN = 2;

  lift_name: string;
  request_time: Time;
  session_id: string;
  request_type: number;
  destination_floor: string;
  door_state: number;

  constructor(fields: Partial<LiftRequest> = {}) {
    this.lift_name = fields.lift_name || '';
    this.request_time = fields.request_time || new Time();
    this.session_id = fields.session_id || '';
    this.request_type = fields.request_type || 0;
    this.destination_floor = fields.destination_floor || '';
    this.door_state = fields.door_state || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['lift_name'] !== 'string') {
      throw new Error('expected "lift_name" to be "string"');
    }
    try {
      Time.validate(obj['request_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "request_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['session_id'] !== 'string') {
      throw new Error('expected "session_id" to be "string"');
    }
    if (typeof obj['request_type'] !== 'number') {
      throw new Error('expected "request_type" to be "number"');
    }
    if (typeof obj['destination_floor'] !== 'string') {
      throw new Error('expected "destination_floor" to be "string"');
    }
    if (typeof obj['door_state'] !== 'number') {
      throw new Error('expected "door_state" to be "number"');
    }
  }
}

/*
string lift_name
builtin_interfaces/Time request_time

# session_id should be unique at least between different requesters.
# For example, session_id could be the requester's node name.
string session_id

# AGV mode means that the doors are always open when the lift is stopped
# Human mode means that LiftDoorRequest messages must be used to open/close
# the doors explicitly, since they may "time out" and close automatically.
uint8 request_type
uint8 REQUEST_END_SESSION=0
uint8 REQUEST_AGV_MODE=1
uint8 REQUEST_HUMAN_MODE=2

# The destination_floor must be one of the values returned in a LiftState.
string destination_floor

# Explicit door requests are necessary in "human" mode to open/close doors.
# Door requests are not necessary in "AGV" mode, when the doors are always
# held open when the lift cabin is stopped.
uint8 door_state
uint8 DOOR_CLOSED=0
uint8 DOOR_OPEN=2

*/

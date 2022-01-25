/* This is a generated file, do not edit */

export class RobotMode {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/RobotMode';

  static readonly MODE_IDLE = 0;
  static readonly MODE_CHARGING = 1;
  static readonly MODE_MOVING = 2;
  static readonly MODE_PAUSED = 3;
  static readonly MODE_WAITING = 4;
  static readonly MODE_EMERGENCY = 5;
  static readonly MODE_GOING_HOME = 6;
  static readonly MODE_DOCKING = 7;
  static readonly MODE_ADAPTER_ERROR = 8;
  static readonly MODE_CLEANING = 9;

  mode: number;
  mode_request_id: number;

  constructor(fields: Partial<RobotMode> = {}) {
    this.mode = fields.mode || 0;
    this.mode_request_id = fields.mode_request_id || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['mode'] !== 'number') {
      throw new Error('expected "mode" to be "number"');
    }
    if (typeof obj['mode_request_id'] !== 'number') {
      throw new Error('expected "mode_request_id" to be "number"');
    }
  }
}

/*
uint32 mode
uint32 MODE_IDLE=0
uint32 MODE_CHARGING=1
uint32 MODE_MOVING=2
uint32 MODE_PAUSED=3
uint32 MODE_WAITING=4
uint32 MODE_EMERGENCY=5
uint32 MODE_GOING_HOME=6
uint32 MODE_DOCKING=7

# Use this when a command received from the fleet adapter
# has a problem and needs to be recomputed.
uint32 MODE_ADAPTER_ERROR=8

uint32 MODE_CLEANING=9

uint64 mode_request_id

*/

/* This is a generated file, do not edit */

export class PauseRequest {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/PauseRequest';

  static readonly TYPE_PAUSE_IMMEDIATELY = 0;
  static readonly TYPE_PAUSE_AT_CHECKPOINT = 1;
  static readonly TYPE_RESUME = 2;

  fleet_name: string;
  robot_name: string;
  mode_request_id: number;
  type: number;
  at_checkpoint: number;

  constructor(fields: Partial<PauseRequest> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.robot_name = fields.robot_name || '';
    this.mode_request_id = fields.mode_request_id || 0;
    this.type = fields.type || 0;
    this.at_checkpoint = fields.at_checkpoint || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
    if (typeof obj['mode_request_id'] !== 'number') {
      throw new Error('expected "mode_request_id" to be "number"');
    }
    if (typeof obj['type'] !== 'number') {
      throw new Error('expected "type" to be "number"');
    }
    if (typeof obj['at_checkpoint'] !== 'number') {
      throw new Error('expected "at_checkpoint" to be "number"');
    }
  }
}

/*
string fleet_name
string robot_name
uint64 mode_request_id

uint32 TYPE_PAUSE_IMMEDIATELY=0
uint32 TYPE_PAUSE_AT_CHECKPOINT=1
uint32 TYPE_RESUME=2
uint32 type

uint32 at_checkpoint

*/

/* This is a generated file, do not edit */

export class RobotMode {
  static readonly FullTypeName = '';

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
  static readonly MODE_PERFORMING_ACTION = 10;
  static readonly MODE_ACTION_COMPLETED = 11;

  mode: number;
  mode_request_id: number;
  performing_action: string;

  constructor(fields: Partial<RobotMode> = {}) {
    this.mode = fields.mode || 0;
    this.mode_request_id = fields.mode_request_id || 0;
    this.performing_action = fields.performing_action || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['mode'] !== 'number') {
      throw new Error('expected "mode" to be "number"');
    }
    if (typeof obj['mode_request_id'] !== 'number') {
      throw new Error('expected "mode_request_id" to be "number"');
    }
    if (typeof obj['performing_action'] !== 'string') {
      throw new Error('expected "performing_action" to be "string"');
    }
  }
}

export default RobotMode;

/* This is a generated file, do not edit */

export class ChargingAssignment {
  static readonly FullTypeName = '';

  static readonly MODE_CHARGE = 0;
  static readonly MODE_WAIT = 1;

  robot_name: string;
  waypoint_name: string;
  mode: number;

  constructor(fields: Partial<ChargingAssignment> = {}) {
    this.robot_name = fields.robot_name || '';
    this.waypoint_name = fields.waypoint_name || '';
    this.mode = fields.mode || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
    if (typeof obj['waypoint_name'] !== 'string') {
      throw new Error('expected "waypoint_name" to be "string"');
    }
    if (typeof obj['mode'] !== 'number') {
      throw new Error('expected "mode" to be "number"');
    }
  }
}

export default ChargingAssignment;

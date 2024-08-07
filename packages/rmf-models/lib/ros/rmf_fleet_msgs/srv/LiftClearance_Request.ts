/* This is a generated file, do not edit */

export class LiftClearance_Request {
  static readonly FullTypeName = '';

  robot_name: string;
  lift_name: string;

  constructor(fields: Partial<LiftClearance_Request> = {}) {
    this.robot_name = fields.robot_name || '';
    this.lift_name = fields.lift_name || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
    if (typeof obj['lift_name'] !== 'string') {
      throw new Error('expected "lift_name" to be "string"');
    }
  }
}

export default LiftClearance_Request;

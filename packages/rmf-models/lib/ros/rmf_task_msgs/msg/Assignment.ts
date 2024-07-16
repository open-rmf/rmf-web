/* This is a generated file, do not edit */

export class Assignment {
  static readonly FullTypeName = '';

  is_assigned: boolean;
  fleet_name: string;
  expected_robot_name: string;

  constructor(fields: Partial<Assignment> = {}) {
    this.is_assigned = fields.is_assigned || false;
    this.fleet_name = fields.fleet_name || '';
    this.expected_robot_name = fields.expected_robot_name || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['is_assigned'] !== 'boolean') {
      throw new Error('expected "is_assigned" to be "boolean"');
    }
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['expected_robot_name'] !== 'string') {
      throw new Error('expected "expected_robot_name" to be "string"');
    }
  }
}

export default Assignment;

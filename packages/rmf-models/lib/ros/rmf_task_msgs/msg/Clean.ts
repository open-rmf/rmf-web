/* This is a generated file, do not edit */

export class Clean {
  static readonly FullTypeName = '';

  start_waypoint: string;

  constructor(fields: Partial<Clean> = {}) {
    this.start_waypoint = fields.start_waypoint || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['start_waypoint'] !== 'string') {
      throw new Error('expected "start_waypoint" to be "string"');
    }
  }
}

export default Clean;

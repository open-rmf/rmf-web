/* This is a generated file, do not edit */

export class Door {
  static readonly FullTypeName = '';

  static readonly DOOR_TYPE_UNDEFINED = 0;
  static readonly DOOR_TYPE_SINGLE_SLIDING = 1;
  static readonly DOOR_TYPE_DOUBLE_SLIDING = 2;
  static readonly DOOR_TYPE_SINGLE_TELESCOPE = 3;
  static readonly DOOR_TYPE_DOUBLE_TELESCOPE = 4;
  static readonly DOOR_TYPE_SINGLE_SWING = 5;
  static readonly DOOR_TYPE_DOUBLE_SWING = 6;

  name: string;
  v1_x: number;
  v1_y: number;
  v2_x: number;
  v2_y: number;
  door_type: number;
  motion_range: number;
  motion_direction: number;

  constructor(fields: Partial<Door> = {}) {
    this.name = fields.name || '';
    this.v1_x = fields.v1_x || 0;
    this.v1_y = fields.v1_y || 0;
    this.v2_x = fields.v2_x || 0;
    this.v2_y = fields.v2_y || 0;
    this.door_type = fields.door_type || 0;
    this.motion_range = fields.motion_range || 0;
    this.motion_direction = fields.motion_direction || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (typeof obj['v1_x'] !== 'number') {
      throw new Error('expected "v1_x" to be "number"');
    }
    if (typeof obj['v1_y'] !== 'number') {
      throw new Error('expected "v1_y" to be "number"');
    }
    if (typeof obj['v2_x'] !== 'number') {
      throw new Error('expected "v2_x" to be "number"');
    }
    if (typeof obj['v2_y'] !== 'number') {
      throw new Error('expected "v2_y" to be "number"');
    }
    if (typeof obj['door_type'] !== 'number') {
      throw new Error('expected "door_type" to be "number"');
    }
    if (typeof obj['motion_range'] !== 'number') {
      throw new Error('expected "motion_range" to be "number"');
    }
    if (typeof obj['motion_direction'] !== 'number') {
      throw new Error('expected "motion_direction" to be "number"');
    }
  }
}

export default Door;

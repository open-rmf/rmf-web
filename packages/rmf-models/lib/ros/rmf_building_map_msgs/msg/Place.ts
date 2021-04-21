/* This is a generated file, do not edit */

export class Place {
  static readonly FullTypeName = 'rmf_building_map_msgs/msg/Place';

  name: string;
  x: number;
  y: number;
  yaw: number;
  position_tolerance: number;
  yaw_tolerance: number;

  constructor(fields: Partial<Place> = {}) {
    this.name = fields.name || '';
    this.x = fields.x || 0;
    this.y = fields.y || 0;
    this.yaw = fields.yaw || 0;
    this.position_tolerance = fields.position_tolerance || 0;
    this.yaw_tolerance = fields.yaw_tolerance || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (typeof obj['x'] !== 'number') {
      throw new Error('expected "x" to be "number"');
    }
    if (typeof obj['y'] !== 'number') {
      throw new Error('expected "y" to be "number"');
    }
    if (typeof obj['yaw'] !== 'number') {
      throw new Error('expected "yaw" to be "number"');
    }
    if (typeof obj['position_tolerance'] !== 'number') {
      throw new Error('expected "position_tolerance" to be "number"');
    }
    if (typeof obj['yaw_tolerance'] !== 'number') {
      throw new Error('expected "yaw_tolerance" to be "number"');
    }
  }
}

/*
string name
float32 x
float32 y
float32 yaw
float32 position_tolerance
float32 yaw_tolerance

*/

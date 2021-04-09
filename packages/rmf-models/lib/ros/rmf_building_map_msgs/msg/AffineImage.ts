/* This is a generated file, do not edit */

export class AffineImage {
  static readonly FullTypeName = 'rmf_building_map_msgs/msg/AffineImage';

  name: string;
  x_offset: number;
  y_offset: number;
  yaw: number;
  scale: number;
  encoding: string;
  data: Uint8Array | number[];

  constructor(fields: Partial<AffineImage> = {}) {
    this.name = fields.name || '';
    this.x_offset = fields.x_offset || 0;
    this.y_offset = fields.y_offset || 0;
    this.yaw = fields.yaw || 0;
    this.scale = fields.scale || 0;
    this.encoding = fields.encoding || '';
    this.data = fields.data || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (typeof obj['x_offset'] !== 'number') {
      throw new Error('expected "x_offset" to be "number"');
    }
    if (typeof obj['y_offset'] !== 'number') {
      throw new Error('expected "y_offset" to be "number"');
    }
    if (typeof obj['yaw'] !== 'number') {
      throw new Error('expected "yaw" to be "number"');
    }
    if (typeof obj['scale'] !== 'number') {
      throw new Error('expected "scale" to be "number"');
    }
    if (typeof obj['encoding'] !== 'string') {
      throw new Error('expected "encoding" to be "string"');
    }
    if (!(obj['data'] instanceof Uint8Array) && !Array.isArray(obj['data'])) {
      throw new Error('expected "data" to be "Uint8Array" or an array');
    }
    if (Array.isArray(obj['data'])) {
      for (const [i, v] of obj['data'].entries()) {
        if (typeof v !== 'number') {
          throw new Error(`expected index ${i} of "data" to be "number"`);
        }
      }
    }
  }
}

/*
string name
float32 x_offset
float32 y_offset
float32 yaw
float32 scale
string encoding
uint8[] data

*/

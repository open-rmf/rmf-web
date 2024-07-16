/* This is a generated file, do not edit */

import * as rmf_building_map_msgs from '../../rmf_building_map_msgs';

export class GraphNode {
  static readonly FullTypeName = '';

  x: number;
  y: number;
  name: string;
  params: Array<rmf_building_map_msgs.msg.Param>;

  constructor(fields: Partial<GraphNode> = {}) {
    this.x = fields.x || 0;
    this.y = fields.y || 0;
    this.name = fields.name || '';
    this.params = fields.params || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['x'] !== 'number') {
      throw new Error('expected "x" to be "number"');
    }
    if (typeof obj['y'] !== 'number') {
      throw new Error('expected "y" to be "number"');
    }
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (!Array.isArray(obj['params'])) {
      throw new Error('expected "params" to be an array');
    }
    for (const [i, v] of obj['params'].entries()) {
      try {
        rmf_building_map_msgs.msg.Param.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "params":\n  ` + (e as Error).message);
      }
    }
  }
}

export default GraphNode;

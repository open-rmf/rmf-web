/* This is a generated file, do not edit */

import * as rmf_building_map_msgs from '../../rmf_building_map_msgs';

export class Graph {
  static readonly FullTypeName = '';

  name: string;
  vertices: Array<rmf_building_map_msgs.msg.GraphNode>;
  edges: Array<rmf_building_map_msgs.msg.GraphEdge>;
  params: Array<rmf_building_map_msgs.msg.Param>;

  constructor(fields: Partial<Graph> = {}) {
    this.name = fields.name || '';
    this.vertices = fields.vertices || [];
    this.edges = fields.edges || [];
    this.params = fields.params || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (!Array.isArray(obj['vertices'])) {
      throw new Error('expected "vertices" to be an array');
    }
    for (const [i, v] of obj['vertices'].entries()) {
      try {
        rmf_building_map_msgs.msg.GraphNode.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "vertices":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['edges'])) {
      throw new Error('expected "edges" to be an array');
    }
    for (const [i, v] of obj['edges'].entries()) {
      try {
        rmf_building_map_msgs.msg.GraphEdge.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "edges":\n  ` + (e as Error).message);
      }
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

export default Graph;

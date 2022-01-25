/* This is a generated file, do not edit */

import { Param } from '../../rmf_building_map_msgs/msg/Param';

export class GraphEdge {
  static readonly FullTypeName = 'rmf_building_map_msgs/msg/GraphEdge';

  static readonly EDGE_TYPE_BIDIRECTIONAL = 0;
  static readonly EDGE_TYPE_UNIDIRECTIONAL = 1;

  v1_idx: number;
  v2_idx: number;
  params: Param[];
  edge_type: number;

  constructor(fields: Partial<GraphEdge> = {}) {
    this.v1_idx = fields.v1_idx || 0;
    this.v2_idx = fields.v2_idx || 0;
    this.params = fields.params || [];
    this.edge_type = fields.edge_type || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['v1_idx'] !== 'number') {
      throw new Error('expected "v1_idx" to be "number"');
    }
    if (typeof obj['v2_idx'] !== 'number') {
      throw new Error('expected "v2_idx" to be "number"');
    }
    if (!Array.isArray(obj['params'])) {
      throw new Error('expected "params" to be an array');
    }
    for (const [i, v] of obj['params'].entries()) {
      try {
        Param.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "params":\n  ` + (e as Error).message);
      }
    }
    if (typeof obj['edge_type'] !== 'number') {
      throw new Error('expected "edge_type" to be "number"');
    }
  }
}

/*
uint32 v1_idx
uint32 v2_idx
Param[] params

# when edge_type is UNIDIRECTIONAL, it means v1 -> v2
# when edge_type is BIDIRECTIONAL, it means v1 <-> v2
uint8 edge_type
uint8 EDGE_TYPE_BIDIRECTIONAL=0
uint8 EDGE_TYPE_UNIDIRECTIONAL=1

*/

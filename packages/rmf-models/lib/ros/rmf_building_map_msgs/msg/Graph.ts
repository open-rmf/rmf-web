/* This is a generated file, do not edit */

import { GraphNode } from '../../rmf_building_map_msgs/msg/GraphNode';
import { GraphEdge } from '../../rmf_building_map_msgs/msg/GraphEdge';
import { Param } from '../../rmf_building_map_msgs/msg/Param';

export class Graph {
  static readonly FullTypeName = 'rmf_building_map_msgs/msg/Graph';

  name: string;
  vertices: GraphNode[];
  edges: GraphEdge[];
  params: Param[];

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
        GraphNode.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "vertices":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['edges'])) {
      throw new Error('expected "edges" to be an array');
    }
    for (const [i, v] of obj['edges'].entries()) {
      try {
        GraphEdge.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "edges":\n  ` + (e as Error).message);
      }
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
  }
}

/*
string name
GraphNode[] vertices
GraphEdge[] edges
Param[] params

*/

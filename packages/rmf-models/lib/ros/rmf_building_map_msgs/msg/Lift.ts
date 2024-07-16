/* This is a generated file, do not edit */

import * as rmf_building_map_msgs from '../../rmf_building_map_msgs';

export class Lift {
  static readonly FullTypeName = '';

  name: string;
  levels: Array<string>;
  doors: Array<rmf_building_map_msgs.msg.Door>;
  wall_graph: rmf_building_map_msgs.msg.Graph;
  ref_x: number;
  ref_y: number;
  ref_yaw: number;
  width: number;
  depth: number;

  constructor(fields: Partial<Lift> = {}) {
    this.name = fields.name || '';
    this.levels = fields.levels || [];
    this.doors = fields.doors || [];
    this.wall_graph = fields.wall_graph || new rmf_building_map_msgs.msg.Graph();
    this.ref_x = fields.ref_x || 0;
    this.ref_y = fields.ref_y || 0;
    this.ref_yaw = fields.ref_yaw || 0;
    this.width = fields.width || 0;
    this.depth = fields.depth || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (!Array.isArray(obj['levels'])) {
      throw new Error('expected "levels" to be an array');
    }
    for (const [i, v] of obj['levels'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "levels" to be "string"`);
      }
    }
    if (!Array.isArray(obj['doors'])) {
      throw new Error('expected "doors" to be an array');
    }
    for (const [i, v] of obj['doors'].entries()) {
      try {
        rmf_building_map_msgs.msg.Door.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "doors":\n  ` + (e as Error).message);
      }
    }
    try {
      rmf_building_map_msgs.msg.Graph.validate(obj['wall_graph'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "wall_graph":\n  ' + (e as Error).message);
    }
    if (typeof obj['ref_x'] !== 'number') {
      throw new Error('expected "ref_x" to be "number"');
    }
    if (typeof obj['ref_y'] !== 'number') {
      throw new Error('expected "ref_y" to be "number"');
    }
    if (typeof obj['ref_yaw'] !== 'number') {
      throw new Error('expected "ref_yaw" to be "number"');
    }
    if (typeof obj['width'] !== 'number') {
      throw new Error('expected "width" to be "number"');
    }
    if (typeof obj['depth'] !== 'number') {
      throw new Error('expected "depth" to be "number"');
    }
  }
}

export default Lift;

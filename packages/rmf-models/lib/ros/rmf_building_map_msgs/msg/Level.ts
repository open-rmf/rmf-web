/* This is a generated file, do not edit */

import * as rmf_building_map_msgs from '../../rmf_building_map_msgs';

export class Level {
  static readonly FullTypeName = '';

  name: string;
  elevation: number;
  images: Array<rmf_building_map_msgs.msg.AffineImage>;
  places: Array<rmf_building_map_msgs.msg.Place>;
  doors: Array<rmf_building_map_msgs.msg.Door>;
  nav_graphs: Array<rmf_building_map_msgs.msg.Graph>;
  wall_graph: rmf_building_map_msgs.msg.Graph;

  constructor(fields: Partial<Level> = {}) {
    this.name = fields.name || '';
    this.elevation = fields.elevation || 0;
    this.images = fields.images || [];
    this.places = fields.places || [];
    this.doors = fields.doors || [];
    this.nav_graphs = fields.nav_graphs || [];
    this.wall_graph = fields.wall_graph || new rmf_building_map_msgs.msg.Graph();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (typeof obj['elevation'] !== 'number') {
      throw new Error('expected "elevation" to be "number"');
    }
    if (!Array.isArray(obj['images'])) {
      throw new Error('expected "images" to be an array');
    }
    for (const [i, v] of obj['images'].entries()) {
      try {
        rmf_building_map_msgs.msg.AffineImage.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "images":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['places'])) {
      throw new Error('expected "places" to be an array');
    }
    for (const [i, v] of obj['places'].entries()) {
      try {
        rmf_building_map_msgs.msg.Place.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "places":\n  ` + (e as Error).message);
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
    if (!Array.isArray(obj['nav_graphs'])) {
      throw new Error('expected "nav_graphs" to be an array');
    }
    for (const [i, v] of obj['nav_graphs'].entries()) {
      try {
        rmf_building_map_msgs.msg.Graph.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "nav_graphs":\n  ` + (e as Error).message);
      }
    }
    try {
      rmf_building_map_msgs.msg.Graph.validate(obj['wall_graph'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "wall_graph":\n  ' + (e as Error).message);
    }
  }
}

export default Level;

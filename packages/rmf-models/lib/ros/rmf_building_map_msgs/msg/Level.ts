/* This is a generated file, do not edit */

import { AffineImage } from '../../rmf_building_map_msgs/msg/AffineImage';
import { Place } from '../../rmf_building_map_msgs/msg/Place';
import { Door } from '../../rmf_building_map_msgs/msg/Door';
import { Graph } from '../../rmf_building_map_msgs/msg/Graph';

export class Level {
  static readonly FullTypeName = 'rmf_building_map_msgs/msg/Level';

  name: string;
  elevation: number;
  images: AffineImage[];
  places: Place[];
  doors: Door[];
  nav_graphs: Graph[];
  wall_graph: Graph;

  constructor(fields: Partial<Level> = {}) {
    this.name = fields.name || '';
    this.elevation = fields.elevation || 0;
    this.images = fields.images || [];
    this.places = fields.places || [];
    this.doors = fields.doors || [];
    this.nav_graphs = fields.nav_graphs || [];
    this.wall_graph = fields.wall_graph || new Graph();
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
        AffineImage.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "images":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['places'])) {
      throw new Error('expected "places" to be an array');
    }
    for (const [i, v] of obj['places'].entries()) {
      try {
        Place.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "places":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['doors'])) {
      throw new Error('expected "doors" to be an array');
    }
    for (const [i, v] of obj['doors'].entries()) {
      try {
        Door.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "doors":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['nav_graphs'])) {
      throw new Error('expected "nav_graphs" to be an array');
    }
    for (const [i, v] of obj['nav_graphs'].entries()) {
      try {
        Graph.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "nav_graphs":\n  ` + (e as Error).message);
      }
    }
    try {
      Graph.validate(obj['wall_graph'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "wall_graph":\n  ' + (e as Error).message);
    }
  }
}

/*
string name
float32 elevation
AffineImage[] images
Place[] places
Door[] doors
Graph[] nav_graphs
Graph wall_graph

*/

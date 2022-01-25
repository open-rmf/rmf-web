/* This is a generated file, do not edit */

import { Level } from '../../rmf_building_map_msgs/msg/Level';
import { Lift } from '../../rmf_building_map_msgs/msg/Lift';

export class BuildingMap {
  static readonly FullTypeName = 'rmf_building_map_msgs/msg/BuildingMap';

  name: string;
  levels: Level[];
  lifts: Lift[];

  constructor(fields: Partial<BuildingMap> = {}) {
    this.name = fields.name || '';
    this.levels = fields.levels || [];
    this.lifts = fields.lifts || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (!Array.isArray(obj['levels'])) {
      throw new Error('expected "levels" to be an array');
    }
    for (const [i, v] of obj['levels'].entries()) {
      try {
        Level.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "levels":\n  ` + (e as Error).message);
      }
    }
    if (!Array.isArray(obj['lifts'])) {
      throw new Error('expected "lifts" to be an array');
    }
    for (const [i, v] of obj['lifts'].entries()) {
      try {
        Lift.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "lifts":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
string name
Level[] levels
Lift[] lifts

*/

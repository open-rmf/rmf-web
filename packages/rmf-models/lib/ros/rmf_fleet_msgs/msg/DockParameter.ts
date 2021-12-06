/* This is a generated file, do not edit */

import { Location } from '../../rmf_fleet_msgs/msg/Location';

export class DockParameter {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/DockParameter';

  start: string;
  finish: string;
  path: Location[];

  constructor(fields: Partial<DockParameter> = {}) {
    this.start = fields.start || '';
    this.finish = fields.finish || '';
    this.path = fields.path || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['start'] !== 'string') {
      throw new Error('expected "start" to be "string"');
    }
    if (typeof obj['finish'] !== 'string') {
      throw new Error('expected "finish" to be "string"');
    }
    if (!Array.isArray(obj['path'])) {
      throw new Error('expected "path" to be an array');
    }
    for (const [i, v] of obj['path'].entries()) {
      try {
        Location.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "path":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
# The name of the waypoint where the docking begins
string start

# The name of the waypoint where the docking ends
string finish

# The points in the docking path
Location[] path
*/

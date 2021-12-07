/* This is a generated file, do not edit */

import { DockParameter } from '../../rmf_fleet_msgs/msg/DockParameter';

export class Dock {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/Dock';

  fleet_name: string;
  params: DockParameter[];

  constructor(fields: Partial<Dock> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.params = fields.params || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (!Array.isArray(obj['params'])) {
      throw new Error('expected "params" to be an array');
    }
    for (const [i, v] of obj['params'].entries()) {
      try {
        DockParameter.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "params":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
string fleet_name
DockParameter[] params
*/

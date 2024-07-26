/* This is a generated file, do not edit */

import * as rmf_fleet_msgs from '../../rmf_fleet_msgs';

export class Dock {
  static readonly FullTypeName = '';

  fleet_name: string;
  params: Array<rmf_fleet_msgs.msg.DockParameter>;

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
        rmf_fleet_msgs.msg.DockParameter.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "params":\n  ` + (e as Error).message);
      }
    }
  }
}

export default Dock;

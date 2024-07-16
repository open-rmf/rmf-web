/* This is a generated file, do not edit */

import * as rmf_fleet_msgs from '../../rmf_fleet_msgs';

export class DockSummary {
  static readonly FullTypeName = '';

  docks: Array<rmf_fleet_msgs.msg.Dock>;

  constructor(fields: Partial<DockSummary> = {}) {
    this.docks = fields.docks || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['docks'])) {
      throw new Error('expected "docks" to be an array');
    }
    for (const [i, v] of obj['docks'].entries()) {
      try {
        rmf_fleet_msgs.msg.Dock.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "docks":\n  ` + (e as Error).message);
      }
    }
  }
}

export default DockSummary;

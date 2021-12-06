/* This is a generated file, do not edit */

import { Dock } from '../../rmf_fleet_msgs/msg/Dock';

export class DockSummary {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/DockSummary';

  docks: Dock[];

  constructor(fields: Partial<DockSummary> = {}) {
    this.docks = fields.docks || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['docks'])) {
      throw new Error('expected "docks" to be an array');
    }
    for (const [i, v] of obj['docks'].entries()) {
      try {
        Dock.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "docks":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
Dock[] docks
*/

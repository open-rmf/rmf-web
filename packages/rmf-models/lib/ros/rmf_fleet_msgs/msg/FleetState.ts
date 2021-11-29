/* This is a generated file, do not edit */

import { RobotState } from '../../rmf_fleet_msgs/msg/RobotState';

export class FleetState {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/FleetState';

  name: string;
  robots: RobotState[];

  constructor(fields: Partial<FleetState> = {}) {
    this.name = fields.name || '';
    this.robots = fields.robots || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (!Array.isArray(obj['robots'])) {
      throw new Error('expected "robots" to be an array');
    }
    for (const [i, v] of obj['robots'].entries()) {
      try {
        RobotState.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "robots":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
string name
RobotState[] robots

*/

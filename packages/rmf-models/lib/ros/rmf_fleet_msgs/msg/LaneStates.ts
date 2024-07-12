/* This is a generated file, do not edit */

import { SpeedLimitedLane } from '../../rmf_fleet_msgs/msg/SpeedLimitedLane';

export class LaneStates {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/LaneStates';

  fleet_name: string;
  closed_lanes: BigUint64Array | number[];
  speed_limits: SpeedLimitedLane[];

  constructor(fields: Partial<LaneStates> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.closed_lanes = fields.closed_lanes || [];
    this.speed_limits = fields.speed_limits || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (!(obj['closed_lanes'] instanceof BigUint64Array) && !Array.isArray(obj['closed_lanes'])) {
      throw new Error('expected "closed_lanes" to be "BigUint64Array" or an array');
    }
    if (Array.isArray(obj['closed_lanes'])) {
      for (const [i, v] of obj['closed_lanes'].entries()) {
        if (typeof v !== 'number') {
          throw new Error(`expected index ${i} of "closed_lanes" to be "number"`);
        }
      }
    }
    if (!Array.isArray(obj['speed_limits'])) {
      throw new Error('expected "speed_limits" to be an array');
    }
    for (const [i, v] of obj['speed_limits'].entries()) {
      try {
        SpeedLimitedLane.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "speed_limits":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
# The name of the fleet with closed or speed limited lanes
string fleet_name

# The indices of the lanes that are currently closed
uint64[] closed_lanes

# Lanes that have speed limits
SpeedLimitedLane[] speed_limits

*/

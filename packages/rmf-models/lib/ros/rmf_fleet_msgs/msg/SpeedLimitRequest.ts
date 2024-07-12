/* This is a generated file, do not edit */

import { SpeedLimitedLane } from '../../rmf_fleet_msgs/msg/SpeedLimitedLane';

export class SpeedLimitRequest {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/SpeedLimitRequest';

  fleet_name: string;
  speed_limits: SpeedLimitedLane[];
  remove_limits: BigUint64Array | number[];

  constructor(fields: Partial<SpeedLimitRequest> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.speed_limits = fields.speed_limits || [];
    this.remove_limits = fields.remove_limits || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
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
    if (!(obj['remove_limits'] instanceof BigUint64Array) && !Array.isArray(obj['remove_limits'])) {
      throw new Error('expected "remove_limits" to be "BigUint64Array" or an array');
    }
    if (Array.isArray(obj['remove_limits'])) {
      for (const [i, v] of obj['remove_limits'].entries()) {
        if (typeof v !== 'number') {
          throw new Error(`expected index ${i} of "remove_limits" to be "number"`);
        }
      }
    }
  }
}

/*
# The name of the fleet
string fleet_name

# The lanes to impose speed limits upon.
SpeedLimitedLane[] speed_limits

# The indices of lanes to remove speed limits
uint64[] remove_limits

*/

/* This is a generated file, do not edit */

import * as rmf_fleet_msgs from '../../rmf_fleet_msgs';

export class LaneStates {
  static readonly FullTypeName = '';

  fleet_name: string;
  closed_lanes: BigUint64Array | number[];
  speed_limits: Array<rmf_fleet_msgs.msg.SpeedLimitedLane>;

  constructor(fields: Partial<LaneStates> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.closed_lanes = fields.closed_lanes || new BigUint64Array(0);
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
    } else if (!(obj['closed_lanes'] instanceof BigUint64Array)) {
      throw new Error('"closed_lanes" must be either an array of number or BigUint64Array');
    }
    if (!Array.isArray(obj['speed_limits'])) {
      throw new Error('expected "speed_limits" to be an array');
    }
    for (const [i, v] of obj['speed_limits'].entries()) {
      try {
        rmf_fleet_msgs.msg.SpeedLimitedLane.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "speed_limits":\n  ` + (e as Error).message);
      }
    }
  }
}

export default LaneStates;

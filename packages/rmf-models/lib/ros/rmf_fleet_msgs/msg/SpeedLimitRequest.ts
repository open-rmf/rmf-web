/* This is a generated file, do not edit */

import * as rmf_fleet_msgs from '../../rmf_fleet_msgs';

export class SpeedLimitRequest {
  static readonly FullTypeName = '';

  fleet_name: string;
  speed_limits: Array<rmf_fleet_msgs.msg.SpeedLimitedLane>;
  remove_limits: BigUint64Array | number[];

  constructor(fields: Partial<SpeedLimitRequest> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.speed_limits = fields.speed_limits || [];
    this.remove_limits = fields.remove_limits || new BigUint64Array(0);
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
        rmf_fleet_msgs.msg.SpeedLimitedLane.validate(v);
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
    } else if (!(obj['remove_limits'] instanceof BigUint64Array)) {
      throw new Error('"remove_limits" must be either an array of number or BigUint64Array');
    }
  }
}

export default SpeedLimitRequest;

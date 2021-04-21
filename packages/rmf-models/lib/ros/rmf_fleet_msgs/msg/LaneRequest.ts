/* This is a generated file, do not edit */

export class LaneRequest {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/LaneRequest';

  fleet_name: string;
  open_lanes: BigUint64Array | number[];
  close_lanes: BigUint64Array | number[];

  constructor(fields: Partial<LaneRequest> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.open_lanes = fields.open_lanes || [];
    this.close_lanes = fields.close_lanes || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (!(obj['open_lanes'] instanceof BigUint64Array) && !Array.isArray(obj['open_lanes'])) {
      throw new Error('expected "open_lanes" to be "BigUint64Array" or an array');
    }
    if (Array.isArray(obj['open_lanes'])) {
      for (const [i, v] of obj['open_lanes'].entries()) {
        if (typeof v !== 'number') {
          throw new Error(`expected index ${i} of "open_lanes" to be "number"`);
        }
      }
    }
    if (!(obj['close_lanes'] instanceof BigUint64Array) && !Array.isArray(obj['close_lanes'])) {
      throw new Error('expected "close_lanes" to be "BigUint64Array" or an array');
    }
    if (Array.isArray(obj['close_lanes'])) {
      for (const [i, v] of obj['close_lanes'].entries()) {
        if (typeof v !== 'number') {
          throw new Error(`expected index ${i} of "close_lanes" to be "number"`);
        }
      }
    }
  }
}

/*

string fleet_name
uint64[] open_lanes
uint64[] close_lanes

*/

/* This is a generated file, do not edit */

export class ClosedLanes {
  static readonly FullTypeName = '';

  fleet_name: string;
  closed_lanes: BigUint64Array | number[];

  constructor(fields: Partial<ClosedLanes> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.closed_lanes = fields.closed_lanes || new BigUint64Array(0);
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
  }
}

export default ClosedLanes;

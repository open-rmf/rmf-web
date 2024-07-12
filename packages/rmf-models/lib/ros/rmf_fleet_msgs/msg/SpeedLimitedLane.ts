/* This is a generated file, do not edit */

export class SpeedLimitedLane {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/SpeedLimitedLane';

  lane_index: number;
  speed_limit: number;

  constructor(fields: Partial<SpeedLimitedLane> = {}) {
    this.lane_index = fields.lane_index || 0;
    this.speed_limit = fields.speed_limit || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['lane_index'] !== 'number') {
      throw new Error('expected "lane_index" to be "number"');
    }
    if (typeof obj['speed_limit'] !== 'number') {
      throw new Error('expected "speed_limit" to be "number"');
    }
  }
}

/*
# The index of the lane with a speed limit
uint64 lane_index

# The imposed speed limit for the lane
float64 speed_limit
*/

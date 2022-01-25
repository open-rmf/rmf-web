/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';

export class Location {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/Location';

  t: Time;
  x: number;
  y: number;
  yaw: number;
  obey_approach_speed_limit: boolean;
  approach_speed_limit: number;
  level_name: string;
  index: number;

  constructor(fields: Partial<Location> = {}) {
    this.t = fields.t || new Time();
    this.x = fields.x || 0;
    this.y = fields.y || 0;
    this.yaw = fields.yaw || 0;
    this.obey_approach_speed_limit = fields.obey_approach_speed_limit || false;
    this.approach_speed_limit = fields.approach_speed_limit || 0;
    this.level_name = fields.level_name || '';
    this.index = fields.index || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['t'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "t":\n  ' + (e as Error).message);
    }
    if (typeof obj['x'] !== 'number') {
      throw new Error('expected "x" to be "number"');
    }
    if (typeof obj['y'] !== 'number') {
      throw new Error('expected "y" to be "number"');
    }
    if (typeof obj['yaw'] !== 'number') {
      throw new Error('expected "yaw" to be "number"');
    }
    if (typeof obj['obey_approach_speed_limit'] !== 'boolean') {
      throw new Error('expected "obey_approach_speed_limit" to be "boolean"');
    }
    if (typeof obj['approach_speed_limit'] !== 'number') {
      throw new Error('expected "approach_speed_limit" to be "number"');
    }
    if (typeof obj['level_name'] !== 'string') {
      throw new Error('expected "level_name" to be "string"');
    }
    if (typeof obj['index'] !== 'number') {
      throw new Error('expected "index" to be "number"');
    }
  }
}

/*
builtin_interfaces/Time t
float32 x
float32 y
float32 yaw

bool obey_approach_speed_limit false
# Speed limit of the lane leading to this waypoint in m/s
float32 approach_speed_limit

string level_name
uint64 index

*/

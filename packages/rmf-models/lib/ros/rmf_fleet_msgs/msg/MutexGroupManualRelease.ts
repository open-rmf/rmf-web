/* This is a generated file, do not edit */

export class MutexGroupManualRelease {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/MutexGroupManualRelease';

  release_mutex_groups: string[];
  fleet: string;
  robot: string;

  constructor(fields: Partial<MutexGroupManualRelease> = {}) {
    this.release_mutex_groups = fields.release_mutex_groups || [];
    this.fleet = fields.fleet || '';
    this.robot = fields.robot || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['release_mutex_groups'])) {
      throw new Error('expected "release_mutex_groups" to be an array');
    }
    for (const [i, v] of obj['release_mutex_groups'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "release_mutex_groups" to be "string"`);
      }
    }
    if (typeof obj['fleet'] !== 'string') {
      throw new Error('expected "fleet" to be "string"');
    }
    if (typeof obj['robot'] !== 'string') {
      throw new Error('expected "robot" to be "string"');
    }
  }
}

/*
# This message allows operators to manually request that a robot release one or
# more mutex groups that it is currently holding.

# Name of the mutex groups to release
string[] release_mutex_groups

# The name of the fleet that the robot belongs to
string fleet

# The name of the robot that needs to release the mutex groups
string robot

*/

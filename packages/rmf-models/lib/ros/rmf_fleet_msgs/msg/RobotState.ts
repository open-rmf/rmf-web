/* This is a generated file, do not edit */

import * as rmf_fleet_msgs from '../../rmf_fleet_msgs';

export class RobotState {
  static readonly FullTypeName = '';

  name: string;
  model: string;
  task_id: string;
  seq: number;
  mode: rmf_fleet_msgs.msg.RobotMode;
  battery_percent: number;
  location: rmf_fleet_msgs.msg.Location;
  path: Array<rmf_fleet_msgs.msg.Location>;

  constructor(fields: Partial<RobotState> = {}) {
    this.name = fields.name || '';
    this.model = fields.model || '';
    this.task_id = fields.task_id || '';
    this.seq = fields.seq || 0;
    this.mode = fields.mode || new rmf_fleet_msgs.msg.RobotMode();
    this.battery_percent = fields.battery_percent || 0;
    this.location = fields.location || new rmf_fleet_msgs.msg.Location();
    this.path = fields.path || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (typeof obj['model'] !== 'string') {
      throw new Error('expected "model" to be "string"');
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['seq'] !== 'number') {
      throw new Error('expected "seq" to be "number"');
    }
    try {
      rmf_fleet_msgs.msg.RobotMode.validate(obj['mode'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "mode":\n  ' + (e as Error).message);
    }
    if (typeof obj['battery_percent'] !== 'number') {
      throw new Error('expected "battery_percent" to be "number"');
    }
    try {
      rmf_fleet_msgs.msg.Location.validate(obj['location'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "location":\n  ' + (e as Error).message);
    }
    if (!Array.isArray(obj['path'])) {
      throw new Error('expected "path" to be an array');
    }
    for (const [i, v] of obj['path'].entries()) {
      try {
        rmf_fleet_msgs.msg.Location.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "path":\n  ` + (e as Error).message);
      }
    }
  }
}

export default RobotState;

/* This is a generated file, do not edit */

import * as rmf_fleet_msgs from '../../rmf_fleet_msgs';

export class ModeRequest {
  static readonly FullTypeName = '';

  fleet_name: string;
  robot_name: string;
  mode: rmf_fleet_msgs.msg.RobotMode;
  task_id: string;
  parameters: Array<rmf_fleet_msgs.msg.ModeParameter>;

  constructor(fields: Partial<ModeRequest> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.robot_name = fields.robot_name || '';
    this.mode = fields.mode || new rmf_fleet_msgs.msg.RobotMode();
    this.task_id = fields.task_id || '';
    this.parameters = fields.parameters || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
    try {
      rmf_fleet_msgs.msg.RobotMode.validate(obj['mode'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "mode":\n  ' + (e as Error).message);
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (!Array.isArray(obj['parameters'])) {
      throw new Error('expected "parameters" to be an array');
    }
    for (const [i, v] of obj['parameters'].entries()) {
      try {
        rmf_fleet_msgs.msg.ModeParameter.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "parameters":\n  ` + (e as Error).message);
      }
    }
  }
}

export default ModeRequest;

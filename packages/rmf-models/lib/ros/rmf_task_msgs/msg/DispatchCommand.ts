/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class DispatchCommand {
  static readonly FullTypeName = '';

  static readonly TYPE_AWARD = 1;
  static readonly TYPE_REMOVE = 2;

  fleet_name: string;
  task_id: string;
  dispatch_id: number;
  timestamp: builtin_interfaces.msg.Time;
  type: number;

  constructor(fields: Partial<DispatchCommand> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.task_id = fields.task_id || '';
    this.dispatch_id = fields.dispatch_id || 0;
    this.timestamp = fields.timestamp || new builtin_interfaces.msg.Time();
    this.type = fields.type || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['dispatch_id'] !== 'number') {
      throw new Error('expected "dispatch_id" to be "number"');
    }
    try {
      builtin_interfaces.msg.Time.validate(obj['timestamp'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "timestamp":\n  ' + (e as Error).message);
    }
    if (typeof obj['type'] !== 'number') {
      throw new Error('expected "type" to be "number"');
    }
  }
}

export default DispatchCommand;

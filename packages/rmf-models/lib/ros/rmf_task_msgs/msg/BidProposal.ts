/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class BidProposal {
  static readonly FullTypeName = '';

  fleet_name: string;
  expected_robot_name: string;
  prev_cost: number;
  new_cost: number;
  finish_time: builtin_interfaces.msg.Time;

  constructor(fields: Partial<BidProposal> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.expected_robot_name = fields.expected_robot_name || '';
    this.prev_cost = fields.prev_cost || 0;
    this.new_cost = fields.new_cost || 0;
    this.finish_time = fields.finish_time || new builtin_interfaces.msg.Time();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['expected_robot_name'] !== 'string') {
      throw new Error('expected "expected_robot_name" to be "string"');
    }
    if (typeof obj['prev_cost'] !== 'number') {
      throw new Error('expected "prev_cost" to be "number"');
    }
    if (typeof obj['new_cost'] !== 'number') {
      throw new Error('expected "new_cost" to be "number"');
    }
    try {
      builtin_interfaces.msg.Time.validate(obj['finish_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "finish_time":\n  ' + (e as Error).message);
    }
  }
}

export default BidProposal;

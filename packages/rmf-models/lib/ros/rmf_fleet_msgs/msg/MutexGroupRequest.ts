/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';

export class MutexGroupRequest {
  static readonly FullTypeName = '';

  static readonly MODE_RELEASE = 0;
  static readonly MODE_LOCK = 1;

  group: string;
  claimant: number;
  claim_time: builtin_interfaces.msg.Time;
  mode: number;

  constructor(fields: Partial<MutexGroupRequest> = {}) {
    this.group = fields.group || '';
    this.claimant = fields.claimant || 0;
    this.claim_time = fields.claim_time || new builtin_interfaces.msg.Time();
    this.mode = fields.mode || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['group'] !== 'string') {
      throw new Error('expected "group" to be "string"');
    }
    if (typeof obj['claimant'] !== 'number') {
      throw new Error('expected "claimant" to be "number"');
    }
    try {
      builtin_interfaces.msg.Time.validate(obj['claim_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "claim_time":\n  ' + (e as Error).message);
    }
    if (typeof obj['mode'] !== 'number') {
      throw new Error('expected "mode" to be "number"');
    }
  }
}

export default MutexGroupRequest;

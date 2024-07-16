/* This is a generated file, do not edit */

import * as rmf_task_msgs from '../../rmf_task_msgs';

export class Behavior {
  static readonly FullTypeName = '';

  name: string;
  parameters: Array<rmf_task_msgs.msg.BehaviorParameter>;

  constructor(fields: Partial<Behavior> = {}) {
    this.name = fields.name || '';
    this.parameters = fields.parameters || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (!Array.isArray(obj['parameters'])) {
      throw new Error('expected "parameters" to be an array');
    }
    for (const [i, v] of obj['parameters'].entries()) {
      try {
        rmf_task_msgs.msg.BehaviorParameter.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "parameters":\n  ` + (e as Error).message);
      }
    }
  }
}

export default Behavior;

/* This is a generated file, do not edit */

import { BehaviorParameter } from '../../rmf_task_msgs/msg/BehaviorParameter';

export class Behavior {
  static readonly FullTypeName = 'rmf_task_msgs/msg/Behavior';

  name: string;
  parameters: BehaviorParameter[];

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
        BehaviorParameter.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "parameters":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
string name
BehaviorParameter[] parameters

*/

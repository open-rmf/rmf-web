/* This is a generated file, do not edit */

export class BehaviorParameter {
  static readonly FullTypeName = 'rmf_task_msgs/msg/BehaviorParameter';

  name: string;
  value: string;

  constructor(fields: Partial<BehaviorParameter> = {}) {
    this.name = fields.name || '';
    this.value = fields.value || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (typeof obj['value'] !== 'string') {
      throw new Error('expected "value" to be "string"');
    }
  }
}

/*
string name
string value

*/

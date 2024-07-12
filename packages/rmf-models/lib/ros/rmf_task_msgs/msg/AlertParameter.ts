/* This is a generated file, do not edit */

export class AlertParameter {
  static readonly FullTypeName = 'rmf_task_msgs/msg/AlertParameter';

  name: string;
  value: string;

  constructor(fields: Partial<AlertParameter> = {}) {
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
# Generic key-value pair to be used in Alert
string name
string value

*/

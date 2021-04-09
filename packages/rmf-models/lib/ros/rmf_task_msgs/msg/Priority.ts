/* This is a generated file, do not edit */

export class Priority {
  static readonly FullTypeName = 'rmf_task_msgs/msg/Priority';

  value: number;

  constructor(fields: Partial<Priority> = {}) {
    this.value = fields.value || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['value'] !== 'number') {
      throw new Error('expected "value" to be "number"');
    }
  }
}

/*
uint64 value 0
*/

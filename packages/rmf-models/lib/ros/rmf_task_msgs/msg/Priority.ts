/* This is a generated file, do not edit */

export class Priority {
  static readonly FullTypeName = '';

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

export default Priority;

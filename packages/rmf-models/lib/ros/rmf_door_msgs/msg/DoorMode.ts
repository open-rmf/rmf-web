/* This is a generated file, do not edit */

export class DoorMode {
  static readonly FullTypeName = '';

  static readonly MODE_CLOSED = 0;
  static readonly MODE_MOVING = 1;
  static readonly MODE_OPEN = 2;
  static readonly MODE_OFFLINE = 3;
  static readonly MODE_UNKNOWN = 4;

  value: number;

  constructor(fields: Partial<DoorMode> = {}) {
    this.value = fields.value || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['value'] !== 'number') {
      throw new Error('expected "value" to be "number"');
    }
  }
}

export default DoorMode;

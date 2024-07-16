/* This is a generated file, do not edit */

export class ChargerCancel {
  static readonly FullTypeName = '';

  charger_name: string;
  request_id: string;

  constructor(fields: Partial<ChargerCancel> = {}) {
    this.charger_name = fields.charger_name || '';
    this.request_id = fields.request_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['charger_name'] !== 'string') {
      throw new Error('expected "charger_name" to be "string"');
    }
    if (typeof obj['request_id'] !== 'string') {
      throw new Error('expected "request_id" to be "string"');
    }
  }
}

export default ChargerCancel;

/* This is a generated file, do not edit */

export class AlertResponse {
  static readonly FullTypeName = '';

  id: string;
  response: string;

  constructor(fields: Partial<AlertResponse> = {}) {
    this.id = fields.id || '';
    this.response = fields.response || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['id'] !== 'string') {
      throw new Error('expected "id" to be "string"');
    }
    if (typeof obj['response'] !== 'string') {
      throw new Error('expected "response" to be "string"');
    }
  }
}

export default AlertResponse;

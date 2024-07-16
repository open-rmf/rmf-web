/* This is a generated file, do not edit */

export class ApiService_Response {
  static readonly FullTypeName = '';

  json_msg: string;

  constructor(fields: Partial<ApiService_Response> = {}) {
    this.json_msg = fields.json_msg || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['json_msg'] !== 'string') {
      throw new Error('expected "json_msg" to be "string"');
    }
  }
}

export default ApiService_Response;

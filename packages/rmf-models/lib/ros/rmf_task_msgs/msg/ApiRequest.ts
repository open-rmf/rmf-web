/* This is a generated file, do not edit */

export class ApiRequest {
  static readonly FullTypeName = '';

  json_msg: string;
  request_id: string;

  constructor(fields: Partial<ApiRequest> = {}) {
    this.json_msg = fields.json_msg || '';
    this.request_id = fields.request_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['json_msg'] !== 'string') {
      throw new Error('expected "json_msg" to be "string"');
    }
    if (typeof obj['request_id'] !== 'string') {
      throw new Error('expected "request_id" to be "string"');
    }
  }
}

export default ApiRequest;

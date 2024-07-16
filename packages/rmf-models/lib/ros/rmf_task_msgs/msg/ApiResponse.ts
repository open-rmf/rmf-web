/* This is a generated file, do not edit */

export class ApiResponse {
  static readonly FullTypeName = '';

  static readonly TYPE_UNINITIALIZED = 0;
  static readonly TYPE_ACKNOWLEDGE = 1;
  static readonly TYPE_RESPONDING = 2;

  type: number;
  json_msg: string;
  request_id: string;

  constructor(fields: Partial<ApiResponse> = {}) {
    this.type = fields.type || 0;
    this.json_msg = fields.json_msg || '';
    this.request_id = fields.request_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['type'] !== 'number') {
      throw new Error('expected "type" to be "number"');
    }
    if (typeof obj['json_msg'] !== 'string') {
      throw new Error('expected "json_msg" to be "string"');
    }
    if (typeof obj['request_id'] !== 'string') {
      throw new Error('expected "request_id" to be "string"');
    }
  }
}

export default ApiResponse;

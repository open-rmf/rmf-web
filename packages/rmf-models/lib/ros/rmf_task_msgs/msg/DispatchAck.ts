/* This is a generated file, do not edit */

export class DispatchAck {
  static readonly FullTypeName = '';

  dispatch_id: number;
  success: boolean;
  errors: Array<string>;

  constructor(fields: Partial<DispatchAck> = {}) {
    this.dispatch_id = fields.dispatch_id || 0;
    this.success = fields.success || false;
    this.errors = fields.errors || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['dispatch_id'] !== 'number') {
      throw new Error('expected "dispatch_id" to be "number"');
    }
    if (typeof obj['success'] !== 'boolean') {
      throw new Error('expected "success" to be "boolean"');
    }
    if (!Array.isArray(obj['errors'])) {
      throw new Error('expected "errors" to be an array');
    }
    for (const [i, v] of obj['errors'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "errors" to be "string"`);
      }
    }
  }
}

export default DispatchAck;

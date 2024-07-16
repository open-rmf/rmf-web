/* This is a generated file, do not edit */

export class SubmitTask_Response {
  static readonly FullTypeName = '';

  success: boolean;
  task_id: string;
  message: string;

  constructor(fields: Partial<SubmitTask_Response> = {}) {
    this.success = fields.success || false;
    this.task_id = fields.task_id || '';
    this.message = fields.message || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['success'] !== 'boolean') {
      throw new Error('expected "success" to be "boolean"');
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['message'] !== 'string') {
      throw new Error('expected "message" to be "string"');
    }
  }
}

export default SubmitTask_Response;

/* This is a generated file, do not edit */

export class ReviveTask_Request {
  static readonly FullTypeName = '';

  requester: string;
  task_id: string;

  constructor(fields: Partial<ReviveTask_Request> = {}) {
    this.requester = fields.requester || '';
    this.task_id = fields.task_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['requester'] !== 'string') {
      throw new Error('expected "requester" to be "string"');
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
  }
}

export default ReviveTask_Request;

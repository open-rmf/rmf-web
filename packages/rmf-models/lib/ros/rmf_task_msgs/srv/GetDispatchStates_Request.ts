/* This is a generated file, do not edit */

export class GetDispatchStates_Request {
  static readonly FullTypeName = '';

  task_ids: Array<string>;

  constructor(fields: Partial<GetDispatchStates_Request> = {}) {
    this.task_ids = fields.task_ids || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (!Array.isArray(obj['task_ids'])) {
      throw new Error('expected "task_ids" to be an array');
    }
    for (const [i, v] of obj['task_ids'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "task_ids" to be "string"`);
      }
    }
  }
}

export default GetDispatchStates_Request;

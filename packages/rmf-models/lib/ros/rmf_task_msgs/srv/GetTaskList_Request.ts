/* This is a generated file, do not edit */

export class GetTaskList_Request {
  static readonly FullTypeName = 'rmf_task_msgs/srv/GetTaskList_Request';

  requester: string;
  task_id: string[];

  constructor(fields: Partial<GetTaskList_Request> = {}) {
    this.requester = fields.requester || '';
    this.task_id = fields.task_id || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['requester'] !== 'string') {
      throw new Error('expected "requester" to be "string"');
    }
    if (!Array.isArray(obj['task_id'])) {
      throw new Error('expected "task_id" to be an array');
    }
    for (const [i, v] of obj['task_id'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "task_id" to be "string"`);
      }
    }
  }
}

/*
# Query list of submitted tasks | Get service call

# Identifier for who is requesting the service
string requester

# Input the generated task ID during submission
# if empty, provide all Submitted Tasks
string[] task_id


*/

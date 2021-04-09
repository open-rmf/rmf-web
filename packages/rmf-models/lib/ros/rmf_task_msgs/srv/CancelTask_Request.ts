/* This is a generated file, do not edit */

export class CancelTask_Request {
  static readonly FullTypeName = 'rmf_task_msgs/srv/CancelTask_Request';

  requester: string;
  task_id: string;

  constructor(fields: Partial<CancelTask_Request> = {}) {
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

/*
# Cancel Task | "Delete" service call

# Identifier for who is requesting the service
string requester

# generated task ID by dispatcher node
string task_id


*/

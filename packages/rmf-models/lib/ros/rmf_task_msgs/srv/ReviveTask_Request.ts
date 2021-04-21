/* This is a generated file, do not edit */

export class ReviveTask_Request {
  static readonly FullTypeName = 'rmf_task_msgs/srv/ReviveTask_Request';

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

/*
# Revive a previously cancelled or failed task. This will reinitiate
# a bidding sequence to reassign this task.

# Identifier for who is requesting the service
string requester

# task that was previously cancelled or failed
string task_id


*/

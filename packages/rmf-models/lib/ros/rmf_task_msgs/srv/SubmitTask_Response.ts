/* This is a generated file, do not edit */

export class SubmitTask_Response {
  static readonly FullTypeName = 'rmf_task_msgs/srv/SubmitTask_Response';

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

/*


# Confirmation that this service call is processed
bool success

# generated task ID by dispatcher node
string task_id

# This will provide a verbose message regarding task submission
string message


*/

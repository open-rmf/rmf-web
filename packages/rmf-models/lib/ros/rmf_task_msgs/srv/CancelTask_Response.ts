/* This is a generated file, do not edit */

export class CancelTask_Response {
  static readonly FullTypeName = 'rmf_task_msgs/srv/CancelTask_Response';

  success: boolean;
  message: string;

  constructor(fields: Partial<CancelTask_Response> = {}) {
    this.success = fields.success || false;
    this.message = fields.message || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['success'] !== 'boolean') {
      throw new Error('expected "success" to be "boolean"');
    }
    if (typeof obj['message'] !== 'string') {
      throw new Error('expected "message" to be "string"');
    }
  }
}

/*


# Confirmation that this service call is processed
bool success

# This will provide a verbose message regarding task cancellation
string message


*/

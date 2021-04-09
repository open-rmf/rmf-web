/* This is a generated file, do not edit */

export class ReviveTask_Response {
  static readonly FullTypeName = 'rmf_task_msgs/srv/ReviveTask_Response';

  success: boolean;

  constructor(fields: Partial<ReviveTask_Response> = {}) {
    this.success = fields.success || false;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['success'] !== 'boolean') {
      throw new Error('expected "success" to be "boolean"');
    }
  }
}

/*


# Confirmation that this service call is processed
bool success


*/

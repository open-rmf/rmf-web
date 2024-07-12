/* This is a generated file, do not edit */

export class AlertResponse {
  static readonly FullTypeName = 'rmf_task_msgs/msg/AlertResponse';

  id: string;
  response: string;

  constructor(fields: Partial<AlertResponse> = {}) {
    this.id = fields.id || '';
    this.response = fields.response || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['id'] !== 'string') {
      throw new Error('expected "id" to be "string"');
    }
    if (typeof obj['response'] !== 'string') {
      throw new Error('expected "response" to be "string"');
    }
  }
}

/*
# The unique ID of the Alert this response is for
string id

# This response must be one of the available options
# in the Alert.
string response

*/

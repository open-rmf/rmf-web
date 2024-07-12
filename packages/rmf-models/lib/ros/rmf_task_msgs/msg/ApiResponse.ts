/* This is a generated file, do not edit */

export class ApiResponse {
  static readonly FullTypeName = 'rmf_task_msgs/msg/ApiResponse';

  static readonly TYPE_UNINITIALIZED = 0;
  static readonly TYPE_ACKNOWLEDGE = 1;
  static readonly TYPE_RESPONDING = 2;

  type: number;
  json_msg: string;
  request_id: string;

  constructor(fields: Partial<ApiResponse> = {}) {
    this.type = fields.type || 0;
    this.json_msg = fields.json_msg || '';
    this.request_id = fields.request_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['type'] !== 'number') {
      throw new Error('expected "type" to be "number"');
    }
    if (typeof obj['json_msg'] !== 'string') {
      throw new Error('expected "json_msg" to be "string"');
    }
    if (typeof obj['request_id'] !== 'string') {
      throw new Error('expected "request_id" to be "string"');
    }
  }
}

/*

# This response type means the message was not initialized correctly and will
# result in an error
uint8 TYPE_UNINITIALIZED = 0

# This response type means the request is being acknowledged which will grant it
# some extra time before the API Node has its response timeout. This can be used
# to extend the lifetime of a request which may take a long time to complete.
# Each time an acknowledgment is sent the lifetime will be extended.
uint8 TYPE_ACKNOWLEDGE = 1

# This response type means this message is responding to the request and
# therefore fulfilling the request.
uint8 TYPE_RESPONDING = 2

# The type of response this is: Acknowledging or Responding
# (Uninitialized will result in the API Node issuing an error response)
uint8 type

# The JSON message that represents the response
string json_msg

# The unique ID of the request that this response is targeted at
string request_id

*/

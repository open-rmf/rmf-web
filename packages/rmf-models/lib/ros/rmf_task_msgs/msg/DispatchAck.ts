/* This is a generated file, do not edit */

import { DispatchRequest } from '../../rmf_task_msgs/msg/DispatchRequest';

export class DispatchAck {
  static readonly FullTypeName = 'rmf_task_msgs/msg/DispatchAck';

  dispatch_request: DispatchRequest;
  success: boolean;

  constructor(fields: Partial<DispatchAck> = {}) {
    this.dispatch_request = fields.dispatch_request || new DispatchRequest();
    this.success = fields.success || false;
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      DispatchRequest.validate(obj['dispatch_request'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "dispatch_request":\n  ' + (e as Error).message);
    }
    if (typeof obj['success'] !== 'boolean') {
      throw new Error('expected "success" to be "boolean"');
    }
  }
}

/*
# This message is published by the fleet adapter in response to a
# DispatchRequest message. It indicates whether the requested task addition or
# cancellation was successful. 

# The DispatchRequest message received by the Fleet Adapter
DispatchRequest dispatch_request

# True if the addition or cancellation operation was successful
bool success

*/

/* This is a generated file, do not edit */

import * as rmf_task_msgs from '../../rmf_task_msgs';

export class GetDispatchStates_Response {
  static readonly FullTypeName = '';

  success: boolean;
  states: rmf_task_msgs.msg.DispatchStates;

  constructor(fields: Partial<GetDispatchStates_Response> = {}) {
    this.success = fields.success || false;
    this.states = fields.states || new rmf_task_msgs.msg.DispatchStates();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['success'] !== 'boolean') {
      throw new Error('expected "success" to be "boolean"');
    }
    try {
      rmf_task_msgs.msg.DispatchStates.validate(obj['states'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "states":\n  ' + (e as Error).message);
    }
  }
}

export default GetDispatchStates_Response;

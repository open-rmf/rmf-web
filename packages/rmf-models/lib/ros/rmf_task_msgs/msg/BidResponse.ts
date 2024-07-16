/* This is a generated file, do not edit */

import * as rmf_task_msgs from '../../rmf_task_msgs';

export class BidResponse {
  static readonly FullTypeName = '';

  task_id: string;
  has_proposal: boolean;
  proposal: rmf_task_msgs.msg.BidProposal;
  errors: Array<string>;

  constructor(fields: Partial<BidResponse> = {}) {
    this.task_id = fields.task_id || '';
    this.has_proposal = fields.has_proposal || false;
    this.proposal = fields.proposal || new rmf_task_msgs.msg.BidProposal();
    this.errors = fields.errors || [];
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['has_proposal'] !== 'boolean') {
      throw new Error('expected "has_proposal" to be "boolean"');
    }
    try {
      rmf_task_msgs.msg.BidProposal.validate(obj['proposal'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "proposal":\n  ' + (e as Error).message);
    }
    if (!Array.isArray(obj['errors'])) {
      throw new Error('expected "errors" to be an array');
    }
    for (const [i, v] of obj['errors'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "errors" to be "string"`);
      }
    }
  }
}

export default BidResponse;

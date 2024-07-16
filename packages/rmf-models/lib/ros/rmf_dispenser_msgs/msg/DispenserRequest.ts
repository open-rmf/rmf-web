/* This is a generated file, do not edit */

import * as rmf_dispenser_msgs from '../../rmf_dispenser_msgs';
import * as builtin_interfaces from '../../builtin_interfaces';

export class DispenserRequest {
  static readonly FullTypeName = '';

  time: builtin_interfaces.msg.Time;
  request_guid: string;
  target_guid: string;
  transporter_type: string;
  items: Array<rmf_dispenser_msgs.msg.DispenserRequestItem>;

  constructor(fields: Partial<DispenserRequest> = {}) {
    this.time = fields.time || new builtin_interfaces.msg.Time();
    this.request_guid = fields.request_guid || '';
    this.target_guid = fields.target_guid || '';
    this.transporter_type = fields.transporter_type || '';
    this.items = fields.items || [];
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      builtin_interfaces.msg.Time.validate(obj['time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "time":\n  ' + (e as Error).message);
    }
    if (typeof obj['request_guid'] !== 'string') {
      throw new Error('expected "request_guid" to be "string"');
    }
    if (typeof obj['target_guid'] !== 'string') {
      throw new Error('expected "target_guid" to be "string"');
    }
    if (typeof obj['transporter_type'] !== 'string') {
      throw new Error('expected "transporter_type" to be "string"');
    }
    if (!Array.isArray(obj['items'])) {
      throw new Error('expected "items" to be an array');
    }
    for (const [i, v] of obj['items'].entries()) {
      try {
        rmf_dispenser_msgs.msg.DispenserRequestItem.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "items":\n  ` + (e as Error).message);
      }
    }
  }
}

export default DispenserRequest;

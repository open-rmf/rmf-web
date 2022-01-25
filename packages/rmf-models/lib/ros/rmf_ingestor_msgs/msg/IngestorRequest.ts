/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';
import { IngestorRequestItem } from '../../rmf_ingestor_msgs/msg/IngestorRequestItem';

export class IngestorRequest {
  static readonly FullTypeName = 'rmf_ingestor_msgs/msg/IngestorRequest';

  time: Time;
  request_guid: string;
  target_guid: string;
  transporter_type: string;
  items: IngestorRequestItem[];

  constructor(fields: Partial<IngestorRequest> = {}) {
    this.time = fields.time || new Time();
    this.request_guid = fields.request_guid || '';
    this.target_guid = fields.target_guid || '';
    this.transporter_type = fields.transporter_type || '';
    this.items = fields.items || [];
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['time'] as Record<string, unknown>);
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
        IngestorRequestItem.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "items":\n  ` + (e as Error).message);
      }
    }
  }
}

/*
builtin_interfaces/Time time

# A unique ID for this request
string request_guid

# The unique name of the ingestor that this request is aimed at
string target_guid

# below are custom workcell message fields
string transporter_type
IngestorRequestItem[] items

*/

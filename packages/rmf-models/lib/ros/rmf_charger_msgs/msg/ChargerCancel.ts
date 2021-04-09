/* This is a generated file, do not edit */

export class ChargerCancel {
  static readonly FullTypeName = 'rmf_charger_msgs/msg/ChargerCancel';

  charger_name: string;
  request_id: string;

  constructor(fields: Partial<ChargerCancel> = {}) {
    this.charger_name = fields.charger_name || '';
    this.request_id = fields.request_id || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['charger_name'] !== 'string') {
      throw new Error('expected "charger_name" to be "string"');
    }
    if (typeof obj['request_id'] !== 'string') {
      throw new Error('expected "request_id" to be "string"');
    }
  }
}

/*
string charger_name  # the charger that should process this message

# A unique ID for each request. It is advised that you prefix this
# with the sender's node name. This is used for error tracking
# later on
string request_id

*/

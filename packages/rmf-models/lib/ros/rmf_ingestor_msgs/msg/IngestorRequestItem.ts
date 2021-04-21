/* This is a generated file, do not edit */

export class IngestorRequestItem {
  static readonly FullTypeName = 'rmf_ingestor_msgs/msg/IngestorRequestItem';

  type_guid: string;
  quantity: number;
  compartment_name: string;

  constructor(fields: Partial<IngestorRequestItem> = {}) {
    this.type_guid = fields.type_guid || '';
    this.quantity = fields.quantity || 0;
    this.compartment_name = fields.compartment_name || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['type_guid'] !== 'string') {
      throw new Error('expected "type_guid" to be "string"');
    }
    if (typeof obj['quantity'] !== 'number') {
      throw new Error('expected "quantity" to be "number"');
    }
    if (typeof obj['compartment_name'] !== 'string') {
      throw new Error('expected "compartment_name" to be "string"');
    }
  }
}

/*
string type_guid
int32 quantity
string compartment_name

*/

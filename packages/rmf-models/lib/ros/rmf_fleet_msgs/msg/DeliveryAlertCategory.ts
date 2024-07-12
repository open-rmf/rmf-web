/* This is a generated file, do not edit */

export class DeliveryAlertCategory {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/DeliveryAlertCategory';

  static readonly MISSING = 0;
  static readonly WRONG = 1;
  static readonly OBSTRUCTED = 2;
  static readonly CANCELLED = 3;

  value: number;

  constructor(fields: Partial<DeliveryAlertCategory> = {}) {
    this.value = fields.value || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['value'] !== 'number') {
      throw new Error('expected "value" to be "number"');
    }
  }
}

/*
uint32 value
uint32 MISSING=0
uint32 WRONG=1
uint32 OBSTRUCTED=2
uint32 CANCELLED=3

*/

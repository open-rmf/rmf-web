/* This is a generated file, do not edit */

export class DeliveryAlertAction {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/DeliveryAlertAction';

  static readonly WAITING = 0;
  static readonly CANCEL = 1;
  static readonly OVERRIDE = 2;
  static readonly RESUME = 3;

  value: number;

  constructor(fields: Partial<DeliveryAlertAction> = {}) {
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
uint32 WAITING=0
uint32 CANCEL=1
uint32 OVERRIDE=2
uint32 RESUME=3

*/

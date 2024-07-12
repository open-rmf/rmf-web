/* This is a generated file, do not edit */

export class DeliveryAlertTier {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/DeliveryAlertTier';

  static readonly WARNING = 0;
  static readonly ERROR = 1;

  value: number;

  constructor(fields: Partial<DeliveryAlertTier> = {}) {
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
uint32 WARNING=0
uint32 ERROR=1

*/

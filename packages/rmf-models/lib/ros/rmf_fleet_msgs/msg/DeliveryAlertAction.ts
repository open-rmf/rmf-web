/* This is a generated file, do not edit */

export class DeliveryAlertAction {
  static readonly FullTypeName = '';

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

export default DeliveryAlertAction;

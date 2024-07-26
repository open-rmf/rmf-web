/* This is a generated file, do not edit */

import * as rmf_fleet_msgs from '../../rmf_fleet_msgs';

export class DeliveryAlert {
  static readonly FullTypeName = '';

  id: string;
  category: rmf_fleet_msgs.msg.DeliveryAlertCategory;
  tier: rmf_fleet_msgs.msg.DeliveryAlertTier;
  task_id: string;
  action: rmf_fleet_msgs.msg.DeliveryAlertAction;
  message: string;

  constructor(fields: Partial<DeliveryAlert> = {}) {
    this.id = fields.id || '';
    this.category = fields.category || new rmf_fleet_msgs.msg.DeliveryAlertCategory();
    this.tier = fields.tier || new rmf_fleet_msgs.msg.DeliveryAlertTier();
    this.task_id = fields.task_id || '';
    this.action = fields.action || new rmf_fleet_msgs.msg.DeliveryAlertAction();
    this.message = fields.message || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['id'] !== 'string') {
      throw new Error('expected "id" to be "string"');
    }
    try {
      rmf_fleet_msgs.msg.DeliveryAlertCategory.validate(obj['category'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "category":\n  ' + (e as Error).message);
    }
    try {
      rmf_fleet_msgs.msg.DeliveryAlertTier.validate(obj['tier'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "tier":\n  ' + (e as Error).message);
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    try {
      rmf_fleet_msgs.msg.DeliveryAlertAction.validate(obj['action'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "action":\n  ' + (e as Error).message);
    }
    if (typeof obj['message'] !== 'string') {
      throw new Error('expected "message" to be "string"');
    }
  }
}

export default DeliveryAlert;

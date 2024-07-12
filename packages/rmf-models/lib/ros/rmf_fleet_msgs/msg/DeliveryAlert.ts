/* This is a generated file, do not edit */

import { DeliveryAlertCategory } from '../../rmf_fleet_msgs/msg/DeliveryAlertCategory';
import { DeliveryAlertTier } from '../../rmf_fleet_msgs/msg/DeliveryAlertTier';
import { DeliveryAlertAction } from '../../rmf_fleet_msgs/msg/DeliveryAlertAction';

export class DeliveryAlert {
  static readonly FullTypeName = 'rmf_fleet_msgs/msg/DeliveryAlert';

  id: string;
  category: DeliveryAlertCategory;
  tier: DeliveryAlertTier;
  task_id: string;
  action: DeliveryAlertAction;
  message: string;

  constructor(fields: Partial<DeliveryAlert> = {}) {
    this.id = fields.id || '';
    this.category = fields.category || new DeliveryAlertCategory();
    this.tier = fields.tier || new DeliveryAlertTier();
    this.task_id = fields.task_id || '';
    this.action = fields.action || new DeliveryAlertAction();
    this.message = fields.message || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['id'] !== 'string') {
      throw new Error('expected "id" to be "string"');
    }
    try {
      DeliveryAlertCategory.validate(obj['category'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "category":\n  ' + (e as Error).message);
    }
    try {
      DeliveryAlertTier.validate(obj['tier'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "tier":\n  ' + (e as Error).message);
    }
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    try {
      DeliveryAlertAction.validate(obj['action'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "action":\n  ' + (e as Error).message);
    }
    if (typeof obj['message'] !== 'string') {
      throw new Error('expected "message" to be "string"');
    }
  }
}

/*
string id
DeliveryAlertCategory category
DeliveryAlertTier tier
string task_id
DeliveryAlertAction action
string message

*/

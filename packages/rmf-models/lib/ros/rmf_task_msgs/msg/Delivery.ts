/* This is a generated file, do not edit */

import { DispenserRequestItem } from '../../rmf_dispenser_msgs/msg/DispenserRequestItem';
import { Behavior } from '../../rmf_task_msgs/msg/Behavior';

export class Delivery {
  static readonly FullTypeName = 'rmf_task_msgs/msg/Delivery';

  task_id: string;
  items: DispenserRequestItem[];
  pickup_place_name: string;
  pickup_dispenser: string;
  pickup_behavior: Behavior;
  dropoff_place_name: string;
  dropoff_ingestor: string;
  dropoff_behavior: Behavior;

  constructor(fields: Partial<Delivery> = {}) {
    this.task_id = fields.task_id || '';
    this.items = fields.items || [];
    this.pickup_place_name = fields.pickup_place_name || '';
    this.pickup_dispenser = fields.pickup_dispenser || '';
    this.pickup_behavior = fields.pickup_behavior || new Behavior();
    this.dropoff_place_name = fields.dropoff_place_name || '';
    this.dropoff_ingestor = fields.dropoff_ingestor || '';
    this.dropoff_behavior = fields.dropoff_behavior || new Behavior();
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (!Array.isArray(obj['items'])) {
      throw new Error('expected "items" to be an array');
    }
    for (const [i, v] of obj['items'].entries()) {
      try {
        DispenserRequestItem.validate(v);
      } catch (e) {
        throw new Error(`in index ${i} of "items":\n  ` + (e as Error).message);
      }
    }
    if (typeof obj['pickup_place_name'] !== 'string') {
      throw new Error('expected "pickup_place_name" to be "string"');
    }
    if (typeof obj['pickup_dispenser'] !== 'string') {
      throw new Error('expected "pickup_dispenser" to be "string"');
    }
    try {
      Behavior.validate(obj['pickup_behavior'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "pickup_behavior":\n  ' + (e as Error).message);
    }
    if (typeof obj['dropoff_place_name'] !== 'string') {
      throw new Error('expected "dropoff_place_name" to be "string"');
    }
    if (typeof obj['dropoff_ingestor'] !== 'string') {
      throw new Error('expected "dropoff_ingestor" to be "string"');
    }
    try {
      Behavior.validate(obj['dropoff_behavior'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "dropoff_behavior":\n  ' + (e as Error).message);
    }
  }
}

/*
# task_id is intended to be a pseudo-random string generated
# by the caller which can be used to identify this task as it
# moves between the queues to completion (or failure).
string task_id

rmf_dispenser_msgs/DispenserRequestItem[] items

string pickup_place_name
string pickup_dispenser
Behavior pickup_behavior

string dropoff_place_name
string dropoff_ingestor
Behavior dropoff_behavior

*/

/* This is a generated file, do not edit */

export class Tow {
  static readonly FullTypeName = 'rmf_task_msgs/msg/Tow';

  task_id: string;
  object_type: string;
  is_object_id_known: boolean;
  object_id: string;
  pickup_place_name: string;
  is_dropoff_place_known: boolean;
  dropoff_place_name: string;

  constructor(fields: Partial<Tow> = {}) {
    this.task_id = fields.task_id || '';
    this.object_type = fields.object_type || '';
    this.is_object_id_known = fields.is_object_id_known || false;
    this.object_id = fields.object_id || '';
    this.pickup_place_name = fields.pickup_place_name || '';
    this.is_dropoff_place_known = fields.is_dropoff_place_known || false;
    this.dropoff_place_name = fields.dropoff_place_name || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['object_type'] !== 'string') {
      throw new Error('expected "object_type" to be "string"');
    }
    if (typeof obj['is_object_id_known'] !== 'boolean') {
      throw new Error('expected "is_object_id_known" to be "boolean"');
    }
    if (typeof obj['object_id'] !== 'string') {
      throw new Error('expected "object_id" to be "string"');
    }
    if (typeof obj['pickup_place_name'] !== 'string') {
      throw new Error('expected "pickup_place_name" to be "string"');
    }
    if (typeof obj['is_dropoff_place_known'] !== 'boolean') {
      throw new Error('expected "is_dropoff_place_known" to be "boolean"');
    }
    if (typeof obj['dropoff_place_name'] !== 'string') {
      throw new Error('expected "dropoff_place_name" to be "string"');
    }
  }
}

/*
# task_id is intended to be a pseudo-random string generated
# by the caller which can be used to identify this task as it
# moves between the queues to completion (or failure).
string task_id

string object_type

bool is_object_id_known
string object_id

string pickup_place_name

bool is_dropoff_place_known
string dropoff_place_name

*/

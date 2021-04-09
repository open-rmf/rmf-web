/* This is a generated file, do not edit */

export class TaskType {
  static readonly FullTypeName = 'rmf_task_msgs/msg/TaskType';

  static readonly TYPE_STATION = 0;
  static readonly TYPE_LOOP = 1;
  static readonly TYPE_DELIVERY = 2;
  static readonly TYPE_CHARGE_BATTERY = 3;
  static readonly TYPE_CLEAN = 4;
  static readonly TYPE_PATROL = 5;

  type: number;

  constructor(fields: Partial<TaskType> = {}) {
    this.type = fields.type || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['type'] !== 'number') {
      throw new Error('expected "type" to be "number"');
    }
  }
}

/*
uint32 type
uint32 TYPE_STATION=0
uint32 TYPE_LOOP=1
uint32 TYPE_DELIVERY=2
uint32 TYPE_CHARGE_BATTERY=3
uint32 TYPE_CLEAN=4
uint32 TYPE_PATROL=5


*/

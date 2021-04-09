/* This is a generated file, do not edit */

export class Station {
  static readonly FullTypeName = 'rmf_task_msgs/msg/Station';

  task_id: string;
  robot_type: string;
  place_name: string;

  constructor(fields: Partial<Station> = {}) {
    this.task_id = fields.task_id || '';
    this.robot_type = fields.robot_type || '';
    this.place_name = fields.place_name || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['robot_type'] !== 'string') {
      throw new Error('expected "robot_type" to be "string"');
    }
    if (typeof obj['place_name'] !== 'string') {
      throw new Error('expected "place_name" to be "string"');
    }
  }
}

/*
# task_id is intended to be a pseudo-random string generated
# by the caller which can be used to identify this task as it
# moves between the queues to completion (or failure).
string task_id

# robot_type can be used to specify a particular robot fleet
# for this request
string robot_type

# the place name where the robot is requested to station (park)
string place_name

*/

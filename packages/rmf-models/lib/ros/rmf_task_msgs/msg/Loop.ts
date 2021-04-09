/* This is a generated file, do not edit */

export class Loop {
  static readonly FullTypeName = 'rmf_task_msgs/msg/Loop';

  task_id: string;
  robot_type: string;
  num_loops: number;
  start_name: string;
  finish_name: string;

  constructor(fields: Partial<Loop> = {}) {
    this.task_id = fields.task_id || '';
    this.robot_type = fields.robot_type || '';
    this.num_loops = fields.num_loops || 0;
    this.start_name = fields.start_name || '';
    this.finish_name = fields.finish_name || '';
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['task_id'] !== 'string') {
      throw new Error('expected "task_id" to be "string"');
    }
    if (typeof obj['robot_type'] !== 'string') {
      throw new Error('expected "robot_type" to be "string"');
    }
    if (typeof obj['num_loops'] !== 'number') {
      throw new Error('expected "num_loops" to be "number"');
    }
    if (typeof obj['start_name'] !== 'string') {
      throw new Error('expected "start_name" to be "string"');
    }
    if (typeof obj['finish_name'] !== 'string') {
      throw new Error('expected "finish_name" to be "string"');
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

# The number of times the robot should loop between the specified points.
uint32 num_loops

# The name of the waypoint where the robot should begin its loop. If the robot
# is not already at this point, it will begin the task by moving there.
string start_name

# The name of the waypoint where the robot should end its looping. The robot
# will visit this waypoint num_loops times and then stop here on the last
# visit.
string finish_name

*/

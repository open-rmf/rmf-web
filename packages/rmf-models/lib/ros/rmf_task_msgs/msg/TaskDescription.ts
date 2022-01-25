/* This is a generated file, do not edit */

import { Time } from '../../builtin_interfaces/msg/Time';
import { Priority } from '../../rmf_task_msgs/msg/Priority';
import { TaskType } from '../../rmf_task_msgs/msg/TaskType';
import { Station } from '../../rmf_task_msgs/msg/Station';
import { Loop } from '../../rmf_task_msgs/msg/Loop';
import { Delivery } from '../../rmf_task_msgs/msg/Delivery';
import { Clean } from '../../rmf_task_msgs/msg/Clean';

export class TaskDescription {
  static readonly FullTypeName = 'rmf_task_msgs/msg/TaskDescription';

  start_time: Time;
  priority: Priority;
  task_type: TaskType;
  station: Station;
  loop: Loop;
  delivery: Delivery;
  clean: Clean;

  constructor(fields: Partial<TaskDescription> = {}) {
    this.start_time = fields.start_time || new Time();
    this.priority = fields.priority || new Priority();
    this.task_type = fields.task_type || new TaskType();
    this.station = fields.station || new Station();
    this.loop = fields.loop || new Loop();
    this.delivery = fields.delivery || new Delivery();
    this.clean = fields.clean || new Clean();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      Time.validate(obj['start_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "start_time":\n  ' + (e as Error).message);
    }
    try {
      Priority.validate(obj['priority'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "priority":\n  ' + (e as Error).message);
    }
    try {
      TaskType.validate(obj['task_type'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "task_type":\n  ' + (e as Error).message);
    }
    try {
      Station.validate(obj['station'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "station":\n  ' + (e as Error).message);
    }
    try {
      Loop.validate(obj['loop'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "loop":\n  ' + (e as Error).message);
    }
    try {
      Delivery.validate(obj['delivery'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "delivery":\n  ' + (e as Error).message);
    }
    try {
      Clean.validate(obj['clean'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "clean":\n  ' + (e as Error).message);
    }
  }
}

/*
# Desired start time of a task
builtin_interfaces/Time start_time

# Priority of the task
Priority priority

# Task type
TaskType task_type

# The corresponding field for the above TaskType should be populated
Station station
Loop loop
Delivery delivery
Clean clean

*/

/* This is a generated file, do not edit */

import * as builtin_interfaces from '../../builtin_interfaces';
import * as rmf_task_msgs from '../../rmf_task_msgs';

export class TaskDescription {
  static readonly FullTypeName = '';

  start_time: builtin_interfaces.msg.Time;
  priority: rmf_task_msgs.msg.Priority;
  task_type: rmf_task_msgs.msg.TaskType;
  station: rmf_task_msgs.msg.Station;
  loop: rmf_task_msgs.msg.Loop;
  delivery: rmf_task_msgs.msg.Delivery;
  clean: rmf_task_msgs.msg.Clean;

  constructor(fields: Partial<TaskDescription> = {}) {
    this.start_time = fields.start_time || new builtin_interfaces.msg.Time();
    this.priority = fields.priority || new rmf_task_msgs.msg.Priority();
    this.task_type = fields.task_type || new rmf_task_msgs.msg.TaskType();
    this.station = fields.station || new rmf_task_msgs.msg.Station();
    this.loop = fields.loop || new rmf_task_msgs.msg.Loop();
    this.delivery = fields.delivery || new rmf_task_msgs.msg.Delivery();
    this.clean = fields.clean || new rmf_task_msgs.msg.Clean();
  }

  static validate(obj: Record<string, unknown>): void {
    try {
      builtin_interfaces.msg.Time.validate(obj['start_time'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "start_time":\n  ' + (e as Error).message);
    }
    try {
      rmf_task_msgs.msg.Priority.validate(obj['priority'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "priority":\n  ' + (e as Error).message);
    }
    try {
      rmf_task_msgs.msg.TaskType.validate(obj['task_type'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "task_type":\n  ' + (e as Error).message);
    }
    try {
      rmf_task_msgs.msg.Station.validate(obj['station'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "station":\n  ' + (e as Error).message);
    }
    try {
      rmf_task_msgs.msg.Loop.validate(obj['loop'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "loop":\n  ' + (e as Error).message);
    }
    try {
      rmf_task_msgs.msg.Delivery.validate(obj['delivery'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "delivery":\n  ' + (e as Error).message);
    }
    try {
      rmf_task_msgs.msg.Clean.validate(obj['clean'] as Record<string, unknown>);
    } catch (e) {
      throw new Error('in "clean":\n  ' + (e as Error).message);
    }
  }
}

export default TaskDescription;

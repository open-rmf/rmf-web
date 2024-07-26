/* This is a generated file, do not edit */

export class Loop {
  static readonly FullTypeName = '';

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

export default Loop;

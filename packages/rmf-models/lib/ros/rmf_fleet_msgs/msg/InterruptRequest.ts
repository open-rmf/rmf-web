/* This is a generated file, do not edit */

export class InterruptRequest {
  static readonly FullTypeName = '';

  static readonly TYPE_INTERRUPT = 0;
  static readonly TYPE_RESUME = 1;

  fleet_name: string;
  robot_name: string;
  interrupt_id: string;
  labels: Array<string>;
  type: number;

  constructor(fields: Partial<InterruptRequest> = {}) {
    this.fleet_name = fields.fleet_name || '';
    this.robot_name = fields.robot_name || '';
    this.interrupt_id = fields.interrupt_id || '';
    this.labels = fields.labels || [];
    this.type = fields.type || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['fleet_name'] !== 'string') {
      throw new Error('expected "fleet_name" to be "string"');
    }
    if (typeof obj['robot_name'] !== 'string') {
      throw new Error('expected "robot_name" to be "string"');
    }
    if (typeof obj['interrupt_id'] !== 'string') {
      throw new Error('expected "interrupt_id" to be "string"');
    }
    if (!Array.isArray(obj['labels'])) {
      throw new Error('expected "labels" to be an array');
    }
    for (const [i, v] of obj['labels'].entries()) {
      if (typeof v !== 'string') {
        throw new Error(`expected index ${i} of "labels" to be "string"`);
      }
    }
    if (typeof obj['type'] !== 'number') {
      throw new Error('expected "type" to be "number"');
    }
  }
}

export default InterruptRequest;

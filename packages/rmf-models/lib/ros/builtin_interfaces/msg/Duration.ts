/* This is a generated file, do not edit */

export class Duration {
  static readonly FullTypeName = '';

  sec: number;
  nanosec: number;

  constructor(fields: Partial<Duration> = {}) {
    this.sec = fields.sec || 0;
    this.nanosec = fields.nanosec || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['sec'] !== 'number') {
      throw new Error('expected "sec" to be "number"');
    }
    if (typeof obj['nanosec'] !== 'number') {
      throw new Error('expected "nanosec" to be "number"');
    }
  }
}

export default Duration;

/* This is a generated file, do not edit */

export class Time {
  static readonly FullTypeName = 'builtin_interfaces/msg/Time';

  sec: number;
  nanosec: number;

  constructor(fields: Partial<Time> = {}) {
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

/*
# Time indicates a specific point in time, relative to a clock's 0 point.

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 10e9).
uint32 nanosec

*/

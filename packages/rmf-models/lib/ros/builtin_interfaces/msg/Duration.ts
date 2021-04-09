/* This is a generated file, do not edit */

export class Duration {
  static readonly FullTypeName = 'builtin_interfaces/msg/Duration';

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

/*
# Duration defines a period between two time points. It is comprised of a
# seconds component and a nanoseconds component.

# Seconds component, range is valid over any possible int32 value.
int32 sec

# Nanoseconds component in the range of [0, 10e9).
uint32 nanosec

*/

/* This is a generated file, do not edit */

export class DoorMode {
  static readonly FullTypeName = 'rmf_door_msgs/msg/DoorMode';

  static readonly MODE_CLOSED = 0;
  static readonly MODE_MOVING = 1;
  static readonly MODE_OPEN = 2;
  static readonly MODE_OFFLINE = 3;
  static readonly MODE_UNKNOWN = 4;

  value: number;

  constructor(fields: Partial<DoorMode> = {}) {
    this.value = fields.value || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['value'] !== 'number') {
      throw new Error('expected "value" to be "number"');
    }
  }
}

/*
# The DoorMode message captures the "mode" of an automatic door controller.
# Most door controllers default to running in "closed" mode, and transition
# through some sort of "moving" mode until reaching the "open" mode.

uint32 value

# "value" must be one of the following enumerations:
uint32 MODE_CLOSED=0
uint32 MODE_MOVING=1
uint32 MODE_OPEN=2
uint32 MODE_OFFLINE=3
uint32 MODE_UNKNOWN=4

*/

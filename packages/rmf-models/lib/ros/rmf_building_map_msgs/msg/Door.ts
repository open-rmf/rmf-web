/* This is a generated file, do not edit */

export class Door {
  static readonly FullTypeName = 'rmf_building_map_msgs/msg/Door';

  static readonly DOOR_TYPE_UNDEFINED = 0;
  static readonly DOOR_TYPE_SINGLE_SLIDING = 1;
  static readonly DOOR_TYPE_DOUBLE_SLIDING = 2;
  static readonly DOOR_TYPE_SINGLE_TELESCOPE = 3;
  static readonly DOOR_TYPE_DOUBLE_TELESCOPE = 4;
  static readonly DOOR_TYPE_SINGLE_SWING = 5;
  static readonly DOOR_TYPE_DOUBLE_SWING = 6;

  name: string;
  v1_x: number;
  v1_y: number;
  v2_x: number;
  v2_y: number;
  door_type: number;
  motion_range: number;
  motion_direction: number;

  constructor(fields: Partial<Door> = {}) {
    this.name = fields.name || '';
    this.v1_x = fields.v1_x || 0;
    this.v1_y = fields.v1_y || 0;
    this.v2_x = fields.v2_x || 0;
    this.v2_y = fields.v2_y || 0;
    this.door_type = fields.door_type || 0;
    this.motion_range = fields.motion_range || 0;
    this.motion_direction = fields.motion_direction || 0;
  }

  static validate(obj: Record<string, unknown>): void {
    if (typeof obj['name'] !== 'string') {
      throw new Error('expected "name" to be "string"');
    }
    if (typeof obj['v1_x'] !== 'number') {
      throw new Error('expected "v1_x" to be "number"');
    }
    if (typeof obj['v1_y'] !== 'number') {
      throw new Error('expected "v1_y" to be "number"');
    }
    if (typeof obj['v2_x'] !== 'number') {
      throw new Error('expected "v2_x" to be "number"');
    }
    if (typeof obj['v2_y'] !== 'number') {
      throw new Error('expected "v2_y" to be "number"');
    }
    if (typeof obj['door_type'] !== 'number') {
      throw new Error('expected "door_type" to be "number"');
    }
    if (typeof obj['motion_range'] !== 'number') {
      throw new Error('expected "motion_range" to be "number"');
    }
    if (typeof obj['motion_direction'] !== 'number') {
      throw new Error('expected "motion_direction" to be "number"');
    }
  }
}

/*
string name

# CONVENTIONS
# ===========
# single hinge doors:
#   * hinge is located at (v1_x, v1_y)
#   * door extends till (v2_x, v2_y)
#   * motion_range = door swing range in DEGREES
#   * there are two possible motions: clockwise and anti-clockwise
#     selected by the motion_direction parameter, which is +1 or -1
#
# double hinge doors:
#   * hinges are located at both (v1_x, v1_y) and (v2_x, v2_y)
#   * motion range = door swing ranges in DEGREES (assume symmetric)
#   * same motion-direction selection as single hinge
#
# single sliding doors:
#   * the door slides from (v2_x, v2_y) towards (v1_x, v1_y)
#   * range of motion is entire distance from v2->v1. No need to specify.
#
# double sliding doors:
#   * door panels slide from the centerpoint of v1<->v2 towards v1 and v2
#
# single/double telescoping doors:
#   * common in elevators; same parameters as sliding doors; they just
#     open/close faster and take up less space inside the wall.

float32 v1_x
float32 v1_y

float32 v2_x
float32 v2_y

uint8 door_type
uint8 DOOR_TYPE_UNDEFINED=0
uint8 DOOR_TYPE_SINGLE_SLIDING=1
uint8 DOOR_TYPE_DOUBLE_SLIDING=2
uint8 DOOR_TYPE_SINGLE_TELESCOPE=3
uint8 DOOR_TYPE_DOUBLE_TELESCOPE=4
uint8 DOOR_TYPE_SINGLE_SWING=5
uint8 DOOR_TYPE_DOUBLE_SWING=6

float32 motion_range
int32 motion_direction

*/

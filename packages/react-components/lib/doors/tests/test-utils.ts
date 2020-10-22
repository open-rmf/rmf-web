import * as RomiCore from '@osrf/romi-js-core-interfaces';

export function makeDoor(door?: Partial<RomiCore.Door>): RomiCore.Door {
  return {
    name: 'test',
    door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING,
    motion_direction: 1,
    motion_range: Math.PI / 2,
    v1_x: 0,
    v1_y: 0,
    v2_x: 1,
    v2_y: 1,
    ...door,
  };
}

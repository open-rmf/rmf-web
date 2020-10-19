import * as RomiCore from '@osrf/romi-js-core-interfaces';

export function makeLift(lift?: Partial<RomiCore.Lift>): RomiCore.Lift {
  return {
    name: 'test',
    depth: 0,
    doors: [],
    levels: ['test'],
    ref_x: 0,
    ref_y: 0,
    ref_yaw: 0,
    width: 1,
    wall_graph: { name: 'test', edges: [], params: [], vertices: [] },
    ...lift,
  };
}

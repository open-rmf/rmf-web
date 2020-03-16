import * as RomiCore from '@osrf/romi-js-core-interfaces';
import officePng from './office.png';

export default async function buildingMap(): Promise<RomiCore.BuildingMap> {
  const resp = await fetch(officePng);
  const officeImageData = new Uint8Array(await resp.arrayBuffer());

  const doors = [
    {
      name: 'Door1',
      v1_x: 0,
      v1_y: 0,
      v2_x: 1,
      v2_y: 1,
      door_type: 1,
      motion_range: 1.571,
      motion_direction: 1,
    },
    {
      name: 'Door2',
      v1_x: 10,
      v1_y: 10,
      v2_x: 11,
      v2_y: 11,
      door_type: 2,
      motion_range: 1.571,
      motion_direction: -1,
    },
  ];
  for (let i = 3; i < 10; i++) {
    doors.push({
      name: `Door${i}`,
      v1_x: i,
      v1_y: i,
      v2_x: i + 1,
      v2_y: i + 1,
      door_type: 1,
      motion_range: 1.571,
      motion_direction: 1,
    });
  }

  return {
    name: 'test building',
    levels: [
      {
        name: 'L1',
        elevation: 0,
        images: [
          {
            data: officeImageData,
            encoding: 'png',
            name: 'office',
            scale: 0.008465494960546494,
            x_offset: 0,
            y_offset: 0,
            yaw: 0,
          },
        ],
        places: [],
        doors: doors,
        nav_graphs: [],
        wall_graph: {
          name: 'wallgraph',
          vertices: [],
          edges: [],
          params: [],
        },
      },
    ],
    lifts: [
      {
        name: 'Lift1',
        doors: [],
        levels: ['L1', 'L2', 'L3'],
        ref_x: 0,
        ref_y: 0,
        ref_yaw: 0,
        wall_graph: {
          name: 'wallgraph',
          vertices: [],
          edges: [],
          params: [],
        },
      },
      {
        name: 'Lift2',
        doors: [],
        levels: ['L1', 'L2', 'L3', 'L4'],
        ref_x: 10,
        ref_y: 10,
        ref_yaw: 1.571,
        wall_graph: {
          name: 'wallgraph',
          vertices: [],
          edges: [],
          params: [],
        },
      },
    ],
  };
}

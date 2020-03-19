import * as RomiCore from '@osrf/romi-js-core-interfaces';
import officePng from './office.png';

export default async function buildingMap(): Promise<RomiCore.BuildingMap> {
  const resp = await fetch(officePng);
  const officeImageData = new Uint8Array(await resp.arrayBuffer());

  const doors = [
    {
      name: 'main_door',
      v1_x: 7,
      v1_y: -6,
      v2_x: 8,
      v2_y: -6.2,
      door_type: 1,
      motion_range: 1.571,
      motion_direction: 1,
    },
    {
      name: 'hardware_door',
      v1_x: 4,
      v1_y: -4,
      v2_x: 4.1,
      v2_y: -3.8,
      door_type: 1,
      motion_range: 1.571,
      motion_direction: -1,
    },
    {
      name: 'coe_door',
      v1_x: 6,
      v1_y: -6,
      v2_x: 6.2,
      v2_y: -5.9,
      door_type: 2,
      motion_range: -1.571,
      motion_direction: 1,
    },
  ];

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
        places: [{
          name: 'Place1',
          x: 2,
          y: -2,
          yaw: 0,
          position_tolerance: 0,
          yaw_tolerance: 0,
        },
        {
          name: 'Place2',
          x: 8,
          y: -4,
          yaw: 0,
          position_tolerance: 0,
          yaw_tolerance: 0,
        }],
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
      {
        name: 'mysterious_lift',
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
      }
    ],
  };
}

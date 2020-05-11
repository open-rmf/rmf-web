import * as RomiCore from '@osrf/romi-js-core-interfaces';
import airportPngUrl from './airport_terminal.png';
import officePngUrl from './office.png';

export default async function buildingMap(): Promise<RomiCore.BuildingMap> {
  let officePng: Uint8Array;
  let airportPng: Uint8Array;
  if (process.env.NODE_ENV === 'test') {
    const fs = await import('fs');
    officePng = fs.readFileSync(`${__dirname}/office.png`);
    airportPng = fs.readFileSync(`${__dirname}/airport_terminal.png`);
  } else {
    const officePngResp = await fetch(officePngUrl);
    officePng = new Uint8Array(await officePngResp.arrayBuffer());
    const airportPngResp = await fetch(airportPngUrl);
    airportPng = new Uint8Array(await airportPngResp.arrayBuffer());
  }

  const doors = [
    {
      name: 'main_door',
      v1_x: 8.2,
      v1_y: -5.5,
      v2_x: 7.85,
      v2_y: -6.2,
      door_type: 2,
      motion_range: -1.571,
      motion_direction: 1,
    },
    {
      name: 'hardware_door',
      v1_x: 4.9,
      v1_y: -4,
      v2_x: 4.4,
      v2_y: -5,
      door_type: 1,
      motion_range: 1.571,
      motion_direction: -1,
    },
    {
      name: 'coe_door',
      v1_x: 19.5,
      v1_y: -10.8,
      v2_x: 19.5,
      v2_y: -9.9,
      door_type: 1,
      motion_range: 1.571,
      motion_direction: 1,
    },
    {
      name: 'exit_door',
      v1_x: 12.2,
      v1_y: -2.7,
      v2_x: 14.1,
      v2_y: -2.7,
      door_type: 1,
      motion_range: -1.571,
      motion_direction: 1,
    },
  ];

  const lifts = [
    {
      name: 'Lift1',
      doors: [{
        name: 'lift1_front_door',
        v1_x: 8.80,
        v1_y: -2.3,
        v2_x: 7.70,
        v2_y: -4.5,
        door_type: 1,
        motion_range: 0,
        motion_direction: 1,
      }],
      levels: ['L1', 'L2', 'L3'],
      ref_x: 7.1,
      ref_y: -2.8,
      ref_yaw: 26,
      width: 2.5,
      depth: 2.5,
      wall_graph: {
        name: 'wallgraph',
        vertices: [],
        edges: [],
        params: [],
      },
    },
    {
      name: 'Lift2',
      doors: [
        {
          name: 'lift2_front_door',
          v1_x: 9,
          v1_y: -12.5,
          v2_x: 10,
          v2_y: -12.5,
          door_type: 1,
          motion_range: 0,
          motion_direction: 1,
        }
      ],
      levels: ['L1', 'L2', 'L3', 'L4'],
      ref_x: 9.5,
      ref_y: -13,
      ref_yaw: 1.571,
      width: 1,
      depth: 1,
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
      ref_x: 22,
      ref_y: -10,
      ref_yaw: 1.571,
      width: 1,
      depth: 1,
      wall_graph: {
        name: 'wallgraph',
        vertices: [],
        edges: [],
        params: [],
      },
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
            data: officePng,
            encoding: 'png',
            name: 'office',
            scale: 0.008465494960546494,
            x_offset: 0,
            y_offset: 0,
            yaw: 0,
          },
        ],
        places: [
          {
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
          },
        ],
        doors: doors,
        nav_graphs: [],
        wall_graph: {
          name: 'wallgraph',
          vertices: [],
          edges: [],
          params: [],
        },
      },
      {
        name: 'Airport Terminal',
        elevation: 10,
        doors: [],
        images: [
          {
            data: airportPng,
            encoding: 'png',
            name: 'airport_terminal',
            scale: 0.08212187141180038,
            x_offset: 0,
            y_offset: 0,
            yaw: 0,
          },
        ],
        nav_graphs: [],
        places: [],
        wall_graph: {
          name: 'wallgraph',
          vertices: [],
          edges: [],
          params: [],
        },
      },
    ],
    lifts: lifts
  };
}

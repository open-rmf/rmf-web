import React from 'react';
import ReactDOM from 'react-dom';
import DoorsPanel from '../doors-panel';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

const buildingMap: RomiCore.BuildingMap = {
  name: 'test building',
  levels: [
    {
      name: 'L1',
      elevation: 0,
      images: [],
      places: [],
      doors: [
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
      ],
      nav_graphs: [],
      wall_graph: {
        name: 'wallgraph',
        vertices: [],
        edges: [],
        params: [],
      },
    },
  ],
  lifts: [],
};

const doorStates: {[key: string]: RomiCore.DoorState} = {
  Door1: {
    door_name: 'Door1',
    current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
    door_time: { sec: 0, nanosec: 0 },
  },
  Door2: {
    door_name: 'Door2',
    current_mode: { value: RomiCore.DoorMode.MODE_CLOSED },
    door_time: { sec: 0, nanosec: 0 },
  },
};

ReactDOM.render(
  <DoorsPanel
    buildingMap={buildingMap}
    doorStates={doorStates}
  />,
  document.getElementById('root')
);

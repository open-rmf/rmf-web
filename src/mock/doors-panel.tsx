import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import ReactDOM from 'react-dom';
import DoorsPanel from '../doors-panel';
import buildingMap from './data/building-map';

const doorStates: { [key: string]: RomiCore.DoorState } = {
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
    onOpenClick={door => {
      console.log(`${door.name} open clicked`);
    }}
    onCloseClick={door => {
      console.log(`${door.name} close clicked`);
    }}
  />,
  document.getElementById('root'),
);

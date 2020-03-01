import React from 'react';
import ReactDOM from 'react-dom';

import './index.css';
import 'leaflet/dist/leaflet.css'
import { extendControlPositions } from './leaflet/control-positions';

import App from './App';
import OmniPanel from './omni-panel';
import DoorsPanel from './doors-panel';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as serviceWorker from './serviceWorker';
import { ClockSource } from './clock';
// import { WebSocketManager } from './util/websocket';

extendControlPositions();
// export const webSocketManager = new WebSocketManager('ws://localhost:8006')

/*
webSocketManager.addOnOpenCallback(async (event: Event) => {
  if (!webSocketManager.client) return
  webSocketManager.client.send(JSON.stringify({
    request: 'time',
    param: {}
  }))
})
*/

// webSocketManager.connect()

export const clockSource = new ClockSource()

/*
webSocketManager.addOnMessageCallback(async (event: WebSocketMessageEvent) => {
  clockSource.timeDiff =
})
*/

clockSource.start()

window.addEventListener('unload', (_event) => {
  // webSocketManager.disconnect()
  clockSource.stop()
})

// ReactDOM.render(<App />, document.getElementById('root'));
// ReactDOM.render(<OmniPanel />, document.getElementById('root'));
const doors = new Map<RomiCore.Door, RomiCore.DoorState>();
doors.set({
  name: 'door1',
  door_type: 1,
  motion_direction: 1,
  motion_range: 1,
  v1_x: 0,
  v1_y: 0,
  v2_x: 0,
  v2_y: 0,
}, {
  door_name: 'door1',
  current_mode: { value: RomiCore.DoorMode.MODE_CLOSED },
  door_time: { sec: 0, nanosec: 0 },
});
doors.set({
  name: 'door2',
  door_type: 1,
  motion_direction: 1,
  motion_range: 1,
  v1_x: 0,
  v1_y: 0,
  v2_x: 0,
  v2_y: 0,
}, {
  door_name: 'door1',
  current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
  door_time: { sec: 0, nanosec: 0 },
});

ReactDOM.render(<DoorsPanel doors={doors} />, document.getElementById('root'));

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();

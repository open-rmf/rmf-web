import React from 'react';
import ReactDOM from 'react-dom';
import LiftsPanel from '../lifts-panel';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import buildingMap from './data/building-map';

const liftStates: { [key: string]: RomiCore.LiftState } = {
  Lift1: {
    lift_name: 'Lift1',
    available_floors: ['L1', 'L2', 'L3'],
    available_modes: new Uint8Array([RomiCore.LiftState.MODE_AGV]),
    current_floor: 'L1',
    current_mode: RomiCore.LiftState.MODE_AGV,
    destination_floor: 'L1',
    door_state: RomiCore.LiftState.DOOR_OPEN,
    lift_time: { sec: 0, nanosec: 0 },
    motion_state: RomiCore.LiftState.MOTION_STOPPED,
    session_id: '',
  },
  Lift2: {
    lift_name: 'Lift2',
    available_floors: ['L1', 'L2', 'L3', 'L4'],
    available_modes: new Uint8Array([RomiCore.LiftState.MODE_AGV]),
    current_floor: 'L2',
    current_mode: RomiCore.LiftState.MODE_AGV,
    destination_floor: 'L4',
    door_state: RomiCore.LiftState.DOOR_CLOSED,
    lift_time: { sec: 0, nanosec: 0 },
    motion_state: RomiCore.LiftState.MOTION_UP,
    session_id: '',
  },
};

ReactDOM.render(
  <LiftsPanel buildingMap={buildingMap} liftStates={liftStates} />,
  document.getElementById('root'),
);

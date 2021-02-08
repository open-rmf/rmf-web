import React from 'react';
import ReactDOM from 'react-dom';
import MainMenu, { ItemState } from '../main-menu';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

const mockItemState: ItemState = {
  doors: {
    testDoor: {
      door_name: 'testDoor',
      door_time: { sec: 0, nanosec: 0 },
      current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
    },
  },
  dispensers: {
    testDispenser: {
      time: { sec: 0, nanosec: 0 },
      guid: 'testDispenser',
      mode: RomiCore.DispenserState.IDLE,
      request_guid_queue: [],
      seconds_remaining: 0,
    },
  },
  lifts: {
    testLift: {
      lift_name: 'testLift',
      lift_time: { sec: 0, nanosec: 0 },
      available_floors: [],
      current_floor: '',
      destination_floor: '',
      door_state: RomiCore.DoorMode.MODE_OPEN,
      motion_state: RomiCore.LiftState.MOTION_STOPPED,
      available_modes: new Uint8Array(),
      current_mode: RomiCore.LiftState.DOOR_OPEN,
      session_id: '',
    },
  },
  robots: {
    fleet: {
      name: 'fleet',
      robots: [
        {
          name: 'testRobot',
          model: 'model',
          task_id: '',
          battery_percent: 20,
          mode: { mode: RomiCore.RobotMode.MODE_MOVING },
          location: {
            level_name: 'L1',
            x: 0,
            y: 0,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
          },
          path: [],
        },
      ],
    },
  },
};

it('renders without crashing', () => {
  const div = document.createElement('div');
  ReactDOM.render(
    <MainMenu pushView={jest.fn()} itemState={mockItemState} tasks={[]} notifications={[]} />,
    div,
  );
  ReactDOM.unmountComponentAtNode(div);
});

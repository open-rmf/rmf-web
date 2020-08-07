import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

export const viewBoxCoords: string = '0 0 25.794363144785166 14.53525484725833';

export interface StyleTyping {
  [key: string]: React.CSSProperties;
}

export interface SampleStyleTyping {
  colorSample: {
    [key: string]: React.CSSProperties;
  };
}

export interface FormProps {
  errorState: boolean[];
  errorMessage: string[];
  labels: string[];
}

/***
 * Start of Door utils
 * Use door if you just need a door and doors if you need an array of doors
 * If you need just one door state and not the whole object, use doorStates.main_door
 */

const door: RomiCore.Door = {
  door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
  motion_direction: 1,
  motion_range: -1.571,
  name: 'main_door',
  v1_x: 10.8,
  v1_y: -2.3,
  v2_x: 7.7,
  v2_y: -5.5,
};

const doors: RomiCore.Door[] = [
  {
    door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
    motion_direction: 1,
    motion_range: -1.571,
    name: 'main_door',
    v1_x: 10.8,
    v1_y: -2.3,
    v2_x: 7.7,
    v2_y: -5.5,
  },
  {
    door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
    motion_direction: 1,
    motion_range: -1.571,
    name: 'second_door',
    v1_x: 10.8,
    v1_y: -2.3,
    v2_x: 7.7,
    v2_y: -5.5,
  },
  {
    door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
    motion_direction: 1,
    motion_range: -1.571,
    name: 'third_door',
    v1_x: 10.8,
    v1_y: -2.3,
    v2_x: 7.7,
    v2_y: -5.5,
  },
];

const doorStates: { [key: string]: RomiCore.DoorState } = {
  main_door: {
    current_mode: {
      value: RomiCore.DoorMode.MODE_CLOSED,
    },
    door_name: 'main_door',
    door_time: { sec: 0, nanosec: 0 },
  },
  second_door: {
    current_mode: {
      value: RomiCore.DoorMode.MODE_OPEN,
    },
    door_name: 'second_door',
    door_time: { sec: 0, nanosec: 0 },
  },
  third_door: {
    current_mode: {
      value: RomiCore.DoorMode.MODE_MOVING,
    },
    door_name: 'third_door',
    door_time: { sec: 0, nanosec: 0 },
  },
};

// end of door utils

export { door, doors, doorStates };

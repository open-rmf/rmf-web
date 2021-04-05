import React from 'react';
import * as RmfModels from 'rmf-models';
import * as L from 'leaflet';

import { colorPalette } from '../../util/css-utils';

export const viewBoxCoords: string = '0 0 25.794363144785166 14.53525484725833';
export const footprint: number = 0.5;
export const mapBound = new L.LatLngBounds([0, 25.794363144785166], [-17.53525484725833, 0]);
export const maxBound = new L.LatLngBounds(
  [12.84386068880558, 338.5063539594412],
  [-77.06316413283348, -56.417725659906864],
);

export interface StyleTyping {
  [key: string]: React.CSSProperties;
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
 * The above applies to the Dispenser, Lift and Robot util state
 */

const door: RmfModels.Door = {
  door_type: RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING,
  motion_direction: 1,
  motion_range: -1.571,
  name: 'main_door',
  v1_x: 10.8,
  v1_y: -2.3,
  v2_x: 7.7,
  v2_y: -5.5,
};

const doors: RmfModels.Door[] = [
  { ...door },
  { ...door, name: 'second_door' },
  { ...door, name: 'third_door' },
  { ...door, name: 'fourth_door' },
];

const doorStateTemplate: RmfModels.DoorState = {
  current_mode: {
    value: RmfModels.DoorMode.MODE_CLOSED,
  },
  door_name: 'main_door',
  door_time: { sec: 0, nanosec: 0 },
};

const doorStates: { [key: string]: RmfModels.DoorState } = {
  main_door: {
    ...doorStateTemplate,
  },
  second_door: {
    ...doorStateTemplate,
    current_mode: {
      value: RmfModels.DoorMode.MODE_OPEN,
    },
    door_name: 'second_door',
  },
  third_door: {
    ...doorStateTemplate,
    current_mode: {
      value: RmfModels.DoorMode.MODE_MOVING,
    },
    door_name: 'third_door',
  },
  fourth_door: {
    ...doorStateTemplate,
    current_mode: {
      value: 3,
    },
    door_name: 'fourth_door',
  },
};

// end of door utils

/***
 * Start of Dispenser Utils
 */
const dispenserStateTemplate: RmfModels.DispenserState = {
  guid: 'main_dispenser',
  mode: RmfModels.DispenserState.IDLE,
  request_guid_queue: [],
  seconds_remaining: 0,
  time: { sec: 0, nanosec: 0 },
};

const dispenserStates: { [key: string]: RmfModels.DispenserState } = {
  main_dispenser: { ...dispenserStateTemplate },
  second_dispenser: {
    ...dispenserStateTemplate,
    guid: 'second_dispenser',
    mode: RmfModels.DispenserState.BUSY,
  },
  third_dispenser: {
    ...dispenserStateTemplate,
    guid: 'third_dispenser',
    mode: RmfModels.DispenserState.OFFLINE,
  },
  fourth_dispenser: {
    ...dispenserStateTemplate,
    guid: 'fourth_dispenser',
    mode: 3,
  },
};

// end of dispenser utils

/***
 * Start of lift utils
 */

const lift: RmfModels.Lift = {
  depth: 2.5,
  doors: [
    {
      door_type: 1,
      motion_direction: 1,
      motion_range: 0,
      name: 'main_lift_front_door',
      v1_x: 8.8,
      v1_y: -2.3,
      v2_x: 7.7,
      v2_y: -4.5,
    },
  ],
  levels: ['L1', 'L2', 'L3', 'L4'],
  name: 'main_lift',
  ref_x: 7.1,
  ref_y: -2.8,
  ref_yaw: 0.5,
  wall_graph: {
    edges: [],
    name: 'wallgraph',
    params: [],
    vertices: [],
  },
  width: 2.5,
};

const lifts: RmfModels.Lift[] = [
  { ...lift },
  {
    ...lift,
    doors: [{ ...lift.doors[0], name: 'second_lift_front_door' }],
    name: 'second_lift',
  },
];

const liftStateTemplate: RmfModels.LiftState = {
  available_floors: ['L1', 'L2', 'L3', 'L4'],
  available_modes: new Uint8Array(0),
  current_floor: 'L1',
  current_mode: RmfModels.LiftState.MODE_UNKNOWN,
  destination_floor: 'L1',
  door_state: RmfModels.LiftState.DOOR_CLOSED,
  lift_name: 'main_lift',
  lift_time: { sec: 0, nanosec: 0 },
  motion_state: RmfModels.LiftState.MOTION_STOPPED,
  session_id: '',
};

const liftStates: { [key: string]: RmfModels.LiftState } = {
  main_lift: { ...liftStateTemplate },
  second_lift: {
    ...liftStateTemplate,
    lift_name: 'second_lift',
    motion_state: RmfModels.LiftState.MOTION_UP,
  },
};

// End of lift utils

/***
 * Start of Robot utils
 */
const robotState: RmfModels.RobotState = {
  battery_percent: 100,
  location: {
    level_name: 'L1',
    t: { sec: 0, nanosec: 0 },
    x: 8,
    y: -4,
    yaw: 0,
    index: 0,
  },
  mode: { mode: RmfModels.RobotMode.MODE_IDLE, mode_request_id: 0 },
  model: '40_hours',
  name: 'main_robot',
  path: [],
  task_id: 'taskA',
  seq: 0,
};

const robotStates: RmfModels.FleetState[] = [
  {
    name: 'main_fleet',
    robots: [
      { ...robotState },
      {
        ...robotState,
        name: 'second_robot',
        mode: { mode: RmfModels.RobotMode.MODE_CHARGING, mode_request_id: 0 },
      },
      {
        ...robotState,
        name: 'third_robot',
        mode: { mode: RmfModels.RobotMode.MODE_DOCKING, mode_request_id: 0 },
      },
      {
        ...robotState,
        name: 'fourth_robot',
        mode: { mode: RmfModels.RobotMode.MODE_EMERGENCY, mode_request_id: 0 },
      },
      {
        ...robotState,
        name: 'fifth_robot',
        mode: { mode: RmfModels.RobotMode.MODE_GOING_HOME, mode_request_id: 0 },
      },
      {
        ...robotState,
        name: 'sixth_robot',
        mode: { mode: RmfModels.RobotMode.MODE_MOVING, mode_request_id: 0 },
      },
      {
        ...robotState,
        name: 'seventh_robot',
        mode: { mode: RmfModels.RobotMode.MODE_PAUSED, mode_request_id: 0 },
      },
      {
        ...robotState,
        name: 'eigth_robot',
        mode: { mode: RmfModels.RobotMode.MODE_WAITING, mode_request_id: 0 },
      },
    ],
  },
];

// end of Robot utils

export const componentDisplayStyle: StyleTyping = {
  display: {
    display: 'grid',
    gridTemplateColumns: '1fr 3fr',
  },
  modeInfoPanel: {
    padding: '2rem',
  },
  modeInfoItem: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: '0.5rem',
  },
  modeInfoLink: {
    marginTop: '0.5rem',
    padding: '0.5rem',
  },
  aTag: {
    textDecoration: 'none',
    color: 'rgb(20, 116, 243)',
  },
};

export const defaultStyles: StyleTyping = {
  root: {
    margin: '0 auto',
    width: '40%',
  },
  heading: {
    padding: '0.5rem',
  },
  aTag: {
    textDecoration: 'none',
    color: 'rgb(20, 116, 243)',
  },
};

// colors used outside material ui
export const colorPaletteUtil: StyleTyping = {
  unknown: {
    backgroundColor: colorPalette.unknown,
  },
};

export {
  door,
  doors,
  doorStates,
  dispenserStates,
  lift,
  lifts,
  liftStates,
  robotState,
  robotStates,
};

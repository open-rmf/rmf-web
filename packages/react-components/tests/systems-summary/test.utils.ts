import { ItemSummary, Notification } from '../../lib';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

export const itemSummary: ItemSummary = {
  item: 'Door',
  summary: { operational: 0, outOfOrder: 0 },
  spoiltItemList: [],
};

export const notifications: Notification[] = [
  { id: 1, time: 'time', error: 'message', severity: 'High' },
];

export const tasks: RomiCore.TaskSummary[] = [
  {
    task_id: '1',
    state: 0,
    start_time: { sec: 0, nanosec: 0 },
    status: 'state',
    submission_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
  },
  {
    task_id: '2',
    state: 1,
    start_time: { sec: 0, nanosec: 0 },
    status: 'state',
    submission_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
  },
  {
    task_id: '3',
    state: 2,
    start_time: { sec: 0, nanosec: 0 },
    status: 'state',
    submission_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
  },
  {
    task_id: '4',
    state: 3,
    start_time: { sec: 0, nanosec: 0 },
    status: 'state',
    submission_time: { sec: 0, nanosec: 0 },
    end_time: { sec: 0, nanosec: 0 },
  },
];

export const door: RomiCore.Door = {
  name: 'door',
  v1_x: 8.2,
  v1_y: -5.5,
  v2_x: 7.85,
  v2_y: -6.2,
  door_type: 2,
  motion_range: -1.571,
  motion_direction: 1,
};

export const lift: RomiCore.Lift = {
  name: 'lift',
  doors: [door],
  levels: ['L1', 'L2', 'L3'],
  ref_x: 7.1,
  ref_y: -2.8,
  ref_yaw: -0.5,
  width: 2.5,
  depth: 2.5,
  wall_graph: {
    name: 'wallgraph',
    vertices: [],
    edges: [],
    params: [],
  },
};

export const fleet: RomiCore.FleetState = {
  name: 'fleet',
  robots: [
    {
      name: 'robot',
      model: 'Model1',
      mode: { mode: RomiCore.RobotMode.MODE_EMERGENCY },
      location: {
        level_name: 'L1',
        x: 4,
        y: -12,
        yaw: 0,
        t: { sec: 0, nanosec: 0 },
      },
      battery_percent: 100,
      path: [],
      task_id: 'task1',
    },
  ],
};

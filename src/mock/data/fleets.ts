import * as RomiCore from '@osrf/romi-js-core-interfaces';

const fleets: RomiCore.FleetState[] = [
  {
    name: 'Fleet1',
    robots: [
      {
        name: 'RobotA',
        model: 'ModelA',
        mode: { mode: RomiCore.RobotMode.MODE_MOVING },
        location: {
          level_name: 'L1',
          x: 0,
          y: 0,
          yaw: 0,
          t: { sec: 0, nanosec: 0 },
        },
        battery_percent: 100,
        path: [],
        task_id: 'taskA',
      },
      {
        name: 'RobotB',
        model: 'ModelB',
        mode: { mode: RomiCore.RobotMode.MODE_WAITING },
        location: {
          level_name: 'L1',
          x: 10,
          y: 10,
          yaw: 0,
          t: { sec: 0, nanosec: 0 },
        },
        battery_percent: 100,
        path: [],
        task_id: 'taskB',
      },
    ],
  },
  {
    name: 'Fleet2',
    robots: [
      {
        name: 'RobotC',
        model: 'ModelC',
        mode: { mode: RomiCore.RobotMode.MODE_EMERGENCY },
        location: {
          level_name: 'L2',
          x: 0,
          y: 0,
          yaw: 0,
          t: { sec: 0, nanosec: 0 },
        },
        battery_percent: 100,
        path: [],
        task_id: 'taskC',
      },
    ],
  },
];

export default fleets;

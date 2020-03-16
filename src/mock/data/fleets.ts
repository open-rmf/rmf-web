import * as RomiCore from '@osrf/romi-js-core-interfaces';

const robots: RomiCore.RobotState[] = [];
for (let i = 1; i <= 20; i++) {
  robots.push({
    name: `Geth${i}`,
    model: 'TeRmInAtOr',
    mode: { mode: RomiCore.RobotMode.MODE_MOVING },
    location: {
      level_name: 'L1',
      x: i,
      y: 0,
      yaw: (i * 0.1) % 2 * Math.PI,
      t: { sec: 0, nanosec: 0 },
    },
    battery_percent: 100,
    path: [],
    task_id: 'taskA',
  });
}

const superFleet: RomiCore.FleetState = {
  name: 'SuperFleet',
  robots: robots,
};

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
          y: -5,
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
          y: -10,
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
          y: -15,
          yaw: 0,
          t: { sec: 0, nanosec: 0 },
        },
        battery_percent: 100,
        path: [],
        task_id: 'taskC',
      },
    ],
  },
  superFleet,
];

export default fleets;

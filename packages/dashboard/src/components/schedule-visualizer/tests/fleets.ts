import * as RmfModels from 'rmf-models';

function superFleet(): RmfModels.FleetState {
  const robots: RmfModels.RobotState[] = [];
  for (let i = 1; i <= 20; i++) {
    robots.push({
      name: `Geth${i}`,
      model: 'TeRmInAtOr',
      mode: { mode: RmfModels.RobotMode.MODE_MOVING, mode_request_id: 0 },
      location: {
        level_name: 'L1',
        x: i,
        y: -2,
        yaw: ((i * 0.1) % 2) * Math.PI,
        t: { sec: 0, nanosec: 0 },
        index: 0,
      },
      battery_percent: 100,
      path: [],
      task_id: 'taskA',
      seq: 0,
    });
  }

  return {
    name: 'SuperFleet',
    robots: robots,
  };
}

export default function fakeFleets(): RmfModels.FleetState[] {
  return [
    {
      name: 'Fleet1',
      robots: [
        {
          name: 'Robot1',
          model: 'Model1',
          mode: { mode: RmfModels.RobotMode.MODE_EMERGENCY, mode_request_id: 0 },
          location: {
            level_name: 'L1',
            x: 4,
            y: -12,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
            index: 0,
          },
          battery_percent: 100,
          path: [],
          task_id: 'task1',
          seq: 0,
        },
      ],
    },
    {
      name: 'Fleet2',
      robots: [
        {
          name: 'Robot2',
          model: 'Model2',
          mode: { mode: RmfModels.RobotMode.MODE_EMERGENCY, mode_request_id: 0 },
          location: {
            level_name: 'L2',
            x: 4,
            y: -12,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
            index: 0,
          },
          battery_percent: 100,
          path: [],
          task_id: 'task2',
          seq: 0,
        },
      ],
    },
    {
      name: 'tinyRobot',
      robots: [
        {
          name: 'tinyRobot1',
          model: 'tinyRobot',
          mode: { mode: RmfModels.RobotMode.MODE_MOVING, mode_request_id: 0 },
          location: {
            level_name: 'L1',
            x: 2,
            y: -5,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
            index: 0,
          },
          battery_percent: 100,
          path: [],
          task_id: 'taskA',
          seq: 0,
        },
        {
          name: 'tinyRobot2',
          model: 'tinyRobot',
          mode: { mode: RmfModels.RobotMode.MODE_MOVING, mode_request_id: 0 },
          location: {
            level_name: 'L1',
            x: 12,
            y: -10,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
            index: 0,
          },
          battery_percent: 100,
          path: [],
          task_id: 'taskB',
          seq: 0,
        },
      ],
    },
    {
      name: 'FleetA',
      robots: [
        {
          name: 'RobotA',
          model: 'ModelA',
          mode: { mode: RmfModels.RobotMode.MODE_MOVING, mode_request_id: 0 },
          location: {
            level_name: 'L2',
            x: 2,
            y: -5,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
            index: 0,
          },
          battery_percent: 100,
          path: [],
          task_id: 'taskA',
          seq: 0,
        },
      ],
    },
    {
      name: 'FleetB',
      robots: [
        {
          name: 'RobotC',
          model: 'ModelC',
          mode: { mode: RmfModels.RobotMode.MODE_EMERGENCY, mode_request_id: 0 },
          location: {
            level_name: 'L2',
            x: 4,
            y: -12,
            yaw: 0,
            t: { sec: 0, nanosec: 0 },
            index: 0,
          },
          battery_percent: 100,
          path: [],
          task_id: 'taskC',
          seq: 0,
        },
      ],
    },
    superFleet(),
  ];
}

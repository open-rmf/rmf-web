import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { cleanup, render } from '@testing-library/react';
import React from 'react';
import { RobotAccordion } from '../../lib';
import { allRobotModes, makeRobot } from './test-utils';

it('smoke test with different robot modes', () => {
  allRobotModes()
    .map((mode) =>
      makeRobot({
        mode,
      }),
    )
    .forEach((robot) => {
      render(<RobotAccordion fleetName="test_fleet" robot={robot} />);
      cleanup();
    });
});

it('renders basic robot information', () => {
  const robot = makeRobot({
    name: 'test_robot',
    battery_percent: 1,
    location: { level_name: 'test_level', x: 0, y: 0, yaw: 0, t: { sec: 0, nanosec: 0 } },
    mode: { mode: RomiCore.RobotMode.MODE_PAUSED },
    model: 'test_model',
    task_id: 'test_task_id',
    path: [],
  });
  const accordion = render(<RobotAccordion fleetName="test_fleet" robot={robot} />);
  expect(accordion.queryAllByText('test_robot')).toHaveSize(2); // summary and details
  expect(accordion.queryByText('test_model')).toBeTruthy();
  expect(accordion.queryByText('test_fleet')).toBeTruthy();
  expect(accordion.queryByText('test_level (0.000, 0.000)')).toBeTruthy();
  expect(accordion.queryByText('0.000')).toBeTruthy(); // yaw
  expect(accordion.queryByText('test_task_id')).toBeTruthy();
  expect(accordion.queryByText('1')).toBeTruthy(); // battery
});

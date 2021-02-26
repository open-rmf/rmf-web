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

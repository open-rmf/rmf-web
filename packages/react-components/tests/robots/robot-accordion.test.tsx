import { render } from '@testing-library/react';
import React from 'react';
import { RobotAccordion } from '../../lib';
import { makeRobot } from './test-utils';

test('smoke test', () => {
  render(<RobotAccordion fleetName="test_fleet" robot={makeRobot()} />);
});

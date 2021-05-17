import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { RobotPanel } from '../../lib';
import { makeRandomRobot } from '../../tests/robots/test-utils';
import { makeDefinedTask } from '../test-data/tasks';

describe('RobotPanel', () => {
  it('shows detailed information when robot is clicked', () => {
    const tasks = [makeDefinedTask('Delivery', 'test_robot1', 'active_task_1', 3, 3)];
    const robots = [makeRandomRobot('test_robot1', 'test_fleet', 2)];
    const root = render(<RobotPanel robots={robots} tasks={tasks} />);
    userEvent.click(root.getByText('pickup_1'));
    root.getByRole('heading', { name: 'test_robot1' });
  });
});

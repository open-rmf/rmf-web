import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { Task } from 'api-client';
import React from 'react';
import { RobotPanel } from '../../lib';
import { makeRandomRobot } from '../../tests/robots/test-utils';
import { makeDefinedTask } from '../test-data/tasks';

describe('RobotPanel', () => {
  it('shows empty information when robot is clicked when there are no assigned tasks', () => {
    const tasks: Task[] = [];
    const robots = [makeRandomRobot('test_robot1', 'test_fleet', 2)];
    const root = render(<RobotPanel robots={robots} tasks={tasks} />);
    userEvent.click(root.getByText('test_robot1'));
    root.getByRole('heading', { name: 'test_robot1' });
    expect(root.getAllByRole('button', { name: '-' }).length).toBe(5);
  });

  it('shows detailed information when robot is clicked', () => {
    const tasks = [makeDefinedTask('Delivery', 'test_robot1', 'active_task_1', 3, 3)];
    const robots = [makeRandomRobot('test_robot1', 'test_fleet', 2)];
    const root = render(<RobotPanel robots={robots} tasks={tasks} />);
    userEvent.click(root.getByText('pickup_1'));
    root.getByRole('heading', { name: 'test_robot1' });
    root.getByRole('button', { name: 'active_task_1' });
  });
});

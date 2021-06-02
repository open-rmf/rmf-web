import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { TaskProgress } from 'api-client';
import React from 'react';
import { RobotPanel, RobotPanelProps } from '../../lib';
import { makeRandomRobot } from '../../tests/robots/test-utils';
import { makeDefinedTask } from '../test-data/tasks';

function makeFetchTasks(tasks: TaskProgress[]): RobotPanelProps['fetchTasks'] {
  return async (limit, offset) => {
    if (limit && offset) return tasks.slice(offset, offset + limit);
    else {
      return tasks;
    }
  };
}

describe('RobotPanel', () => {
  it('shows empty information when robot is clicked when there are no assigned tasks', () => {
    const robots = [makeRandomRobot('test_robot1', 'test_fleet', 2)];
    const root = render(<RobotPanel robots={robots} fetchTasks={makeFetchTasks([])} />);
    userEvent.click(root.getByText('test_robot1'));
    expect(root.getByRole('heading', { name: 'test_robot1' })).toBeTruthy();
    expect(root.getAllByRole('button', { name: '-' }).length).toBe(5);
  });

  it('shows detailed information when robot is clicked', () => {
    const tasks = [makeDefinedTask('Loop', 'test_robot1', 'task_1', 3, 3)];
    const robots = [makeRandomRobot('test_robot1', 'test_fleet', 2)];
    const root = render(<RobotPanel robots={robots} fetchTasks={makeFetchTasks(tasks)} />);
    userEvent.click(root.getByText('test_robot1'));
    expect(root.getByRole('progressbar')).toBeTruthy();
  });
});

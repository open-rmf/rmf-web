import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { makeDefinedTask } from '../tasks/test-data.spec';
import { RobotPanel, RobotPanelProps } from './robot-panel';
import { makeRandomRobot } from './test-utils.spec';
import { VerboseRobot } from './utils';

function makeFetchRobots(robots: VerboseRobot[]): RobotPanelProps['fetchVerboseRobots'] {
  return async () => {
    return robots;
  };
}

describe('RobotPanel', () => {
  it('shows empty information when robot is clicked when there are no assigned tasks', async () => {
    const robots = [makeRandomRobot('test_robot1', 'test_fleet', 2)];
    const root = render(
      <RobotPanel verboseRobots={robots} fetchVerboseRobots={makeFetchRobots(robots)} />,
    );
    await userEvent.click(root.getByText('test_robot1'));
    expect(root.getByRole('heading', { name: 'test_robot1' })).toBeTruthy();
    expect(root.getAllByRole('button', { name: '-' }).length).toBe(4);
  });

  it('shows detailed information when robot is clicked', async () => {
    const tasks = [
      { ...makeDefinedTask('Loop', 'test_robot1', 'task_1', 3, 3), progress: { status: '10%' } },
    ];
    const robot = makeRandomRobot('test_robot1', 'test_fleet', 2);
    const verboseRobot = [{ ...robot, tasks }];
    const root = render(
      <RobotPanel
        verboseRobots={verboseRobot}
        fetchVerboseRobots={makeFetchRobots(verboseRobot)}
      />,
    );
    await userEvent.click(root.getByText('test_robot1'));
    expect(root.getByRole('progressbar')).toBeTruthy();
  });
});

import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { makeTaskState } from './../tasks/make-tasks.test';
import { RobotsTable } from './robots-table';
import { makeRobot } from './test-utils.test';

describe('Robots table', () => {
  const rmfApi = new MockRmfApi();
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('Renders robots table', async () => {
    const mockReturnFleets = vi.fn().mockResolvedValue({
      data: [
        {
          name: 'test_fleet',
          robots: {
            ['test_robot1']: makeRobot({ name: 'test_robot1', task_id: 'test_task_id' }),
            ['test_robot2']: makeRobot({ name: 'test_robot2', task_id: undefined }),
          },
        },
      ],
    });
    const mockTaskState = makeTaskState('test_task_id');
    mockTaskState.unix_millis_finish_time = 0;
    const mockReturnTasks = vi.fn().mockResolvedValue({
      data: [mockTaskState],
    });

    rmfApi.fleetsApi.getFleetsFleetsGet = mockReturnFleets;
    rmfApi.tasksApi.queryTaskStatesTasksGet = mockReturnTasks;

    const root = render(
      <Base>
        <RobotsTable />
      </Base>,
    );

    await root.findByText(/test_robot1/i);
    expect(root.getByText(/test_robot1/i)).toBeTruthy();
    expect(root.getByText(/test_robot2/i)).toBeTruthy();
  });
});

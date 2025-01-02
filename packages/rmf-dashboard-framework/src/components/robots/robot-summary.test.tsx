import { waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { ApiServerModelsRmfApiRobotStateStatus as Status, FleetState } from 'api-client';
import { act } from 'react';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { makeTaskState } from './../tasks/make-tasks.test';
import { RobotSummary } from './robot-summary';
import { RobotTableData } from './robot-table-datagrid';
import { makeRobot } from './test-utils.test';

describe('Robot summary', () => {
  const rmfApi = new MockRmfApi();
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('Renders robot summary', async () => {
    const onCloseMock = vi.fn();
    const robotTableData: RobotTableData = {
      fleet: 'test_fleet',
      name: 'test_robot',
      status: Status.Idle,
      battery: 0.6,
      estFinishTime: 1000000,
      lastUpdateTime: 900000,
      level: 'L1',
      commission: {
        dispatch_tasks: true,
        direct_tasks: true,
        idle_behavior: true,
      },
    };

    const root = render(
      <Base>
        <RobotSummary onClose={onCloseMock} robot={robotTableData} />
      </Base>,
    );

    // Create the subject for the fleet
    rmfApi.getFleetStateObs('test_fleet');
    const mockFleetState: FleetState = {
      name: 'test_fleet',
      robots: {
        ['test_robot']: makeRobot({ name: 'test_robot', task_id: 'test_task_id' }),
      },
    };
    act(() => {
      rmfApi.fleetStateObsStore['test_fleet'].next(mockFleetState);
    });

    // Create the subject for the task
    rmfApi.getTaskStateObs('test_task_id');
    const mockTaskState = makeTaskState('test_task_id');
    act(() => {
      rmfApi.taskStateObsStore['test_task_id'].next(mockTaskState);
    });

    expect(root.getByText(/Robot summary/i)).toBeTruthy();
    expect(root.getByText(/test_robot/i)).toBeTruthy();
    expect(root.getByText(/Assigned tasks/i)).toBeTruthy();
    expect(root.getByText(/test_task_id/i)).toBeTruthy();
    userEvent.keyboard('{Escape}');
    await waitFor(() => expect(onCloseMock).toHaveBeenCalledTimes(1));
  });
});

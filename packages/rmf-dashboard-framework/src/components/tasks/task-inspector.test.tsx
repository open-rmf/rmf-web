import React from 'react';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { TaskInspector } from './task-inspector';
import { makeTaskLog, makeTaskState } from './test-data.test';

describe('Task inspector', () => {
  const rmfApi = new MockRmfApi();
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('Task inspector without task', async () => {
    const onClose = vi.fn();
    const root = render(
      <Base>
        <TaskInspector task={null} onClose={onClose} />
      </Base>,
    );
    expect(root.getByText(/Click on a task to view more information/i)).toBeTruthy();
  });

  it('Task inspector renders', async () => {
    const mockTaskState = makeTaskState('mock_task_id');
    const mockTaskLogs = makeTaskLog('mock_task_id');
    rmfApi.tasksApi.getTaskLogTasksTaskIdLogGet = vi.fn().mockResolvedValue({ data: mockTaskLogs });

    const onClose = vi.fn();
    const root = render(
      <Base>
        <TaskInspector task={mockTaskState} onClose={onClose} />
      </Base>,
    );

    expect(root.getByText(/mock_task_id/i)).toBeTruthy();
    expect(root.getByTestId('task-cancel-button')).toBeTruthy();
  });
});

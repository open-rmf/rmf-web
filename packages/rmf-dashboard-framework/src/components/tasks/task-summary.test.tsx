import React from 'react';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { TaskSummary } from './task-summary';
import { makeTaskState } from './test-data.test';

describe('Task Summary', () => {
  const rmfApi = new MockRmfApi();
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('Task Summary renders', async () => {
    const mockTaskState = makeTaskState('mock_task_id');

    const onClose = vi.fn();
    const root = render(
      <Base>
        <TaskSummary task={mockTaskState} onClose={onClose} />
      </Base>,
    );

    expect(root.getByText(/mock_task_id/i)).toBeTruthy();
  });
});

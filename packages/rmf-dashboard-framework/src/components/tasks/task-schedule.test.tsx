import React from 'react';
import { describe, it } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { TaskSchedule } from './task-schedule';

describe('Task schedule', () => {
  const rmfApi = new MockRmfApi();
  rmfApi.tasksApi.getScheduledTasksScheduledTasksGet = () => new Promise(() => {});
  rmfApi.tasksApi.addExceptDateScheduledTasksTaskIdExceptDatePost = () => new Promise(() => {});
  rmfApi.tasksApi.delScheduledTasksScheduledTasksTaskIdDelete = () => new Promise(() => {});

  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('Task schedule renders', async () => {
    render(
      <Base>
        <TaskSchedule />
      </Base>,
    );
  });
});

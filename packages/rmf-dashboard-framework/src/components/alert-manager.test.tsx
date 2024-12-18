import React from 'react';
import { describe, it, vi } from 'vitest';

import { RmfApiProvider } from '../hooks';
import { MockRmfApi, render, TestProviders } from '../utils/test-utils.test';
import { AlertManager } from './alert-manager';

describe('Alert manager', () => {
  const rmfApi = new MockRmfApi();
  rmfApi.alertsApi.getAlertResponseAlertsRequestAlertIdResponseGet = vi
    .fn()
    .mockResolvedValue({ data: [] });
  rmfApi.tasksApi.getTaskLogTasksTaskIdLogGet = () => new Promise(() => {});

  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('starts without crashing', () => {
    render(
      <Base>
        <AlertManager />
      </Base>,
    );
  });
});

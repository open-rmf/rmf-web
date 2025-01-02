import { AlertRequest, ApiServerModelsAlertsAlertRequestTier } from 'api-client';
import React from 'react';
import { describe, expect, it, vi } from 'vitest';

import { RmfApiProvider } from '../hooks';
import { MockRmfApi, render, TestProviders } from '../utils/test-utils.test';
import { AlertDialog, AlertManager } from './alert-manager';

describe('Alert dialog', () => {
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

  it('renders without crashing', () => {
    const alertRequest: AlertRequest = {
      id: 'test-alert',
      unix_millis_alert_time: 0,
      title: 'Test Alert',
      subtitle: 'Test subtitle',
      message: 'This is a test alert',
      tier: ApiServerModelsAlertsAlertRequestTier.Error,
      responses_available: ['ok'],
      display: true,
      task_id: 'test-task',
      alert_parameters: [],
    };
    const onDismiss = vi.fn();

    const root = render(
      <Base>
        <AlertDialog alertRequest={alertRequest} onDismiss={onDismiss} />
      </Base>,
    );
    expect(root.getByText('Test Alert')).toBeTruthy();
    expect(root.getByText('This is a test alert')).toBeTruthy();
    expect(root.getByTestId('test-alert-ok-button')).toBeTruthy();
    expect(root.getByTestId('task-cancel-button')).toBeTruthy();
    expect(root.getByTestId('dismiss-button')).toBeTruthy();
  });
});

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

import { Tab } from '@mui/material';
import { waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { describe, expect, it, vi } from 'vitest';

import { AuthenticatorProvider } from '../hooks/use-authenticator';
import { RmfApiProvider } from '../hooks/use-rmf-api';
import { RmfApi } from '../services/rmf-api';
import { StubAuthenticator } from '../services/stub-authenticator';
import { MockRmfApi, render, TestProviders } from '../utils/test-utils.test';
import AppBar from './appbar';

describe('AppBar', () => {
  const Base = (props: React.PropsWithChildren<{}>) => {
    const rmfApi = React.useMemo<RmfApi>(() => {
      const mockRmfApi = new MockRmfApi();
      // mock out some api calls so they never resolves
      mockRmfApi.tasksApi.getFavoritesTasksFavoriteTasksGet = () => new Promise(() => {});
      mockRmfApi.alertsApi.getUnrespondedAlertsAlertsUnrespondedRequestsGet = () =>
        new Promise(() => {});
      mockRmfApi.buildingApi.getPreviousFireAlarmTriggerBuildingMapPreviousFireAlarmTriggerGet =
        () => new Promise(() => {});
      return mockRmfApi;
    }, []);
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders with navigation bar', () => {
    const root = render(
      <Base>
        <AppBar
          tabs={[<Tab key="test" label="test" value="test" />]}
          tabValue="test"
          helpLink=""
          reportIssueLink=""
        />
      </Base>,
    );
    expect(root.getAllByRole('tablist').length > 0).toBeTruthy();
  });

  it('user button is shown when there is an authenticated user', () => {
    const root = render(
      <Base>
        <AppBar
          tabs={[<Tab key="test" label="test" value="test" />]}
          tabValue="test"
          helpLink=""
          reportIssueLink=""
        />
      </Base>,
    );
    expect(root.getByLabelText('user-btn')).toBeTruthy();
  });

  it('logout is triggered when logout button is clicked', async () => {
    const authenticator = new StubAuthenticator();
    const spy = vi.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
    const root = render(
      <Base>
        <AuthenticatorProvider value={authenticator}>
          <AppBar
            tabs={[<Tab key="test" label="test" value="test" />]}
            tabValue="test"
            helpLink=""
            reportIssueLink=""
          />
        </AuthenticatorProvider>
      </Base>,
    );
    userEvent.click(root.getByLabelText('user-btn'));
    await expect(waitFor(() => root.getByText('Logout'))).resolves.not.toThrow();
    userEvent.click(root.getByText('Logout'));
    await expect(waitFor(() => expect(spy).toHaveBeenCalledTimes(1))).resolves.not.toThrow();
  });

  it('uses headerLogo from logo resources manager', async () => {
    const root = render(
      <Base>
        <AppBar
          tabs={[<Tab key="test" label="test" value="test" />]}
          tabValue="test"
          helpLink=""
          reportIssueLink=""
        />
      </Base>,
    );
    await expect(
      waitFor(() => {
        const q = root.container.querySelector('[src="/test-logo.png"][alt="logo"]');
        if (!q) throw new Error();
        return q;
      }),
    ).resolves.not.toThrow();
  });
});

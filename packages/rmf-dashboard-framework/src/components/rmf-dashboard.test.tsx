import React from 'react';
import {
  InitialWindow,
  LocallyPersistentWorkspace,
  MicroAppManifest,
  Workspace,
} from 'rmf-dashboard-framework/components';
import {
  createMapApp,
  doorsApp,
  liftsApp,
  robotMutexGroupsApp,
  robotsApp,
  tasksApp,
} from 'rmf-dashboard-framework/micro-apps';
import { StubAuthenticator } from 'rmf-dashboard-framework/services';
import { describe, it, vi } from 'vitest';

import { RmfApiProvider } from '../hooks';
import { MockRmfApi, render, TestProviders } from '../utils/test-utils.test';
import { RmfDashboard } from './rmf-dashboard';

describe('RmfDashboard', () => {
  const rmfApi = new MockRmfApi();
  rmfApi.tasksApi.queryTaskStatesTasksGet = vi.fn().mockResolvedValue({ data: [] });

  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  const mapApp = createMapApp({
    attributionPrefix: 'Open-RMF',
    defaultMapLevel: 'L1',
    defaultRobotZoom: 20,
    defaultZoom: 6,
  });

  const appRegistry: MicroAppManifest[] = [
    mapApp,
    doorsApp,
    liftsApp,
    robotsApp,
    robotMutexGroupsApp,
    tasksApp,
  ];

  const homeWorkspace: InitialWindow[] = [
    {
      layout: { x: 0, y: 0, w: 12, h: 6 },
      microApp: mapApp,
    },
  ];

  const robotsWorkspace: InitialWindow[] = [
    {
      layout: { x: 0, y: 0, w: 7, h: 4 },
      microApp: robotsApp,
    },
    { layout: { x: 8, y: 0, w: 5, h: 8 }, microApp: mapApp },
    { layout: { x: 0, y: 0, w: 7, h: 4 }, microApp: doorsApp },
    { layout: { x: 0, y: 0, w: 7, h: 4 }, microApp: liftsApp },
    { layout: { x: 8, y: 0, w: 5, h: 4 }, microApp: robotMutexGroupsApp },
  ];

  const tasksWorkspace: InitialWindow[] = [
    { layout: { x: 0, y: 0, w: 7, h: 8 }, microApp: tasksApp },
    { layout: { x: 8, y: 0, w: 5, h: 8 }, microApp: mapApp },
  ];

  it('renders without crashing', () => {
    render(
      <Base>
        <RmfDashboard
          apiServerUrl="http://localhost:8000"
          trajectoryServerUrl="http://localhost:8006"
          authenticator={new StubAuthenticator()}
          helpLink="https://osrf.github.io/ros2multirobotbook/rmf-core.html"
          reportIssueLink="https://github.com/open-rmf/rmf-web/issues"
          resources={{ fleets: {}, logos: { header: '/resources/defaultLogo.png' } }}
          tasks={{
            allowedTasks: [
              { taskDefinitionId: 'patrol' },
              { taskDefinitionId: 'delivery' },
              { taskDefinitionId: 'compose-clean' },
              { taskDefinitionId: 'custom_compose' },
            ],
            pickupZones: [],
            cartIds: [],
          }}
          tabs={[
            {
              name: 'Map',
              route: '',
              element: <Workspace initialWindows={homeWorkspace} />,
            },
            {
              name: 'Robots',
              route: 'robots',
              element: <Workspace initialWindows={robotsWorkspace} />,
            },
            {
              name: 'Tasks',
              route: 'tasks',
              element: <Workspace initialWindows={tasksWorkspace} />,
            },
            {
              name: 'Custom',
              route: 'custom',
              element: (
                <LocallyPersistentWorkspace
                  defaultWindows={[]}
                  allowDesignMode
                  appRegistry={appRegistry}
                  storageKey="custom-workspace"
                />
              ),
            },
          ]}
        />
      </Base>,
    );
  });
});

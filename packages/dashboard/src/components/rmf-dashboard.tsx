import { Tab, Typography } from '@mui/material';
import React, { startTransition, useTransition } from 'react';
import { getDefaultTaskDefinition, LocalizationProvider } from 'react-components';
import { RouterProvider } from 'react-router';
import { createBrowserRouter } from 'react-router-dom';

import { AuthenticatorProvider } from '../hooks/use-authenticator';
import { Resources, ResourcesProvider } from '../hooks/use-resources';
import { RmfApiProvider } from '../hooks/use-rmf-api';
import { TaskRegistry, TaskRegistryProvider } from '../hooks/use-task-registry';
import { UserProfileProvider } from '../hooks/use-user-profile';
import { Authenticator, UserProfile } from '../services/authenticator';
import { RmfApi } from '../services/rmf-api';
import AppBar, { APP_BAR_HEIGHT } from './appbar';

export * from '../services/rmf-api';

export interface DashboardHome {}

export interface DashboardTab {
  name: string;
  route: string;
  element: React.ReactNode;
}

export interface AllowedTask {
  /**
   * The task definition to configure.
   */
  taskDefinitionId: 'patrol' | 'delivery' | 'compose-clean' | 'custom_compose';

  /**
   * Configure the display name for the task definition.
   */
  displayName?: string;
}

export interface TaskRegistryInput extends Omit<TaskRegistry, 'taskDefinitions'> {
  allowedTasks: AllowedTask[];
}

export interface RmfDashboardProps {
  /**
   * Url of the RMF api server.
   */
  apiServerUrl: string;

  /**
   * Url of the RMF trajectory server.
   */
  trajectoryServerUrl: string;

  authenticator: Authenticator;

  /**
   * Url to be linked for the "help" button.
   */
  helpLink: string;

  /**
   * Url to be linked for the "report issue" button.
   */
  reportIssueLink: string;

  resources: Resources;
  tasks: TaskRegistryInput;

  /**
   * List of tabs on the app bar.
   */
  tabs: DashboardTab[];
}

export function RmfDashboard({
  apiServerUrl,
  trajectoryServerUrl,
  authenticator,
  helpLink,
  reportIssueLink,
  resources,
  tasks,
  tabs,
}: RmfDashboardProps) {
  const rmfApi = React.useMemo(
    () => new RmfApi(apiServerUrl, trajectoryServerUrl, authenticator),
    [apiServerUrl, trajectoryServerUrl, authenticator],
  );

  // FIXME(koonepng): This should be fully definition in tasks resources when the dashboard actually
  // supports configuring all the fields.
  const taskRegistry = React.useMemo<TaskRegistry>(
    () => ({
      taskDefinitions: tasks.allowedTasks.map((t) => {
        const defaultTaskDefinition = getDefaultTaskDefinition(t.taskDefinitionId);
        if (!defaultTaskDefinition) {
          throw Error(`Invalid tasks configured for dashboard: [${t.taskDefinitionId}]`);
        }
        if (t.displayName !== undefined) {
          return {
            ...defaultTaskDefinition,
            taskDisplayName: t.displayName,
          };
        } else {
          return defaultTaskDefinition;
        }
      }),
      pickupZones: tasks.pickupZones,
      cartIds: tasks.cartIds,
    }),
    [tasks.allowedTasks, tasks.pickupZones, tasks.cartIds],
  );

  const [userProfile, setUserProfile] = React.useState<UserProfile | null>(null);
  React.useEffect(() => {
    (async () => {
      await authenticator.init();
      const user = (await rmfApi.defaultApi.getUserUserGet()).data;
      const perm = (await rmfApi.defaultApi.getEffectivePermissionsPermissionsGet()).data;
      setUserProfile({ user, permissions: perm });
    })();
  }, [authenticator, rmfApi]);

  const [tabValue, setTabValue] = React.useState(tabs[0].name);

  const WithTabControl = React.useCallback(({ t }: { t: DashboardTab }) => {
    setTabValue(t.name);
    return t.element;
  }, []);

  const router = React.useMemo(
    () =>
      createBrowserRouter(
        tabs.map((t) => ({
          path: t.route,
          element: <WithTabControl t={t} />,
        })),
      ),
    [tabs, WithTabControl],
  );

  const [pendingTransition, startTransition] = useTransition();

  const AllThemProviders = React.useCallback(
    ({ children }: React.PropsWithChildren<{}>) =>
      userProfile && (
        <LocalizationProvider>
          <AuthenticatorProvider value={authenticator}>
            <ResourcesProvider value={resources}>
              <TaskRegistryProvider value={taskRegistry}>
                <RmfApiProvider value={rmfApi}>
                  <UserProfileProvider value={userProfile}>{children}</UserProfileProvider>
                </RmfApiProvider>
              </TaskRegistryProvider>
            </ResourcesProvider>
          </AuthenticatorProvider>
        </LocalizationProvider>
      ),
    [authenticator, resources, rmfApi, taskRegistry, userProfile],
  );

  return (
    <AllThemProviders>
      <AppBar
        tabs={tabs.map((t) => (
          <Tab
            key={t.name}
            sx={{ height: APP_BAR_HEIGHT }}
            label={<Typography variant="h6">{t.name}</Typography>}
            value={t.name}
            onClick={() => {
              setTabValue(t.name);
              startTransition(() => {
                router.navigate(t.route);
              });
            }}
          />
        ))}
        tabValue={tabValue}
        helpLink={helpLink}
        reportIssueLink={reportIssueLink}
      ></AppBar>
      {!pendingTransition && <RouterProvider router={router} />}
    </AllThemProviders>
  );
}

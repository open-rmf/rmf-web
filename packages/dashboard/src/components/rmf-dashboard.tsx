import { Box, Container, Tab, Typography } from '@mui/material';
import React, { useTransition } from 'react';
import { getDefaultTaskDefinition, LocalizationProvider } from 'react-components';
import { Navigate, Route, Routes, useNavigate } from 'react-router';
import { BrowserRouter } from 'react-router-dom';

import { authenticator } from '../app-config';
import { AuthenticatorProvider } from '../hooks/use-authenticator';
import { Resources, ResourcesProvider } from '../hooks/use-resources';
import { RmfApiProvider } from '../hooks/use-rmf-api';
import { TaskRegistry, TaskRegistryProvider } from '../hooks/use-task-registry';
import { UserProfileProvider, useUserProfile } from '../hooks/use-user-profile';
import { LoginPage } from '../pages';
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

  /**
   * Set various resources (icons, logo etc) used. Different resource can be used based on the theme, `default` is always required.
   */
  resources: Resources;

  /**
   * List of allowed tasks that can be requested
   */
  tasks: TaskRegistryInput;

  /**
   * List of tabs on the app bar.
   */
  tabs: DashboardTab[];

  /**
   * Prefix where other routes will be based on, defaults to `import.meta.env.BASE_URL`.
   * Must end with a slash
   */
  baseUrl?: string;
}

export function RmfDashboard(props: RmfDashboardProps) {
  const { apiServerUrl, trajectoryServerUrl, authenticator, resources, tasks } = props;

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

  return (
    userProfile && (
      <LocalizationProvider>
        <AuthenticatorProvider value={authenticator}>
          <ResourcesProvider value={resources}>
            <TaskRegistryProvider value={taskRegistry}>
              <RmfApiProvider value={rmfApi}>
                <UserProfileProvider value={userProfile}>
                  <BrowserRouter>
                    <DashboardContents {...props} />
                  </BrowserRouter>
                </UserProfileProvider>
              </RmfApiProvider>
            </TaskRegistryProvider>
          </ResourcesProvider>
        </AuthenticatorProvider>
      </LocalizationProvider>
    )
  );
}

interface RequireAuthProps {
  redirectTo: string;
  children: React.ReactNode;
}

function RequireAuth({ redirectTo, children }: RequireAuthProps) {
  const userProfile = useUserProfile();
  return userProfile ? children : <Navigate to={redirectTo} />;
}

function NotFound() {
  return (
    <Container maxWidth="md" sx={{ textAlign: 'center', py: 8 }}>
      <Typography variant="h3" gutterBottom>
        404 - Not Found
      </Typography>
      <Typography variant="body1" color="textSecondary" gutterBottom>
        The page you're looking for doesn't exist.
      </Typography>
    </Container>
  );
}

function DashboardContents({
  helpLink,
  reportIssueLink,
  resources,
  tabs,
  baseUrl = import.meta.env.BASE_URL,
}: RmfDashboardProps) {
  const [tabValue, setTabValue] = React.useState(tabs[0].name);

  const WithTabControl = React.useCallback(({ t }: { t: DashboardTab }) => {
    setTabValue(t.name);
    return t.element;
  }, []);

  const [pendingTransition, startTransition] = useTransition();
  const navigate = useNavigate();

  // TODO(koonpeng): enable admin tab when authz is implemented.
  const allTabs = tabs;
  // const allTabs = React.useMemo<DashboardTab[]>(
  //   () => [...tabs, { name: 'Admin', route: 'admin', element: <AdminDrawer /> }],
  //   [tabs],
  // );

  return (
    <>
      <AppBar
        tabs={allTabs.map((t) => (
          <Tab
            key={t.name}
            sx={{ height: APP_BAR_HEIGHT }}
            label={<Typography variant="h6">{t.name}</Typography>}
            value={t.name}
            onClick={() => {
              setTabValue(t.name);
              startTransition(() => {
                navigate(t.route);
              });
            }}
          />
        ))}
        tabValue={tabValue}
        helpLink={helpLink}
        reportIssueLink={reportIssueLink}
      />
      <Box sx={{ marginTop: APP_BAR_HEIGHT }} />
      {!pendingTransition && (
        <Routes>
          <Route path={baseUrl}>
            <Route
              path="login"
              element={
                <LoginPage
                  title="Dashboard"
                  logo={resources.logos.header}
                  onLoginClick={() => authenticator.login(`${window.location.origin}${baseUrl}`)}
                />
              }
            />
            {allTabs.map((t) => (
              <Route
                key={t.name}
                path={t.route}
                element={
                  <RequireAuth redirectTo={`${baseUrl}login`}>
                    <WithTabControl t={t} />
                  </RequireAuth>
                }
              />
            ))}
            <Route path="*" element={<NotFound />} />
          </Route>
          <Route path="*" element={<NotFound />} />
        </Routes>
      )}
    </>
  );
}

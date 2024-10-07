import {
  Alert,
  AlertProps,
  Container,
  CssBaseline,
  Snackbar,
  Tab,
  ThemeProvider,
  Typography,
} from '@mui/material';
import React, { useTransition } from 'react';
import { getDefaultTaskDefinition, LocalizationProvider } from 'react-components';
import { matchPath, Navigate, Outlet, Route, Routes, useLocation, useNavigate } from 'react-router';
import { BrowserRouter } from 'react-router-dom';

import { AppController, AppControllerProvider } from '../hooks/use-app-controller';
import { AuthenticatorProvider } from '../hooks/use-authenticator';
import { Resources, ResourcesProvider } from '../hooks/use-resources';
import { RmfApiProvider } from '../hooks/use-rmf-api';
import { SettingsProvider } from '../hooks/use-settings';
import { TaskRegistry, TaskRegistryProvider } from '../hooks/use-task-registry';
import { UserProfileProvider } from '../hooks/use-user-profile';
import { LoginPage } from '../pages';
import { Authenticator, UserProfile } from '../services/authenticator';
import { DefaultRmfApi } from '../services/rmf-api';
import { loadSettings, saveSettings, Settings } from '../services/settings';
import { AlertManager } from './alert-manager';
import AppBar, { APP_BAR_HEIGHT } from './appbar';
import { DeliveryAlertStore } from './delivery-alert-store';
import { DashboardThemes } from './theme';

const DefaultAlertDuration = 2000;

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

  /**
   * The color of the event when rendered on the task scheduler in the form of a CSS color string.
   */
  scheduleEventColor?: string;
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

  themes?: DashboardThemes;

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

  /**
   * Url to a file to be played when an alert occurs on the dashboard.
   */
  alertAudioPath?: string;
}

export function RmfDashboard(props: RmfDashboardProps) {
  const {
    apiServerUrl,
    trajectoryServerUrl,
    authenticator,
    themes,
    resources,
    tasks,
    baseUrl = import.meta.env.BASE_URL,
    alertAudioPath,
  } = props;

  const rmfApi = React.useMemo(
    () => new DefaultRmfApi(apiServerUrl, trajectoryServerUrl, authenticator),
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
        const taskDefinition = { ...defaultTaskDefinition };
        if (t.displayName !== undefined) {
          taskDefinition.taskDisplayName = t.displayName;
        }
        if (t.scheduleEventColor !== undefined) {
          taskDefinition.scheduleEventColor = t.scheduleEventColor;
        }
        return taskDefinition;
      }),
      pickupZones: tasks.pickupZones,
      cartIds: tasks.cartIds,
    }),
    [tasks.allowedTasks, tasks.pickupZones, tasks.cartIds],
  );

  const [userProfile, setUserProfile] = React.useState<UserProfile | null>(null);
  const [authReady, setAuthReady] = React.useState(false);
  React.useEffect(() => {
    (async () => {
      await authenticator.init();
      if (!authenticator.user) {
        setUserProfile(null);
        setAuthReady(true);
        return;
      }
      try {
        const user = (await rmfApi.defaultApi.getUserUserGet()).data;
        const perm = (await rmfApi.defaultApi.getEffectivePermissionsPermissionsGet()).data;
        setUserProfile({ user, permissions: perm });
      } catch (e) {
        console.error(e);
        setUserProfile(null);
      } finally {
        setAuthReady(true);
      }
    })();
  }, [authenticator, rmfApi]);

  const [settings, setSettings] = React.useState(() => loadSettings());
  const updateSettings = React.useCallback((newSettings: Settings) => {
    saveSettings(newSettings);
    setSettings(newSettings);
  }, []);

  const [showAlert, setShowAlert] = React.useState(false);
  const [alertSeverity, setAlertSeverity] = React.useState<AlertProps['severity']>('error');
  const [alertMessage, setAlertMessage] = React.useState('');
  const [alertDuration, setAlertDuration] = React.useState(DefaultAlertDuration);
  const [extraAppbarItems, setExtraAppbarItems] = React.useState<React.ReactNode>(null);
  const appController = React.useMemo<AppController>(
    () => ({
      updateSettings,
      showAlert: (severity, message, autoHideDuration) => {
        setAlertSeverity(severity);
        setAlertMessage(message);
        setShowAlert(true);
        setAlertDuration(autoHideDuration || DefaultAlertDuration);
      },
      setExtraAppbarItems,
    }),
    [updateSettings],
  );

  const theme = React.useMemo(() => {
    if (!themes) {
      return null;
    }
    return themes[settings.themeMode] || themes.default;
  }, [themes, settings.themeMode]);

  const providers = authReady && (
    <BrowserRouter>
      <Routes>
        <Route
          path={`${baseUrl}login`}
          element={
            userProfile ? (
              <Navigate to={baseUrl} />
            ) : (
              <LoginPage
                title="Dashboard"
                logo={resources.logos.header}
                onLoginClick={() => authenticator.login(`${window.location.origin}${baseUrl}`)}
              />
            )
          }
        />
        <Route
          path="*"
          element={userProfile ? <Outlet /> : <Navigate to={`${baseUrl}login`} />}
        ></Route>
      </Routes>
      {userProfile && (
        <LocalizationProvider>
          <CssBaseline />
          <AuthenticatorProvider value={authenticator}>
            <UserProfileProvider value={userProfile}>
              <ResourcesProvider value={resources}>
                <TaskRegistryProvider value={taskRegistry}>
                  <RmfApiProvider value={rmfApi}>
                    <SettingsProvider value={settings}>
                      <AppControllerProvider value={appController}>
                        <DeliveryAlertStore />
                        <AlertManager alertAudioPath={alertAudioPath} />
                        <DashboardContents {...props} extraAppbarItems={extraAppbarItems} />
                        {/* TODO: Support stacking of alerts */}
                        <Snackbar
                          open={showAlert}
                          message={alertMessage}
                          onClose={() => setShowAlert(false)}
                          autoHideDuration={alertDuration}
                        >
                          <Alert
                            onClose={() => setShowAlert(false)}
                            severity={alertSeverity}
                            sx={{ width: '100%' }}
                          >
                            {alertMessage}
                          </Alert>
                        </Snackbar>
                      </AppControllerProvider>
                    </SettingsProvider>
                  </RmfApiProvider>
                </TaskRegistryProvider>
              </ResourcesProvider>
            </UserProfileProvider>
          </AuthenticatorProvider>
        </LocalizationProvider>
      )}
    </BrowserRouter>
  );

  return theme ? <ThemeProvider theme={theme}>{providers}</ThemeProvider> : providers;
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

interface DashboardContentsProps extends RmfDashboardProps {
  extraAppbarItems: React.ReactNode;
}

function DashboardContents({
  helpLink,
  reportIssueLink,
  themes,
  tabs,
  baseUrl = import.meta.env.BASE_URL,
  extraAppbarItems,
}: DashboardContentsProps) {
  const location = useLocation();
  const currentTab = tabs.find((t) => matchPath(`${baseUrl}${t.route}`, location.pathname));

  const [pendingTransition, startTransition] = useTransition();
  const navigate = useNavigate();

  // TODO(koonpeng): enable admin tab when authz is implemented.
  const allTabs = tabs;
  // const allTabs = React.useMemo<DashboardTab[]>(
  //   () => [...tabs, { name: 'Admin', route: 'admin', element: <AdminDrawer /> }],
  //   [tabs],
  // );

  return (
    <Routes>
      <Route path={baseUrl}>
        <Route
          element={
            <>
              <AppBar
                tabs={allTabs.map((t) => (
                  <Tab
                    key={t.name}
                    sx={{ height: APP_BAR_HEIGHT }}
                    label={<Typography variant="h6">{t.name}</Typography>}
                    value={t.name}
                    onClick={() => {
                      startTransition(() => {
                        navigate(`${baseUrl}${t.route}`);
                      });
                    }}
                  />
                ))}
                tabValue={currentTab!.name}
                themes={themes}
                helpLink={helpLink}
                reportIssueLink={reportIssueLink}
                extraToolbarItems={extraAppbarItems}
              />
              {!pendingTransition && <Outlet />}
            </>
          }
        >
          {allTabs.map((t) => (
            <Route key={t.name} path={t.route} element={t.element} />
          ))}
        </Route>
        <Route path="*" element={<NotFound />} />
      </Route>
      <Route path="*" element={<NotFound />} />
    </Routes>
  );
}

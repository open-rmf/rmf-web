import { AccountCircle, AddOutlined, Notifications, Settings } from '@mui/icons-material';
import {
  Badge,
  Button,
  CardContent,
  Divider,
  FormControl,
  FormControlLabel,
  FormLabel,
  IconButton,
  Menu,
  MenuItem,
  Radio,
  RadioGroup,
  Toolbar,
  Tooltip,
  Typography,
} from '@mui/material';
import {
  ApiServerModelsTortoiseModelsAlertsAlertLeaf as Alert,
  TaskRequest,
  TaskFavoritePydantic as TaskFavorite,
} from 'api-client';
import React from 'react';
import {
  AppBarTab,
  CreateTaskForm,
  CreateTaskFormProps,
  getPlaces,
  HeaderBar,
  LogoButton,
  NavigationBar,
  useAsync,
} from 'react-components';
import { useHistory, useLocation } from 'react-router-dom';
import { UserProfileContext } from 'rmf-auth';
import { logoSize } from '../managers/resource-manager';
import { ThemeMode } from '../settings';
import {
  AdminRoute,
  CustomRoute1,
  CustomRoute2,
  DashboardRoute,
  RobotsRoute,
  TasksRoute,
} from '../util/url';
import {
  AppConfigContext,
  AppControllerContext,
  ResourcesContext,
  SettingsContext,
} from './app-contexts';
import { RmfAppContext } from './rmf-app';
import { parseTasksFile } from './tasks/utils';
import { AppEvents } from './app-events';
import { Subscription } from 'rxjs';
import { format } from 'date-fns';

export type TabValue = 'infrastructure' | 'robots' | 'tasks' | 'custom1' | 'custom2' | 'admin';

function locationToTabValue(pathname: string): TabValue | undefined {
  // `DashboardRoute` being the root, it is a prefix to all routes, so we need to check exactly.
  if (pathname === DashboardRoute) return 'infrastructure';
  if (pathname.startsWith(RobotsRoute)) return 'robots';
  if (pathname.startsWith(TasksRoute)) return 'tasks';
  if (pathname.startsWith(CustomRoute1)) return 'custom1';
  if (pathname.startsWith(CustomRoute2)) return 'custom2';
  if (pathname.startsWith(AdminRoute)) return 'admin';
  return undefined;
}

function AppSettings() {
  const settings = React.useContext(SettingsContext);
  const appController = React.useContext(AppControllerContext);
  return (
    <FormControl>
      <FormLabel id="theme-label">Theme</FormLabel>
      <RadioGroup row aria-labelledby="theme-label">
        <FormControlLabel
          value={ThemeMode.Default}
          control={<Radio />}
          label="Default"
          checked={settings.themeMode === ThemeMode.Default}
          onChange={() =>
            appController.updateSettings({ ...settings, themeMode: ThemeMode.Default })
          }
        />
        <FormControlLabel
          value={ThemeMode.RmfLight}
          control={<Radio />}
          label="RMF Light"
          checked={settings.themeMode === ThemeMode.RmfLight}
          onChange={() =>
            appController.updateSettings({ ...settings, themeMode: ThemeMode.RmfLight })
          }
        />
        <FormControlLabel
          value={ThemeMode.RmfDark}
          control={<Radio />}
          label="RMF Dark"
          checked={settings.themeMode === ThemeMode.RmfDark}
          onChange={() =>
            appController.updateSettings({ ...settings, themeMode: ThemeMode.RmfDark })
          }
        />
      </RadioGroup>
    </FormControl>
  );
}

export interface AppBarProps {
  extraToolbarItems?: React.ReactNode;

  // TODO: change the alarm status to required when we have an alarm
  // service working properly in the backend
  alarmState?: boolean | null;
}

export const AppBar = React.memo(({ extraToolbarItems }: AppBarProps): React.ReactElement => {
  const rmf = React.useContext(RmfAppContext);
  const resourceManager = React.useContext(ResourcesContext);
  const { showAlert } = React.useContext(AppControllerContext);
  const history = useHistory();
  const location = useLocation();
  const tabValue = React.useMemo(() => locationToTabValue(location.pathname), [location]);
  const logoResourcesContext = React.useContext(ResourcesContext)?.logos;
  const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
  const { authenticator } = React.useContext(AppConfigContext);
  const profile = React.useContext(UserProfileContext);
  const safeAsync = useAsync();
  const [brandingIconPath, setBrandingIconPath] = React.useState<string>('');
  const [settingsAnchor, setSettingsAnchor] = React.useState<HTMLElement | null>(null);
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [placeNames, setPlaceNames] = React.useState<string[]>([]);
  const [workcells, setWorkcells] = React.useState<string[]>();
  const [favoritesTasks, setFavoritesTasks] = React.useState<TaskFavorite[]>([]);
  const [refreshTaskQueueTableCount, setRefreshTaskQueueTableCount] = React.useState(0);
  const [alertListAnchor, setAlertListAnchor] = React.useState<HTMLElement | null>(null);
  const [unacknowledgedAlertsNum, setUnacknowledgedAlertsNum] = React.useState(0);
  const [unacknowledgedAlertList, setUnacknowledgedAlertList] = React.useState<Alert[]>([]);
  const [acknowledgedAlertList, setAcknowledgedAlertList] = React.useState<Alert[]>([]);

  const curTheme = React.useContext(SettingsContext).themeMode;

  async function handleLogout(): Promise<void> {
    try {
      await authenticator.logout();
    } catch (e) {
      console.error(`error logging out: ${(e as Error).message}`);
    }
  }

  React.useEffect(() => {
    const sub = AppEvents.refreshTaskQueueTableCount.subscribe((currentValue) => {
      setRefreshTaskQueueTableCount(currentValue);
    });
    return () => sub.unsubscribe();
  }, []);

  React.useEffect(() => {
    if (!logoResourcesContext) return;
    (async () => {
      setBrandingIconPath(await safeAsync(logoResourcesContext.getHeaderLogoPath(curTheme)));
    })();
  }, [logoResourcesContext, safeAsync, curTheme]);

  //#region CreateTaskForm props
  React.useEffect(() => {
    if (!resourceManager?.dispensers) {
      return;
    }
    setWorkcells(Object.keys(resourceManager.dispensers.dispensers));
  }, [resourceManager]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const subs: Subscription[] = [];
    subs.push(
      rmf.buildingMapObs.subscribe((map) =>
        setPlaceNames(getPlaces(map).map((p) => p.vertex.name)),
      ),
    );
    subs.push(
      AppEvents.refreshAlertCount.subscribe((_) => {
        (async () => {
          const resp = await rmf.alertsApi.getAlertsAlertsGet();
          const alerts = resp.data as Alert[];
          setUnacknowledgedAlertsNum(
            alerts.filter(
              (alert) => !(alert.acknowledged_by && alert.unix_millis_acknowledged_time),
            ).length,
          );
        })();
      }),
    );
    // Get the initial number of unacknowledged alerts
    (async () => {
      const resp = await rmf.alertsApi.getAlertsAlertsGet();
      const alerts = resp.data as Alert[];
      setUnacknowledgedAlertsNum(
        alerts.filter((alert) => !(alert.acknowledged_by && alert.unix_millis_acknowledged_time))
          .length,
      );
    })();
    return () => subs.forEach((s) => s.unsubscribe());
  }, [rmf]);

  const submitTasks = React.useCallback<Required<CreateTaskFormProps>['submitTasks']>(
    async (taskRequests) => {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      await Promise.all(
        taskRequests.map((taskReq) =>
          rmf.tasksApi.postDispatchTaskTasksDispatchTaskPost({
            type: 'dispatch_task_request',
            request: taskReq,
          }),
        ),
      );
      AppEvents.refreshTaskQueueTableCount.next(refreshTaskQueueTableCount + 1);
    },
    [rmf, refreshTaskQueueTableCount],
  );

  const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
  const tasksFromFile = (): Promise<TaskRequest[]> => {
    return new Promise((res) => {
      const fileInputEl = uploadFileInputRef.current;
      if (!fileInputEl) {
        return [];
      }
      let taskFiles: TaskRequest[];
      const listener = async () => {
        try {
          if (!fileInputEl.files || fileInputEl.files.length === 0) {
            return res([]);
          }
          try {
            taskFiles = parseTasksFile(await fileInputEl.files[0].text());
          } catch (err) {
            showAlert('error', (err as Error).message, 5000);
            return res([]);
          }
          // only submit tasks when all tasks are error free
          return res(taskFiles);
        } finally {
          fileInputEl.removeEventListener('input', listener);
          fileInputEl.value = '';
        }
      };
      fileInputEl.addEventListener('input', listener);
      fileInputEl.click();
    });
  };
  //#endregion CreateTaskForm props

  //#region 'Favorite Task'
  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    (async () => {
      const resp = await rmf.tasksApi.getFavoritesTasksFavoriteTasksGet();

      const results = resp.data as TaskFavorite[];
      setFavoritesTasks(results);
    })();

    return () => {
      setFavoritesTasks([]);
    };
  }, [rmf, refreshTaskQueueTableCount]);

  const submitFavoriteTask = React.useCallback<Required<CreateTaskFormProps>['submitFavoriteTask']>(
    async (taskFavoriteRequest) => {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      await rmf.tasksApi.postFavoriteTaskFavoriteTasksPost(taskFavoriteRequest);
      AppEvents.refreshTaskQueueTableCount.next(refreshTaskQueueTableCount + 1);
    },
    [rmf, refreshTaskQueueTableCount],
  );

  const deleteFavoriteTask = React.useCallback<Required<CreateTaskFormProps>['deleteFavoriteTask']>(
    async (favoriteTask) => {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      if (!favoriteTask.id) {
        throw new Error('Id is needed');
      }

      await rmf.tasksApi.deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete(favoriteTask.id);
      AppEvents.refreshTaskQueueTableCount.next(refreshTaskQueueTableCount + 1);
    },
    [rmf, refreshTaskQueueTableCount],
  );
  //#endregion 'Favorite Task'

  const handleOpenAlertList = (event: React.MouseEvent<HTMLButtonElement, MouseEvent>) => {
    if (!rmf) {
      return;
    }
    (async () => {
      const resp = await rmf.alertsApi.getAlertsAlertsGet();
      const alerts = resp.data as Alert[];
      const ackList: Alert[] = [];
      const unackList: Alert[] = [];
      for (let alert of alerts) {
        if (alert.acknowledged_by || alert.unix_millis_acknowledged_time) {
          ackList.push(alert);
        } else {
          unackList.push(alert);
        }
      }
      setAcknowledgedAlertList(ackList.reverse());
      setUnacknowledgedAlertList(unackList.reverse());
    })();
    setAlertListAnchor(event.currentTarget);
  };

  const openAlertDialog = (alert: Alert) => {
    AppEvents.alertListOpenedAlert.next(alert);
  };

  return (
    <>
      <HeaderBar>
        <LogoButton src={brandingIconPath} alt="logo" sx={{ width: logoSize }} />
        <NavigationBar value={tabValue}>
          <AppBarTab
            label="Infrastructure"
            value="infrastructure"
            aria-label="Infrastructure"
            onTabClick={() => history.push(DashboardRoute)}
          />
          <AppBarTab
            label="Robots"
            value="robots"
            aria-label="Robots"
            onTabClick={() => history.push(RobotsRoute)}
          />
          <AppBarTab
            label="Tasks"
            value="tasks"
            aria-label="Tasks"
            onTabClick={() => history.push(TasksRoute)}
          />
          <AppBarTab
            label="Custom 1"
            value="custom1"
            aria-label="Custom 1"
            onTabClick={() => history.push(CustomRoute1)}
          />
          <AppBarTab
            label="Custom 2"
            value="custom2"
            aria-label="Custom 2"
            onTabClick={() => history.push(CustomRoute2)}
          />
          {profile?.user.is_admin && (
            <AppBarTab
              label="Admin"
              value="admin"
              aria-label="Admin"
              onTabClick={() => history.push(AdminRoute)}
            />
          )}
        </NavigationBar>
        <Toolbar variant="dense" sx={{ textAlign: 'right', flexGrow: -1 }}>
          <Button
            id="create-new-task-button"
            aria-label="new task"
            color="secondary"
            variant="contained"
            size="small"
            onClick={() => setOpenCreateTaskForm(true)}
          >
            <AddOutlined />
            New Task
          </Button>
          <IconButton
            id="alert-list-button"
            aria-label="alert-list-button"
            color="inherit"
            onClick={handleOpenAlertList}
          >
            <Badge badgeContent={unacknowledgedAlertsNum} color="secondary">
              <Notifications />
            </Badge>
          </IconButton>
          <Menu
            anchorEl={alertListAnchor}
            open={!!alertListAnchor}
            onClose={() => setAlertListAnchor(null)}
            transformOrigin={{ horizontal: 'right', vertical: 'top' }}
            anchorOrigin={{ horizontal: 'right', vertical: 'bottom' }}
            PaperProps={{
              style: {
                maxHeight: '20rem',
                maxWidth: '30rem',
              },
            }}
          >
            <MenuItem dense disabled>
              <Typography variant="body2" noWrap>
                Unacknowledged
              </Typography>
            </MenuItem>
            {unacknowledgedAlertList.map((alert) => (
              <Tooltip
                title={
                  <React.Fragment>
                    <Typography>ID: {alert.original_id}</Typography>
                  </React.Fragment>
                }
                placement="right"
              >
                <MenuItem
                  key={alert.id}
                  dense
                  onClick={() => {
                    openAlertDialog(alert);
                    setAlertListAnchor(null);
                  }}
                >
                  <Typography variant="body2" mr={2} noWrap>
                    {new Date(alert.unix_millis_created_time).toLocaleString()}
                  </Typography>
                  <Typography variant="body2" noWrap>
                    {alert.category.toUpperCase()} alert
                  </Typography>
                </MenuItem>
              </Tooltip>
            ))}
            <Divider />
            <MenuItem dense disabled>
              <Typography variant="body2" noWrap>
                Acknowledged
              </Typography>
            </MenuItem>
            {acknowledgedAlertList.map((alert) => (
              <Tooltip
                title={
                  <div>
                    <Typography display="block">ID: {alert.original_id}</Typography>
                    <Typography display="block">
                      On:{' '}
                      {alert.unix_millis_acknowledged_time
                        ? new Date(alert.unix_millis_acknowledged_time).toLocaleString()
                        : 'N/A'}
                    </Typography>
                    <Typography display="block">By: {alert.acknowledged_by ?? 'N/A'}</Typography>
                  </div>
                }
                placement="right"
              >
                <MenuItem
                  key={alert.id}
                  dense
                  onClick={() => {
                    openAlertDialog(alert);
                    setAlertListAnchor(null);
                  }}
                >
                  <Typography variant="body2" mr={2} noWrap>
                    {new Date(alert.unix_millis_created_time).toLocaleString()}
                  </Typography>
                  <Typography variant="body2" noWrap>
                    {alert.category.toUpperCase()} alert
                  </Typography>
                </MenuItem>
              </Tooltip>
            ))}
          </Menu>
          <Divider orientation="vertical" sx={{ marginLeft: 1, marginRight: 2 }} />
          <Typography variant="caption">Powered by Open-RMF</Typography>
          {extraToolbarItems}
          <IconButton
            id="show-settings-btn"
            aria-label="settings"
            color="inherit"
            onClick={(ev) => setSettingsAnchor(ev.currentTarget)}
          >
            <Settings />
          </IconButton>
          {profile && (
            <>
              <IconButton
                id="user-btn"
                aria-label={'user-btn'}
                color="inherit"
                onClick={(event) => setAnchorEl(event.currentTarget)}
              >
                <AccountCircle />
              </IconButton>
              <Menu
                anchorEl={anchorEl}
                anchorOrigin={{
                  vertical: 'bottom',
                  horizontal: 'right',
                }}
                transformOrigin={{
                  vertical: 'top',
                  horizontal: 'right',
                }}
                open={!!anchorEl}
                onClose={() => setAnchorEl(null)}
              >
                <MenuItem id="logout-btn" onClick={handleLogout}>
                  Logout
                </MenuItem>
              </Menu>
            </>
          )}
        </Toolbar>
      </HeaderBar>
      <Menu
        anchorEl={settingsAnchor}
        open={!!settingsAnchor}
        onClose={() => setSettingsAnchor(null)}
      >
        <CardContent>
          <AppSettings />
        </CardContent>
      </Menu>
      {openCreateTaskForm && (
        <CreateTaskForm
          cleaningZones={placeNames}
          loopWaypoints={placeNames}
          deliveryWaypoints={placeNames}
          dispensers={workcells}
          ingestors={workcells}
          favoritesTasks={favoritesTasks}
          open={openCreateTaskForm}
          onClose={() => setOpenCreateTaskForm(false)}
          submitTasks={submitTasks}
          submitFavoriteTask={submitFavoriteTask}
          deleteFavoriteTask={deleteFavoriteTask}
          tasksFromFile={tasksFromFile}
          onSuccess={() => {
            setOpenCreateTaskForm(false);
            showAlert('success', 'Successfully created task');
          }}
          onFail={(e) => {
            showAlert('error', `Failed to create task: ${e.message}`);
          }}
          onSuccessFavoriteTask={(message) => {
            showAlert('success', message);
          }}
          onFailFavoriteTask={(e) => {
            showAlert('error', `Failed to create or delete favorite task: ${e.message}`);
          }}
        />
      )}
    </>
  );
});

export default AppBar;

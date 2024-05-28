import {
  AccountCircle,
  AdminPanelSettings,
  AddOutlined,
  Help,
  LocalFireDepartment,
  Logout,
  Notifications,
  Report,
  // Settings,
  Warning as Issue,
} from '@mui/icons-material';
import {
  Badge,
  Button,
  CardContent,
  Chip,
  Dialog,
  DialogActions,
  DialogTitle,
  Divider,
  FormControl,
  FormControlLabel,
  FormLabel,
  IconButton,
  ListItemIcon,
  ListItemText,
  Menu,
  MenuItem,
  Radio,
  RadioGroup,
  Stack,
  Toolbar,
  Tooltip,
  Typography,
  useMediaQuery,
} from '@mui/material';
import { AlertRequest, FireAlarmTriggerState, TaskFavorite, TaskRequest } from 'api-client';
import React from 'react';
import {
  AppBarTab,
  ConfirmationDialog,
  CreateTaskForm,
  CreateTaskFormProps,
  HeaderBar,
  LogoButton,
  NavigationBar,
  useAsync,
} from 'react-components';
import { useNavigate, useLocation } from 'react-router-dom';
import { UserProfileContext } from 'rmf-auth';
import { logoSize } from '../managers/resource-manager';
import { ThemeMode } from '../settings';
import { DashboardRoute, RobotsRoute, TasksRoute } from '../util/url';
import {
  AppConfigContext,
  AppControllerContext,
  ResourcesContext,
  SettingsContext,
} from './app-contexts';
import { AppEvents } from './app-events';
import { RmfAppContext } from './rmf-app';
import { parseTasksFile } from './tasks/utils';
import { Subscription } from 'rxjs';
import { formatDistance } from 'date-fns';
import { styled } from '@mui/system';
import { useCreateTaskFormData } from '../hooks/useCreateTaskForm';
import { toApiSchedule } from './tasks/utils';
import useGetUsername from '../hooks/useFetchUser';

const StyledIconButton = styled(IconButton)(({ theme }) => ({
  fontSize: theme.spacing(4), // spacing = 8
}));

export type TabValue = 'infrastructure' | 'robots' | 'tasks';

const locationToTabValue = (pathname: string): TabValue | undefined => {
  const routes: { prefix: string; tabValue: TabValue }[] = [
    { prefix: RobotsRoute, tabValue: 'robots' },
    { prefix: TasksRoute, tabValue: 'tasks' },
    { prefix: DashboardRoute, tabValue: 'infrastructure' },
  ];

  // `DashboardRoute` being the root, it is a prefix to all routes, so we need to check exactly.
  const matchingRoute = routes.find((route) => pathname.startsWith(route.prefix));
  return matchingRoute?.tabValue;
};

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
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  const [largerResolution, setLargerResolution] = React.useState(false);

  const StyledAppBarTab = styled(AppBarTab)(({ theme }) => ({
    fontSize: theme.spacing(largerResolution ? 2 : 4),
  }));

  const StyledAppBarButton = styled(Button)(({ theme }) => ({
    fontSize: theme.spacing(largerResolution ? 1.5 : 4), // spacing = 8
    paddingTop: 0,
    paddingBottom: 0,
  }));

  React.useEffect(() => {
    setLargerResolution(isScreenHeightLessThan800);
  }, [isScreenHeightLessThan800]);

  const rmf = React.useContext(RmfAppContext);
  const resourceManager = React.useContext(ResourcesContext);
  const { showAlert } = React.useContext(AppControllerContext);
  const navigate = useNavigate();
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
  const [favoritesTasks, setFavoritesTasks] = React.useState<TaskFavorite[]>([]);
  const [alertListAnchor, setAlertListAnchor] = React.useState<HTMLElement | null>(null);
  const [unacknowledgedAlertsNum, setUnacknowledgedAlertsNum] = React.useState(0);
  const [unacknowledgedAlertList, setUnacknowledgedAlertList] = React.useState<AlertRequest[]>([]);
  const [openAdminActionsDialog, setOpenAdminActionsDialog] = React.useState(false);
  const [openFireAlarmTriggerResetDialog, setOpenFireAlarmTriggerResetDialog] =
    React.useState(false);
  const [fireAlarmPreviousTrigger, setFireAlarmPreviousTrigger] = React.useState<
    FireAlarmTriggerState | undefined
  >(undefined);

  const curTheme = React.useContext(SettingsContext).themeMode;
  const { waypointNames, pickupPoints, dropoffPoints, cleaningZoneNames } =
    useCreateTaskFormData(rmf);
  const username = useGetUsername(rmf);

  async function handleLogout(): Promise<void> {
    try {
      await authenticator.logout();
    } catch (e) {
      console.error(`error logging out: ${(e as Error).message}`);
    }
  }

  React.useEffect(() => {
    if (!logoResourcesContext) return;
    (async () => {
      setBrandingIconPath(await safeAsync(logoResourcesContext.getHeaderLogoPath(curTheme)));
    })();
  }, [logoResourcesContext, safeAsync, curTheme]);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const subs: Subscription[] = [];
    subs.push(
      AppEvents.refreshAlert.subscribe({
        next: () => {
          (async () => {
            const resp = await rmf.alertsApi.getUnrespondedAlertsAlertsUnrespondedRequestsGet();
            const alerts = resp.data as AlertRequest[];
            setUnacknowledgedAlertsNum(alerts.length);
          })();
        },
      }),
    );

    // Get the initial number of unacknowledged alerts
    (async () => {
      const resp = await rmf.alertsApi.getUnrespondedAlertsAlertsUnrespondedRequestsGet();
      const alerts = resp.data as AlertRequest[];
      setUnacknowledgedAlertsNum(alerts.length);
    })();
    return () => subs.forEach((s) => s.unsubscribe());
  }, [rmf]);

  const submitTasks = React.useCallback<Required<CreateTaskFormProps>['submitTasks']>(
    async (taskRequests, schedule) => {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      if (!schedule) {
        await Promise.all(
          taskRequests.map((request) => {
            console.debug('submitTask:');
            console.debug(request);
            return rmf.tasksApi.postDispatchTaskTasksDispatchTaskPost({
              type: 'dispatch_task_request',
              request,
            });
          }),
        );
      } else {
        const scheduleRequests = taskRequests.map((req) => {
          console.debug('schedule task:');
          console.debug(req);
          console.debug(schedule);
          return toApiSchedule(req, schedule);
        });
        await Promise.all(
          scheduleRequests.map((req) => rmf.tasksApi.postScheduledTaskScheduledTasksPost(req)),
        );
      }
      AppEvents.refreshTaskApp.next();
    },
    [rmf],
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

  //#region 'Favorite Task'
  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const getFavoriteTasks = async () => {
      const resp = await rmf.tasksApi.getFavoritesTasksFavoriteTasksGet();
      const results = resp.data as TaskFavorite[];
      setFavoritesTasks(results);
    };
    getFavoriteTasks();

    const sub = AppEvents.refreshFavoriteTasks.subscribe({ next: getFavoriteTasks });
    return () => sub.unsubscribe();
  }, [rmf]);

  const submitFavoriteTask = React.useCallback<Required<CreateTaskFormProps>['submitFavoriteTask']>(
    async (taskFavoriteRequest) => {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      await rmf.tasksApi.postFavoriteTaskFavoriteTasksPost(taskFavoriteRequest);
      AppEvents.refreshFavoriteTasks.next();
    },
    [rmf],
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
      AppEvents.refreshFavoriteTasks.next();
    },
    [rmf],
  );
  //#endregion 'Favorite Task'

  const handleOpenAlertList = (event: React.MouseEvent<HTMLButtonElement, MouseEvent>) => {
    if (!rmf) {
      return;
    }
    (async () => {
      const { data: alerts } =
        await rmf.alertsApi.getUnrespondedAlertsAlertsUnrespondedRequestsGet();
      setUnacknowledgedAlertList(alerts.reverse());
    })();
    setAlertListAnchor(event.currentTarget);
  };

  const openAlertDialog = (alert: AlertRequest) => {
    AppEvents.alertListOpenedAlert.next(alert);
  };

  const timeDistance = (time: number) => {
    return formatDistance(new Date(), new Date(time));
  };

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    (async () => {
      try {
        const resp =
          await rmf.buildingApi.getPreviousFireAlarmTriggerBuildingMapPreviousFireAlarmTriggerGet();
        setFireAlarmPreviousTrigger(resp.data);
      } catch (e) {
        console.error(`Failed to get previous fire alarm trigger: ${(e as Error).message}`);
      }
    })();
  }, [rmf, openAdminActionsDialog]);

  const handleResetFireAlarmTrigger = React.useCallback<React.MouseEventHandler>(async () => {
    try {
      if (!rmf) {
        throw new Error('building map api not available');
      }

      const resp =
        await rmf.buildingApi.resetFireAlarmTriggerBuildingMapResetFireAlarmTriggerPost();
      if (!resp.data.trigger) {
        showAlert('success', 'Requested to reset fire alarm trigger');
      } else {
        showAlert('error', 'Failed to reset fire alarm trigger');
      }
    } catch (e) {
      showAlert('error', `Failed to reset fire alarm trigger: ${(e as Error).message}`);
    }

    setOpenFireAlarmTriggerResetDialog(false);
    setOpenAdminActionsDialog(false);
  }, [rmf, showAlert]);

  return (
    <>
      <HeaderBar>
        <LogoButton src={brandingIconPath} alt="logo" sx={{ width: logoSize }} />
        <NavigationBar value={tabValue}>
          <StyledAppBarTab
            label="Map"
            value="infrastructure"
            aria-label="Map"
            onTabClick={() => navigate(DashboardRoute)}
          />
          <StyledAppBarTab
            label="System Overview"
            value="robots"
            aria-label="System Overview"
            onTabClick={() => navigate(RobotsRoute)}
          />
          <StyledAppBarTab
            label="Tasks"
            value="tasks"
            aria-label="Tasks"
            onTabClick={() => navigate(TasksRoute)}
          />
        </NavigationBar>
        <Toolbar variant="dense" sx={{ textAlign: 'right', flexGrow: -1 }}>
          <StyledAppBarButton
            id="create-new-task-button"
            aria-label="new task"
            color="secondary"
            variant="contained"
            size={largerResolution ? 'small' : 'medium'}
            onClick={() => setOpenCreateTaskForm(true)}
          >
            <AddOutlined transform={`scale(${largerResolution ? 0.5 : 1})`} />
            New Task
          </StyledAppBarButton>
          <Tooltip title="Notifications">
            <StyledIconButton
              id="alert-list-button"
              aria-label="alert-list-button"
              color="inherit"
              onClick={handleOpenAlertList}
            >
              <Badge badgeContent={unacknowledgedAlertsNum} color="secondary">
                <Notifications fontSize="inherit" />
              </Badge>
            </StyledIconButton>
          </Tooltip>
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
            {unacknowledgedAlertList.length === 0 ? (
              <MenuItem dense disabled>
                <Typography variant="body2" noWrap>
                  No unacknowledged alerts
                </Typography>
              </MenuItem>
            ) : (
              unacknowledgedAlertList.map((alert) => (
                <Tooltip
                  key={alert.id}
                  title={
                    <React.Fragment>
                      <Typography>Alert</Typography>
                      <Typography>ID: {alert.id}</Typography>
                      <Typography>Title: {alert.title}</Typography>
                      <Typography>
                        Created: {new Date(alert.unix_millis_alert_time).toLocaleString()}
                      </Typography>
                    </React.Fragment>
                  }
                  placement="right"
                >
                  <MenuItem
                    dense
                    onClick={() => {
                      openAlertDialog(alert);
                      setAlertListAnchor(null);
                    }}
                    divider
                  >
                    <Report />
                    <Typography variant="body2" mx={1} noWrap>
                      {alert.task_id ? `Task ${alert.task_id} had an alert ` : 'Alert occured '}
                      {timeDistance(alert.unix_millis_alert_time)} ago
                    </Typography>
                  </MenuItem>
                </Tooltip>
              ))
            )}
          </Menu>
          <Divider orientation="vertical" sx={{ marginLeft: 1, marginRight: 2 }} />
          {/* <Typography variant="subtitle1" fontSize={largerResolution ? 12 : 16}>
            Powered by Open-RMF
          </Typography> */}
          {extraToolbarItems}
          {/* <Tooltip title="Settings">
            <StyledIconButton
              id="show-settings-btn"
              aria-label="settings"
              color="inherit"
              onClick={(ev) => setSettingsAnchor(ev.currentTarget)}
            >
              <Settings fontSize="inherit" />
            </StyledIconButton>
          </Tooltip> */}
          <Tooltip title="Help">
            <StyledIconButton
              id="show-help-btn"
              aria-label="help"
              color="inherit"
              onClick={() => window.open(resourceManager?.helpLink, '_blank')}
            >
              <Help fontSize="inherit" />
            </StyledIconButton>
          </Tooltip>
          <Tooltip title="Report issues">
            <StyledIconButton
              id="show-warning-btn"
              aria-label="warning"
              color="inherit"
              onClick={() => window.open(resourceManager?.reportIssue, '_blank')}
            >
              <Issue fontSize="inherit" />
            </StyledIconButton>
          </Tooltip>
          {profile && (
            <>
              <Tooltip title="Profile">
                <StyledIconButton
                  id="user-btn"
                  aria-label={'user-btn'}
                  color="inherit"
                  onClick={(event) => setAnchorEl(event.currentTarget)}
                >
                  <AccountCircle fontSize="inherit" />
                </StyledIconButton>
              </Tooltip>
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
                <MenuItem disabled>
                  <ListItemText>{`Logged in as ${username ?? 'unknown user'}`}</ListItemText>
                  <Divider orientation="vertical" sx={{ marginLeft: 1, marginRight: 2 }} />
                  <ListItemIcon>
                    <Chip
                      color="primary"
                      size="small"
                      label={profile.user.is_admin ? 'Admin' : 'User'}
                    />
                  </ListItemIcon>
                </MenuItem>
                <Divider />
                <MenuItem
                  disabled={!profile.user.is_admin}
                  id="admin-action-btn"
                  onClick={() => {
                    setOpenAdminActionsDialog(true);
                    setAnchorEl(null);
                  }}
                >
                  <ListItemIcon>
                    <AdminPanelSettings fontSize="small" />
                  </ListItemIcon>
                  Admin actions
                </MenuItem>
                <Divider />
                <MenuItem id="logout-btn" onClick={handleLogout}>
                  <ListItemIcon>
                    <Logout fontSize="small" />
                  </ListItemIcon>
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
          user={username ? username : 'unknown user'}
          patrolWaypoints={waypointNames}
          cleaningZones={cleaningZoneNames}
          pickupZones={resourceManager?.pickupZones}
          cartIds={resourceManager?.cartIds}
          pickupPoints={pickupPoints}
          dropoffPoints={dropoffPoints}
          favoritesTasks={favoritesTasks}
          open={openCreateTaskForm}
          onClose={() => setOpenCreateTaskForm(false)}
          submitTasks={submitTasks}
          submitFavoriteTask={submitFavoriteTask}
          deleteFavoriteTask={deleteFavoriteTask}
          tasksFromFile={tasksFromFile}
          onSuccess={() => {
            console.log('Dispatch task requested');
            setOpenCreateTaskForm(false);
            showAlert('success', 'Dispatch task requested');
          }}
          onFail={(e) => {
            console.error(`Failed to dispatch task: ${e.message}`);
            showAlert('error', `Failed to dispatch task: ${e.message}`);
          }}
          onSuccessFavoriteTask={(message) => {
            console.log(`Created favorite task: ${message}`);
            showAlert('success', message);
          }}
          onFailFavoriteTask={(e) => {
            console.error(`Failed to create favorite task: ${e.message}`);
            showAlert('error', `Failed to create or delete favorite task: ${e.message}`);
          }}
          onSuccessScheduling={() => {
            console.log('Create schedule requested');
            setOpenCreateTaskForm(false);
            showAlert('success', 'Create schedule requested');
          }}
          onFailScheduling={(e) => {
            console.error(`Failed to create schedule: ${e.message}`);
            showAlert('error', `Failed to submit schedule: ${e.message}`);
          }}
        />
      )}
      {openAdminActionsDialog && (
        <Dialog onClose={() => setOpenAdminActionsDialog(false)} open={openAdminActionsDialog}>
          <DialogTitle>Admin actions</DialogTitle>
          <DialogActions sx={{ px: 2 }}>
            <Stack direction="column" spacing={2}>
              <Stack direction="row" spacing={2}>
                {fireAlarmPreviousTrigger && fireAlarmPreviousTrigger.trigger ? (
                  <div>
                    <Typography variant="subtitle2" alignContent="center">
                      Last fire alarm triggered on:
                    </Typography>
                    <Typography variant="body2" alignContent="center">
                      {new Date(fireAlarmPreviousTrigger.unix_millis_time).toLocaleString()}
                    </Typography>
                  </div>
                ) : fireAlarmPreviousTrigger && !fireAlarmPreviousTrigger.trigger ? (
                  <div>
                    <Typography variant="subtitle2" alignContent="center">
                      Last fire alarm reset on:
                    </Typography>
                    <Typography variant="body2" alignContent="center">
                      {new Date(fireAlarmPreviousTrigger.unix_millis_time).toLocaleString()}
                    </Typography>
                  </div>
                ) : (
                  <div>
                    <Typography variant="subtitle2" alignContent="center">
                      Last fire alarm triggered on:
                    </Typography>
                    <Typography variant="body2" alignContent="center">
                      n/a
                    </Typography>
                  </div>
                )}
                <Button
                  variant="contained"
                  onClick={() => setOpenFireAlarmTriggerResetDialog(true)}
                  startIcon={<LocalFireDepartment />}
                >
                  Reset fire alarm
                </Button>
              </Stack>
            </Stack>
          </DialogActions>
        </Dialog>
      )}
      {openFireAlarmTriggerResetDialog && (
        <ConfirmationDialog
          confirmText="Confirm reset"
          cancelText="Cancel"
          open={true}
          title={'Reset fire alarm trigger'}
          submitting={undefined}
          onClose={() => setOpenFireAlarmTriggerResetDialog(false)}
          onSubmit={handleResetFireAlarmTrigger}
        >
          <Typography color="warning.main">
            Warning: Please ensure that all other systems are back online and that it is safe to
            resume robot operations.
          </Typography>
        </ConfirmationDialog>
      )}
    </>
  );
});

export default AppBar;

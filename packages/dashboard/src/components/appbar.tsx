import {
  AccountCircle,
  AdminPanelSettings,
  Help,
  LocalFireDepartment,
  Logout,
  Notifications,
  Report,
  Warning as Issue,
} from '@mui/icons-material';
import Brightness4Icon from '@mui/icons-material/Brightness4';
import Brightness7Icon from '@mui/icons-material/Brightness7';
import {
  AppBar as MuiAppBar,
  Badge,
  Box,
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
  Grid,
  IconButton,
  ListItemIcon,
  ListItemText,
  Menu,
  MenuItem,
  Radio,
  RadioGroup,
  Stack,
  Tab,
  Tabs,
  Toolbar,
  Tooltip,
  Typography,
  useTheme,
} from '@mui/material';
import { AlertRequest, FireAlarmTriggerState, TaskFavorite } from 'api-client';
import { formatDistance } from 'date-fns';
import React from 'react';
import { ConfirmationDialog, TaskForm, TaskFormProps } from 'react-components';
import { Subscription } from 'rxjs';

import { useAppController } from '../hooks/use-app-controller';
import { useAuthenticator } from '../hooks/use-authenticator';
import { useTaskFormData } from '../hooks/use-create-task-form';
import { useResources } from '../hooks/use-resources';
import { useRmfApi } from '../hooks/use-rmf-api';
import { useSettings } from '../hooks/use-settings';
import { useTaskRegistry } from '../hooks/use-task-registry';
import { useUserProfile } from '../hooks/use-user-profile';
import { AppEvents } from './app-events';
import { dispatchTask, scheduleTask } from './tasks/utils';
import { DashboardThemes } from './theme';

export const APP_BAR_HEIGHT = '3.5rem';

const ToolbarIconButton = React.forwardRef<
  HTMLButtonElement,
  React.ComponentProps<typeof IconButton>
>((props, ref) => <IconButton size="large" ref={ref} {...props} />);

function AppSettings() {
  const settings = useSettings();
  const appController = useAppController();
  return (
    <FormControl>
      <FormLabel id="theme-label">Theme</FormLabel>
      <RadioGroup row aria-labelledby="theme-label">
        <FormControlLabel
          value={'default'}
          control={<Radio />}
          label="Default"
          checked={settings.themeMode === 'default'}
          onChange={() => appController.updateSettings({ ...settings, themeMode: 'default' })}
        />
      </RadioGroup>
    </FormControl>
  );
}

export interface AppBarProps {
  tabs: React.ReactElement<React.ComponentProps<typeof Tab>>[];
  tabValue: string;
  themes?: DashboardThemes;
  helpLink: string;
  reportIssueLink: string;
  extraToolbarItems?: React.ReactNode;
}

export const AppBar = React.memo(
  ({ tabs, tabValue, themes, helpLink, reportIssueLink, extraToolbarItems }: AppBarProps) => {
    const authenticator = useAuthenticator();
    const rmfApi = useRmfApi();
    const resources = useResources();
    const taskRegistry = useTaskRegistry();
    const { showAlert } = useAppController();
    const [anchorEl, setAnchorEl] = React.useState<HTMLElement | null>(null);
    const profile = useUserProfile();
    const [settingsAnchor, setSettingsAnchor] = React.useState<HTMLElement | null>(null);
    const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
    const [favoritesTasks, setFavoritesTasks] = React.useState<TaskFavorite[]>([]);
    const [alertListAnchor, setAlertListAnchor] = React.useState<HTMLElement | null>(null);
    const [unacknowledgedAlertList, setUnacknowledgedAlertList] = React.useState<AlertRequest[]>(
      [],
    );
    const [openAdminActionsDialog, setOpenAdminActionsDialog] = React.useState(false);
    const [openFireAlarmTriggerResetDialog, setOpenFireAlarmTriggerResetDialog] =
      React.useState(false);
    const [fireAlarmPreviousTrigger, setFireAlarmPreviousTrigger] = React.useState<
      FireAlarmTriggerState | undefined
    >(undefined);

    const { waypointNames, pickupPoints, dropoffPoints, cleaningZoneNames, fleets } =
      useTaskFormData(rmfApi);
    const username = profile.user.username;

    async function handleLogout(): Promise<void> {
      try {
        await authenticator.logout();
      } catch (e) {
        console.error(`error logging out: ${(e as Error).message}`);
      }
    }

    React.useEffect(() => {
      const updateUnrespondedAlerts = async () => {
        const { data: alerts } =
          await rmfApi.alertsApi.getUnrespondedAlertsAlertsUnrespondedRequestsGet();
        // alert.display is checked to verify that the dashboard should display it
        // in the first place
        const alertsToBeDisplayed = alerts.filter((alert) => alert.display);
        setUnacknowledgedAlertList(alertsToBeDisplayed.reverse());
      };

      const subs: Subscription[] = [];
      subs.push(rmfApi.alertRequestsObsStore.subscribe(updateUnrespondedAlerts));
      subs.push(rmfApi.alertResponsesObsStore.subscribe(updateUnrespondedAlerts));

      // Get the initial number of unacknowledged alerts
      updateUnrespondedAlerts();
      return () => subs.forEach((s) => s.unsubscribe());
    }, [rmfApi]);

    const dispatchTaskCallback = React.useCallback<Required<TaskFormProps>['onDispatchTask']>(
      async (taskRequest, robotDispatchTarget) => {
        if (!rmfApi) {
          throw new Error('tasks api not available');
        }
        await dispatchTask(rmfApi, taskRequest, robotDispatchTarget);
        AppEvents.refreshTaskApp.next();
      },
      [rmfApi],
    );

    const scheduleTaskCallback = React.useCallback<Required<TaskFormProps>['onScheduleTask']>(
      async (taskRequest, schedule) => {
        if (!rmfApi) {
          throw new Error('tasks api not available');
        }
        await scheduleTask(rmfApi, taskRequest, schedule);
        AppEvents.refreshTaskApp.next();
      },
      [rmfApi],
    );

    //#region 'Favorite Task'
    React.useEffect(() => {
      const getFavoriteTasks = async () => {
        const resp = await rmfApi.tasksApi.getFavoritesTasksFavoriteTasksGet();
        const results = resp.data as TaskFavorite[];
        setFavoritesTasks(results);
      };
      getFavoriteTasks();

      const sub = AppEvents.refreshFavoriteTasks.subscribe({ next: getFavoriteTasks });
      return () => sub.unsubscribe();
    }, [rmfApi]);

    const submitFavoriteTask = React.useCallback<Required<TaskFormProps>['submitFavoriteTask']>(
      async (taskFavoriteRequest) => {
        await rmfApi.tasksApi.postFavoriteTaskFavoriteTasksPost(taskFavoriteRequest);
        AppEvents.refreshFavoriteTasks.next();
      },
      [rmfApi],
    );

    const deleteFavoriteTask = React.useCallback<Required<TaskFormProps>['deleteFavoriteTask']>(
      async (favoriteTask) => {
        if (!favoriteTask.id) {
          throw new Error('Id is needed');
        }

        await rmfApi.tasksApi.deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete(favoriteTask.id);
        AppEvents.refreshFavoriteTasks.next();
      },
      [rmfApi],
    );
    //#endregion 'Favorite Task'

    const handleOpenAlertList = (event: React.MouseEvent<HTMLButtonElement, MouseEvent>) => {
      setAlertListAnchor(event.currentTarget);
    };

    const openAlertDialog = (alert: AlertRequest) => {
      AppEvents.pushAlert.next(alert);
    };

    const timeDistance = (time: number) => {
      return formatDistance(new Date(), new Date(time));
    };

    React.useEffect(() => {
      (async () => {
        try {
          const resp =
            await rmfApi.buildingApi.getPreviousFireAlarmTriggerBuildingMapPreviousFireAlarmTriggerGet();
          setFireAlarmPreviousTrigger(resp.data);
        } catch (e) {
          console.error(`Failed to get previous fire alarm trigger: ${(e as Error).message}`);
        }
      })();
    }, [rmfApi, openAdminActionsDialog]);

    const handleResetFireAlarmTrigger = React.useCallback<React.MouseEventHandler>(async () => {
      try {
        const resp =
          await rmfApi.buildingApi.resetFireAlarmTriggerBuildingMapResetFireAlarmTriggerPost();
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
    }, [rmfApi, showAlert]);

    const theme = useTheme();
    const settings = useSettings();
    const appController = useAppController();

    return (
      <MuiAppBar position="sticky" sx={{ height: APP_BAR_HEIGHT, zIndex: theme.zIndex.drawer + 1 }}>
        <Grid container alignItems="center" justifyContent="space-between" wrap="nowrap">
          <Box display="flex" alignItems="center">
            <img src={resources.logos.header} alt="logo" style={{ height: APP_BAR_HEIGHT }} />
            <Divider orientation="vertical" flexItem />
            <Tabs
              variant="scrollable"
              value={tabValue}
              textColor="inherit"
              indicatorColor="secondary"
            >
              {tabs}
            </Tabs>
          </Box>
          <Toolbar variant="dense">
            <Box display="flex" alignItems="center" gap={2}>
              <Typography variant="subtitle1">Powered by Open-RMF</Typography>
              <Button
                id="create-new-task-button"
                aria-label="new task"
                color="secondary"
                variant="contained"
                sx={{ marginRight: 2 }}
                onClick={() => setOpenCreateTaskForm(true)}
              >
                New Task
              </Button>
            </Box>
            <Divider orientation="vertical" flexItem />
            <Tooltip title="Notifications">
              <ToolbarIconButton
                id="alert-list-button"
                aria-label="alert-list-button"
                color="inherit"
                onClick={handleOpenAlertList}
              >
                <Badge badgeContent={unacknowledgedAlertList.length} color="secondary">
                  <Notifications fontSize="inherit" />
                </Badge>
              </ToolbarIconButton>
            </Tooltip>
            <Menu
              anchorEl={alertListAnchor}
              open={!!alertListAnchor}
              onClose={() => setAlertListAnchor(null)}
              transformOrigin={{ horizontal: 'right', vertical: 'top' }}
              anchorOrigin={{ horizontal: 'right', vertical: 'bottom' }}
              slotProps={{
                paper: {
                  style: {
                    maxHeight: '20rem',
                    maxWidth: '30rem',
                  },
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
                      <>
                        <Typography>Alert</Typography>
                        <Typography>ID: {alert.id}</Typography>
                        <Typography>Title: {alert.title}</Typography>
                        <Typography>
                          Created: {new Date(alert.unix_millis_alert_time).toLocaleString()}
                        </Typography>
                      </>
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
            {extraToolbarItems}
            {themes?.dark && (
              <Tooltip title="Toggle Dark Mode">
                <ToolbarIconButton
                  id="toggle-dark-mode-btn"
                  aria-label="toggle dark mode"
                  color="inherit"
                  onClick={() =>
                    appController.updateSettings({
                      ...settings,
                      themeMode: settings.themeMode === 'default' ? 'dark' : 'default',
                    })
                  }
                >
                  {settings.themeMode === 'dark' ? <Brightness7Icon /> : <Brightness4Icon />}
                </ToolbarIconButton>
              </Tooltip>
            )}
            <Tooltip title="Help">
              <ToolbarIconButton
                id="show-help-btn"
                aria-label="help"
                color="inherit"
                onClick={() => window.open(helpLink, '_blank')}
              >
                <Help fontSize="inherit" />
              </ToolbarIconButton>
            </Tooltip>
            <Tooltip title="Report issues">
              <ToolbarIconButton
                id="show-warning-btn"
                aria-label="warning"
                color="inherit"
                onClick={() => window.open(reportIssueLink, '_blank')}
              >
                <Issue fontSize="inherit" />
              </ToolbarIconButton>
            </Tooltip>
            <Tooltip title="Profile">
              <ToolbarIconButton
                id="user-btn"
                aria-label={'user-btn'}
                color="inherit"
                onClick={(event) => setAnchorEl(event.currentTarget)}
              >
                <AccountCircle fontSize="inherit" />
              </ToolbarIconButton>
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
          </Toolbar>
        </Grid>

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
          <TaskForm
            user={username ? username : 'unknown user'}
            fleets={fleets}
            tasksToDisplay={taskRegistry.taskDefinitions}
            patrolWaypoints={waypointNames}
            cleaningZones={cleaningZoneNames}
            pickupZones={taskRegistry.pickupZones}
            cartIds={taskRegistry.cartIds}
            pickupPoints={pickupPoints}
            dropoffPoints={dropoffPoints}
            favoritesTasks={favoritesTasks}
            open={openCreateTaskForm}
            onClose={() => setOpenCreateTaskForm(false)}
            onDispatchTask={dispatchTaskCallback}
            onScheduleTask={scheduleTaskCallback}
            onEditScheduleTask={undefined}
            submitFavoriteTask={submitFavoriteTask}
            deleteFavoriteTask={deleteFavoriteTask}
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
      </MuiAppBar>
    );
  },
);

export default AppBar;

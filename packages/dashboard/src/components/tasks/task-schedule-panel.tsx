import {
  Button,
  Grid,
  IconButton,
  makeStyles,
  Paper,
  Snackbar,
  TableContainer,
  TablePagination,
  Toolbar,
  Tooltip,
  Tab,
  Tabs,
  Typography,
  useTheme,
} from '@material-ui/core';
import { AddOutlined as AddOutlinedIcon } from '@material-ui/icons';
import { Alert, AlertProps } from '@material-ui/lab';
import { Task } from 'api-client';
import React from 'react';
import {
  ScheduledTask,
  ScheduledTaskTable,
  ScheduleTaskForm,
  ScheduleTaskFormProps,
  SubmitTaskSchedule,
  TaskRule,
  TaskRuleTable,
  TaskRuleInfo,
  ScheduledTaskInfo,
} from 'react-components';
import * as RmfModels from 'rmf-models';
import {
  createTaskRuleAPI,
  deleteScheduledTaskAPI,
  deleteTaskRuleAPI,
  getScheduledTasksAPI,
  getTaskRulesAPI,
} from '../../managers/task-scheduler-manager';

// import { UserContext } from '../auth/contexts';
import { a11yProps, TabPanel } from './tab-panel';

const useStyles = makeStyles((theme) => ({
  tableContainer: {
    display: 'flex',
    flexDirection: 'column',
  },
  tableTitle: {
    flex: '1 1 100%',
  },
  detailPanelContainer: {
    width: 350,
    padding: theme.spacing(2),
    marginLeft: theme.spacing(1),
    flex: '0 0 auto',
  },
  enabledToggleButton: {
    background: theme.palette.action.selected,
  },
}));

function NoSelectedTask() {
  return (
    <Grid container wrap="nowrap" alignItems="center" style={{ height: '100%' }}>
      <Typography variant="h6" align="center" color="textSecondary">
        Click on a task to view more information
      </Typography>
    </Grid>
  );
}

export interface TaskSchedulerPanelProps extends React.HTMLProps<HTMLDivElement> {
  /**
   * Should only contain the tasks of the current page.
   */
  tasks: Task[];
  paginationOptions?: Omit<React.ComponentPropsWithoutRef<typeof TablePagination>, 'component'>;
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  onRefresh?: () => void;
  onAutoRefresh?: (enabled: boolean) => void;
}

export function TaskSchedulerPanel({
  tasks,
  paginationOptions,
  cleaningZones,
  loopWaypoints,
  deliveryWaypoints,
  dispensers,
  ingestors,
  onRefresh,
  onAutoRefresh,
  ...divProps
}: TaskSchedulerPanelProps): JSX.Element {
  const classes = useStyles();
  const theme = useTheme();
  const [selectedTask, setSelectedTask] = React.useState<ScheduledTask | undefined>(undefined);
  const [selectedRule, setSelectedRule] = React.useState<TaskRule | undefined>(undefined);
  const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');
  // const [autoRefresh, setAutoRefresh] = React.useState(true);
  const [taskRules, setTaskRules] = React.useState([] as TaskRule[]);
  const [scheduledTasks, setScheduledTasks] = React.useState([] as ScheduledTask[]);

  // const user = React.useContext(UserContext);

  const [value, setValue] = React.useState(0);
  const handleChange = (event: any, newValue: number) => {
    setValue(newValue);
  };

  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!selectedTask) {
      return;
    }
    try {
      await deleteScheduledTaskAPI(selectedTask.id);
      setSnackbarMessage('Successfully cancelled task');
      setSnackbarSeverity('success');
      setOpenSnackbar(true);
      setSelectedTask(undefined);
    } catch (e) {
      setSnackbarMessage(`Failed to cancel task: ${e.message}`);
      setSnackbarSeverity('error');
      setOpenSnackbar(true);
    }
  }, [selectedTask]);

  const handleDeleteRuleClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!selectedRule) {
      return;
    }

    try {
      await deleteTaskRuleAPI(selectedRule.id);
      setSnackbarMessage('Rule successfully deleted');
      setSnackbarSeverity('success');
      setOpenSnackbar(true);
      setSelectedTask(undefined);
    } catch (e) {
      setSnackbarMessage(`Failed to delete rule: ${e.message}`);
      setSnackbarSeverity('error');
      setOpenSnackbar(true);
    }
  }, [selectedRule]);

  // const autoRefreshTooltipPrefix = autoRefresh ? 'Disable' : 'Enable';

  /*
  ---------------
  */
  const taskTypeParser = (task: number): string => {
    switch (task) {
      case RmfModels.TaskType.TYPE_CLEAN:
        return 'clean';

      case RmfModels.TaskType.TYPE_LOOP:
        return 'loop';

      case RmfModels.TaskType.TYPE_DELIVERY:
        return 'delivery';

      default:
        return 'error';
    }
  };

  const submitTaskSchedule = React.useCallback<Required<ScheduleTaskFormProps>['submitTask']>(
    async (task: SubmitTaskSchedule) => {
      const frequencyType =
        task.frequencyType === 'Custom' ? task.frequencyTypeCustom : task.frequencyType;

      await createTaskRuleAPI({
        name: task.ruleName,
        days_of_week: task.daysOfWeek,
        start_datetime: task.startDatetime.toISOString(),
        end_datetime: task.endDatetime?.toISOString(),
        frequency: task.frequency,
        frequency_type: frequencyType,
        task_type: taskTypeParser(task.task.task_type),
        args: task.task,
      });
    },
    [],
  );

  const getTaskRules = React.useCallback(async () => {
    const response = await getTaskRulesAPI(0);
    setTaskRules(response);
    // await Promise.all(tasks.map((t) => tasksApi.submitTaskTasksSubmitTaskPost(t)));
    // handleRefresh();
  }, []);

  const getScheduledTasks = React.useCallback(async () => {
    const response = await getScheduledTasksAPI(0);
    // await Promise.all(tasks.map((t) => tasksApi.submitTaskTasksSubmitTaskPost(t)));
    // handleRefresh();
    setScheduledTasks(response);
  }, []);

  const taskCancellable =
    selectedTask?.task_datetime && new Date(selectedTask?.task_datetime) < new Date();
  // const taskCancellable =
  //   selectedTask &&
  //   user &&
  //   Enforcer.canCancelTask(user, selectedTask) &&
  //   (selectedTask.summary.state === RmfModels.TaskSummary.STATE_ACTIVE ||
  //     selectedTask.summary.state === RmfModels.TaskSummary.STATE_PENDING ||
  //     selectedTask.summary.state === RmfModels.TaskSummary.STATE_QUEUED);

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Paper className={classes.tableContainer}>
          <Toolbar>
            {/* <AppBar position="static"> */}
            <Tabs
              value={value}
              className={classes.tableTitle}
              onChange={handleChange}
              aria-label="schedule-task-tab"
            >
              <Tab label="Scheduled Task" {...a11yProps(0)} />
              <Tab label="Task Rules" {...a11yProps(1)} />
            </Tabs>
            <Tooltip title="Create Task Rule. The task rule will generate the correspondent scheduled tasks">
              <IconButton onClick={() => setOpenCreateTaskForm(true)} aria-label="Create Task">
                <AddOutlinedIcon />
              </IconButton>
            </Tooltip>
          </Toolbar>

          <TabPanel value={value} index={0}>
            <TableContainer>
              <ScheduledTaskTable
                onLoad={getScheduledTasks}
                scheduledTasks={scheduledTasks}
                onTaskClick={(_ev, task) =>
                  setSelectedTask(scheduledTasks.find((t) => t.id === task.id))
                }
              />
            </TableContainer>
            {paginationOptions && (
              <TablePagination
                component="div"
                {...paginationOptions}
                style={{ flex: '0 0 auto' }}
              />
            )}
          </TabPanel>

          <TabPanel value={value} index={1}>
            <TableContainer>
              <TaskRuleTable
                onLoad={getTaskRules}
                taskRules={taskRules}
                onTaskClick={(_ev, task) => {
                  setSelectedRule(taskRules.find((t) => t.id === task.id));
                }}
              />
            </TableContainer>
            {paginationOptions && (
              <TablePagination
                component="div"
                {...paginationOptions}
                style={{ flex: '0 0 auto' }}
              />
            )}
          </TabPanel>
        </Paper>
        <Paper className={classes.detailPanelContainer}>
          {value === 0 &&
            (selectedTask ? (
              <>
                <ScheduledTaskInfo task={selectedTask} />
                <Button
                  style={{ marginTop: theme.spacing(1) }}
                  fullWidth
                  variant="contained"
                  color="secondary"
                  aria-label="Cancel Task"
                  disabled={!!taskCancellable}
                  onClick={handleCancelTaskClick}
                >
                  Delete Task
                </Button>
              </>
            ) : (
              <NoSelectedTask />
            ))}
          {value === 1 &&
            (selectedRule ? (
              <>
                <TaskRuleInfo task={selectedRule} />
                <Button
                  style={{ marginTop: theme.spacing(1) }}
                  fullWidth
                  variant="contained"
                  color="secondary"
                  aria-label="Cancel Task"
                  onClick={handleDeleteRuleClick}
                >
                  Delete Rule
                </Button>
              </>
            ) : (
              <NoSelectedTask />
            ))}
        </Paper>
      </Grid>
      {openCreateTaskForm && (
        <ScheduleTaskForm
          cleaningZones={cleaningZones}
          loopWaypoints={loopWaypoints}
          deliveryWaypoints={deliveryWaypoints}
          dispensers={dispensers}
          ingestors={ingestors}
          open={openCreateTaskForm}
          onClose={() => setOpenCreateTaskForm(false)}
          submitTask={submitTaskSchedule}
          onSuccess={() => {
            setOpenCreateTaskForm(false);
            setSnackbarSeverity('success');
            setSnackbarMessage('Successfully created task');
            setOpenSnackbar(true);
          }}
          onFail={(e) => {
            setSnackbarSeverity('error');
            setSnackbarMessage(`Failed to create task: ${e.message}`);
            setOpenSnackbar(true);
          }}
        />
      )}
      <input type="file" style={{ display: 'none' }} ref={uploadFileInputRef} />
      <Snackbar open={openSnackbar} onClose={() => setOpenSnackbar(false)} autoHideDuration={2000}>
        <Alert severity={snackbarSeverity}>{snackbarMessage}</Alert>
      </Snackbar>
    </div>
  );
}

export default TaskSchedulerPanel;

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
import {
  AddOutlined as AddOutlinedIcon,
  Autorenew as AutorenewIcon,
  Refresh as RefreshIcon,
} from '@material-ui/icons';
import { Alert, AlertProps } from '@material-ui/lab';
import { SubmitTask, Task } from 'api-client';
import React from 'react';
import {
  ScheduledTask,
  ScheduledTaskTable,
  ScheduleTaskForm,
  ScheduleTaskFormProps,
  TaskInfo,
  TaskRule,
  TaskRuleTable,
  TaskTable,
} from 'react-components';
import * as RmfModels from 'rmf-models';
import { UserContext } from '../auth/contexts';
import { Enforcer } from '../permissions';
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
  getTaskRules: (offset: number) => Promise<TaskRule[]>;
  getScheduledTasks: (offset: number) => Promise<ScheduledTask[]>;
  submitTask?: ScheduleTaskFormProps['submitTask'];
  cancelTask?: (task: RmfModels.TaskSummary) => Promise<void>;
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
  getTaskRules,
  getScheduledTasks,
  submitTask,
  cancelTask,
  onRefresh,
  onAutoRefresh,
  ...divProps
}: TaskSchedulerPanelProps): JSX.Element {
  const classes = useStyles();
  const theme = useTheme();
  const [selectedTask, setSelectedTask] = React.useState<Task | undefined>(undefined);
  const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');
  const [autoRefresh, setAutoRefresh] = React.useState(true);
  const user = React.useContext(UserContext);

  const [value, setValue] = React.useState(0);
  const handleChange = (event: any, newValue: number) => {
    setValue(newValue);
  };
  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!cancelTask || !selectedTask) {
      return;
    }
    try {
      await cancelTask(selectedTask.summary);
      setSnackbarMessage('Successfully cancelled task');
      setSnackbarSeverity('success');
      setOpenSnackbar(true);
      setSelectedTask(undefined);
    } catch (e) {
      setSnackbarMessage(`Failed to cancel task: ${e.message}`);
      setSnackbarSeverity('error');
      setOpenSnackbar(true);
    }
  }, [cancelTask, selectedTask]);

  const autoRefreshTooltipPrefix = autoRefresh ? 'Disable' : 'Enable';

  const taskCancellable =
    selectedTask &&
    user &&
    Enforcer.canCancelTask(user, selectedTask) &&
    (selectedTask.summary.state === RmfModels.TaskSummary.STATE_ACTIVE ||
      selectedTask.summary.state === RmfModels.TaskSummary.STATE_PENDING ||
      selectedTask.summary.state === RmfModels.TaskSummary.STATE_QUEUED);

  React.useEffect(() => {
    getTaskRules(0);
    // getScheduledTasks(0);
  }, [getTaskRules]);

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
            {/* </AppBar> */}
            {/* <Typography className={classes.tableTitle} variant="h6">
              Scheduled Tasks
            </Typography> */}
            <Tooltip title="Schedule task">
              <IconButton onClick={() => setOpenCreateTaskForm(true)} aria-label="Create Task">
                <AddOutlinedIcon />
              </IconButton>
            </Tooltip>
          </Toolbar>

          <TabPanel value={value} index={0}>
            <TableContainer>
              <ScheduledTaskTable
                scheduledTasks={tasks.map((t) => t.summary)}
                onTaskClick={(_ev, task) =>
                  setSelectedTask(tasks.find((t) => t.task_id === task.id))
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
                taskRules={tasks.map((t) => t.summary)}
                onTaskClick={(_ev, task) =>
                  setSelectedTask(tasks.find((t) => t.task_id === task.id))
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
        </Paper>
        <Paper className={classes.detailPanelContainer}>
          {selectedTask ? (
            <>
              <TaskInfo task={selectedTask.summary} />
              <Button
                style={{ marginTop: theme.spacing(1) }}
                fullWidth
                variant="contained"
                color="secondary"
                aria-label="Cancel Task"
                disabled={!taskCancellable}
                onClick={handleCancelTaskClick}
              >
                Cancel Task
              </Button>
            </>
          ) : (
            <NoSelectedTask />
          )}
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
          submitTask={submitTask}
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

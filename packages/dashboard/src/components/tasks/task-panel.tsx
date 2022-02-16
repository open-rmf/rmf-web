import {
  AddOutlined as AddOutlinedIcon,
  Autorenew as AutorenewIcon,
  Refresh as RefreshIcon,
} from '@mui/icons-material';
import {
  Alert,
  AlertProps,
  Button,
  Grid,
  IconButton,
  Paper,
  Snackbar,
  styled,
  TableContainer,
  TablePagination,
  Toolbar,
  Tooltip,
  Typography,
  useTheme,
} from '@mui/material';
import { TaskEventLog, TaskState } from 'api-client';
import React from 'react';
import { CreateTaskForm, CreateTaskFormProps, TaskInfo, TaskTable } from 'react-components';
import { UserProfileContext } from 'rmf-auth';
//import { AppControllerContext } from '../app-contexts';
import { Enforcer } from '../permissions';
import { RmfIngressContext } from '../rmf-app';
//import { parseTasksFile } from './utils';
import { TaskLogs } from './task-logs';

const prefix = 'task-panel';
const classes = {
  tableContainer: `${prefix}-table-container`,
  tableTitle: `${prefix}-table-title`,
  taskInfoContainer: `${prefix}-task-info-container`,
  enabledToggleButton: `${prefix}-enable-toggle-button`,
  logsPanelContainer: `${prefix}-task-logs-container`,
  tableGrid: `${prefix}-table-grid`,
  timelineGrid: `${prefix}-timeline-grid`,
  logsGrid: `${prefix}-logs-grid`,
};

const StyledDiv = styled('div')(({ theme }) => ({
  [`& .${classes.tableContainer}`]: {
    display: 'flex',
    flex: '1 1 100%',
    flexDirection: 'column',
    overflow: 'auto',
  },
  [`& .${classes.tableTitle}`]: {
    flex: '1 1 100%',
  },
  [`& .${classes.taskInfoContainer}`]: {
    display: 'flex',
    padding: theme.spacing(2),
    flex: '1 1 100%',
    flexDirection: 'column',
    maxHeight: '100%',
    overflow: 'auto',
  },
  [`& .${classes.enabledToggleButton}`]: {
    background: theme.palette.action.selected,
  },
  [`&.${classes.tableGrid}`]: {
    xs: 5,
  },
  [`& .${classes.timelineGrid}`]: {
    marginLeft: theme.spacing(2),
    marginButtom: theme.spacing(2),
    padding: theme.spacing(2),
    maxHeight: '100%',
    xs: 4,
  },
  [`& .${classes.logsGrid}`]: {
    display: 'flex',
    marginLeft: theme.spacing(2),
    flex: '1 1 100%',
    flexDirection: 'column',
    xs: 4,
  },
  height: '90%',
  flexDirection: 'column',
}));

function NoSelectedTask() {
  return (
    <Grid container wrap="nowrap" alignItems="center" style={{ height: '100%' }}>
      <Typography variant="h6" align="center">
        Click on a task to view more information
      </Typography>
    </Grid>
  );
}

export interface TaskPanelProps
  extends React.DetailedHTMLProps<React.HTMLAttributes<HTMLDivElement>, HTMLDivElement> {
  /**
   * Should only contain the tasks of the current page.
   */
  tasks: TaskState[];
  paginationOptions?: Omit<React.ComponentPropsWithoutRef<typeof TablePagination>, 'component'>;
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  submitTasks?: CreateTaskFormProps['submitTasks'];
  cancelTask?: (task: TaskState) => Promise<void>;
  onRefresh?: () => void;
  onAutoRefresh?: (enabled: boolean) => void;
}

export function TaskPanel({
  tasks,
  paginationOptions,
  cleaningZones,
  loopWaypoints,
  deliveryWaypoints,
  dispensers,
  ingestors,
  submitTasks,
  cancelTask,
  onRefresh,
  onAutoRefresh,
  ...divProps
}: TaskPanelProps): JSX.Element {
  const theme = useTheme();
  const [selectedTask, setSelectedTask] = React.useState<TaskState | undefined>(undefined);
  const [selectedTaskState, setSelectedTaskState] = React.useState<TaskState | undefined>(
    undefined,
  );
  const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');
  const [autoRefresh, setAutoRefresh] = React.useState(true);
  const [selectedTaskLog, setSelectedTaskLog] = React.useState<TaskEventLog | undefined>(undefined);
  const profile = React.useContext(UserProfileContext);
  //const { showErrorAlert } = React.useContext(AppControllerContext);
  const { tasksApi } = React.useContext(RmfIngressContext) || {};

  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!cancelTask || !selectedTask) {
      return;
    }
    try {
      await cancelTask(selectedTask);
      setSnackbarMessage('Successfully cancelled task');
      setSnackbarSeverity('success');
      setOpenSnackbar(true);
      setSelectedTask(undefined);
    } catch (e) {
      setSnackbarMessage(`Failed to cancel task: ${(e as Error).message}`);
      setSnackbarSeverity('error');
      setOpenSnackbar(true);
    }
  }, [cancelTask, selectedTask]);

  // /* istanbul ignore next */
  //const tasksFromFile = (): Promise<TaskState[]> => {
  //return new Promise((res) => {
  //const fileInputEl = uploadFileInputRef.current;
  //if (!fileInputEl) {
  //return [];
  //}
  //let taskFiles: TaskState[];
  //const listener = async () => {
  //try {
  //if (!fileInputEl.files || fileInputEl.files.length === 0) {
  //return res([]);
  //}
  //try {
  //taskFiles = parseTasksFile(await fileInputEl.files[0].text());
  //} catch (err) {
  //showErrorAlert((err as Error).message, 5000);
  //return res([]);
  //}
  //// only submit tasks when all tasks are error free
  //return res(taskFiles);
  //} finally {
  //fileInputEl.removeEventListener('input', listener);
  //fileInputEl.value = '';
  //}
  //};
  //fileInputEl.addEventListener('input', listener);
  //fileInputEl.click();
  //});
  //};

  const fetchLogs = React.useCallback(async () => {
    if (!tasksApi) {
      return [];
    }
    if (selectedTask) {
      const logs = await tasksApi.getTaskLogTasksTaskIdLogGet(
        selectedTask.booking.id,
        '0,99999999',
      );
      const state = await tasksApi.getTaskStateTasksTaskIdStateGet(selectedTask.booking.id);
      setSelectedTaskLog(logs.data);
      setSelectedTaskState(state.data);
    }
  }, [tasksApi, selectedTask]);

  React.useEffect(() => {
    fetchLogs();
  }, [selectedTask, fetchLogs]);

  const autoRefreshTooltipPrefix = autoRefresh ? 'Disable' : 'Enable';

  const taskCancellable =
    selectedTask &&
    profile &&
    Enforcer.canCancelTask(profile) &&
    (selectedTask.active || selectedTask.pending);
  // selectedTask. === RmfTaskSummary.STATE_QUEUED

  return (
    <StyledDiv {...divProps}>
      <Grid container wrap="nowrap" justifyContent="center" style={{ height: 'inherit' }}>
        <Grid item xs={5} className={classes.tableGrid}>
          <Paper className={classes.tableContainer}>
            <Toolbar>
              <Typography className={classes.tableTitle} variant="h6">
                Tasks
              </Typography>
              <Tooltip title={`${autoRefreshTooltipPrefix} auto refresh`}>
                <IconButton
                  className={autoRefresh ? classes.enabledToggleButton : undefined}
                  onClick={() => {
                    setAutoRefresh((prev) => !prev);
                    onAutoRefresh && onAutoRefresh(!autoRefresh);
                  }}
                  aria-label={`${autoRefreshTooltipPrefix} auto refresh`}
                >
                  <AutorenewIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="Refresh">
                <IconButton onClick={() => onRefresh && onRefresh()} aria-label="Refresh">
                  <RefreshIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="Create task">
                <IconButton onClick={() => setOpenCreateTaskForm(true)} aria-label="Create Task">
                  <AddOutlinedIcon />
                </IconButton>
              </Tooltip>
            </Toolbar>
            <TableContainer>
              <TaskTable
                tasks={tasks.map((t) => t)}
                onTaskClick={(_ev, task) =>
                  setSelectedTask(tasks.find((t) => t.booking.id === task.booking.id))
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
          </Paper>
        </Grid>
        <Grid className={classes.timelineGrid} item xs={4}>
          <Paper className={classes.taskInfoContainer}>
            {selectedTask ? (
              <>
                {selectedTaskState && taskCancellable ? (
                  <TaskInfo task={selectedTaskState} />
                ) : null}
              </>
            ) : (
              <NoSelectedTask />
            )}
          </Paper>
          <Button
            style={{ marginTop: theme.spacing(1), marginBottom: theme.spacing(1) }}
            fullWidth
            variant="contained"
            color="secondary"
            aria-label="Cancel Task"
            disabled={!taskCancellable}
            onClick={handleCancelTaskClick}
          >
            Cancel Task
          </Button>
        </Grid>
        <Grid item xs={4} className={classes.logsGrid}>
          {selectedTaskLog && selectedTaskState && taskCancellable ? (
            <TaskLogs taskLog={selectedTaskLog} taskState={selectedTaskState} />
          ) : null}
        </Grid>
      </Grid>
      {openCreateTaskForm && (
        <CreateTaskForm
          cleaningZones={cleaningZones}
          loopWaypoints={loopWaypoints}
          deliveryWaypoints={deliveryWaypoints}
          dispensers={dispensers}
          ingestors={ingestors}
          open={openCreateTaskForm}
          onClose={() => setOpenCreateTaskForm(false)}
          submitTasks={submitTasks}
          // tasksFromFile={tasksFromFile}
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
    </StyledDiv>
  );
}

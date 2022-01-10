import {
  Alert,
  AlertProps,
  Button,
  Grid,
  IconButton,
  Paper,
  Snackbar,
  TableContainer,
  TablePagination,
  Toolbar,
  Tooltip,
  Typography,
  useTheme,
  styled,
} from '@mui/material';
import {
  AddOutlined as AddOutlinedIcon,
  Autorenew as AutorenewIcon,
  Refresh as RefreshIcon,
} from '@mui/icons-material';
import { TaskState, TaskEventLog } from 'api-client';
import React from 'react';
import { CreateTaskForm, CreateTaskFormProps, TaskInfo, TaskTable } from 'react-components';
import { UserProfileContext } from 'rmf-auth';
import { AppControllerContext } from '../app-contexts';
import { Enforcer } from '../permissions';
import { parseTasksFile } from './utils';
import { TaskLogs } from './task-logs';
import { RmfIngressContext } from '../rmf-app';

const prefix = 'task-panel';
const classes = {
  tableContainer: `${prefix}-table-container`,
  tableTitle: `${prefix}-table-title`,
  detailPanelContainer: `${prefix}-detail-panel-container`,
  enabledToggleButton: `${prefix}-enable-toggle-button`,
};

const StyledDiv = styled('div')(({ theme }) => ({
  [`& .${classes.tableContainer}`]: {
    display: 'flex',
    flexDirection: 'column',
  },
  [`& .${classes.tableTitle}`]: {
    flex: '1 1 100%',
  },
  [`& .${classes.detailPanelContainer}`]: {
    width: 350,
    padding: theme.spacing(2),
    marginLeft: theme.spacing(2),
    flex: '0 0 auto',
  },
  [`& .${classes.enabledToggleButton}`]: {
    background: theme.palette.action.selected,
  },
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
  const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');
  const [autoRefresh, setAutoRefresh] = React.useState(true);
  const [showLogs, setShowLogs] = React.useState(false);
  const [selectedTaskLog, setSelectedTaskLog] = React.useState<TaskEventLog | undefined>(undefined);
  const profile = React.useContext(UserProfileContext);
  const { showErrorAlert } = React.useContext(AppControllerContext);
  const { tasksApi } = React.useContext(RmfIngressContext) || {};

  const handleCancelTaskClick = React.useCallback<React.MouseEventHandler>(async () => {
    if (!cancelTask || !selectedTask) {
      return;
    }
    try {
      // await cancelTask(selectedTask.summary);
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
  const tasksFromFile = (): Promise<TaskState[]> => {
    return new Promise((res) => {
      const fileInputEl = uploadFileInputRef.current;
      if (!fileInputEl) {
        return [];
      }
      let taskFiles: TaskState[];
      const listener = async () => {
        try {
          if (!fileInputEl.files || fileInputEl.files.length === 0) {
            return res([]);
          }
          try {
            taskFiles = parseTasksFile(await fileInputEl.files[0].text());
          } catch (err) {
            showErrorAlert((err as Error).message, 5000);
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

  const fetchLogs = React.useCallback(async () => {
    if (!tasksApi) {
      return [];
    }
    if (selectedTask) {
      const logs = await tasksApi.getTaskLogTasksTaskIdLogGet(selectedTask.booking.id);
      setSelectedTaskLog(logs.data);
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
            <TablePagination component="div" {...paginationOptions} style={{ flex: '0 0 auto' }} />
          )}
        </Paper>
        <Paper className={classes.detailPanelContainer}>
          {selectedTask ? (
            <>
              <TaskInfo task={selectedTask} showLogs={showLogs} onShowLogs={setShowLogs} />
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
        {showLogs && selectedTaskLog ? <TaskLogs taskLog={selectedTaskLog} /> : null}
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
          tasksFromFile={tasksFromFile}
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

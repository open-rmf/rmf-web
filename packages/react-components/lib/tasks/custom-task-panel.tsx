import {
  Grid,
  IconButton,
  makeStyles,
  Paper,
  Snackbar,
  TableContainer,
  TablePagination,
  Toolbar,
  Tooltip,
  Typography,
} from '@material-ui/core';
import { Autorenew as AutorenewIcon } from '@material-ui/icons';
import { Alert, AlertProps } from '@material-ui/lab';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskInfo } from './task-info';
import { CustomTaskTable } from './custom-task-table';

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

export interface FetchCustomTasksResult {
  tasks: RmfModels.TaskSummary[];
  totalCount: number;
}

export interface CustomTaskPanelProps extends React.HTMLProps<HTMLDivElement> {
  /**
   * Should only contain the tasks of the current page.
   */
  tasks: RmfModels.TaskSummary[];
  paginationOptions?: Omit<React.ComponentPropsWithoutRef<typeof TablePagination>, 'component'>;
  cancelTask?: (task: RmfModels.TaskSummary) => Promise<void>;
  onAutoRefresh?: (enabled: boolean) => void;
}

export function CustomTaskPanel({
  tasks,
  paginationOptions,
  cancelTask,
  onAutoRefresh,
  ...divProps
}: CustomTaskPanelProps): JSX.Element {
  const classes = useStyles();
  const [selectedTask, setSelectedTask] = React.useState<RmfModels.TaskSummary | undefined>(
    undefined,
  );

  const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');
  const [autoRefresh, setAutoRefresh] = React.useState(true);

  const handleCancelTask = React.useCallback(
    async (task: RmfModels.TaskSummary) => {
      if (!cancelTask) {
        return;
      }
      try {
        await cancelTask(task);
        setSnackbarMessage('Successfully cancelled task');
        setSnackbarSeverity('success');
        setOpenSnackbar(true);
        setSelectedTask(undefined);
      } catch (e) {
        setSnackbarMessage(`Failed to cancel task: ${e.message}`);
        setSnackbarSeverity('error');
        setOpenSnackbar(true);
      }
    },
    [cancelTask],
  );

  const autoRefreshTooltipPrefix = autoRefresh ? 'Disable' : 'Enable';

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Paper className={classes.tableContainer}>
          <Toolbar>
            <Typography className={classes.tableTitle} variant="h6">
              Beverage Station Tasks
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
          </Toolbar>
          <TableContainer>
            <CustomTaskTable
              tasks={tasks}
              onTaskClick={(_ev, task) => {
                setSelectedTask(task);
              }}
            />
          </TableContainer>
          {paginationOptions && (
            <TablePagination component="div" {...paginationOptions} style={{ flex: '0 0 auto' }} />
          )}
        </Paper>
        <Paper className={classes.detailPanelContainer}>
          {selectedTask ? (
            <TaskInfo
              task={selectedTask}
              onCancelTaskClick={() => handleCancelTask(selectedTask)}
            />
          ) : (
            <NoSelectedTask />
          )}
        </Paper>
      </Grid>
      <input type="file" style={{ display: 'none' }} ref={uploadFileInputRef} />
      <Snackbar open={openSnackbar} onClose={() => setOpenSnackbar(false)} autoHideDuration={2000}>
        <Alert severity={snackbarSeverity}>{snackbarMessage}</Alert>
      </Snackbar>
    </div>
  );
}

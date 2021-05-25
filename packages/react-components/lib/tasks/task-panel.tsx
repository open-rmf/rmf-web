import { Grid, makeStyles, Paper, Snackbar, Typography } from '@material-ui/core';
import { Alert, AlertProps } from '@material-ui/lab';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { CreateTaskForm, CreateTaskFormProps } from './create-task';
import { TaskInfo } from './task-info';
import { TaskTable } from './task-table';

const useStyles = makeStyles((theme) => ({
  detailPanelContainer: {
    width: 350,
    padding: theme.spacing(2),
    marginLeft: theme.spacing(2),
    flex: '0 0 auto',
    backgroundColor: theme.mainBackground,
    color: theme.fontColors,
  },
  taskTable: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
    backgroundColor: theme.mainBackground,
    color: theme.fontColors,
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

export interface FetchTasksResult {
  tasks: RmfModels.TaskSummary[];
  totalCount: number;
}

export interface TaskPanelProps extends React.HTMLProps<HTMLDivElement> {
  fetchTasks: (limit: number, offset: number) => Promise<FetchTasksResult>;
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  submitTask?: CreateTaskFormProps['submitTask'];
}

export function TaskPanel({
  fetchTasks,
  cleaningZones,
  loopWaypoints,
  deliveryWaypoints,
  dispensers,
  ingestors,
  submitTask,
  ...divProps
}: TaskPanelProps): JSX.Element {
  const classes = useStyles();
  const [tasks, setTasks] = React.useState<RmfModels.TaskSummary[]>([]);
  const [totalCount, setTotalCount] = React.useState(-1);
  const [page, setPage] = React.useState(0);
  const [selectedTask, setSelectedTask] = React.useState<RmfModels.TaskSummary | undefined>(
    undefined,
  );
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');

  const handleRefresh = React.useCallback(async () => {
    (async () => {
      const result = await fetchTasks(10, page * 10);
      setTasks(result.tasks);
      setTotalCount(result.totalCount);
    })();
  }, [fetchTasks, page]);

  React.useEffect(() => {
    handleRefresh();
  }, [handleRefresh]);

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Grid style={{ flex: '1 1 auto' }}>
          <TaskTable
            elevation={6}
            className={classes.taskTable}
            tasks={tasks}
            paginationOptions={{
              count: totalCount,
              rowsPerPage: 10,
              rowsPerPageOptions: [10],
              page,
              onChangePage: (_ev, newPage) => setPage(newPage),
            }}
            onCreateTaskClick={() => setOpenCreateTaskForm(true)}
            onTaskClick={(_ev, task) => setSelectedTask(task)}
            onRefreshClick={handleRefresh}
          />
        </Grid>
        <Paper elevation={6} className={classes.detailPanelContainer}>
          {selectedTask ? <TaskInfo task={selectedTask} /> : <NoSelectedTask />}
        </Paper>
      </Grid>
      <CreateTaskForm
        cleaningZones={cleaningZones}
        loopWaypoints={loopWaypoints}
        deliveryWaypoints={deliveryWaypoints}
        dispensers={dispensers}
        ingestors={ingestors}
        open={openCreateTaskForm}
        onClose={() => setOpenCreateTaskForm(false)}
        submitTask={submitTask}
        onCancelClick={() => setOpenCreateTaskForm(false)}
        onSuccess={() => {
          setOpenCreateTaskForm(false);
          setSnackbarSeverity('success');
          setSnackbarMessage('Successfully created task');
          setOpenSnackbar(true);
          handleRefresh();
        }}
        onFail={(e) => {
          setSnackbarSeverity('error');
          setSnackbarMessage(`Failed to create task: ${e.message}`);
          setOpenSnackbar(true);
        }}
      />
      <Snackbar open={openSnackbar} onClose={() => setOpenSnackbar(false)} autoHideDuration={2000}>
        <Alert severity={snackbarSeverity}>{snackbarMessage}</Alert>
      </Snackbar>
    </div>
  );
}

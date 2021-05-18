import { Grid, makeStyles, Paper, Snackbar, Typography } from '@material-ui/core';
import { Alert, AlertProps } from '@material-ui/lab';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { CreateTaskForm, CreateTaskFormProps } from './create-task';
import { TaskInfo } from './task-info';
import { TaskTable, TaskTableProps } from './task-table';

const useStyles = makeStyles((theme) => ({
  detailPanelContainer: {
    width: 350,
    padding: theme.spacing(2),
    marginLeft: theme.spacing(2),
    flex: '0 0 auto',
    backgroundColor: theme.palette.primary.main,
    color: theme.fontColors,
  },
  taskTable: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
    backgroundColor: theme.palette.primary.main,
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

export interface TaskPanelProps extends React.HTMLProps<HTMLDivElement> {
  tasks: RmfModels.TaskSummary[];
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  submitTask?: CreateTaskFormProps['submitTask'];
  onRefreshClick?: TaskTableProps['onRefreshClick'];
}

export function TaskPanel({
  tasks,
  cleaningZones,
  loopWaypoints,
  deliveryWaypoints,
  dispensers,
  ingestors,
  submitTask,
  onRefreshClick,
  ...divProps
}: TaskPanelProps): JSX.Element {
  const classes = useStyles();
  const [page, setPage] = React.useState(0);
  const [selectedTask, setSelectedTask] = React.useState<RmfModels.TaskSummary | undefined>(
    undefined,
  );
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Grid style={{ flex: '1 1 auto' }}>
          <TaskTable
            elevation={6}
            className={classes.taskTable}
            tasks={tasks.slice(page * 10, (page + 1) * 10)}
            paginationOptions={{
              count: tasks.length,
              rowsPerPage: 10,
              rowsPerPageOptions: [10],
              page,
              onChangePage: (_ev, newPage) => setPage(newPage),
            }}
            onCreateTaskClick={() => setOpenCreateTaskForm(true)}
            onTaskClick={(_ev, task) => setSelectedTask(task)}
            onRefreshClick={onRefreshClick}
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

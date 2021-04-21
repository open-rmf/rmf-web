import { Grid, makeStyles, Paper, Typography } from '@material-ui/core';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskInfo } from './task-info';
import { TaskTable, TaskTableProps } from './task-table';

const useStyles = makeStyles((theme) => ({
  detailPanelContainer: {
    width: 350,
    padding: theme.spacing(2),
    marginLeft: theme.spacing(1),
    flex: '0 0 auto',
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

export interface TaskPanelProps extends React.HTMLProps<HTMLDivElement> {
  tasks: RmfModels.TaskSummary[];
  submitTask?: TaskTableProps['submitTask'];
  onRefreshClick?: React.MouseEventHandler<HTMLButtonElement>;
}

export function TaskPanel({
  tasks,
  submitTask,
  onRefreshClick,
  ...divProps
}: TaskPanelProps): JSX.Element {
  const classes = useStyles();
  const [page, setPage] = React.useState(0);
  const [selectedTask, setSelectedTask] = React.useState<RmfModels.TaskSummary | undefined>(
    undefined,
  );

  return (
    <div {...divProps}>
      <Grid container wrap="nowrap" justify="center" style={{ height: 'inherit' }}>
        <Grid style={{ flex: '1 1 auto' }}>
          <TaskTable
            tasks={tasks.slice(page * 10, (page + 1) * 10)}
            paginationOptions={{
              count: tasks.length,
              rowsPerPage: 10,
              rowsPerPageOptions: [10],
              page,
              onChangePage: (_ev, newPage) => setPage(newPage),
            }}
            submitTask={submitTask}
            onTaskClick={(_ev, task) => setSelectedTask(task)}
            onRefreshClick={onRefreshClick}
          />
        </Grid>
        <Paper className={classes.detailPanelContainer}>
          {selectedTask ? <TaskInfo task={selectedTask} /> : <NoSelectedTask />}
        </Paper>
      </Grid>
    </div>
  );
}

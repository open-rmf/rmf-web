import React from 'react';
import { makeStyles, TableContainer, Paper } from '@material-ui/core';
import type { TaskSummary } from 'api-client';
import { TaskTable } from 'react-components';

interface TaskDisplayProps {
  tasks: TaskSummary[];
}

const useStyles = makeStyles(() => ({
  root: {
    height: '100%',
  },
}));

export const TaskDisplay = (props: TaskDisplayProps) => {
  const classes = useStyles();
  const { tasks } = props;
  return (
    <Paper className={classes.root} variant="outlined" aria-label="task-display">
      <TableContainer>
        <TaskTable tasks={tasks} />
      </TableContainer>
    </Paper>
  );
};

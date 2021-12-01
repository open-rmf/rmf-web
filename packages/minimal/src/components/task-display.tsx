import React from 'react';
import { TableContainer, Paper, PaperProps, styled } from '@mui/material';
import type { TaskSummary } from 'api-client';
import { TaskTable } from 'react-components';

interface TaskDisplayProps {
  tasks: TaskSummary[];
}

const classes = {
  root: 'task-display',
};
const StyledPaper = styled((props: PaperProps) => <Paper {...props} />)(() => ({
  [`&.${classes.root}`]: {
    height: '100%',
  },
}));

export const TaskDisplay = (props: TaskDisplayProps) => {
  const { tasks } = props;
  return (
    <StyledPaper className={classes.root} variant="outlined" aria-label="task-display">
      <TableContainer>
        <TaskTable tasks={tasks} />
      </TableContainer>
    </StyledPaper>
  );
};

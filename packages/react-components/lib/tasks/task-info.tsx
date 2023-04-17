import { Divider, Typography, useTheme } from '@mui/material';
import { styled } from '@mui/material';
import type { TaskState } from 'api-client';
import React from 'react';
import { TaskTimeline } from './task-timeline';

const classes = {
  infoValue: 'task-info-info-value',
};
const StyledDiv = styled('div')(() => ({
  [`& .${classes.infoValue}`]: {
    float: 'right',
    textAlign: 'right',
  },
}));

function InfoLine({ children }: React.PropsWithChildren<unknown>) {
  return (
    <Typography variant="body1" gutterBottom>
      {children}
    </Typography>
  );
}

function InfoValue({ children }: React.PropsWithChildren<unknown>) {
  return <span className={classes.infoValue}>{children}</span>;
}

export interface TaskInfoProps {
  task: TaskState;
  title?: string;
}

export function TaskInfo({ task, title }: TaskInfoProps): JSX.Element {
  const theme = useTheme();

  return (
    <StyledDiv>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {title ? title : task.booking.id}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <InfoLine>
        <span>State:</span>
        <InfoValue>{task.status || 'unknown'}</InfoValue>
      </InfoLine>
      <div style={{ padding: '4px' }}>
        <TaskTimeline taskState={task} />
      </div>
    </StyledDiv>
  );
}

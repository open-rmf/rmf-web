import { Divider, makeStyles, Typography, useTheme } from '@material-ui/core';
import React from 'react';
import { ScheduledTask } from './scheduled-task-table';

const useStyles = makeStyles({
  infoValue: {
    float: 'right',
    textAlign: 'right',
  },
});

function InfoLine({ children }: React.PropsWithChildren<unknown>) {
  return (
    <Typography variant="body1" gutterBottom>
      {children}
    </Typography>
  );
}

function InfoValue({ children }: React.PropsWithChildren<unknown>) {
  const classes = useStyles();
  return <span className={classes.infoValue}>{children}</span>;
}

export interface ScheduledTaskInfoProps {
  task: ScheduledTask;
}

export function ScheduledTaskInfo({ task }: ScheduledTaskInfoProps): JSX.Element {
  const theme = useTheme();
  console.log(task);
  return (
    <div>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {task.id}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <InfoLine>
        <span>Enabled:</span>
        <InfoValue>{task.enabled.toString()}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Created by Rule:</span>
        <InfoValue>{task.rule?.name}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Task Type:</span>
        <InfoValue>{task.task_type}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Start Time:</span>
        <InfoValue>{task.task_datetime}</InfoValue>
      </InfoLine>
    </div>
  );
}

export default ScheduledTaskInfo;

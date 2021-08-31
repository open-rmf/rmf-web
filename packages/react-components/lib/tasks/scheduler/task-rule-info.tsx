import { Divider, makeStyles, Typography, useTheme } from '@material-ui/core';
import React from 'react';
import { TaskRule } from './task-rules-table';

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

export interface TaskRuleInfoProps {
  task: TaskRule;
}

export function TaskRuleInfo({ task }: TaskRuleInfoProps): JSX.Element {
  const theme = useTheme();

  return (
    <div>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {task.id}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <InfoLine>
        <span>Name:</span>
        <InfoValue>{task.name}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Task Type:</span>
        <InfoValue>{task.task_type}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Frequency:</span>
        <InfoValue>{task.frequency}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Frequency Type:</span>
        <InfoValue>{task.frequency_type}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Start Time:</span>
        <InfoValue>{task.start_datetime}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>End Time:</span>
        <InfoValue>{task.end_datetime}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Day of week:</span>
        {/* <InfoValue>{task.days_of_week}</InfoValue> */}
      </InfoLine>
    </div>
  );
}

export default TaskRuleInfo;

import { Button, Divider, Typography, useTheme } from '@mui/material';
import { styled } from '@mui/material';
import type { TaskState } from 'api-client';
import React from 'react';
import { TaskTimeline } from './task-timeline';
import { parseTaskDetail, getState } from './utils';

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

interface CleanTaskInfoProps {
  task: TaskState;
}

function CleanTaskInfo({ task }: CleanTaskInfoProps) {
  return (
    <InfoLine>
      <span>Start Waypoint:</span>
      <span style={{ float: 'right' }}>{parseTaskDetail(task, task?.category).to}</span>
    </InfoLine>
  );
}

interface LoopTaskInfoProps {
  task: TaskState;
}

function LoopTaskInfo({ task }: LoopTaskInfoProps) {
  return (
    <>
      <InfoLine>
        <span>Start Waypoint:</span>
        <InfoValue>{parseTaskDetail(task, task?.category).from}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Finish Waypoint:</span>
        <InfoValue>{parseTaskDetail(task, task?.category).to}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Num of Loops:</span>
        <InfoValue>{task.phases ? Object.keys(task.phases).length / 2 : null}</InfoValue>
      </InfoLine>
    </>
  );
}

interface DeliveryTaskInfoProps {
  task: TaskState;
}

function DeliveryTaskInfoProps({ task }: DeliveryTaskInfoProps) {
  // TODO - replace all temp values
  return (
    <>
      <InfoLine>
        <span>Pickup Location:</span>
        <span style={{ float: 'right' }}>{'temp'}</span>
      </InfoLine>
      <InfoLine>
        <span>Pickup Dispenser:</span>
        <span style={{ float: 'right' }}>{'temp'}</span>
      </InfoLine>
      <InfoLine>
        <span>Dropoff Location:</span>
        <span style={{ float: 'right' }}>{parseTaskDetail(task, task?.category).from}</span>
      </InfoLine>
      <InfoLine>
        <span>Dropoff Ingestor:</span>
        <span style={{ float: 'right' }}>{parseTaskDetail(task, task?.category).to}</span>
      </InfoLine>
    </>
  );
}

export interface TaskInfoProps {
  task: TaskState;
  showLogs: boolean;
  onShowLogs?: React.Dispatch<React.SetStateAction<boolean>>;
}

export function TaskInfo({ task, showLogs, onShowLogs }: TaskInfoProps): JSX.Element {
  const theme = useTheme();
  const taskType = task.category;
  const detailInfo = (() => {
    switch (taskType) {
      case 'Clean':
        return <CleanTaskInfo task={task} />;
      case 'Loop':
        return <LoopTaskInfo task={task} />;
      case 'Delivery':
        return <DeliveryTaskInfoProps task={task} />;
      default:
        return null;
    }
  })();

  return (
    <StyledDiv>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {task.booking.id}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <InfoLine>
        <span>State:</span>
        <InfoValue>{getState(task)}</InfoValue>
      </InfoLine>
      {detailInfo}
      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
        <Typography variant="h6">Progress</Typography>
        <Button variant="contained" onClick={() => onShowLogs && onShowLogs(!showLogs)}>
          {showLogs ? 'CLOSE LOGS' : 'SHOW LOGS'}
        </Button>
      </div>
      <div style={{ padding: '4px' }}>
        <TaskTimeline taskState={task} />
      </div>
    </StyledDiv>
  );
}

import { Divider, makeStyles, Typography, useTheme } from '@material-ui/core';
import type { TaskSummary } from 'api-client';
import React from 'react';
import { TaskSummary as RmfTaskSummary, TaskType as RmfTaskType } from 'rmf-models';
import { rosTimeToJs } from '../utils';
import { TaskTimeline } from './task-timeline';
import { taskStateToStr, taskTypeToStr } from './utils';

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

interface CleanTaskInfoProps {
  task: TaskSummary;
}

function CleanTaskInfo({ task }: CleanTaskInfoProps) {
  return (
    <InfoLine>
      <span>Start Waypoint:</span>
      <span style={{ float: 'right' }}>{task.task_profile.description.clean.start_waypoint}</span>
    </InfoLine>
  );
}

interface LoopTaskInfoProps {
  task: TaskSummary;
}

function LoopTaskInfo({ task }: LoopTaskInfoProps) {
  return (
    <>
      <InfoLine>
        <span>Start Waypoint:</span>
        <InfoValue>{task.task_profile.description.loop.start_name}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Finish Waypoint:</span>
        <InfoValue>{task.task_profile.description.loop.finish_name}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Num of Loops:</span>
        <InfoValue>{task.task_profile.description.loop.num_loops}</InfoValue>
      </InfoLine>
    </>
  );
}

interface DeliveryTaskInfoProps {
  task: TaskSummary;
}

function DeliveryTaskInfoProps({ task }: DeliveryTaskInfoProps) {
  return (
    <>
      <InfoLine>
        <span>Pickup Location:</span>
        <span style={{ float: 'right' }}>
          {task.task_profile.description.delivery.pickup_place_name}
        </span>
      </InfoLine>
      <InfoLine>
        <span>Pickup Dispenser:</span>
        <span style={{ float: 'right' }}>
          {task.task_profile.description.delivery.pickup_dispenser}
        </span>
      </InfoLine>
      <InfoLine>
        <span>Dropoff Location:</span>
        <span style={{ float: 'right' }}>
          {task.task_profile.description.delivery.dropoff_place_name}
        </span>
      </InfoLine>
      <InfoLine>
        <span>Dropoff Ingestor:</span>
        <span style={{ float: 'right' }}>
          {task.task_profile.description.delivery.dropoff_ingestor}
        </span>
      </InfoLine>
    </>
  );
}

export interface TaskInfoProps {
  task: TaskSummary;
}

export function TaskInfo({ task }: TaskInfoProps): JSX.Element {
  const theme = useTheme();
  const taskType = task.task_profile.description.task_type.type;
  const hasConcreteEndTime = [
    RmfTaskSummary.STATE_CANCELED,
    RmfTaskSummary.STATE_COMPLETED,
    RmfTaskSummary.STATE_FAILED,
  ].includes(task.state);

  const detailInfo = (() => {
    switch (taskType) {
      case RmfTaskType.TYPE_CLEAN:
        return <CleanTaskInfo task={task} />;
      case RmfTaskType.TYPE_LOOP:
        return <LoopTaskInfo task={task} />;
      case RmfTaskType.TYPE_DELIVERY:
        return <DeliveryTaskInfoProps task={task} />;
      default:
        return null;
    }
  })();

  return (
    <div>
      <Typography variant="h6" style={{ textAlign: 'center' }} gutterBottom>
        {task.task_id}
      </Typography>
      <Divider />
      <div style={{ marginBottom: theme.spacing(1) }}></div>
      <InfoLine>
        <span>Task Type:</span>
        <InfoValue>{taskTypeToStr(taskType)}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Priority:</span>
        <InfoValue>{task.task_profile.description.priority.value}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Assigned Robot:</span>
        <InfoValue>{task.robot_name}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>Start Time:</span>
        <InfoValue>{rosTimeToJs(task.start_time).toLocaleString()}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>{!hasConcreteEndTime && 'Est. '}End Time:</span>
        <InfoValue>{rosTimeToJs(task.end_time).toLocaleString()}</InfoValue>
      </InfoLine>
      <InfoLine>
        <span>State:</span>
        <InfoValue>{taskStateToStr(task.state)}</InfoValue>
      </InfoLine>
      {detailInfo}
      <Typography variant="h6">Progress</Typography>
      <TaskTimeline taskSummary={task} />
    </div>
  );
}

import { Divider, Typography, useTheme } from '@material-ui/core';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { rosTimeToJs } from '../utils';
import { taskStateToStr, taskTypeToStr } from './utils';

function InfoLine({ children }: React.PropsWithChildren<unknown>) {
  return (
    <Typography variant="body1" gutterBottom>
      {children}
    </Typography>
  );
}

interface CleanTaskInfoProps {
  task: RmfModels.TaskSummary;
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
  task: RmfModels.TaskSummary;
}

function LoopTaskInfo({ task }: LoopTaskInfoProps) {
  return (
    <>
      <InfoLine>
        <span>Start Waypoint:</span>
        <span style={{ float: 'right' }}>{task.task_profile.description.loop.start_name}</span>
      </InfoLine>
      <InfoLine>
        <span>Finish Waypoint:</span>
        <span style={{ float: 'right' }}>{task.task_profile.description.loop.start_name}</span>
      </InfoLine>
      <InfoLine>
        <span>Num of Loops:</span>
        <span style={{ float: 'right' }}>{task.task_profile.description.loop.num_loops}</span>
      </InfoLine>
    </>
  );
}

interface DeliveryTaskInfoProps {
  task: RmfModels.TaskSummary;
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
  task: RmfModels.TaskSummary;
}

export function TaskInfo({ task }: TaskInfoProps): JSX.Element {
  const theme = useTheme();
  const taskType = task.task_profile.description.task_type.type;
  const hasConcreteEndTime = [
    RmfModels.TaskSummary.STATE_CANCELED,
    RmfModels.TaskSummary.STATE_COMPLETED,
    RmfModels.TaskSummary.STATE_FAILED,
  ].includes(task.state);

  const detailInfo = (() => {
    switch (taskType) {
      case RmfModels.TaskType.TYPE_CLEAN:
        return <CleanTaskInfo task={task} />;
      case RmfModels.TaskType.TYPE_LOOP:
        return <LoopTaskInfo task={task} />;
      case RmfModels.TaskType.TYPE_DELIVERY:
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
        <span style={{ float: 'right' }}>{taskTypeToStr(taskType)}</span>
      </InfoLine>
      <InfoLine>
        <span>Priority:</span>
        <span style={{ float: 'right' }}>{task.task_profile.description.priority.value}</span>
      </InfoLine>
      <InfoLine>
        <span>Assigned Robot:</span>
        <span style={{ float: 'right' }}>{task.robot_name}</span>
      </InfoLine>
      <InfoLine>
        <span>Start Time:</span>
        <span style={{ float: 'right' }}>{rosTimeToJs(task.start_time).toLocaleString()}</span>
      </InfoLine>
      <InfoLine>
        <span>{!hasConcreteEndTime && 'Est. '}End Time:</span>
        <span style={{ float: 'right' }}>{rosTimeToJs(task.end_time).toLocaleString()}</span>
      </InfoLine>
      <InfoLine>
        <span>State:</span>
        <span style={{ float: 'right' }}>{taskStateToStr(task.state)}</span>
      </InfoLine>
      {detailInfo}
    </div>
  );
}

import React from 'react';
import type {
  CleanTaskDescription,
  DeliveryTaskDescription,
  LoopTaskDescription,
  SubmitTask,
} from 'api-client';
import * as RmfModels from 'rmf-models';

import { CleanTaskForm, DeliveryTaskForm, LoopTaskForm } from '../create-task';

type TaskDescription = CleanTaskDescription | LoopTaskDescription | DeliveryTaskDescription;

export interface ScheduleTaskDescriptionFormProps {
  /**
   * Shows extra UI elements suitable for submittng batched tasks. Default to 'false'.
   */
  // allowBatch?: boolean;
  task: SubmitTask;
  handleTaskDescriptionChange: (taskType: number, taskDesc: TaskDescription) => void;
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
}

export const TaskDescriptionForm = (
  props: ScheduleTaskDescriptionFormProps,
): JSX.Element | null => {
  const {
    task,
    handleTaskDescriptionChange,
    cleaningZones = [],
    loopWaypoints = [],
    deliveryWaypoints = [],
    dispensers = [],
    ingestors = [],
  } = props;

  if (task.task_type === -1) {
    return null;
  }
  switch (task.task_type) {
    case RmfModels.TaskType.TYPE_CLEAN:
      return (
        <CleanTaskForm
          taskDesc={task.description as CleanTaskDescription}
          cleaningZones={cleaningZones}
          onChange={(desc) => handleTaskDescriptionChange(RmfModels.TaskType.TYPE_CLEAN, desc)}
        />
      );
    case RmfModels.TaskType.TYPE_LOOP:
      return (
        <LoopTaskForm
          taskDesc={task.description as LoopTaskDescription}
          loopWaypoints={loopWaypoints}
          onChange={(desc) => handleTaskDescriptionChange(RmfModels.TaskType.TYPE_LOOP, desc)}
        />
      );
    case RmfModels.TaskType.TYPE_DELIVERY:
      return (
        <DeliveryTaskForm
          taskDesc={task.description as DeliveryTaskDescription}
          deliveryWaypoints={deliveryWaypoints}
          dispensers={dispensers}
          ingestors={ingestors}
          onChange={(desc) => handleTaskDescriptionChange(RmfModels.TaskType.TYPE_DELIVERY, desc)}
        />
      );
    default:
      return null;
  }
};

export default TaskDescriptionForm;

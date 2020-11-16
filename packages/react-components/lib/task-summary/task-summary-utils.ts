import * as RomiCore from '@osrf/romi-js-core-interfaces';

// TODO: this is a hacky solution to get the actor from the task status,
// this should be changed when then backend sends an actor on the
// taskSummary message (we should use the actor provided by the message).
// https://github.com/osrf/rmf_core/issues/205
export const getActorFromStatus = (status: string) => {
  // Gets the name of the robot if it has any
  // eslint-disable-next-line
  return status.match(/\[[A-Za-z]([a-zA-Z0-9\/]){3,}\]+/gi);
};

export const formatStatus = (status: string) => {
  return status.split('|');
};

export const getStateLabel = (state: number): string => {
  switch (state) {
    case RomiCore.TaskSummary.STATE_QUEUED:
      return 'QUEUED';
    case RomiCore.TaskSummary.STATE_ACTIVE:
      return 'ACTIVE';
    case RomiCore.TaskSummary.STATE_COMPLETED:
      return 'COMPLETED';
    case RomiCore.TaskSummary.STATE_FAILED:
      return 'FAILED';
    default:
      return 'UNKNOWN';
  }
};

/**
 * Sorts tasks by state so what is active is always at the top of the list.
 */
export const sortTasksByState = (
  tasks: Record<string, RomiCore.TaskSummary>,
): RomiCore.TaskSummary[] => {
  let stateTasks = {
    completed: [] as RomiCore.TaskSummary[],
    failed: [] as RomiCore.TaskSummary[],
    active: [] as RomiCore.TaskSummary[],
    queued: [] as RomiCore.TaskSummary[],
    unknown: [] as RomiCore.TaskSummary[],
  };

  Object.keys(tasks).forEach((key) => {
    switch (tasks[key].state) {
      case RomiCore.TaskSummary.STATE_QUEUED:
        stateTasks.queued.push(tasks[key]);
        break;
      case RomiCore.TaskSummary.STATE_ACTIVE:
        stateTasks.active.push(tasks[key]);
        break;
      case RomiCore.TaskSummary.STATE_COMPLETED:
        stateTasks.completed.push(tasks[key]);
        break;
      case RomiCore.TaskSummary.STATE_FAILED:
        stateTasks.failed.push(tasks[key]);
        break;
      default:
        stateTasks.unknown.push(tasks[key]);
        break;
    }
  });
  let sortedTasks: RomiCore.TaskSummary[] = [];
  sortedTasks.push(...stateTasks.active);
  sortedTasks.push(...stateTasks.queued);
  sortedTasks.push(...stateTasks.failed);
  sortedTasks.push(...stateTasks.completed);
  sortedTasks.push(...stateTasks.unknown);

  return sortedTasks;
};

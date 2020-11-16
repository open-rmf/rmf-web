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
 * Sorts the tasks by state so what is active in the moment is always
 * at the top.
 */
export const sortTasksByState = (
  tasks: Record<string, RomiCore.TaskSummary>,
): RomiCore.TaskSummary[] => {
  let completed: RomiCore.TaskSummary[] = [];
  let failed: RomiCore.TaskSummary[] = [];
  let active: RomiCore.TaskSummary[] = [];
  let queued: RomiCore.TaskSummary[] = [];
  let unknown: RomiCore.TaskSummary[] = [];

  Object.keys(tasks).forEach((key) => {
    switch (tasks[key].state) {
      case RomiCore.TaskSummary.STATE_QUEUED:
        queued.push(tasks[key]);
        break;
      case RomiCore.TaskSummary.STATE_ACTIVE:
        active.push(tasks[key]);
        break;
      case RomiCore.TaskSummary.STATE_COMPLETED:
        completed.push(tasks[key]);
        break;
      case RomiCore.TaskSummary.STATE_FAILED:
        failed.push(tasks[key]);
        break;
      default:
        unknown.push(tasks[key]);
        break;
    }
  });
  let sortedTasks: RomiCore.TaskSummary[] = [];
  sortedTasks.push(...active);
  sortedTasks.push(...queued);
  sortedTasks.push(...failed);
  sortedTasks.push(...completed);
  sortedTasks.push(...unknown);

  return sortedTasks;
};

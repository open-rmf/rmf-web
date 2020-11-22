import * as RomiCore from '@osrf/romi-js-core-interfaces';

// TODO: this is a hacky solution to get the actor from the task status,
// this should be changed when then backend sends an actor on the
// taskSummary message (we should use the actor provided by the message).
// https://github.com/osrf/rmf_core/issues/205
export const getActorFromStatus = (status: string): RegExpMatchArray | null => {
  // Gets the name of the robot if it has any
  // eslint-disable-next-line
  return status.match(/\[[A-Za-z]([a-zA-Z0-9\/]){3,}\]+/gi);
};

export const formatStatus = (status: string): string[] => {
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

export const sortTasksBySubmissionTime = (
  tasks: RomiCore.TaskSummary[],
): RomiCore.TaskSummary[] => {
  if (tasks.length === 0) return [];
  return tasks.sort((a, b) => (a.submission_time.nanosec < b.submission_time.nanosec ? 1 : -1));
};

/**
 * Classify and stores each task by its state.
 */
export const separateTasksByState = (
  tasks: Record<string, RomiCore.TaskSummary>,
  states: string[],
): Record<string, RomiCore.TaskSummary[]> => {
  const stateTasks: Record<string, RomiCore.TaskSummary[]> = {};
  states.forEach((state) => {
    stateTasks[state] = [];
  });

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
  return stateTasks;
};

/**
 * Sorts tasks by state and by submission time, so what is active is always at the top of the list.
 */
export const sortTasks = (tasks: Record<string, RomiCore.TaskSummary>): RomiCore.TaskSummary[] => {
  const states = ['active', 'queued', 'failed', 'completed', 'unknown'];

  const stateTasks = separateTasksByState(tasks, states);
  const sortedTasks: RomiCore.TaskSummary[] = [];

  states.forEach((state) => {
    sortedTasks.push(...sortTasksBySubmissionTime(stateTasks[state]));
  });
  return sortedTasks;
};

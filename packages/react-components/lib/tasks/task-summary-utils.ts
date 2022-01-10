// import type { TaskSummary } from 'api-client';
import { TaskSummary as RmfTaskSummary } from 'rmf-models';

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
    case RmfTaskSummary.STATE_QUEUED:
      return 'QUEUED';
    case RmfTaskSummary.STATE_ACTIVE:
      return 'ACTIVE';
    case RmfTaskSummary.STATE_COMPLETED:
      return 'COMPLETED';
    case RmfTaskSummary.STATE_FAILED:
      return 'FAILED';
    default:
      return 'UNKNOWN';
  }
};

export const sortTasksBySubmissionTime = (tasks: any[]): any[] => {
  if (tasks.length === 0) return [];
  return tasks.sort((a, b) => (a.submission_time.nanosec < b.submission_time.nanosec ? 1 : -1));
};

/**
 * Classifies and stores each task by its state.
 */
export const separateTasksByState = (
  tasks: Record<string, any>,
  states: string[],
): Record<string, any[]> => {
  const stateTasks: Record<string, any[]> = {};
  states.forEach((state) => {
    stateTasks[state] = [];
  });

  Object.keys(tasks).forEach((key) => {
    switch (tasks[key].state) {
      case RmfTaskSummary.STATE_QUEUED:
        stateTasks.queued.push(tasks[key]);
        break;
      case RmfTaskSummary.STATE_ACTIVE:
        stateTasks.active.push(tasks[key]);
        break;
      case RmfTaskSummary.STATE_COMPLETED:
        stateTasks.completed.push(tasks[key]);
        break;
      case RmfTaskSummary.STATE_FAILED:
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
 * Sort tasks by state and by submission time, so what is active is always at the top of the list.
 */
export const sortTasks = (tasks: Record<string, any>): any[] => {
  const states = ['active', 'queued', 'failed', 'completed', 'unknown'];

  const stateTasks = separateTasksByState(tasks, states);
  const sortedTasks: any[] = [];

  states.forEach((state) => {
    sortedTasks.push(...sortTasksBySubmissionTime(stateTasks[state]));
  });
  return sortedTasks;
};

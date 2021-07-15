import * as RmfModels from 'rmf-models';

export const sortTasks = (tasks: RmfModels.TaskSummary[]) => {
  let cancelledTask: RmfModels.TaskSummary[] = [];
  let activeTask: RmfModels.TaskSummary[] = [];
  let pendingTask: RmfModels.TaskSummary[] = [];
  let completedTask: RmfModels.TaskSummary[] = [];
  let failedTask: RmfModels.TaskSummary[] = [];
  let queuedTask: RmfModels.TaskSummary[] = [];

  tasks.forEach((task) => {
    switch (task.state) {
      case RmfModels.TaskSummary.STATE_CANCELED:
        cancelledTask.push(task);
        return;
      case RmfModels.TaskSummary.STATE_ACTIVE:
        activeTask.push(task);
        return;
      case RmfModels.TaskSummary.STATE_QUEUED:
        queuedTask.push(task);
        return;
      case RmfModels.TaskSummary.STATE_COMPLETED:
        completedTask.push(task);
        return;
      case RmfModels.TaskSummary.STATE_FAILED:
        failedTask.push(task);
        return;
      case RmfModels.TaskSummary.STATE_PENDING:
        pendingTask.push(task);
        return;
      default:
        return;
    }
  });

  return [
    ...activeTask,
    ...queuedTask,
    ...pendingTask,
    ...completedTask,
    ...cancelledTask,
    ...failedTask,
  ];
};

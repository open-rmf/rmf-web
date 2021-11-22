import { TaskSummary as RmfTaskSummary, TaskType as RmfTaskType } from 'rmf-models';

export function taskStateToStr(state: number): string {
  switch (state) {
    case RmfTaskSummary.STATE_ACTIVE:
      return 'Active';
    case RmfTaskSummary.STATE_CANCELED:
      return 'Cancelled';
    case RmfTaskSummary.STATE_COMPLETED:
      return 'Completed';
    case RmfTaskSummary.STATE_FAILED:
      return 'Failed';
    case RmfTaskSummary.STATE_PENDING:
      return 'Pending';
    case RmfTaskSummary.STATE_QUEUED:
      return 'Queued';
    default:
      return 'Unknown';
  }
}

export function taskTypeToStr(taskType: number): string {
  switch (taskType) {
    case RmfTaskType.TYPE_CHARGE_BATTERY:
      return 'Charge';
    case RmfTaskType.TYPE_CLEAN:
      return 'Clean';
    case RmfTaskType.TYPE_DELIVERY:
      return 'Delivery';
    case RmfTaskType.TYPE_LOOP:
      return 'Loop';
    case RmfTaskType.TYPE_PATROL:
      return 'Patrol';
    case RmfTaskType.TYPE_STATION:
      return 'Station';
    default:
      return 'Unknown';
  }
}

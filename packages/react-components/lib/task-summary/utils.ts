import * as RmfModels from 'rmf-models';

export function taskStateToStr(state: number): string {
  switch (state) {
    case RmfModels.TaskSummary.STATE_ACTIVE:
      return 'Active';
    case RmfModels.TaskSummary.STATE_CANCELED:
      return 'Cancelled';
    case RmfModels.TaskSummary.STATE_COMPLETED:
      return 'Completed';
    case RmfModels.TaskSummary.STATE_FAILED:
      return 'Failed';
    case RmfModels.TaskSummary.STATE_PENDING:
      return 'Pending';
    case RmfModels.TaskSummary.STATE_QUEUED:
      return 'Queued';
    default:
      return 'Unknown';
  }
}

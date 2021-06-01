import type { SubmitTask } from 'api-client';
import * as RmfModels from 'rmf-models';

export function makeSubmitTask(): SubmitTask {
  return {
    description: {
      cleaning_zone: 'zone',
    },
    start_time: Math.floor(Date.now() / 1000),
    task_type: RmfModels.TaskType.TYPE_CLEAN,
    priority: 0,
  };
}

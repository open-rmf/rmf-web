import { TaskType as RmfTaskType } from 'rmf-models';
import type { TaskState } from 'api-client';

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

function parsePhaseDetail(phases: TaskState['phases'], category?: string) {
  if (phases) {
    if (category === 'Loop') {
      const startPhase = phases['1'];
      const endPhase = phases['2'];
      const from = startPhase.category?.split('[place:')[1].split(']')[0];
      const to = endPhase.category?.split('[place:')[1].split(']')[0];
      return { to, from };
    }
  }
  return {};
}

export function parseTaskDetail(
  task: TaskState,
  category?: string,
): { to: string; from: string } | {} {
  if (category?.includes('Loop')) return parsePhaseDetail(task.phases, category);
  if (category?.includes('Delivery')) {
    const from = category?.split('[place:')[1].split(']')[0];
    const to = category?.split('[place:')[2].split(']')[0];
    return { to, from };
  } else {
    return {};
  }
}

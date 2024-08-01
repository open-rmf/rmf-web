import type { TaskRequest, TaskStateOutput as TaskState } from 'api-client';

import { TaskBookingLabels } from './booking-label';

export function serializeTaskBookingLabel(labels: TaskBookingLabels): string[] {
  return Object.entries(labels).map(([k, v]) => `${k}=${v}`);
}

export function parseTaskBookingLabels(taskTags: string[]): TaskBookingLabels {
  return Object.fromEntries(taskTags.map((s) => s.split('=', 2)));
}

export function getTaskBookingLabelFromTaskState(taskState: TaskState): TaskBookingLabels {
  return taskState.booking.labels ? parseTaskBookingLabels(taskState.booking.labels) : {};
}

export function getTaskBookingLabelFromTaskRequest(
  taskRequest: TaskRequest,
): TaskBookingLabels | null {
  return taskRequest.labels ? parseTaskBookingLabels(taskRequest.labels) : {};
}

export function getTaskDefinitionId(labels: TaskBookingLabels): string | null {
  return labels['task_definition_id'] || null;
}

import { ajv } from '../utils/schema-utils';
import schema from 'api-client/dist/schema';
import type { TaskBookingLabel, TaskState } from 'api-client';

const validateTaskBookingLabel = ajv.compile(schema.components.schemas.TaskBookingLabel);

export function serializeTaskBookingLabel(label: TaskBookingLabel): string {
  return JSON.stringify(label);
}

export function getTaskBookingLabelFromJsonString(
  jsonString: string,
): TaskBookingLabel | undefined {
  try {
    // Validate first before parsing again into the interface
    const validated = validateTaskBookingLabel(JSON.parse(jsonString));
    if (validated) {
      const parsedLabel: TaskBookingLabel = JSON.parse(jsonString);
      return parsedLabel;
    }
  } catch (e) {
    console.error(`Failed to parse TaskBookingLabel: ${(e as Error).message}`);
    return undefined;
  }

  console.error(`Failed to validate TaskBookingLabel`);
  return undefined;
}

export function getTaskBookingLabelFromTaskState(taskState: TaskState): TaskBookingLabel | null {
  let requestLabel: TaskBookingLabel | null = null;
  if (taskState.booking.labels) {
    for (const label of taskState.booking.labels) {
      try {
        const parsedLabel = getTaskBookingLabelFromJsonString(label);
        if (parsedLabel) {
          requestLabel = parsedLabel;
          break;
        }
      } catch (e) {
        continue;
      }
    }
  }
  return requestLabel;
}

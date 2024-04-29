import Ajv from 'ajv';
import schema from 'api-client/dist/schema';
import type { TaskState, TaskRequestLabel } from 'api-client';

// FIXME: AJV is duplicated here with dashboard, but this is only a temporary
// measure until we start using an external validation tool.
const ajv = new Ajv();

Object.entries(schema.components.schemas).forEach(([k, v]) => {
  ajv.addSchema(v, `#/components/schemas/${k}`);
});

const validateTaskRequestLabel = ajv.compile(schema.components.schemas.TaskRequestLabel);

export function serializeTaskRequestLabel(label: TaskRequestLabel): string {
  return JSON.stringify(label);
}

export function getTaskRequestLabelFromJsonString(
  jsonString: string,
): TaskRequestLabel | undefined {
  try {
    // Validate first before parsing again into the interface
    const validated = validateTaskRequestLabel(JSON.parse(jsonString));
    if (validated) {
      const parsedLabel: TaskRequestLabel = JSON.parse(jsonString);
      return parsedLabel;
    }
  } catch (e) {
    console.error(`Failed to parse TaskRequestLabel: ${(e as Error).message}`);
    return undefined;
  }

  console.error(`Failed to validate TaskRequestLabel`);
  return undefined;
}

export function getTaskRequestLabelFromTaskState(taskState: TaskState): TaskRequestLabel | null {
  let requestLabel: TaskRequestLabel | null = null;
  if (taskState.booking.labels) {
    for (const label of taskState.booking.labels) {
      try {
        const parsedLabel = getTaskRequestLabelFromJsonString(label);
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

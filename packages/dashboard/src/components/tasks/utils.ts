import { TaskRequest } from 'api-client';
import schema from 'api-client/dist/schema';
import { ajv } from '../utils';

/* istanbul ignore next */
export function parseTasksFile(contents: string): TaskRequest[] {
  const obj = JSON.parse(contents) as unknown[];
  if (!Array.isArray(obj)) {
    throw new Error('Expected an array of tasks');
  }

  const errIdx = obj.findIndex((req) => !ajv.validate(schema.components.schemas.TaskRequest, req));
  if (errIdx !== -1) {
    const errors = ajv.errors!;
    throw new Error(`Validation error on item ${errIdx + 1}: ${errors[0].message}`);
  }
  return obj;
}

import Ajv, { ValidateFunction } from 'ajv';
import { DispatchTaskRequest } from 'api-client';
import schema from 'api-client/openapi/schema';

let validate: ValidateFunction | null = null;

function getValidator() {
  if (validate) {
    return validate;
  }
  const ajv = new Ajv();
  validate = ajv.compile<DispatchTaskRequest>(schema.components.schemas.DispatchTaskRequest);
  return validate;
}

/* istanbul ignore next */
export function parseTasksFile(contents: string): DispatchTaskRequest[] {
  const obj = JSON.parse(contents) as unknown[];
  if (!Array.isArray(obj)) {
    throw new Error('Expected an array of tasks');
  }

  const validate = getValidator();
  const errIdx = obj.findIndex((req) => !validate(req));
  if (errIdx !== -1) {
    const errors = validate.errors!;
    throw new Error(`Validation error on item ${errIdx + 1}: ${errors[0].message}`);
  }
  return obj;
}

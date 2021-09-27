import { SubmitTask } from 'api-client';
import * as RmfModels from 'rmf-models';

/* istanbul ignore next */
function checkField(
  obj: Record<string, unknown>,
  field: string,
  type: string,
  errMsgs: string[],
): void {
  try {
    if (!Object.prototype.hasOwnProperty.call(obj, field) || typeof obj[field] !== type) {
      throw new TypeError(`expected [${field}] to be [${type}]`);
    }
  } catch (err) {
    errMsgs.push(err.message);
  }
}

export interface SubmitTaskDetails {
  submitTask: SubmitTask[];
  errors: { [index: string]: string[] };
}

// TODO: See if we can generate validators from the schema.
/* istanbul ignore next */
export function parseTasksFile(contents: string): SubmitTaskDetails {
  const tasks = JSON.parse(contents);
  let errMsgs: string[] = [];
  let res: SubmitTaskDetails = { submitTask: [], errors: {} };
  if (!Array.isArray(tasks)) {
    throw new TypeError('expected an array');
  }

  tasks.forEach((t, i) => {
    if (typeof t !== 'object') {
      throw new TypeError('expected object');
    }
    checkField(t, 'task_type', 'number', errMsgs);
    checkField(t, 'start_time', 'number', errMsgs);
    checkField(t, 'priority', 'number', errMsgs);
    checkField(t, 'description', 'object', errMsgs);
    const desc = t['description'];
    switch (t['task_type']) {
      case RmfModels.TaskType.TYPE_CLEAN:
        checkField(desc, 'cleaning_zone', 'string', errMsgs);
        break;
      case RmfModels.TaskType.TYPE_DELIVERY:
        checkField(desc, 'pickup_place_name', 'string', errMsgs);
        checkField(desc, 'pickup_dispenser', 'string', errMsgs);
        checkField(desc, 'dropoff_ingestor', 'string', errMsgs);
        checkField(desc, 'dropoff_place_name', 'string', errMsgs);
        break;
      case RmfModels.TaskType.TYPE_LOOP:
        checkField(desc, 'num_loops', 'number', errMsgs);
        checkField(desc, 'start_name', 'string', errMsgs);
        checkField(desc, 'finish_name', 'string', errMsgs);
        break;
      default:
        errMsgs.push('Unknown Task Type');
    }
    if (errMsgs.length > 0) res.errors[i] = [...errMsgs];
    errMsgs = [];
  });
  res.submitTask = tasks;
  return res;
}

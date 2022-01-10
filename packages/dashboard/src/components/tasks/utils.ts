// import { SubmitTask } from 'api-client';
import { TaskType as RmfTaskType } from 'rmf-models';

/**
 * TODO - handle submit task type changes once the backend connection is available
 */

const genericErrors = {
  checkArray: 'Expected an array of tasks',
  checkTaskObject: 'Expected task to be an object',
};

/* istanbul ignore next */
function checkField(
  obj: Record<string, unknown>,
  field: string,
  type: string,
  index: number,
): string | void {
  if (!Object.prototype.hasOwnProperty.call(obj, field) || typeof obj[field] !== type) {
    return `Task ${index + 1}: expected [${field}] to be [${type}]`;
  }
}

// TODO: See if we c>an generate validators from the schema.
/* istanbul ignore next */
export function parseTasksFile(contents: string): any[] {
  const errMsgs: (string | void)[] = [];
  const tasks = JSON.parse(contents);
  if (!Array.isArray(tasks)) {
    throw new TypeError(genericErrors.checkArray);
  }

  tasks.forEach((t, i) => {
    if (typeof t !== 'object') {
      throw new TypeError(genericErrors.checkTaskObject);
    }
    checkField(t, 'task_type', 'number', i) &&
      errMsgs.push(checkField(t, 'task_type', 'number', i));
    checkField(t, 'start_time', 'number', i) &&
      errMsgs.push(checkField(t, 'start_time', 'number', i));
    checkField(t, 'priority', 'number', i) && errMsgs.push(checkField(t, 'priority', 'number', i));
    checkField(t, 'description', 'object', i) &&
      errMsgs.push(checkField(t, 'description', 'object', i));
    const desc = t['description'];
    switch (t['task_type']) {
      case RmfTaskType.TYPE_CLEAN:
        checkField(desc, 'cleaning_zone', 'string', i) &&
          errMsgs.push(checkField(desc, 'cleaning_zone', 'string', i));
        break;
      case RmfTaskType.TYPE_DELIVERY:
        checkField(desc, 'pickup_place_name', 'string', i) &&
          errMsgs.push(checkField(desc, 'pickup_place_name', 'string', i));
        checkField(desc, 'pickup_dispenser', 'string', i) &&
          errMsgs.push(checkField(desc, 'pickup_dispenser', 'string', i));
        checkField(desc, 'dropoff_ingestor', 'string', i) &&
          errMsgs.push(checkField(desc, 'dropoff_ingestor', 'string', i));
        checkField(desc, 'dropoff_place_name', 'string', i) &&
          errMsgs.push(checkField(desc, 'dropoff_place_name', 'string', i));
        break;
      case RmfTaskType.TYPE_LOOP:
        checkField(desc, 'num_loops', 'number', i) &&
          errMsgs.push(checkField(desc, 'num_loops', 'number', i));
        checkField(desc, 'start_name', 'string', i) &&
          errMsgs.push(checkField(desc, 'start_name', 'string', i));
        checkField(desc, 'finish_name', 'string', i) &&
          errMsgs.push(checkField(desc, 'finish_name', 'string', i));
        break;
      default:
        errMsgs.push(`Task ${i + 1}: Unknown task type`);
    }
  });
  if (errMsgs.length > 0) throw new Error(errMsgs.join('\n'));
  return tasks;
}

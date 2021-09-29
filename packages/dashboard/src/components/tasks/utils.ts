import { SubmitTask } from 'api-client';
import * as RmfModels from 'rmf-models';

const genericErrors = {
  checkArray: 'Expected an array of tasks',
  checkTaskObject: 'Expected task to be an object',
  defaultError: 'Unknown task type',
};

// errors associated with a certain type of task
const customTaskFileErrors: Record<string, Record<string, string>> = {
  // task fields expected of all task types
  commonTaskFields: {
    task_type: 'Expected task_type field to be a number',
    start_time: 'Expected start_time field to be a number',
    priority: 'Expected priority field to be a number',
    description: 'Expected description field to be an object',
  },
  // error for clean task type
  cleanTask: {
    cleaning_zone: 'Cleaning Task - Expected cleaning_zone field to be a string',
  },
  // error for delivery task type
  deliveryTask: {
    pickup_place_name: 'Delivery Task - expected pickup_place_name to be a string',
    pickup_dispenser: 'Delivery Task - expected pickup_dispenser to be a string',
    dropoff_ingestor: 'Delivery Task - expected dropoff_ingestor to be a string',
    dropoff_place_name: 'Delivery Task - expected dropoff_place_name to be a string',
  },
  // error for loop task type
  loopTask: {
    num_loops: 'Loop Task - expected num_loops to be a number',
    start_name: 'Loop Task - expected start_name to be a string',
    finish_name: 'Loop Task - expected finish_name to be a string',
  },
};

/* istanbul ignore next */
function checkField(
  obj: Record<string, unknown>,
  field: string,
  type: string,
  taskType: 'commonTaskFields' | 'cleanTask' | 'deliveryTask' | 'loopTask',
): void {
  if (!Object.prototype.hasOwnProperty.call(obj, field) || typeof obj[field] !== type) {
    throw new TypeError(customTaskFileErrors[taskType][field]);
  }
}

// TODO: See if we can generate validators from the schema.
/* istanbul ignore next */
export function parseTasksFile(contents: string): SubmitTask[] {
  const tasks = JSON.parse(contents);
  if (!Array.isArray(tasks)) {
    throw new TypeError(genericErrors.checkArray);
  }

  tasks.forEach((t, i) => {
    if (typeof t !== 'object') {
      throw new TypeError(genericErrors.checkTaskObject);
    }
    checkField(t, 'task_type', 'number', 'commonTaskFields');
    checkField(t, 'start_time', 'number', 'commonTaskFields');
    checkField(t, 'priority', 'number', 'commonTaskFields');
    checkField(t, 'description', 'object', 'commonTaskFields');
    const desc = t['description'];
    switch (t['task_type']) {
      case RmfModels.TaskType.TYPE_CLEAN:
        checkField(desc, 'cleaning_zone', 'string', 'cleanTask');
        break;
      case RmfModels.TaskType.TYPE_DELIVERY:
        checkField(desc, 'pickup_place_name', 'string', 'deliveryTask');
        checkField(desc, 'pickup_dispenser', 'string', 'deliveryTask');
        checkField(desc, 'dropoff_ingestor', 'string', 'deliveryTask');
        checkField(desc, 'dropoff_place_name', 'string', 'deliveryTask');
        break;
      case RmfModels.TaskType.TYPE_LOOP:
        checkField(desc, 'num_loops', 'number', 'loopTask');
        checkField(desc, 'start_name', 'string', 'loopTask');
        checkField(desc, 'finish_name', 'string', 'loopTask');
        break;
      default:
        throw new TypeError(genericErrors.defaultError);
    }
  });
  return tasks;
}

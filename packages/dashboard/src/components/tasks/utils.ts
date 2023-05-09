import { TaskRequest, TaskState } from 'api-client';
import schema from 'api-client/dist/schema';
import { ajv } from '../utils';

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

export function downloadCsvFull(timestamp: Date, allTasks: TaskState[]) {
  const columnSeparator = ';';
  const rowSeparator = '\n';
  const keys = Object.keys(allTasks[0]);
  let csvContent = keys.join(columnSeparator) + rowSeparator;
  allTasks.forEach((task) => {
    keys.forEach((k) => {
      type TaskStateKey = keyof typeof task;
      const columnKey = k as TaskStateKey;
      const value =
        task[columnKey] === null || task[columnKey] === undefined
          ? ''
          : JSON.stringify(task[columnKey]);
      csvContent += value + columnSeparator;
    });
    csvContent += rowSeparator;
  });
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `${timestamp.toJSON().slice(0, 10)}_full_task_history.csv`;
  a.click();

  setTimeout(() => {
    URL.revokeObjectURL(url);
  });
}

export function downloadCsvMinimal(timestamp: Date, allTasks: TaskState[]) {
  const columnSeparator = ';';
  const rowSeparator = '\n';
  const keys = [
    'Date',
    'Requester',
    'ID',
    'Category',
    'Assignee',
    'Start Time',
    'End Time',
    'State',
  ];
  let csvContent = keys.join(columnSeparator) + rowSeparator;
  allTasks.forEach((task) => {
    const values = [
      task.booking.unix_millis_request_time
        ? `${new Date(task.booking.unix_millis_request_time).toLocaleDateString()}`
        : 'unknown',
      task.booking.requester ? task.booking.requester : 'unknown',
      task.booking.id,
      task.category ? task.category : 'unknown',
      task.assigned_to ? task.assigned_to.name : 'unknown',
      task.unix_millis_start_time
        ? `${new Date(task.unix_millis_start_time).toLocaleDateString()} ${new Date(
            task.unix_millis_start_time,
          ).toLocaleTimeString()}`
        : 'unknown',
      task.unix_millis_finish_time
        ? `${new Date(task.unix_millis_finish_time).toLocaleDateString()} ${new Date(
            task.unix_millis_finish_time,
          ).toLocaleTimeString()}`
        : 'unknown',
      task.status ? task.status : 'unknown',
    ];
    csvContent += values.join(columnSeparator) + rowSeparator;
  });
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `${timestamp.toJSON().slice(0, 10)}_minimal_task_history.csv`;
  a.click();

  setTimeout(() => {
    URL.revokeObjectURL(url);
  });
}

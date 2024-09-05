import {
  AddExceptDateRequest,
  PostScheduledTaskRequest,
  RobotTaskRequest,
  TaskRequest,
  TaskStateOutput as TaskState,
} from 'api-client';
import { getTaskBookingLabelFromTaskState, RobotDispatchTarget, Schedule } from 'react-components';

import { RmfApi } from '../../services/rmf-api';
import { toISOStringWithTimezone } from './task-schedule-utils';

export function exportCsvFull(timestamp: Date, allTasks: TaskState[]) {
  const columnSeparator = ';';
  const rowSeparator = '\n';
  let csvContent = `sep=${columnSeparator}` + rowSeparator;
  const keys = Object.keys(allTasks[0]);
  csvContent += keys.join(columnSeparator) + rowSeparator;
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

export function exportCsvMinimal(timestamp: Date, allTasks: TaskState[]) {
  const columnSeparator = ';';
  const rowSeparator = '\n';
  let csvContent = `sep=${columnSeparator}` + rowSeparator;
  const keys = [
    'Date',
    'Requester',
    'Pickup',
    'Destination',
    'Robot',
    'Start Time',
    'End Time',
    'State',
  ];
  csvContent += keys.join(columnSeparator) + rowSeparator;
  allTasks.forEach((task) => {
    const taskBookingLabels = getTaskBookingLabelFromTaskState(task);

    const values = [
      // Date
      task.booking.unix_millis_request_time
        ? `${new Date(task.booking.unix_millis_request_time).toLocaleDateString()}`
        : 'n/a',
      // Requester
      task.booking.requester ? task.booking.requester : 'n/a',
      // Pickup
      taskBookingLabels && 'pickup' in taskBookingLabels ? taskBookingLabels['pickup'] : 'n/a',
      // Destination
      taskBookingLabels && 'destination' in taskBookingLabels
        ? taskBookingLabels['destination']
        : 'n/a',
      // Robot
      task.assigned_to ? task.assigned_to.name : 'n/a',
      // Start Time
      task.unix_millis_start_time
        ? `${new Date(task.unix_millis_start_time).toLocaleDateString()} ${new Date(
            task.unix_millis_start_time,
          ).toLocaleTimeString()}`
        : 'n/a',
      // End Time
      task.unix_millis_finish_time
        ? `${new Date(task.unix_millis_finish_time).toLocaleDateString()} ${new Date(
            task.unix_millis_finish_time,
          ).toLocaleTimeString()}`
        : 'n/a',
      // State
      task.status ? task.status : 'n/a',
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

export const toApiSchedule = (
  taskRequest: TaskRequest,
  schedule: Schedule,
): PostScheduledTaskRequest => {
  const start = schedule.startOn;
  const apiSchedules: PostScheduledTaskRequest['schedules'] = [];
  const date = new Date(start);
  const hours = date.getHours().toString().padStart(2, '0');
  const minutes = date.getMinutes().toString().padStart(2, '0');
  const at = `${hours}:${minutes}`;
  schedule.days[0] && apiSchedules.push({ period: 'monday', at });
  schedule.days[1] && apiSchedules.push({ period: 'tuesday', at });
  schedule.days[2] && apiSchedules.push({ period: 'wednesday', at });
  schedule.days[3] && apiSchedules.push({ period: 'thursday', at });
  schedule.days[4] && apiSchedules.push({ period: 'friday', at });
  schedule.days[5] && apiSchedules.push({ period: 'saturday', at });
  schedule.days[6] && apiSchedules.push({ period: 'sunday', at });
  return {
    task_request: taskRequest,
    schedules: apiSchedules,
  };
};

export async function dispatchTask(
  rmf: RmfApi,
  taskRequest: TaskRequest,
  robotDispatchTarget: RobotDispatchTarget | null,
) {
  if (robotDispatchTarget) {
    const robotTask: RobotTaskRequest = {
      type: 'robot_task_request',
      robot: robotDispatchTarget.robot,
      fleet: robotDispatchTarget.fleet,
      request: taskRequest,
    };
    console.debug(`dispatch robot task: ${robotTask}`);
    await rmf.tasksApi.postRobotTaskTasksRobotTaskPost(robotTask);
    return;
  }

  console.debug(`dispatch task: ${taskRequest}`);
  await rmf.tasksApi.postDispatchTaskTasksDispatchTaskPost({
    type: 'dispatch_task_request',
    request: taskRequest,
  });
}

export async function scheduleTask(rmf: RmfApi, taskRequest: TaskRequest, schedule: Schedule) {
  console.debug(`schedule task: ${taskRequest}\nschedule: ${schedule}`);
  const scheduleRequest = toApiSchedule(taskRequest, schedule);
  await rmf.tasksApi.postScheduledTaskScheduledTasksPost(scheduleRequest);
}

export async function editScheduledTaskEvent(
  rmf: RmfApi,
  taskRequest: TaskRequest,
  newEventSchedule: Schedule,
  newEventDate: Date,
  originalScheduleTaskId: number,
) {
  const addExceptDateRequest: AddExceptDateRequest = {
    except_date: toISOStringWithTimezone(newEventDate),
  };
  console.debug(
    `Adding [${addExceptDateRequest.except_date}] to except date of scheduled task [${originalScheduleTaskId}]`,
  );
  await rmf.tasksApi.addExceptDateScheduledTasksTaskIdExceptDatePost(
    originalScheduleTaskId,
    addExceptDateRequest,
  );

  console.debug(
    `creating new schedule for edited event: ${taskRequest}\nschedule: ${newEventSchedule}`,
  );
  const newScheduleRequest = toApiSchedule(taskRequest, newEventSchedule);
  await rmf.tasksApi.postScheduledTaskScheduledTasksPost(newScheduleRequest);
}

export async function editScheduledTaskSchedule(
  rmf: RmfApi,
  taskRequest: TaskRequest,
  newSchedule: Schedule,
  scheduleTaskId: number,
) {
  const scheduleRequest = toApiSchedule(taskRequest, newSchedule);

  await rmf.tasksApi.updateScheduleTaskScheduledTasksTaskIdUpdatePost(
    scheduleTaskId,
    scheduleRequest,
  );
}

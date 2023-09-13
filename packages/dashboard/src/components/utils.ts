import Ajv from 'ajv';
import { PostScheduledTaskRequest, TaskRequest } from 'api-client';
import schema from 'api-client/schema';
import { AxiosError } from 'axios';
import { Schedule } from 'react-components';

export function getApiErrorMessage(error: unknown): string {
  return (error as AxiosError).response?.data.detail || '';
}

export const ajv = new Ajv();

Object.entries(schema.components.schemas).forEach(([k, v]) => {
  ajv.addSchema(v, `#/components/schemas/${k}`);
});

export const toApiSchedule = (
  taskRequest: TaskRequest,
  schedule: Schedule,
): PostScheduledTaskRequest => {
  const start = schedule.startOn;
  const apiSchedules: PostScheduledTaskRequest['schedules'] = [];
  const date = new Date(start);
  const start_from = start.toISOString();
  const until = schedule.until?.toISOString();
  const hours = date.getHours().toString().padStart(2, '0');
  const minutes = date.getMinutes().toString().padStart(2, '0');
  const at = `${hours}:${minutes}`;
  schedule.days[0] && apiSchedules.push({ period: 'monday', start_from, at, until });
  schedule.days[1] && apiSchedules.push({ period: 'tuesday', start_from, at, until });
  schedule.days[2] && apiSchedules.push({ period: 'wednesday', start_from, at, until });
  schedule.days[3] && apiSchedules.push({ period: 'thursday', start_from, at, until });
  schedule.days[4] && apiSchedules.push({ period: 'friday', start_from, at, until });
  schedule.days[5] && apiSchedules.push({ period: 'saturday', start_from, at, until });
  schedule.days[6] && apiSchedules.push({ period: 'sunday', start_from, at, until });
  return {
    task_request: taskRequest,
    schedules: apiSchedules,
  };
};

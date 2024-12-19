import { PostScheduledTaskRequest, TaskRequest } from 'api-client';
import { describe, expect, it, vi } from 'vitest';

import { MockRmfApi } from '../../utils/test-utils.test';
import { Schedule } from './task-form';
import { makeTaskRequest } from './test-data.test';
import {
  createTaskPriority,
  dispatchTask,
  editScheduledTaskEvent,
  editScheduledTaskSchedule,
  parseTaskPriority,
  scheduleTask,
  toApiSchedule,
} from './utils';

describe('toApiSchedule', () => {
  it('should convert a Schedule to a PostScheduledTaskRequest with correct schedules', () => {
    const taskRequest: TaskRequest = {
      category: 'test_category',
      description: {},
    };
    const schedule: Schedule = {
      startOn: new Date('2023-10-27T10:30:00'),
      days: [true, false, true, false, true, false, false], // Monday, Wednesday, Friday
      until: undefined,
      at: new Date(),
    };
    const expectedApiSchedules: PostScheduledTaskRequest['schedules'] = [
      { period: 'monday', at: '10:30' },
      { period: 'wednesday', at: '10:30' },
      { period: 'friday', at: '10:30' },
    ];
    const expected: PostScheduledTaskRequest = {
      task_request: taskRequest,
      schedules: expectedApiSchedules,
    };

    const result = toApiSchedule(taskRequest, schedule);
    expect(result).toEqual(expected);
  });

  it('should handle an empty days array', () => {
    const taskRequest: TaskRequest = {
      category: 'test_category',
      description: {},
    };
    const schedule: Schedule = {
      startOn: new Date('2023-10-27T10:30:00'),
      days: [false, false, false, false, false, false, false],
      until: undefined,
      at: new Date(),
    };
    const expected: PostScheduledTaskRequest = {
      task_request: taskRequest,
      schedules: [],
    };

    const result = toApiSchedule(taskRequest, schedule);
    expect(result).toEqual(expected);
  });

  it('should format the time correctly', () => {
    const taskRequest: TaskRequest = {
      category: 'test_category',
      description: {},
    };
    const schedule: Schedule = {
      startOn: new Date('2023-10-27T05:05:00'),
      days: [true, false, false, false, false, false, false], // Monday
      until: undefined,
      at: new Date(),
    };
    const expectedApiSchedules: PostScheduledTaskRequest['schedules'] = [
      { period: 'monday', at: '05:05' },
    ];
    const expected: PostScheduledTaskRequest = {
      task_request: taskRequest,
      schedules: expectedApiSchedules,
    };

    const result = toApiSchedule(taskRequest, schedule);
    expect(result).toEqual(expected);
  });
});

describe('dispatchTask', () => {
  const rmfApi = new MockRmfApi();
  rmfApi.tasksApi.postRobotTaskTasksRobotTaskPost = vi.fn().mockResolvedValue({});
  rmfApi.tasksApi.postDispatchTaskTasksDispatchTaskPost = vi.fn().mockResolvedValue({});

  it('dispatch task', async () => {
    await dispatchTask(rmfApi, makeTaskRequest(), null);
    expect(rmfApi.tasksApi.postDispatchTaskTasksDispatchTaskPost).toHaveBeenCalledOnce();
  });

  it('dispatch direct task', async () => {
    await dispatchTask(rmfApi, makeTaskRequest(), { fleet: 'test_fleet', robot: 'test_robot' });
    expect(rmfApi.tasksApi.postRobotTaskTasksRobotTaskPost).toHaveBeenCalledOnce();
  });
});

describe('scheduleTask', () => {
  const rmfApi = new MockRmfApi();
  rmfApi.tasksApi.postScheduledTaskScheduledTasksPost = vi.fn().mockResolvedValue({});

  it('schedule task', async () => {
    const schedule: Schedule = {
      startOn: new Date('2023-10-27T10:30:00'),
      days: [false, false, false, false, false, false, false],
      until: undefined,
      at: new Date(),
    };
    await scheduleTask(rmfApi, makeTaskRequest(), schedule);
    expect(rmfApi.tasksApi.postScheduledTaskScheduledTasksPost).toHaveBeenCalledOnce();
  });
});

describe('editScheduledTaskEvent', () => {
  const rmfApi = new MockRmfApi();
  rmfApi.tasksApi.addExceptDateScheduledTasksTaskIdExceptDatePost = vi.fn().mockResolvedValue({});
  rmfApi.tasksApi.postScheduledTaskScheduledTasksPost = vi.fn().mockResolvedValue({});

  it('edit scheduled task event', async () => {
    const schedule: Schedule = {
      startOn: new Date('2023-10-27T10:30:00'),
      days: [true, true, true, true, true, true, true],
      until: undefined,
      at: new Date(),
    };
    await editScheduledTaskEvent(
      rmfApi,
      makeTaskRequest(),
      schedule,
      new Date('2023-10-28T10:30:00'),
      10,
    );
    expect(rmfApi.tasksApi.addExceptDateScheduledTasksTaskIdExceptDatePost).toHaveBeenCalledOnce();
    expect(rmfApi.tasksApi.postScheduledTaskScheduledTasksPost).toHaveBeenCalledOnce();
  });
});

describe('editScheduledTaskSchedule', () => {
  const rmfApi = new MockRmfApi();
  rmfApi.tasksApi.updateScheduleTaskScheduledTasksTaskIdUpdatePost = vi.fn().mockResolvedValue({});

  it('edit scheduled task event', async () => {
    const schedule: Schedule = {
      startOn: new Date('2023-10-27T10:30:00'),
      days: [true, true, true, true, true, true, true],
      until: undefined,
      at: new Date(),
    };
    await editScheduledTaskSchedule(rmfApi, makeTaskRequest(), schedule, 10);
    expect(rmfApi.tasksApi.updateScheduleTaskScheduledTasksTaskIdUpdatePost).toHaveBeenCalledOnce();
  });
});

describe('createTaskPriority', () => {
  it('create task priority', () => {
    expect(createTaskPriority(true)).toStrictEqual({ type: 'binary', value: 1 });
    expect(createTaskPriority(false)).toStrictEqual({ type: 'binary', value: 0 });
  });
});

describe('parseTaskPriority', () => {
  it('parse task priority', () => {
    expect(parseTaskPriority(null)).toBe(false);
    expect(parseTaskPriority(undefined)).toBe(false);
    expect(parseTaskPriority({ type: 'binary', value: 0 })).toBe(false);
    expect(parseTaskPriority({ type: 'binary', value: 1 })).toBe(true);
  });
});

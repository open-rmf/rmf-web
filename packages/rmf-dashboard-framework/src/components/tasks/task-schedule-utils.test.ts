import { Period, ScheduledTask, ScheduledTaskScheduleOutput } from 'api-client';
import { addMinutes, endOfDay, endOfMinute, startOfDay } from 'date-fns';
import { describe, expect, it } from 'vitest';

import { RecurringDays } from './task-form';
import {
  apiScheduleToSchedule,
  scheduleToEvents,
  scheduleWithSelectedDay,
} from './task-schedule-utils';
import { makeTaskRequest } from './test-data.test';

describe('scheduleToEvents', () => {
  const getEventId = () => 1;
  const getEventTitle = () => 'Test Event';
  const getEventColor = () => '#ff0000';

  const defaultTask: ScheduledTask = {
    id: 1,
    task_request: makeTaskRequest(),
    created_by: 'user',
    schedules: [],
    last_ran: null,
    start_from: null,
    until: null,
    except_dates: [],
  };

  it('should return an empty array if schedule.at is missing', () => {
    const start = startOfDay(new Date(2023, 0, 1));
    const end = endOfDay(new Date(2023, 0, 7));
    const schedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'day',
      at: '',
    };
    const events = scheduleToEvents(
      start,
      end,
      schedule,
      defaultTask,
      getEventId,
      getEventTitle,
      getEventColor,
    );
    expect(events).toEqual([]);
  });

  it('should return an empty array if schedule.period is invalid', () => {
    const start = startOfDay(new Date(2023, 0, 1));
    const end = endOfDay(new Date(2023, 0, 7));
    const schedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'invalid' as Period,
      at: '09:00',
    };
    const events = scheduleToEvents(
      start,
      end,
      schedule,
      defaultTask,
      getEventId,
      getEventTitle,
      getEventColor,
    );
    expect(events).toEqual([]);
  });

  it('should generate events for a daily schedule', () => {
    const start = startOfDay(new Date(2023, 0, 1));
    const end = endOfDay(new Date(2023, 0, 7));
    const schedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'day',
      at: '09:00',
    };
    const events = scheduleToEvents(
      start,
      end,
      schedule,
      defaultTask,
      getEventId,
      getEventTitle,
      getEventColor,
    );
    expect(events.length).toBe(7);
    expect(events[0].start).toEqual(new Date(2023, 0, 1, 9, 0));
    expect(events[0].end).toEqual(addMinutes(new Date(2023, 0, 1, 9, 0), 45));
    expect(events[6].start).toEqual(new Date(2023, 0, 7, 9, 0));
    expect(events[6].end).toEqual(addMinutes(new Date(2023, 0, 7, 9, 0), 45));
  });

  it('should generate events for a weekly schedule (Monday)', () => {
    const start = startOfDay(new Date(2023, 0, 1)); // Sunday
    const end = endOfDay(new Date(2023, 0, 15));
    const schedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'monday',
      at: '10:00',
    };
    const events = scheduleToEvents(
      start,
      end,
      schedule,
      defaultTask,
      getEventId,
      getEventTitle,
      getEventColor,
    );
    expect(events.length).toBe(2);
    expect(events[0].start).toEqual(new Date(2023, 0, 2, 10, 0));
    expect(events[1].start).toEqual(new Date(2023, 0, 9, 10, 0));
  });

  it('should generate events for a weekly schedule (Wednesday)', () => {
    const start = startOfDay(new Date(2023, 0, 1)); // Sunday
    const end = endOfDay(new Date(2023, 0, 15));
    const schedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'wednesday',
      at: '10:00',
    };
    const events = scheduleToEvents(
      start,
      end,
      schedule,
      defaultTask,
      getEventId,
      getEventTitle,
      getEventColor,
    );
    expect(events.length).toBe(2);
    expect(events[0].start).toEqual(new Date(2023, 0, 4, 10, 0));
    expect(events[1].start).toEqual(new Date(2023, 0, 11, 10, 0));
  });

  it('should respect start_from', () => {
    const start = startOfDay(new Date(2023, 0, 1));
    const end = endOfDay(new Date(2023, 0, 7));
    const schedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'day',
      at: '09:00',
    };
    const task: ScheduledTask = {
      ...defaultTask,
      start_from: '2023-01-03T00:00:00',
    };
    const events = scheduleToEvents(
      start,
      end,
      schedule,
      task,
      getEventId,
      getEventTitle,
      getEventColor,
    );
    expect(events.length).toBe(5);
    expect(events[0].start).toEqual(new Date(2023, 0, 3, 9, 0));
  });

  it('should respect until', () => {
    const start = startOfDay(new Date(2023, 0, 1));
    const end = endOfDay(new Date(2023, 0, 7));
    const schedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'day',
      at: '09:00',
    };
    const task: ScheduledTask = {
      ...defaultTask,
      until: '2023-01-05T23:59:59',
    };
    const events = scheduleToEvents(
      start,
      end,
      schedule,
      task,
      getEventId,
      getEventTitle,
      getEventColor,
    );
    expect(events.length).toBe(5);
    expect(events[4].start).toEqual(new Date(2023, 0, 5, 9, 0));
  });

  it('should respect except_dates', () => {
    const start = startOfDay(new Date(2023, 0, 1));
    const end = endOfDay(new Date(2023, 0, 7));
    const schedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'day',
      at: '09:00',
    };
    const task: ScheduledTask = {
      ...defaultTask,
      except_dates: ['2023-01-03', '2023-01-05'],
    };
    const events = scheduleToEvents(
      start,
      end,
      schedule,
      task,
      getEventId,
      getEventTitle,
      getEventColor,
    );
    expect(events.length).toBe(5);
    expect(events[1].start).toEqual(new Date(2023, 0, 2, 9, 0));
    expect(events[2].start).toEqual(new Date(2023, 0, 4, 9, 0));
    expect(events[3].start).toEqual(new Date(2023, 0, 6, 9, 0));
  });
});

describe('scheduleWithSelectedDay', () => {
  it('should create a schedule for Sunday', () => {
    const date = new Date(2023, 0, 1); // Sunday
    const scheduleTask: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'day',
        at: '09:00',
      },
    ];
    const expectedDays: RecurringDays = [false, false, false, false, false, false, true];
    const result = scheduleWithSelectedDay(scheduleTask, date);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(date);
    expect(result.until).toEqual(endOfDay(date));
    expect(result.at).toEqual(new Date('09:00'));
  });

  it('should create a schedule for Monday', () => {
    const date = new Date(2023, 0, 2); // Monday
    const scheduleTask: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'day',
        at: '10:00',
      },
    ];
    const expectedDays: RecurringDays = [true, false, false, false, false, false, false];
    const result = scheduleWithSelectedDay(scheduleTask, date);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(date);
    expect(result.until).toEqual(endOfDay(date));
    expect(result.at).toEqual(new Date('10:00'));
  });

  it('should create a schedule for Tuesday', () => {
    const date = new Date(2023, 0, 3); // Tuesday
    const scheduleTask: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'day',
        at: '11:00',
      },
    ];
    const expectedDays: RecurringDays = [false, true, false, false, false, false, false];
    const result = scheduleWithSelectedDay(scheduleTask, date);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(date);
    expect(result.until).toEqual(endOfDay(date));
    expect(result.at).toEqual(new Date('11:00'));
  });

  it('should create a schedule for Wednesday', () => {
    const date = new Date(2023, 0, 4); // Wednesday
    const scheduleTask: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'day',
        at: '12:00',
      },
    ];
    const expectedDays: RecurringDays = [false, false, true, false, false, false, false];
    const result = scheduleWithSelectedDay(scheduleTask, date);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(date);
    expect(result.until).toEqual(endOfDay(date));
    expect(result.at).toEqual(new Date('12:00'));
  });

  it('should create a schedule for Thursday', () => {
    const date = new Date(2023, 0, 5); // Thursday
    const scheduleTask: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'day',
        at: '13:00',
      },
    ];
    const expectedDays: RecurringDays = [false, false, false, true, false, false, false];
    const result = scheduleWithSelectedDay(scheduleTask, date);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(date);
    expect(result.until).toEqual(endOfDay(date));
    expect(result.at).toEqual(new Date('13:00'));
  });

  it('should create a schedule for Friday', () => {
    const date = new Date(2023, 0, 6); // Friday
    const scheduleTask: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'day',
        at: '14:00',
      },
    ];
    const expectedDays: RecurringDays = [false, false, false, false, true, false, false];
    const result = scheduleWithSelectedDay(scheduleTask, date);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(date);
    expect(result.until).toEqual(endOfDay(date));
    expect(result.at).toEqual(new Date('14:00'));
  });

  it('should create a schedule for Saturday', () => {
    const date = new Date(2023, 0, 7); // Saturday
    const scheduleTask: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'day',
        at: '15:00',
      },
    ];
    const expectedDays: RecurringDays = [false, false, false, false, false, true, false];
    const result = scheduleWithSelectedDay(scheduleTask, date);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(date);
    expect(result.until).toEqual(endOfDay(date));
    expect(result.at).toEqual(new Date('15:00'));
  });

  it('should use the current date if scheduleTask[0].at is undefined', () => {
    const date = new Date(2023, 0, 1); // Sunday
    const scheduleTask: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'day',
        at: '',
      },
    ];
    const result = scheduleWithSelectedDay(scheduleTask, date);
    expect(result.at.setMilliseconds(0)).toEqual(new Date().setMilliseconds(0));
  });
});

describe('apiScheduleToSchedule', () => {
  it('should convert a single day schedule', () => {
    const apiSchedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'monday',
      at: '09:00',
    };
    const task: ScheduledTask = {
      id: 1,
      task_request: makeTaskRequest(),
      created_by: 'user',
      schedules: [apiSchedule],
      last_ran: null,
      start_from: '2023-01-02T00:00:00',
      until: '2023-01-08T23:59:59',
      except_dates: [],
    };
    const expectedDays: RecurringDays = [true, false, false, false, false, false, false];
    const result = apiScheduleToSchedule(task);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(new Date('2023-01-02T00:00:00'));
    expect(result.until).toEqual(endOfMinute(new Date('2023-01-08T23:59:59')));
    expect(result.at).toEqual(new Date('09:00'));
  });

  it('should convert a multiple day schedule', () => {
    const apiSchedules: ScheduledTaskScheduleOutput[] = [
      {
        every: null,
        period: 'tuesday',
        at: '10:00',
      },
      {
        every: null,
        period: 'friday',
        at: '12:00',
      },
    ];
    const task: ScheduledTask = {
      id: 1,
      task_request: makeTaskRequest(),
      created_by: 'user',
      schedules: apiSchedules,
      last_ran: null,
      start_from: '2023-01-03T00:00:00',
      until: '2023-01-09T23:59:59',
      except_dates: [],
    };
    const expectedDays: RecurringDays = [false, true, false, false, true, false, false];
    const result = apiScheduleToSchedule(task);
    expect(result.days).toEqual(expectedDays);
    expect(result.startOn).toEqual(new Date('2023-01-03T00:00:00'));
    expect(result.until).toEqual(endOfMinute(new Date('2023-01-09T23:59:59')));
    expect(result.at).toEqual(new Date('10:00')); // Should take the time from the first schedule
  });

  it('should throw an error for an invalid day', () => {
    const apiSchedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'invalid' as Period,
      at: '09:00',
    };
    const task: ScheduledTask = {
      id: 1,
      task_request: makeTaskRequest(),
      created_by: 'user',
      schedules: [apiSchedule],
      last_ran: null,
      start_from: null,
      until: null,
      except_dates: [],
    };
    expect(() => apiScheduleToSchedule(task)).toThrowError(`Invalid day: ${apiSchedule}`);
  });

  it('should use current date if start_from is not provided', () => {
    const apiSchedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'monday',
      at: '09:00',
    };
    const task: ScheduledTask = {
      id: 1,
      task_request: makeTaskRequest(),
      created_by: 'user',
      schedules: [apiSchedule],
      last_ran: null,
      start_from: null,
      until: null,
      except_dates: [],
    };
    const result = apiScheduleToSchedule(task);
    expect(result.startOn).toEqual(new Date());
  });

  it('should use undefined for until if not provided', () => {
    const apiSchedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'monday',
      at: '09:00',
    };
    const task: ScheduledTask = {
      id: 1,
      task_request: makeTaskRequest(),
      created_by: 'user',
      schedules: [apiSchedule],
      last_ran: null,
      start_from: null,
      until: null,
      except_dates: [],
    };
    const result = apiScheduleToSchedule(task);
    expect(result.until).toBeUndefined();
  });

  it('should use the current date if schedules[0].at is not provided', () => {
    const apiSchedule: ScheduledTaskScheduleOutput = {
      every: null,
      period: 'monday',
      at: '',
    };
    const task: ScheduledTask = {
      id: 1,
      task_request: makeTaskRequest(),
      created_by: 'user',
      schedules: [apiSchedule],
      last_ran: null,
      start_from: null,
      until: null,
      except_dates: [],
    };
    const result = apiScheduleToSchedule(task);
    expect(result.at).toEqual(new Date());
  });
});

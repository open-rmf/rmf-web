import { ProcessedEvent } from '@aldabil/react-scheduler/types';
import { ScheduledTask, ScheduledTaskSchedule as ApiSchedule } from 'api-client';
import {
  addMinutes,
  endOfDay,
  endOfMinute,
  isFriday,
  isMonday,
  isSaturday,
  isSunday,
  isThursday,
  isTuesday,
  isWednesday,
  nextFriday,
  nextMonday,
  nextSaturday,
  nextSunday,
  nextThursday,
  nextTuesday,
  nextWednesday,
  startOfMinute,
} from 'date-fns';
import { getShortDescription, RecurringDays, Schedule } from 'react-components';

/**
 * Generates a list of ProcessedEvents to occur within the query start and end,
 * based on the provided schedule.
 * @param start The start of the query, which is generally 00:00:00 of the first
 * day in the calendar view.
 * @param end The end of the query, which is generally 23:59:59 of the last day
 * in the calendar view.
 * @param schedule The current schedule, to be checked if there are any events
 * between start and end.
 * @param getEventId Callback function to get the event ID.
 * @param getEventTitle Callback function to get the event title.
 * @returns List of ProcessedEvents to occur within the query start and end.
 */
export const scheduleToEvents = (
  start: Date,
  end: Date,
  schedule: ApiSchedule,
  task: ScheduledTask,
  getEventId: () => number,
  getEventTitle: () => string,
): ProcessedEvent[] => {
  if (!schedule.at) {
    console.warn('Unable to convert schedule without [at] to an event');
    return [];
  }
  const [hours, minutes] = schedule.at.split(':').map((s: string) => Number(s));
  let cur = new Date(start);
  cur.setHours(hours);
  cur.setMinutes(minutes);

  const scheStartFrom = schedule.start_from ? startOfMinute(new Date(schedule.start_from)) : null;
  const scheUntil = schedule.until ? endOfMinute(new Date(schedule.until)) : null;

  let period = 8.64e7; // 1 day
  switch (schedule.period) {
    case 'day':
      break;
    case 'monday':
      cur = isMonday(cur) ? cur : nextMonday(cur);
      period *= 7;
      break;
    case 'tuesday':
      cur = isTuesday(cur) ? cur : nextTuesday(cur);
      period *= 7;
      break;
    case 'wednesday':
      cur = isWednesday(cur) ? cur : nextWednesday(cur);
      period *= 7;
      break;
    case 'thursday':
      cur = isThursday(cur) ? cur : nextThursday(cur);
      period *= 7;
      break;
    case 'friday':
      cur = isFriday(cur) ? cur : nextFriday(cur);
      period *= 7;
      break;
    case 'saturday':
      cur = isSaturday(cur) ? cur : nextSaturday(cur);
      period *= 7;
      break;
    case 'sunday':
      cur = isSunday(cur) ? cur : nextSunday(cur);
      period *= 7;
      break;
    default:
      console.warn(`Unable to convert schedule with period [${schedule.period}] to events`);
      return [];
  }

  const events: ProcessedEvent[] = [];
  while (cur <= end) {
    if (
      (scheStartFrom == null || scheStartFrom <= cur) &&
      (scheUntil == null || scheUntil >= cur)
    ) {
      const curToIso = cur.toISOString();
      const curFormatted = `${curToIso.slice(0, 10)}`;
      if (!task.except_dates?.includes(curFormatted)) {
        events.push({
          start: cur,
          end: addMinutes(cur, 45),
          event_id: getEventId(),
          title: getEventTitle(),
        });
      }
    }

    cur = new Date(cur.valueOf() + period);
  }
  return events;
};

export const scheduleWithSelectedDay = (scheduleTask: ApiSchedule[], date: Date): Schedule => {
  const daysArray: RecurringDays = [false, false, false, false, false, false, false];

  const dayIndex = date.getDay();
  // Calculate the adjusted index to match React Scheduler (1 = Monday, ..., 7 = Sunday)
  const adjustedIndex = dayIndex === 0 ? 7 : dayIndex;

  daysArray[adjustedIndex - 1] = true;

  return {
    startOn: scheduleTask[0].start_from ? new Date(scheduleTask[0].start_from) : new Date(),
    days: daysArray,
    until: endOfDay(new Date(date.toISOString())),
    at: scheduleTask[0].start_from ? new Date(scheduleTask[0].start_from) : new Date(),
  };
};

export const apiScheduleToSchedule = (scheduleTask: ApiSchedule[]): Schedule => {
  const daysOfWeek = ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday'];

  const daysArray: RecurringDays = [false, false, false, false, false, false, false];

  for (const schedule of scheduleTask) {
    const dayIndex = daysOfWeek.indexOf(schedule.period.toLowerCase());
    if (dayIndex === -1) {
      throw new Error(`Invalid day: ${schedule}`);
    }

    daysArray[dayIndex] = true;
  }

  return {
    startOn: scheduleTask[0].start_from ? new Date(scheduleTask[0].start_from) : new Date(),
    days: daysArray,
    until: scheduleTask[0].until ? endOfMinute(new Date(scheduleTask[0].until)) : undefined,
    at: scheduleTask[0].start_from ? new Date(scheduleTask[0].start_from) : new Date(),
  };
};

export const getScheduledTaskTitle = (task: ScheduledTask): string => {
  if (!task.task_request || !task.task_request.category) {
    return `[${task.id}] Unknown`;
  }

  return task.task_request ? getShortDescription(task.task_request) : '';
};

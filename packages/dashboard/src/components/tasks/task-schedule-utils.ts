import { ProcessedEvent } from '@aldabil/react-scheduler/types';
import { ScheduledTask, ScheduledTaskScheduleOutput as ApiSchedule } from 'api-client';
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
import {
  getShortDescription,
  getTaskBookingLabelFromTaskRequest,
  RecurringDays,
  Schedule,
  TaskDefinition,
} from 'react-components';

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
  getEventColor: () => string | undefined,
): ProcessedEvent[] => {
  if (!schedule.at) {
    console.warn('Unable to convert schedule without [at] to an event');
    return [];
  }
  const [hours, minutes] = schedule.at.split(':').map((s: string) => Number(s));
  let cur = new Date(start);
  cur.setHours(hours);
  cur.setMinutes(minutes);

  const scheStartFrom = task.start_from ? startOfMinute(new Date(task.start_from)) : null;
  const scheUntil = task.until ? endOfMinute(new Date(task.until)) : null;

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
      // cur is provided in the dashboard's and server's timezone, therefore
      // cur should be formatted accordingly and checked against except_dates
      // without converting to ISO (this will be UTC).
      const numberToPaddedStr = (num: number): string => {
        return num >= 10 ? num.toString() : `0${num}`;
      };
      const curFormatted = `${cur.getFullYear()}-${numberToPaddedStr(
        cur.getMonth() + 1,
      )}-${numberToPaddedStr(cur.getDate())}`;
      if (!task.except_dates.includes(curFormatted)) {
        events.push({
          start: cur,
          end: addMinutes(cur, 45),
          event_id: getEventId(),
          title: getEventTitle(),
          color: getEventColor(),
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

  // The startOn date should be this exact event's date, to prevent adding
  // events to the weeks/days leading up to this edit.
  return {
    startOn: date,
    days: daysArray,
    until: endOfDay(new Date(date.toISOString())),
    at: scheduleTask[0].at ? new Date(scheduleTask[0].at) : new Date(),
  };
};

export const apiScheduleToSchedule = (scheduleTask: ScheduledTask): Schedule => {
  const daysOfWeek = ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday'];

  const daysArray: RecurringDays = [false, false, false, false, false, false, false];

  for (const schedule of scheduleTask.schedules) {
    const dayIndex = daysOfWeek.indexOf(schedule.period.toLowerCase());
    if (dayIndex === -1) {
      throw new Error(`Invalid day: ${schedule}`);
    }

    daysArray[dayIndex] = true;
  }

  return {
    startOn: scheduleTask.start_from ? new Date(scheduleTask.start_from) : new Date(),
    days: daysArray,
    until: scheduleTask.until ? endOfMinute(new Date(scheduleTask.until)) : undefined,
    at: scheduleTask.schedules[0].at ? new Date(scheduleTask.schedules[0].at) : new Date(),
  };
};

export const getScheduledTaskTitle = (
  task: ScheduledTask,
  supportedTasks?: TaskDefinition[],
): string => {
  const taskBookingLabel = getTaskBookingLabelFromTaskRequest(task.task_request);

  let remappedTaskName: string | undefined = undefined;
  if (supportedTasks && taskBookingLabel && 'task_definition_id' in taskBookingLabel) {
    for (const s of supportedTasks) {
      if (s.taskDefinitionId === taskBookingLabel['task_definition_id']) {
        remappedTaskName = s.taskDisplayName;
      }
    }
  }

  const shortDescription = getShortDescription(task.task_request, remappedTaskName);
  if (!task.task_request || !task.task_request.category || !shortDescription) {
    return `[${task.id}] Unknown`;
  }
  return shortDescription;
};

export const getScheduledTaskColor = (
  task: ScheduledTask,
  supportedTasks?: TaskDefinition[],
): string | undefined => {
  const taskBookingLabel = getTaskBookingLabelFromTaskRequest(task.task_request);

  let customEventColor: string | undefined = undefined;
  if (supportedTasks && taskBookingLabel && 'task_definition_id' in taskBookingLabel) {
    for (const s of supportedTasks) {
      if (s.taskDefinitionId === taskBookingLabel['task_definition_id']) {
        customEventColor = s.scheduleEventColor;
      }
    }
  }
  return customEventColor;
};

// Pad a number to 2 digits
function pad(n: number): string {
  return `${Math.floor(Math.abs(n))}`.padStart(2, '0');
}

// Get timezone offset in ISO format (+hh:mm or -hh:mm)
function getTimezoneOffset(date: Date): string {
  const tzOffset = -date.getTimezoneOffset();
  const diff = tzOffset >= 0 ? '+' : '-';
  return diff + pad(tzOffset / 60) + ':' + pad(tzOffset % 60);
}

// Convert Date to local time zone ISO string that contains timezone information
export function toISOStringWithTimezone(date: Date): string {
  return (
    date.getFullYear() +
    '-' +
    pad(date.getMonth() + 1) +
    '-' +
    pad(date.getDate()) +
    'T' +
    pad(date.getHours()) +
    ':' +
    pad(date.getMinutes()) +
    ':' +
    pad(date.getSeconds()) +
    getTimezoneOffset(date)
  );
}

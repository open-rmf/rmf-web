import { Scheduler } from '@aldabil/react-scheduler';
import {
  CellRenderedProps,
  ProcessedEvent,
  SchedulerHelpers,
  SchedulerProps,
} from '@aldabil/react-scheduler/types';
import { DayProps } from '@aldabil/react-scheduler/views/Day';
import { MonthProps } from '@aldabil/react-scheduler/views/Month';
import { WeekProps } from '@aldabil/react-scheduler/views/Week';
import { Button, Typography } from '@mui/material';
import { ScheduledTask, ScheduledTaskScheduleOutput as ApiSchedule } from 'api-client';
import React from 'react';
import {
  ConfirmationDialog,
  CreateTaskForm,
  CreateTaskFormProps,
  EventEditDeletePopup,
  Schedule,
} from 'react-components';

import { allowedTasks } from '../../app-config';
import { useCreateTaskFormData } from '../../hooks/useCreateTaskForm';
import useGetUsername from '../../hooks/useFetchUser';
import { AppControllerContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { RmfAppContext } from '../rmf-app';
import {
  apiScheduleToSchedule,
  getScheduledTaskColor,
  getScheduledTaskTitle,
  scheduleToEvents,
  scheduleWithSelectedDay,
  toISOStringWithTimezone,
} from './task-schedule-utils';
import { dispatchTask, editScheduledTaskEvent, editScheduledTaskSchedule } from './utils';

enum EventScopes {
  ALL = 'all',
  CURRENT = 'current',
}

interface CustomCalendarEditorProps {
  scheduler: SchedulerHelpers;
  value: string;
  onChange: (event: React.ChangeEvent<HTMLInputElement>) => void;
}

const disablingCellsWithoutEvents = (
  events: ProcessedEvent[],
  { start, ...props }: CellRenderedProps,
): React.ReactElement => {
  const filteredEvents = events.filter((event) => start.getTime() !== event.start.getTime());
  const disabled = filteredEvents.length > 0 || events.length === 0;
  const restProps = disabled ? {} : props;
  return (
    <Button
      style={{
        height: '100%',
        background: disabled ? '#eee' : 'transparent',
        cursor: disabled ? 'default' : 'pointer',
      }}
      disableRipple={disabled}
      {...restProps}
    />
  );
};

export const TaskSchedule = () => {
  const rmf = React.useContext(RmfAppContext);
  const { showAlert } = React.useContext(AppControllerContext);

  const { waypointNames, pickupPoints, dropoffPoints, cleaningZoneNames, fleets } =
    useCreateTaskFormData(rmf);
  const username = useGetUsername(rmf);
  const [eventScope, setEventScope] = React.useState<string>(EventScopes.CURRENT);
  const [refreshTaskScheduleCount, setRefreshTaskScheduleCount] = React.useState(0);
  const exceptDateRef = React.useRef<Date>(new Date());
  const currentEventIdRef = React.useRef<number>(-1);
  const [currentScheduleTask, setCurrentScheduledTask] = React.useState<ScheduledTask | undefined>(
    undefined,
  );
  const [calendarEvents, setCalendarEvents] = React.useState<ProcessedEvent[]>([]);
  const [openDeleteScheduleDialog, setOpenDeleteScheduleDialog] = React.useState(false);
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [scheduleToEdit, setScheduleToEdit] = React.useState<Schedule>({
    startOn: new Date(),
    days: [false, false, false, false, false, false, false],
    until: undefined,
    at: new Date(),
  });
  type ViewEvent = {
    start: Date;
    end: Date;
    view: 'day' | 'week' | 'month';
  };
  const [currentView, setCurrentView] = React.useState<ViewEvent>({
    start: new Date(),
    end: new Date(),
    view: 'week',
  });
  const [selectedDate, setSelectedDate] = React.useState<Date>(new Date());

  React.useEffect(() => {
    const sub = AppEvents.refreshTaskSchedule.subscribe({
      next: () => {
        setRefreshTaskScheduleCount((oldValue) => ++oldValue);
      },
    });
    return () => sub.unsubscribe();
  }, []);

  const eventsMap = React.useRef<Record<number, ScheduledTask>>({});
  const getRemoteEvents = React.useCallback<NonNullable<SchedulerProps['getRemoteEvents']>>(
    async (params) => {
      // Keep track of current view so users can retain the same view after
      // deleting or editing schedules.
      setCurrentView(params);

      if (!rmf) {
        return;
      }
      const tasks = (
        await rmf.tasksApi.getScheduledTasksScheduledTasksGet(
          params.end.toISOString(),
          params.start.toISOString(),
        )
      ).data;
      let counter = 0;
      const getEventId = () => {
        return counter++;
      };
      eventsMap.current = {};
      return tasks.flatMap((t: ScheduledTask) =>
        t.schedules.flatMap<ProcessedEvent>((s: ApiSchedule) => {
          const events = scheduleToEvents(
            params.start,
            params.end,
            s,
            t,
            getEventId,
            () => getScheduledTaskTitle(t, allowedTasks),
            () => getScheduledTaskColor(t, allowedTasks),
          );
          events.forEach((ev) => {
            eventsMap.current[Number(ev.event_id)] = t;
          });
          setCalendarEvents(events);
          return events;
        }),
      );
    },
    [rmf],
  );

  const CustomCalendarEditor = ({ scheduler, value, onChange }: CustomCalendarEditorProps) => {
    return (
      <ConfirmationDialog
        confirmText={'Ok'}
        cancelText="Cancel"
        open={true}
        title={'Edit recurring task'}
        submitting={undefined}
        onClose={() => {
          scheduler.close();
          setEventScope(EventScopes.CURRENT);
          AppEvents.refreshTaskSchedule.next();
        }}
        onSubmit={() => {
          setOpenCreateTaskForm(true);
          const task = eventsMap.current[Number(currentEventIdRef.current)];
          if (!task) {
            throw new Error(`unable to find task for event ${currentEventIdRef.current}`);
          }
          setCurrentScheduledTask(task);
          if (eventScope === EventScopes.CURRENT) {
            setScheduleToEdit(scheduleWithSelectedDay(task.schedules, exceptDateRef.current));
          } else {
            setScheduleToEdit(apiScheduleToSchedule(task));
          }
          AppEvents.refreshTaskSchedule.next();
          scheduler.close();
        }}
      >
        <EventEditDeletePopup
          currentValue={EventScopes.CURRENT}
          allValue={EventScopes.ALL}
          value={value}
          onChange={onChange}
        />
      </ConfirmationDialog>
    );
  };

  const dispatchTaskCallback = React.useCallback<Required<CreateTaskFormProps>['onDispatchTask']>(
    async (taskRequest, robotDispatchTarget) => {
      if (!rmf) {
        throw new Error('tasks api not available');
      }
      await dispatchTask(rmf, taskRequest, robotDispatchTarget);
      AppEvents.refreshTaskApp.next();
    },
    [rmf],
  );

  const editScheduledTaskCallback = React.useCallback<
    Required<CreateTaskFormProps>['onScheduleTask']
  >(
    async (taskRequest, schedule) => {
      if (!rmf) {
        throw new Error('tasks api not available');
      }

      if (!currentScheduleTask) {
        throw new Error('No schedule task selected for submission.');
      }

      // Edit entire schedule
      if (eventScope !== EventScopes.CURRENT) {
        console.debug(
          `Editing schedule id [${currentScheduleTask.id}] with new schedule: ${schedule}`,
        );
        await editScheduledTaskSchedule(rmf, taskRequest, schedule, currentScheduleTask.id);
        return;
      }

      // Edit a single event
      console.debug(
        `Editing schedule id [${currentScheduleTask.id}] event [${exceptDateRef.current}] with new schedule: ${schedule}`,
      );
      await editScheduledTaskEvent(
        rmf,
        taskRequest,
        schedule,
        exceptDateRef.current,
        currentScheduleTask.id,
      );

      setEventScope(EventScopes.CURRENT);
      AppEvents.refreshTaskSchedule.next();
    },
    [rmf, currentScheduleTask, eventScope],
  );

  const handleSubmitDeleteSchedule: React.MouseEventHandler = async (ev) => {
    ev.preventDefault();
    try {
      const task = eventsMap.current[Number(currentEventIdRef.current)];

      if (!task) {
        throw new Error(`unable to find task for event ${currentEventIdRef.current}`);
      }
      if (!rmf) {
        throw new Error('tasks api not available');
      }

      if (eventScope === EventScopes.CURRENT) {
        const eventDate = toISOStringWithTimezone(exceptDateRef.current);
        console.debug(`Deleting schedule id ${task.id}, event date ${eventDate}`);
        await rmf.tasksApi.addExceptDateScheduledTasksTaskIdExceptDatePost(task.id, {
          except_date: eventDate,
        });
      } else {
        console.debug(`Deleting schedule with id ${task.id}`);
        await rmf.tasksApi.delScheduledTasksScheduledTasksTaskIdDelete(task.id);
      }
      AppEvents.refreshTaskSchedule.next();

      // Set the default values
      setOpenDeleteScheduleDialog(false);
      currentEventIdRef.current = -1;
      setEventScope(EventScopes.CURRENT);
    } catch (e) {
      console.error(`Failed to delete scheduled task: ${e}`);
    }
  };

  const defaultDaySettings: DayProps = {
    startHour: 0,
    endHour: 23,
    step: 60,
    cellRenderer: ({ start, ...props }: CellRenderedProps) =>
      disablingCellsWithoutEvents(calendarEvents, { start, ...props }),
  };
  const defaultWeekSettings: WeekProps = {
    weekDays: [0, 1, 2, 3, 4, 5, 6],
    weekStartOn: 1,
    startHour: 0,
    endHour: 23,
    step: 60,
    cellRenderer: ({ start, ...props }: CellRenderedProps) =>
      disablingCellsWithoutEvents(calendarEvents, { start, ...props }),
  };
  const defaultMonthSettings: MonthProps = {
    weekDays: [0, 1, 2, 3, 4, 5, 6],
    weekStartOn: 1,
    startHour: 0,
    endHour: 23,
    cellRenderer: ({ start, ...props }: CellRenderedProps) =>
      disablingCellsWithoutEvents(calendarEvents, { start, ...props }),
  };

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <Scheduler
        // react-scheduler does not support refreshing, workaround by mounting a new instance.
        key={`scheduler-${refreshTaskScheduleCount}`}
        selectedDate={selectedDate}
        view={currentView.view}
        day={defaultDaySettings}
        week={defaultWeekSettings}
        month={defaultMonthSettings}
        draggable={false}
        editable={true}
        getRemoteEvents={getRemoteEvents}
        onEventClick={(event: ProcessedEvent) => {
          currentEventIdRef.current = Number(event.event_id);
          exceptDateRef.current = event.start;
        }}
        onDelete={async (deletedId: number) => {
          currentEventIdRef.current = Number(deletedId);
          setOpenDeleteScheduleDialog(true);
        }}
        customEditor={(scheduler) => (
          <CustomCalendarEditor
            scheduler={scheduler}
            value={eventScope}
            onChange={(event: React.ChangeEvent<HTMLInputElement>) => {
              setEventScope(event.target.value);
            }}
          />
        )}
        viewerExtraComponent={(_fields, event) => {
          return <Typography variant="caption">{event.title}</Typography>;
        }}
        onSelectedDateChange={setSelectedDate}
      />
      {openCreateTaskForm && (
        <CreateTaskForm
          user={username ? username : 'unknown user'}
          fleets={fleets}
          tasksToDisplay={allowedTasks}
          patrolWaypoints={waypointNames}
          cleaningZones={cleaningZoneNames}
          pickupPoints={pickupPoints}
          dropoffPoints={dropoffPoints}
          open={openCreateTaskForm}
          scheduleToEdit={scheduleToEdit}
          requestTask={currentScheduleTask?.task_request}
          onClose={() => {
            setOpenCreateTaskForm(false);
            setEventScope(EventScopes.CURRENT);
            AppEvents.refreshTaskSchedule.next();
          }}
          onDispatchTask={dispatchTaskCallback}
          onScheduleTask={editScheduledTaskCallback}
          onSuccess={() => {
            setOpenCreateTaskForm(false);
            showAlert('success', 'Successfully created task');
          }}
          onFail={(e) => {
            showAlert('error', `Failed to create task: ${e.message}`);
          }}
          onSuccessScheduling={() => {
            setOpenCreateTaskForm(false);
            showAlert('success', 'Successfully created schedule');
          }}
          onFailScheduling={(e) => {
            showAlert('error', `Failed to submit schedule: ${e.message}`);
          }}
        />
      )}
      {openDeleteScheduleDialog && (
        <ConfirmationDialog
          confirmText={'Ok'}
          cancelText="Cancel"
          open={openDeleteScheduleDialog}
          title={'Delete recurring event'}
          submitting={undefined}
          onClose={() => {
            setOpenDeleteScheduleDialog(false);
            setEventScope(EventScopes.CURRENT);
          }}
          onSubmit={handleSubmitDeleteSchedule}
        >
          <EventEditDeletePopup
            currentValue={EventScopes.CURRENT}
            allValue={EventScopes.ALL}
            value={eventScope}
            onChange={(event: React.ChangeEvent<HTMLInputElement>) =>
              setEventScope(event.target.value)
            }
          />
        </ConfirmationDialog>
      )}
    </div>
  );
};

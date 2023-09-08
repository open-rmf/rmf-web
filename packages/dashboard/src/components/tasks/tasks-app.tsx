import { Scheduler } from '@aldabil/react-scheduler';
import { ProcessedEvent, SchedulerProps } from '@aldabil/react-scheduler/types';
import DownloadIcon from '@mui/icons-material/Download';
import RefreshIcon from '@mui/icons-material/Refresh';
import {
  Box,
  FormControl,
  FormControlLabel,
  IconButton,
  Menu,
  MenuItem,
  Radio,
  RadioGroup,
  Tab,
  Tabs,
  TableContainer,
  Toolbar,
  Tooltip,
} from '@mui/material';
import {
  ApiServerModelsTortoiseModelsScheduledTaskScheduledTask as ScheduledTask,
  ApiServerModelsTortoiseModelsScheduledTaskScheduledTaskScheduleLeaf as ApiSchedule,
  TaskState,
} from 'api-client';
import {
  addMinutes,
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
import React from 'react';
import {
  ConfirmationDialog,
  FilterFields,
  MuiMouseEvent,
  SortFields,
  TaskDataGridTable,
  Tasks,
  Window,
} from 'react-components';
import { Subscription } from 'rxjs';
import { AppEvents } from '../app-events';
import { MicroAppProps } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { TaskSummary } from './task-summary';
import { downloadCsvFull, downloadCsvMinimal } from './utils';

const RefreshTaskQueueTableInterval = 5000;

interface TabPanelProps {
  children?: React.ReactNode;
  index: number;
  selectedTabIndex: number;
}

enum ScheduleDeleteOptions {
  ALL = 'all',
  CURRENT = 'current',
}

function tabId(index: number): string {
  return `simple-tab-${index}`;
}

function tabPanelId(index: number): string {
  return `simple-tabpanel-${index}`;
}

function TabPanel(props: TabPanelProps) {
  const { children, selectedTabIndex, index, ...other } = props;
  return (
    <div
      role="tabpanel"
      hidden={selectedTabIndex !== index}
      id={tabPanelId(index)}
      aria-labelledby={tabId(index)}
      {...other}
    >
      {selectedTabIndex === index && <Box sx={{ p: 3 }}>{children}</Box>}
    </div>
  );
}

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
function scheduleToEvents(
  start: Date,
  end: Date,
  schedule: ApiSchedule,
  task: ScheduledTask,
  getEventId: () => number,
  getEventTitle: () => string,
): ProcessedEvent[] {
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
      if (!task.except_dates.includes(cur.toLocaleDateString())) {
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
}

function getScheduledTaskTitle(task: ScheduledTask): string {
  if (!task.task_request || !task.task_request.category) {
    return `[${task.id}] Unknown`;
  }
  return `[${task.id}] ${task.task_request.category}`;
}

export const TasksApp = React.memo(
  React.forwardRef(
    (
      { onClose, children, ...otherProps }: React.PropsWithChildren<MicroAppProps>,
      ref: React.Ref<HTMLDivElement>,
    ) => {
      const rmf = React.useContext(RmfAppContext);
      const [autoRefresh, setAutoRefresh] = React.useState(true);
      const [refreshTaskAppCount, setRefreshTaskAppCount] = React.useState(0);

      const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
      const [openTaskSummary, setOpenTaskSummary] = React.useState(false);
      const [selectedTask, setSelectedTask] = React.useState<TaskState | null>(null);

      const [openDeleteScheduleDialog, setOpenDeleteScheduleDialog] = React.useState(false);
      const [scheduleDeleteValue, setScheduleDeleteValue] = React.useState<string>(
        ScheduleDeleteOptions.CURRENT,
      );
      const [currentEventId, setCurrentEventId] = React.useState<number>(-1);
      const exceptDateRef = React.useRef<Date>(new Date());
      const [tasksState, setTasksState] = React.useState<Tasks>({
        isLoading: true,
        data: [],
        total: 0,
        page: 1,
        pageSize: 10,
      });

      const [filterFields, setFilterFields] = React.useState<FilterFields>({ model: undefined });
      const [sortFields, setSortFields] = React.useState<SortFields>({ model: undefined });

      React.useEffect(() => {
        const sub = AppEvents.refreshTaskApp.subscribe({
          next: () => {
            setRefreshTaskAppCount((oldValue) => ++oldValue);
          },
        });
        return () => sub.unsubscribe();
      }, []);

      React.useEffect(() => {
        if (!autoRefresh) {
          return;
        }

        const refreshTaskQueueTable = async () => {
          AppEvents.refreshTaskApp.next();
        };
        const refreshInterval = window.setInterval(
          refreshTaskQueueTable,
          RefreshTaskQueueTableInterval,
        );
        return () => {
          clearInterval(refreshInterval);
        };
      }, [autoRefresh]);

      // TODO: parameterize this variable
      const GET_LIMIT = 10;
      React.useEffect(() => {
        if (!rmf) {
          return;
        }

        let filterColumn: string | undefined = undefined;
        let filterValue: string | undefined = undefined;
        if (filterFields.model && filterFields.model.items.length >= 1) {
          filterColumn = filterFields.model.items[0].columnField;
          filterValue = filterFields.model.items[0].value;

          const filterOperator: string | undefined = filterFields.model.items[0].operatorValue;
          if (
            (filterColumn === 'unix_millis_start_time' ||
              filterColumn === 'unix_millis_finish_time') &&
            filterValue
          ) {
            const selectedTime = new Date(filterValue);
            if (filterOperator && filterOperator === 'onOrBefore') {
              filterValue = `0,${selectedTime.getTime()}`;
            } else if (filterOperator && filterOperator === 'onOrAfter') {
              // Enforce an upper limit which is 24 hours ahead of the current time
              const now = new Date();
              const upperLimit = now.getTime() + 86400000;
              filterValue = `${selectedTime.getTime()},${upperLimit}`;
            }
          }
        }

        let orderBy: string = '-unix_millis_start_time';
        if (sortFields.model && sortFields.model.length >= 1) {
          orderBy =
            sortFields.model[0].sort === 'desc'
              ? '-' + sortFields.model[0].field
              : sortFields.model[0].field;
        }

        const subs: Subscription[] = [];
        (async () => {
          const resp = await rmf.tasksApi.queryTaskStatesTasksGet(
            filterColumn && filterColumn === 'id_' ? filterValue : undefined,
            filterColumn && filterColumn === 'category' ? filterValue : undefined,
            filterColumn && filterColumn === 'assigned_to' ? filterValue : undefined,
            filterColumn && filterColumn === 'status' ? filterValue : undefined,
            filterColumn && filterColumn === 'unix_millis_start_time' ? filterValue : undefined,
            filterColumn && filterColumn === 'unix_millis_finish_time' ? filterValue : undefined,
            GET_LIMIT,
            (tasksState.page - 1) * GET_LIMIT, // Datagrid component need to start in page 1. Otherwise works wrong
            orderBy,
            undefined,
          );
          const results = resp.data as TaskState[];
          const newTasks = results.slice(0, GET_LIMIT);

          setTasksState((old) => ({
            ...old,
            isLoading: false,
            data: newTasks,
            total:
              results.length === GET_LIMIT
                ? tasksState.page * GET_LIMIT + 1
                : tasksState.page * GET_LIMIT - 9,
          }));

          subs.push(
            ...newTasks.map((task) =>
              rmf
                .getTaskStateObs(task.booking.id)
                .subscribe((task) =>
                  setTasksState((prev) => ({ ...prev, [task.booking.id]: task })),
                ),
            ),
          );
        })();
        return () => subs.forEach((s) => s.unsubscribe());
      }, [rmf, refreshTaskAppCount, tasksState.page, filterFields.model, sortFields.model]);

      const getAllTasks = async (timestamp: Date) => {
        if (!rmf) {
          return [];
        }

        const resp = await rmf.tasksApi.queryTaskStatesTasksGet(
          undefined,
          undefined,
          undefined,
          undefined,
          `0,${timestamp.getTime()}`,
          undefined,
          undefined,
          undefined,
          '-unix_millis_start_time',
          undefined,
        );
        const allTasks = resp.data as TaskState[];
        return allTasks;
      };

      const exportTasksToCsv = async (minimal: boolean) => {
        const now = new Date();
        const allTasks = await getAllTasks(now);
        if (!allTasks || !allTasks.length) {
          return;
        }
        if (minimal) {
          downloadCsvMinimal(now, allTasks);
        } else {
          downloadCsvFull(now, allTasks);
        }
      };

      const [anchorExportElement, setAnchorExportElement] = React.useState<null | HTMLElement>(
        null,
      );
      const openExportMenu = Boolean(anchorExportElement);
      const handleClickExportMenu = (event: React.MouseEvent<HTMLElement>) => {
        setAnchorExportElement(event.currentTarget);
      };
      const handleCloseExportMenu = () => {
        setAnchorExportElement(null);
      };

      const eventsMap = React.useRef<Record<number, ScheduledTask>>({});
      const getRemoteEvents = React.useCallback<NonNullable<SchedulerProps['getRemoteEvents']>>(
        async (params) => {
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
              const events = scheduleToEvents(params.start, params.end, s, t, getEventId, () =>
                getScheduledTaskTitle(t),
              );
              events.forEach((ev) => {
                eventsMap.current[Number(ev.event_id)] = t;
              });
              return events;
            }),
          );
        },
        [rmf],
      );

      enum TaskTablePanel {
        QueueTable = 0,
        Schedule = 1,
      }

      const [selectedPanelIndex, setSelectedPanelIndex] = React.useState(TaskTablePanel.QueueTable);

      const handlePanelChange = (_: React.SyntheticEvent, newSelectedTabIndex: number) => {
        setSelectedPanelIndex(newSelectedTabIndex);
        setAutoRefresh(newSelectedTabIndex === TaskTablePanel.QueueTable);
      };

      const handleSubmitDeleteSchedule: React.MouseEventHandler = async (ev) => {
        ev.preventDefault();
        try {
          const task = eventsMap.current[Number(currentEventId)];

          if (!task) {
            throw new Error(`unable to find task for event ${currentEventId}`);
          }
          if (!rmf) {
            throw new Error('tasks api not available');
          }

          if (scheduleDeleteValue === ScheduleDeleteOptions.CURRENT) {
            await rmf.tasksApi.delScheduledTasksEventScheduledTasksTaskIdClearPut(
              task.id,
              exceptDateRef.current.toISOString(),
            );
          } else {
            await rmf.tasksApi.delScheduledTasksScheduledTasksTaskIdDelete(task.id);
          }
          AppEvents.refreshTaskApp.next();

          // Set the default values
          setOpenDeleteScheduleDialog(false);
          setCurrentEventId(-1);
          setScheduleDeleteValue(ScheduleDeleteOptions.CURRENT);
        } catch (e) {
          console.error(`Failed to delete scheduled task: ${e}`);
        }
      };

      return (
        <Window
          ref={ref}
          title="Tasks"
          onClose={onClose}
          toolbar={
            <Toolbar variant="dense">
              <div>
                <Tooltip title="Download" placement="top">
                  <IconButton
                    id="export-button"
                    aria-controls={openExportMenu ? 'export-menu' : undefined}
                    aria-haspopup="true"
                    aria-expanded={openExportMenu ? 'true' : undefined}
                    onClick={handleClickExportMenu}
                    color="inherit"
                  >
                    <DownloadIcon />
                  </IconButton>
                </Tooltip>
                <Menu
                  id="export-menu"
                  MenuListProps={{
                    'aria-labelledby': 'export-button',
                  }}
                  anchorEl={anchorExportElement}
                  open={openExportMenu}
                  onClose={handleCloseExportMenu}
                >
                  <MenuItem
                    onClick={() => {
                      handleCloseExportMenu();
                      exportTasksToCsv(true);
                    }}
                    disableRipple
                  >
                    Export Minimal
                  </MenuItem>
                  <MenuItem
                    onClick={() => {
                      handleCloseExportMenu();
                      exportTasksToCsv(false);
                    }}
                    disableRipple
                  >
                    Export Full
                  </MenuItem>
                </Menu>
              </div>
              <Tooltip title="Refresh" color="inherit" placement="top">
                <IconButton
                  onClick={() => {
                    AppEvents.refreshTaskApp.next();
                  }}
                  aria-label="Refresh"
                >
                  <RefreshIcon />
                </IconButton>
              </Tooltip>
            </Toolbar>
          }
          {...otherProps}
        >
          <Tabs value={selectedPanelIndex} onChange={handlePanelChange} aria-label="Task App Tabs">
            <Tab
              label="Queue"
              id={tabId(TaskTablePanel.QueueTable)}
              aria-controls={tabPanelId(TaskTablePanel.QueueTable)}
            />
            <Tab
              label="Schedule"
              id={tabId(TaskTablePanel.Schedule)}
              aria-controls={tabPanelId(TaskTablePanel.Schedule)}
            />
          </Tabs>
          <TabPanel selectedTabIndex={selectedPanelIndex} index={TaskTablePanel.QueueTable}>
            <TableContainer>
              <TaskDataGridTable
                tasks={tasksState}
                onTaskClick={(_: MuiMouseEvent, task: TaskState) => {
                  setSelectedTask(task);
                  setOpenTaskSummary(true);
                }}
                setFilterFields={setFilterFields}
                setSortFields={setSortFields}
                onPageChange={(newPage: number) =>
                  setTasksState((old: Tasks) => ({ ...old, page: newPage + 1 }))
                }
                onPageSizeChange={(newPageSize: number) =>
                  setTasksState((old: Tasks) => ({ ...old, pageSize: newPageSize }))
                }
              />
            </TableContainer>
          </TabPanel>
          <TabPanel selectedTabIndex={selectedPanelIndex} index={TaskTablePanel.Schedule}>
            <Scheduler
              // react-scheduler does not support refreshing, workaround by mounting a new instance.
              key={`scheduler-${refreshTaskAppCount}`}
              view="week"
              week={{
                weekDays: [0, 1, 2, 3, 4, 5, 6],
                weekStartOn: 1,
                startHour: 0,
                endHour: 23,
                step: 60,
              }}
              day={{
                startHour: 0,
                endHour: 23,
                step: 60,
              }}
              draggable={false}
              editable={false}
              getRemoteEvents={getRemoteEvents}
              onEventClick={(event: ProcessedEvent) => {
                exceptDateRef.current = event.start;
              }}
              onDelete={async (deletedId: number) => {
                setCurrentEventId(Number(deletedId));
                setOpenDeleteScheduleDialog(true);
              }}
            />
          </TabPanel>
          <input type="file" style={{ display: 'none' }} ref={uploadFileInputRef} />
          {openTaskSummary && (
            <TaskSummary task={selectedTask} onClose={() => setOpenTaskSummary(false)} />
          )}
          {openDeleteScheduleDialog && (
            <ConfirmationDialog
              confirmText={'Ok'}
              cancelText="Cancel"
              open={openDeleteScheduleDialog}
              title={'Delete event'}
              submitting={undefined}
              onClose={() => {
                setOpenDeleteScheduleDialog(false);
                setScheduleDeleteValue(ScheduleDeleteOptions.CURRENT);
              }}
              onSubmit={handleSubmitDeleteSchedule}
            >
              <FormControl fullWidth={true}>
                <RadioGroup
                  aria-labelledby="schedule-delete-options-radio-buttons-group"
                  name="schedule-delete-options-radio-buttons-group"
                  value={scheduleDeleteValue}
                  onChange={(event: React.ChangeEvent<HTMLInputElement>) =>
                    setScheduleDeleteValue(event.target.value)
                  }
                >
                  <FormControlLabel
                    value={ScheduleDeleteOptions.CURRENT}
                    control={<Radio />}
                    label="This event"
                  />
                  <FormControlLabel
                    value={ScheduleDeleteOptions.ALL}
                    control={<Radio />}
                    label="All events"
                  />
                </RadioGroup>
              </FormControl>
            </ConfirmationDialog>
          )}
          {children}
        </Window>
      );
    },
  ),
);

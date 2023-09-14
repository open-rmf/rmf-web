import { Scheduler } from '@aldabil/react-scheduler';
import {
  CellRenderedProps,
  ProcessedEvent,
  SchedulerHelpers,
  SchedulerProps,
} from '@aldabil/react-scheduler/types';
import DownloadIcon from '@mui/icons-material/Download';
import RefreshIcon from '@mui/icons-material/Refresh';
import {
  Box,
  IconButton,
  Menu,
  MenuItem,
  Tab,
  Tabs,
  TableContainer,
  Toolbar,
  Tooltip,
  Button,
} from '@mui/material';
import {
  ApiServerModelsTortoiseModelsScheduledTaskScheduledTask as ScheduledTask,
  ApiServerModelsTortoiseModelsScheduledTaskScheduledTaskScheduleLeaf as ApiSchedule,
  TaskState,
} from 'api-client';
import React from 'react';
import {
  ConfirmationDialog,
  CreateTaskForm,
  CreateTaskFormProps,
  EventEditDeletePopup,
  FilterFields,
  MuiMouseEvent,
  Schedule,
  SortFields,
  TaskDataGridTable,
  Tasks,
  Window,
} from 'react-components';
import { Subscription } from 'rxjs';
import { useCreateTaskFormData } from '../../hooks/useCreateTaskForm';
import useGetUsername from '../../hooks/useFetchUser';
import { AppControllerContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { MicroAppProps } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { toApiSchedule } from '../utils';
import {
  apiScheduleToSchedule,
  getScheduledTaskTitle,
  scheduleToEvents,
  scheduleWithSelectedDay,
} from './task-schedule-utils';
import { TaskSummary } from './task-summary';
import { downloadCsvFull, downloadCsvMinimal } from './utils';

const RefreshTaskQueueTableInterval = 5000;

/*Scheduling TODOS [CR]: 
 - Create logic for editing single instance [x]
 - Create a util file for those repeated functions in appbar and tasks-app [x]
 - Create hooks to return the username [similar to useCreateTaskForm hook] [x]
 - Block all cells if they have no events. Currently it works only for weeks [x]
 - Create test for new react components [x]
 - Check why the first event returns id -1 [x]
 - Clean a little the code []
*/

enum EventScopes {
  ALL = 'all',
  CURRENT = 'current',
}

enum TaskTablePanel {
  QueueTable = 0,
  Schedule = 1,
}

interface TabPanelProps {
  children?: React.ReactNode;
  index: number;
  selectedTabIndex: number;
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

interface CustomEditorProps {
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
        cursor: disabled ? 'not-allowed' : 'pointer',
      }}
      disableRipple={disabled}
      {...restProps}
    />
  );
};

export const TasksApp = React.memo(
  React.forwardRef(
    (
      { onClose, children, ...otherProps }: React.PropsWithChildren<MicroAppProps>,
      ref: React.Ref<HTMLDivElement>,
    ) => {
      const rmf = React.useContext(RmfAppContext);
      const { showAlert } = React.useContext(AppControllerContext);
      const { waypointNames, pickupPoints, dropoffPoints, cleaningZoneNames } =
        useCreateTaskFormData(rmf);
      const username = useGetUsername(rmf);

      const [autoRefresh, setAutoRefresh] = React.useState(true);
      const [refreshTaskAppCount, setRefreshTaskAppCount] = React.useState(0);

      const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
      const [openTaskSummary, setOpenTaskSummary] = React.useState(false);
      const [openDeleteScheduleDialog, setOpenDeleteScheduleDialog] = React.useState(false);
      const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);

      const [selectedSchedule, setSelectedSchedule] = React.useState<Schedule>({
        startOn: new Date(),
        days: [false, false, false, false, false, false, false],
        until: undefined,
      });
      const [currentScheduleTask, setCurrentScheduledTask] = React.useState<
        ScheduledTask | undefined
      >(undefined);
      const [calendarEvents, setCalendarEvents] = React.useState<ProcessedEvent[]>([]);
      const [eventScope, setEventScope] = React.useState<string>(EventScopes.CURRENT);

      const [selectedTask, setSelectedTask] = React.useState<TaskState | null>(null);
      const [tasksState, setTasksState] = React.useState<Tasks>({
        isLoading: true,
        data: [],
        total: 0,
        page: 1,
        pageSize: 10,
      });
      const [filterFields, setFilterFields] = React.useState<FilterFields>({ model: undefined });
      const [sortFields, setSortFields] = React.useState<SortFields>({ model: undefined });

      const exceptDateRef = React.useRef<Date>(new Date());
      const currentEventIdRef = React.useRef<number>(-1);

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
              setCalendarEvents(events);
              return events;
            }),
          );
        },
        [rmf],
      );

      const [selectedPanelIndex, setSelectedPanelIndex] = React.useState(TaskTablePanel.QueueTable);

      const handlePanelChange = (_: React.SyntheticEvent, newSelectedTabIndex: number) => {
        setSelectedPanelIndex(newSelectedTabIndex);
        setAutoRefresh(newSelectedTabIndex === TaskTablePanel.QueueTable);
      };

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
          currentEventIdRef.current = -1;
          setEventScope(EventScopes.CURRENT);
        } catch (e) {
          console.error(`Failed to delete scheduled task: ${e}`);
        }
      };

      const submitTasks = React.useCallback<Required<CreateTaskFormProps>['submitTasks']>(
        async (taskRequests, schedule) => {
          if (!rmf) {
            throw new Error('tasks api not available');
          }

          if (!schedule || !currentScheduleTask) {
            throw new Error('Not schedule or task selected');
          }

          const scheduleRequests = taskRequests.map((req) =>
            toApiSchedule(req, schedule && schedule),
          );

          let exceptDate: string | undefined = undefined;
          if (eventScope === EventScopes.CURRENT) {
            exceptDate = exceptDateRef.current.toISOString();
          }

          await Promise.all(
            scheduleRequests.map((req) =>
              rmf.tasksApi.updateScheduleTaskScheduledTasksTaskIdUpdatePost(
                currentScheduleTask.id,
                req,
                exceptDate,
              ),
            ),
          );

          setEventScope(EventScopes.CURRENT);
          AppEvents.refreshTaskApp.next();
        },
        [rmf, currentScheduleTask, eventScope],
      );

      const CustomEditor = ({ scheduler, value, onChange }: CustomEditorProps) => {
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
              AppEvents.refreshTaskApp.next();
            }}
            onSubmit={() => {
              setOpenCreateTaskForm(true);
              const task = eventsMap.current[Number(currentEventIdRef.current)];
              if (!task) {
                throw new Error('No task found');
              }
              setCurrentScheduledTask(task);
              setSelectedSchedule(apiScheduleToSchedule(task.schedules));
              if (eventScope === EventScopes.CURRENT) {
                setSelectedSchedule(scheduleWithSelectedDay(task.schedules, exceptDateRef.current));
              }
              AppEvents.refreshTaskApp.next();
              scheduler.close();
            }}
          >
            <EventEditDeletePopup
              currentValue={EventScopes.CURRENT}
              allValue={EventScopes.ALL}
              value={value}
              deleting={false}
              onChange={onChange}
            />
          </ConfirmationDialog>
        );
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
              month={{
                weekDays: [0, 1, 2, 3, 4, 5, 6],
                weekStartOn: 1,
                startHour: 0,
                endHour: 23,
                cellRenderer: ({ start, ...props }: CellRenderedProps) =>
                  disablingCellsWithoutEvents(calendarEvents, { start, ...props }),
              }}
              week={{
                weekDays: [0, 1, 2, 3, 4, 5, 6],
                weekStartOn: 1,
                startHour: 0,
                endHour: 23,
                step: 60,
                cellRenderer: ({ start, ...props }: CellRenderedProps) =>
                  disablingCellsWithoutEvents(calendarEvents, { start, ...props }),
              }}
              day={{
                startHour: 0,
                endHour: 23,
                step: 60,
                cellRenderer: ({ start, ...props }: CellRenderedProps) =>
                  disablingCellsWithoutEvents(calendarEvents, { start, ...props }),
              }}
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
                <CustomEditor
                  scheduler={scheduler}
                  value={eventScope}
                  onChange={(event: React.ChangeEvent<HTMLInputElement>) => {
                    setEventScope(event.target.value);
                    AppEvents.refreshTaskApp.next();
                  }}
                />
              )}
            />
          </TabPanel>
          <input type="file" style={{ display: 'none' }} ref={uploadFileInputRef} />
          {openTaskSummary && (
            <TaskSummary task={selectedTask} onClose={() => setOpenTaskSummary(false)} />
          )}
          {openCreateTaskForm && (
            <CreateTaskForm
              user={username ? username : 'unknown user'}
              patrolWaypoints={waypointNames}
              cleaningZones={cleaningZoneNames}
              pickupPoints={pickupPoints}
              dropoffPoints={dropoffPoints}
              open={openCreateTaskForm}
              scheduleUnderEdition={true}
              currentSchedule={selectedSchedule}
              requestTask={currentScheduleTask?.task_request}
              onClose={() => {
                setOpenCreateTaskForm(false);
                setEventScope(EventScopes.CURRENT);
                AppEvents.refreshTaskApp.next();
              }}
              submitTasks={submitTasks}
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
              title={'Delete event'}
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
                deleting={true}
                onChange={(event: React.ChangeEvent<HTMLInputElement>) =>
                  setEventScope(event.target.value)
                }
              />
            </ConfirmationDialog>
          )}
          {children}
        </Window>
      );
    },
  ),
);

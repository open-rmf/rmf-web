import { Scheduler } from '@aldabil/react-scheduler';
import { ProcessedEvent, SchedulerProps } from '@aldabil/react-scheduler/types';
import AddOutlinedIcon from '@mui/icons-material/AddOutlined';
import DownloadIcon from '@mui/icons-material/Download';
import RefreshIcon from '@mui/icons-material/Refresh';
import { Grid, IconButton, TableContainer, Toolbar, Tooltip } from '@mui/material';
import Menu from '@mui/material/Menu';
import MenuItem from '@mui/material/MenuItem';
import {
  ApiServerModelsTortoiseModelsScheduledTaskScheduledTask as ScheduledTask,
  ApiServerModelsTortoiseModelsScheduledTaskScheduledTaskScheduleLeaf as ApiSchedule,
  PostScheduledTaskRequest,
  TaskFavoritePydantic as TaskFavorite,
  TaskRequest,
  TaskState,
} from 'api-client';
import {
  addMinutes,
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
} from 'date-fns';
import React from 'react';
import {
  CreateTaskForm,
  CreateTaskFormProps,
  FilterFields,
  getPlaces,
  Schedule,
  SortFields,
  TaskDataGridTable,
  Tasks,
  Window,
} from 'react-components';
import { Subscription } from 'rxjs';
import { AppControllerContext, ResourcesContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { MicroAppProps } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { downloadCsvFull, downloadCsvMinimal, parseTasksFile } from './utils';

function toApiSchedule(taskRequest: TaskRequest, schedule: Schedule): PostScheduledTaskRequest {
  const start = schedule.startOn;
  const apiSchedules: PostScheduledTaskRequest['schedules'] = [];
  const date = new Date(start);
  const start_from = start.toISOString();
  const hours = date.getHours().toString().padStart(2, '0');
  const minutes = date.getMinutes().toString().padStart(2, '0');
  const at = `${hours}:${minutes}`;
  schedule.days[0] && apiSchedules.push({ period: 'monday', start_from, at });
  schedule.days[1] && apiSchedules.push({ period: 'tuesday', start_from, at });
  schedule.days[2] && apiSchedules.push({ period: 'wednesday', start_from, at });
  schedule.days[3] && apiSchedules.push({ period: 'thursday', start_from, at });
  schedule.days[4] && apiSchedules.push({ period: 'friday', start_from, at });
  schedule.days[5] && apiSchedules.push({ period: 'saturday', start_from, at });
  schedule.days[6] && apiSchedules.push({ period: 'sunday', start_from, at });
  return {
    task_request: taskRequest,
    schedules: apiSchedules,
  };
}

function scheduleToEvents(
  start: Date,
  end: Date,
  schedule: ApiSchedule,
  getEventId: () => number,
  getEventTitle: () => string,
): ProcessedEvent[] {
  if (!schedule.at) {
    console.warn('Unable to convert schedule without [at] to an event');
    return [];
  }

  const [hours, minutes] = schedule.at.split(':').map((s) => Number(s));
  let cur = start;
  cur.setHours(hours);
  cur.setMinutes(minutes);
  const scheStartFrom = schedule.start_from ? new Date(schedule.start_from) : null;
  const scheUntil = schedule.until ? new Date(schedule.until) : null;

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
      events.push({
        start: cur,
        end: addMinutes(cur, 15),
        event_id: getEventId(),
        title: getEventTitle(),
      });
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

      const [forceRefresh, setForceRefresh] = React.useState(0);
      const [favoritesTasks, setFavoritesTasks] = React.useState<TaskFavorite[]>([]);

      const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
      const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
      const { showAlert } = React.useContext(AppControllerContext);

      const tasksFromFile = (): Promise<TaskRequest[]> => {
        return new Promise((res) => {
          const fileInputEl = uploadFileInputRef.current;
          if (!fileInputEl) {
            return [];
          }
          let taskFiles: TaskRequest[];
          const listener = async () => {
            try {
              if (!fileInputEl.files || fileInputEl.files.length === 0) {
                return res([]);
              }
              try {
                taskFiles = parseTasksFile(await fileInputEl.files[0].text());
              } catch (err) {
                showAlert('error', (err as Error).message, 5000);
                return res([]);
              }
              // only submit tasks when all tasks are error free
              return res(taskFiles);
            } finally {
              fileInputEl.removeEventListener('input', listener);
              fileInputEl.value = '';
            }
          };
          fileInputEl.addEventListener('input', listener);
          fileInputEl.click();
        });
      };

      const [tasksState, setTasksState] = React.useState<Tasks>({
        isLoading: true,
        data: [],
        total: 0,
        page: 1,
        pageSize: 10,
      });

      const [filterFields, setFilterFields] = React.useState<FilterFields>({ model: undefined });

      const [sortFields, setSortFields] = React.useState<SortFields>({ model: undefined });

      const [placeNames, setPlaceNames] = React.useState<string[]>([]);
      React.useEffect(() => {
        if (!rmf) {
          return;
        }
        const sub = rmf.buildingMapObs.subscribe((map) =>
          setPlaceNames(getPlaces(map).map((p) => p.vertex.name)),
        );
        return () => sub.unsubscribe();
      }, [rmf]);

      React.useEffect(() => {
        if (!rmf) {
          return;
        }
        (async () => {
          const resp = await rmf.tasksApi.getFavoritesTasksFavoriteTasksGet();

          const results = resp.data as TaskFavorite[];
          setFavoritesTasks(results);
        })();

        return () => {
          setFavoritesTasks([]);
        };
      }, [rmf, forceRefresh]);

      const resourceManager = React.useContext(ResourcesContext);

      const [workcells, setWorkcells] = React.useState<string[]>();
      React.useEffect(() => {
        if (!resourceManager?.dispensers) {
          return;
        }
        setWorkcells(Object.keys(resourceManager.dispensers.dispensers));
      }, [resourceManager]);

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
      }, [rmf, forceRefresh, tasksState.page, filterFields.model, sortFields.model]);

      const submitTasks = React.useCallback<Required<CreateTaskFormProps>['submitTasks']>(
        async (taskRequests, schedule) => {
          if (!rmf) {
            throw new Error('tasks api not available');
          }
          if (!schedule) {
            await Promise.all(
              taskRequests.map((request) =>
                rmf.tasksApi.postDispatchTaskTasksDispatchTaskPost({
                  type: 'dispatch_task_request',
                  request,
                }),
              ),
            );
          } else {
            const scheduleRequests = taskRequests.map((req) => toApiSchedule(req, schedule));
            await Promise.all(
              scheduleRequests.map((req) => rmf.tasksApi.postScheduledTaskScheduledTasksPost(req)),
            );
          }
          setForceRefresh((prev) => prev + 1);
        },
        [rmf],
      );

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
      const submitFavoriteTask = React.useCallback<
        Required<CreateTaskFormProps>['submitFavoriteTask']
      >(
        async (taskFavoriteRequest) => {
          if (!rmf) {
            throw new Error('tasks api not available');
          }
          await rmf.tasksApi.postFavoriteTaskFavoriteTasksPost(taskFavoriteRequest);
          setForceRefresh((prev) => prev + 1);
        },
        [rmf],
      );

      const deleteFavoriteTask = React.useCallback<
        Required<CreateTaskFormProps>['deleteFavoriteTask']
      >(
        async (favoriteTask) => {
          if (!rmf) {
            throw new Error('tasks api not available');
          }
          if (!favoriteTask.id) {
            throw new Error('Id is needed');
          }

          await rmf.tasksApi.deleteFavoriteTaskFavoriteTasksFavoriteTaskIdDelete(favoriteTask.id);
          setForceRefresh((prev) => prev + 1);
        },
        [rmf],
      );

      const eventsMap = React.useRef<Record<number, ScheduledTask>>({});
      const getRemoteEvents = React.useCallback<NonNullable<SchedulerProps['getRemoteEvents']>>(
        async (params) => {
          console.log('asjdksajd');
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
          return tasks.flatMap((t) =>
            t.schedules.flatMap<ProcessedEvent>((s) => {
              const events = scheduleToEvents(params.start, params.end, s, getEventId, () =>
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

      return (
        <Window
          ref={ref}
          title="Tasks"
          onClose={onClose}
          toolbar={
            <Toolbar variant="dense">
              <div>
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
                    setForceRefresh((prev) => prev + 1);
                  }}
                  aria-label="Refresh"
                >
                  <RefreshIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="Create task" color="inherit" placement="top">
                <IconButton onClick={() => setOpenCreateTaskForm(true)} aria-label="Create Task">
                  <AddOutlinedIcon />
                </IconButton>
              </Tooltip>
            </Toolbar>
          }
          {...otherProps}
        >
          <Grid container wrap="nowrap" direction="column" height="100%">
            <Grid item flexGrow={1}>
              <TableContainer>
                <TaskDataGridTable
                  tasks={tasksState}
                  onTaskClick={(_ev, task) => AppEvents.taskSelect.next(task)}
                  setFilterFields={setFilterFields}
                  setSortFields={setSortFields}
                  onPageChange={(newPage: number) =>
                    setTasksState((old) => ({ ...old, page: newPage + 1 }))
                  }
                  onPageSizeChange={(newPageSize: number) =>
                    setTasksState((old) => ({ ...old, pageSize: newPageSize }))
                  }
                />
              </TableContainer>
            </Grid>

            <Grid
              item
              // react-scheduler does not support disabling clicking calendar to add events.
              // Workaround by disabling pointer events.
              sx={{
                '& .rs__cell': { pointerEvents: 'none' },
                '& .rs__event__item': { pointerEvents: 'auto' },
              }}
            >
              <Scheduler
                view="week"
                editable={false}
                disableViewNavigator
                getRemoteEvents={getRemoteEvents}
                onDelete={async (deletedId) => {
                  const task = eventsMap.current[Number(deletedId)];
                  if (!task) {
                    console.error(
                      `Failed to delete scheduled task: unable to find task for event ${deletedId}`,
                    );
                    return;
                  }
                  if (!rmf) {
                    return;
                  }
                  try {
                    await rmf.tasksApi.delScheduledTasksScheduledTasksTaskIdDelete(task.id);
                    setForceRefresh((prev) => prev + 1);
                  } catch (e) {
                    console.error(`Failed to delete scheduled task: ${e}`);
                  }
                }}
              />
            </Grid>
          </Grid>
          {openCreateTaskForm && (
            <CreateTaskForm
              cleaningZones={placeNames}
              loopWaypoints={placeNames}
              deliveryWaypoints={placeNames}
              dispensers={workcells}
              ingestors={workcells}
              favoritesTasks={favoritesTasks}
              open={openCreateTaskForm}
              onClose={() => setOpenCreateTaskForm(false)}
              submitTasks={submitTasks}
              submitFavoriteTask={submitFavoriteTask}
              deleteFavoriteTask={deleteFavoriteTask}
              tasksFromFile={tasksFromFile}
              onSuccess={() => {
                setOpenCreateTaskForm(false);
                showAlert('success', 'Successfully created task');
              }}
              onFail={(e) => {
                showAlert('error', `Failed to create task: ${e.message}`);
              }}
              onSuccessFavoriteTask={(message) => {
                showAlert('success', message);
              }}
              onFailFavoriteTask={(e) => {
                showAlert('error', `Failed to create or delete favorite task: ${e.message}`);
              }}
            />
          )}
          <input type="file" style={{ display: 'none' }} ref={uploadFileInputRef} />
          {children}
        </Window>
      );
    },
  ),
);

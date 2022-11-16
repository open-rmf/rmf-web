import AddOutlinedIcon from '@mui/icons-material/AddOutlined';
import RefreshIcon from '@mui/icons-material/Refresh';
import { Grid, IconButton, TableContainer, Toolbar, Tooltip } from '@mui/material';
import { TaskRequest, TaskState } from 'api-client';
import React from 'react';
import {
  CreateTaskForm,
  CreateTaskFormProps,
  getPlaces,
  Window,
  TaskDataGridTable,
  Tasks,
  FilterFields,
} from 'react-components';
import { AppControllerContext, ResourcesContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { MicroAppProps } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { parseTasksFile } from './utils';

export const TasksApp = React.memo(
  React.forwardRef(
    (
      { onClose, children, ...otherProps }: React.PropsWithChildren<MicroAppProps>,
      ref: React.Ref<HTMLDivElement>,
    ) => {
      const rmf = React.useContext(RmfAppContext);

      const [forceRefresh, setForceRefresh] = React.useState(0);

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

      const [filterFields, setFilterFields] = React.useState<FilterFields>({
        category: undefined,
        taskId: undefined,
        startTime: undefined,
        finisTime: undefined,
      });

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

        (async () => {
          const resp = await rmf.tasksApi.queryTaskStatesTasksGet(
            filterFields.taskId,
            filterFields.category,
            filterFields.startTime,
            filterFields.finisTime,
            GET_LIMIT,
            (tasksState.page - 1) * GET_LIMIT, // Datagrid component need to start in page 1. Otherwise works wrong
            undefined,
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
        })();
      }, [
        rmf,
        forceRefresh,
        tasksState.page,
        filterFields.taskId,
        filterFields.category,
        filterFields.finisTime,
        filterFields.startTime,
      ]);

      const submitTasks = React.useCallback<Required<CreateTaskFormProps>['submitTasks']>(
        async (taskRequests) => {
          if (!rmf) {
            throw new Error('tasks api not available');
          }
          await Promise.all(
            taskRequests.map((taskReq) =>
              rmf.tasksApi.postDispatchTaskTasksDispatchTaskPost({
                type: 'dispatch_task_request',
                request: taskReq,
              }),
            ),
          );
          setForceRefresh((prev) => prev + 1);
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
              <Tooltip title="Refresh" color="inherit">
                <IconButton
                  onClick={() => {
                    setForceRefresh((prev) => prev + 1);
                    setFilterFields((old) => ({
                      ...old,
                      category: undefined,
                      startTime: undefined,
                      finisTime: undefined,
                      taskId: undefined,
                    }));
                  }}
                  aria-label="Refresh"
                >
                  <RefreshIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="Create task" color="inherit">
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
                  onPageChange={(newPage: number) =>
                    setTasksState((old) => ({ ...old, page: newPage + 1 }))
                  }
                  onPageSizeChange={(newPageSize: number) =>
                    setTasksState((old) => ({ ...old, pageSize: newPageSize }))
                  }
                />
              </TableContainer>
            </Grid>
          </Grid>
          {openCreateTaskForm && (
            <CreateTaskForm
              cleaningZones={placeNames}
              loopWaypoints={placeNames}
              deliveryWaypoints={placeNames}
              dispensers={workcells}
              ingestors={workcells}
              open={openCreateTaskForm}
              onClose={() => setOpenCreateTaskForm(false)}
              submitTasks={submitTasks}
              tasksFromFile={tasksFromFile}
              onSuccess={() => {
                setOpenCreateTaskForm(false);
                showAlert('success', 'Successfully created task');
              }}
              onFail={(e) => {
                showAlert('error', `Failed to create task: ${e.message}`);
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

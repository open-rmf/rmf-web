import AddOutlinedIcon from '@mui/icons-material/AddOutlined';
import RefreshIcon from '@mui/icons-material/Refresh';
import { Grid, IconButton, TableContainer, TablePagination, Toolbar, Tooltip } from '@mui/material';
import { DataGrid, GridColDef, GridValueGetterParams } from '@mui/x-data-grid';
import { TaskRequest, TaskState } from 'api-client';
import React from 'react';
import {
  CreateTaskForm,
  CreateTaskFormProps,
  getPlaces,
  TaskTable,
  Window,
} from 'react-components';
import { Subscription } from 'rxjs';
import { AppControllerContext, ResourcesContext } from '../app-contexts';
import { AppEvents } from '../app-events';
import { MicroAppProps } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { parseTasksFile } from './utils';

const columns: GridColDef[] = [
  {
    field: 'id',
    headerName: 'ID',
    width: 90,
    valueGetter: (params: GridValueGetterParams) => params.row.booking.id,
  },
  {
    field: 'category',
    headerName: 'Category',
    width: 150,
    editable: false,
  },
  {
    field: 'name',
    headerName: 'Assignee',
    width: 150,
    editable: true,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.assigned_to ? params.row.assigned_to.name : 'unknown',
  },
  {
    field: 'status',
    headerName: 'State',
    width: 150,
    editable: true,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.status ? params.row.status : 'unknown',
  },
];

export const TasksApp = React.memo(
  React.forwardRef(
    (
      { onClose, children, ...otherProps }: React.PropsWithChildren<MicroAppProps>,
      ref: React.Ref<HTMLDivElement>,
    ) => {
      const rmf = React.useContext(RmfAppContext);

      const [forceRefresh, setForceRefresh] = React.useState(0);
      const [taskStates, setTaskStates] = React.useState<Record<string, TaskState>>({});

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

      const [page, setPage] = React.useState(1);
      const [hasMore, setHasMore] = React.useState(true);

      const [pageState, setPageState] = React.useState({
        isLoading: false,
        data: [] as TaskState[],
        total: 0,
        page: 1,
        pageSize: 10,
      });

      /**
       * This state, if toggled, reverts the chronological order of the tasks listed in the tasks table.
       */
      const [chronologicalOrder, setChronologicalOrder] = React.useState<boolean>(true);

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

      React.useEffect(() => {
        if (!rmf) {
          return;
        }
        const subs: Subscription[] = [];
        (async () => {
          const resp = await rmf.tasksApi.queryTaskStatesTasksGet(
            undefined,
            undefined,
            undefined,
            undefined,
            undefined,
            page * 10,
            chronologicalOrder ? '-unix_millis_start_time' : 'unix_millis_start_time',
            undefined,
          );
          const results = resp.data as TaskState[];
          setHasMore(results.length > 10);
          const newTasks = results.slice(0, 10);

          setPageState((old) => ({
            ...old,
            isLoading: false,
            data: results,
            total: results.length,
          }));

          setTaskStates(
            newTasks.reduce<Record<string, TaskState>>((acc, task) => {
              acc[task.booking.id] = task;
              return acc;
            }, {}),
          );
          subs.push(
            ...newTasks.map((task) =>
              rmf
                .getTaskStateObs(task.booking.id)
                .subscribe((task) =>
                  setTaskStates((prev) => ({ ...prev, [task.booking.id]: task })),
                ),
            ),
          );
        })();
        return () => subs.forEach((s) => s.unsubscribe());
      }, [rmf, page, forceRefresh, chronologicalOrder, pageState.page, pageState.pageSize]);

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
                  onClick={() => setForceRefresh((prev) => prev + 1)}
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
                <div style={{ height: '100%', width: '100%' }}>
                  <DataGrid
                    autoHeight
                    getRowId={(r) => r.booking.id}
                    rows={pageState.data}
                    rowCount={pageState.total}
                    loading={pageState.isLoading}
                    rowsPerPageOptions={[10, 30, 50, 70, 100]}
                    pagination
                    page={pageState.page - 1}
                    pageSize={pageState.pageSize}
                    // paginationMode="server"
                    onPageChange={(newPage) => {
                      setPageState((old) => ({ ...old, page: newPage + 1 }));
                    }}
                    onPageSizeChange={(newPageSize) =>
                      setPageState((old) => ({ ...old, pageSize: newPageSize }))
                    }
                    columns={columns}
                  />
                </div>
                {/* <TaskTableDataGrid
                  tasks={Object.values(taskStates)}
                  onTaskClick={(_ev, task) => AppEvents.taskSelect.next(task)}
                  onDateTitleClick={(_ev) => setChronologicalOrder((prev) => !prev)}
                  chronologicalOrder={chronologicalOrder}
                  addMoreRows={setPage}
                  page={page}
                /> */}
                {/* <TaskTable
                  tasks={Object.values(taskStates)}
                  onTaskClick={(_ev, task) => AppEvents.taskSelect.next(task)}
                  onDateTitleClick={(_ev) => setChronologicalOrder((prev) => !prev)}
                  chronologicalOrder={chronologicalOrder}
                /> */}
              </TableContainer>
            </Grid>
            {/* <Grid item>
              <TablePagination
                component="div"
                page={page}
                count={hasMore ? -1 : page * 10 + Object.keys(taskStates).length}
                rowsPerPage={10}
                rowsPerPageOptions={[10]}
                onPageChange={(_ev, page) => setPage(page)}
                style={{ flex: '0 0 auto' }}
              />
            </Grid> */}
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

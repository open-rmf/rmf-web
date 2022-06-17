import AddOutlinedIcon from '@mui/icons-material/AddOutlined';
import AutorenewIcon from '@mui/icons-material/Autorenew';
import RefreshIcon from '@mui/icons-material/Refresh';
import {
  Alert,
  AlertProps,
  Grid,
  IconButton,
  Snackbar,
  TableContainer,
  TablePagination,
  Toolbar,
  Tooltip,
  useTheme,
} from '@mui/material';
import { TaskRequest, TaskState } from 'api-client';
import React from 'react';
import {
  CreateTaskForm,
  CreateTaskFormProps,
  getPlaces,
  TaskTable,
  Window,
} from 'react-components';
import { AppControllerContext, ResourcesContext } from './app-contexts';
import { AppEvents } from './app-events';
import { MicroAppProps } from './micro-app';
import { RmfAppContext } from './rmf-app';
import { parseTasksFile } from './tasks/utils';

export interface TaskPanelProps
  extends React.DetailedHTMLProps<React.HTMLAttributes<HTMLDivElement>, HTMLDivElement> {
  /**
   * Should only contain the tasks of the current page.
   */
  tasks: TaskState[];
  paginationOptions?: Omit<React.ComponentPropsWithoutRef<typeof TablePagination>, 'component'>;
  cleaningZones?: string[];
  loopWaypoints?: string[];
  deliveryWaypoints?: string[];
  dispensers?: string[];
  ingestors?: string[];
  submitTasks?: CreateTaskFormProps['submitTasks'];
  cancelTask?: (task: TaskState) => Promise<void>;
  onRefresh?: () => void;
  onAutoRefresh?: (enabled: boolean) => void;
}

export const TasksApp = React.memo(
  React.forwardRef(
    (
      { onClose, children, ...otherProps }: React.PropsWithChildren<MicroAppProps>,
      ref: React.Ref<HTMLDivElement>,
    ) => {
      const theme = useTheme();
      const rmf = React.useContext(RmfAppContext);

      const [fetchedTasks, setFetchedTasks] = React.useState<TaskState[]>([]);
      const [latestTasks, setLatestTasks] = React.useState<Record<string, TaskState>>({});

      const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
      const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
      const [openSnackbar, setOpenSnackbar] = React.useState(false);
      const [snackbarMessage, setSnackbarMessage] = React.useState('');
      const [snackbarSeverity, setSnackbarSeverity] =
        React.useState<AlertProps['severity']>('success');
      const { showErrorAlert } = React.useContext(AppControllerContext);

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
                showErrorAlert((err as Error).message, 5000);
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

      const [autoRefresh, setAutoRefresh] = React.useState(true);
      const autoRefreshTooltipPrefix = autoRefresh ? 'Disable' : 'Enable';

      const [page, setPage] = React.useState(0);
      const [hasMore, setHasMore] = React.useState(true);

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

      const handleRefresh = React.useCallback(
        async (page: number) => {
          if (!rmf) {
            return [];
          }
          const resp = await rmf.tasksApi.queryTaskStatesTasksGet(
            undefined,
            undefined,
            undefined,
            undefined,
            11,
            page * 10,
            '-unix_millis_start_time',
            undefined,
          );
          const results = resp.data as TaskState[];
          setHasMore(results.length > 10);
          const newTasks = results.slice(0, 10);
          setFetchedTasks(newTasks);
          setLatestTasks(
            newTasks.reduce<Record<string, TaskState>>((acc, task) => {
              acc[task.booking.id] = task;
              return acc;
            }, {}),
          );
        },
        [rmf],
      );

      React.useEffect(() => {
        if (!autoRefresh || !rmf) {
          return;
        }
        for (const task of fetchedTasks) {
          rmf
            .getTaskStateObs(task.booking.id)
            .subscribe((state) =>
              setLatestTasks((prev) => ({ ...prev, [state.booking.id]: state })),
            );
        }
      }, [rmf, autoRefresh, fetchedTasks]);

      const submitTasks = React.useCallback<Required<TaskPanelProps>['submitTasks']>(
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
          handleRefresh(page);
        },
        [rmf, page, handleRefresh],
      );

      return (
        <Window
          ref={ref}
          title="Tasks"
          onClose={onClose}
          toolbar={
            <Toolbar variant="dense">
              <Tooltip title={`${autoRefreshTooltipPrefix} auto refresh`} color="inherit">
                <IconButton
                  sx={{ background: autoRefresh ? theme.palette.action.selected : undefined }}
                  onClick={() => setAutoRefresh((prev) => !prev)}
                  aria-label={`${autoRefreshTooltipPrefix} auto refresh`}
                >
                  <AutorenewIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="Refresh" color="inherit">
                <IconButton onClick={() => handleRefresh(page)} aria-label="Refresh">
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
                <TaskTable
                  tasks={Object.values(latestTasks)}
                  onTaskClick={(_ev, task) => AppEvents.taskSelect.next(task)}
                />
              </TableContainer>
            </Grid>
            <Grid item>
              <TablePagination
                component="div"
                page={page}
                count={hasMore ? -1 : page * 10 + fetchedTasks.length}
                rowsPerPage={10}
                rowsPerPageOptions={[10]}
                onPageChange={(_ev, page) => setPage(page)}
                style={{ flex: '0 0 auto' }}
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
              open={openCreateTaskForm}
              onClose={() => setOpenCreateTaskForm(false)}
              submitTasks={submitTasks}
              tasksFromFile={tasksFromFile}
              onSuccess={() => {
                setOpenCreateTaskForm(false);
                setSnackbarSeverity('success');
                setSnackbarMessage('Successfully created task');
                setOpenSnackbar(true);
              }}
              onFail={(e) => {
                setSnackbarSeverity('error');
                setSnackbarMessage(`Failed to create task: ${e.message}`);
                setOpenSnackbar(true);
              }}
            />
          )}
          <input type="file" style={{ display: 'none' }} ref={uploadFileInputRef} />
          <Snackbar
            open={openSnackbar}
            onClose={() => setOpenSnackbar(false)}
            autoHideDuration={2000}
          >
            <Alert severity={snackbarSeverity}>{snackbarMessage}</Alert>
          </Snackbar>
          {children}
        </Window>
      );
    },
  ),
);

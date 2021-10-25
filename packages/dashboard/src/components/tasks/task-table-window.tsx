/* istanbul ignore file */

import {
  Box,
  IconButton,
  makeStyles,
  TableContainer,
  TablePagination,
  Tooltip,
} from '@material-ui/core';
import {
  AddOutlined as AddOutlinedIcon,
  Autorenew as AutorenewIcon,
  Refresh as RefreshIcon,
} from '@material-ui/icons';
import { Dispenser, Ingestor, SubmitTask, Task, TaskSummary } from 'api-client';
import React from 'react';
import {
  CreateTaskForm,
  CreateTaskFormProps,
  getPlaces,
  Place,
  TaskTable,
  useAsync,
  Window,
} from 'react-components';
import * as RmfModels from 'rmf-models';
import { throttleTime } from 'rxjs';
import { AppControllerContext } from '../app-contexts';
import { RmfIngressContext, RxRmfContext } from '../rmf-app';
import { ManagedWindowProps } from '../window';
import { parseTasksFile } from './utils';

const useStyles = makeStyles((theme) => ({
  taskPanel: {
    padding: `${theme.spacing(4)}px`,
    height: '100%',
    backgroundColor: theme.palette.background.default,
    maxWidth: 1600,
  },
  enabledToggleButton: {
    background: theme.palette.action.selected,
  },
}));

export const TaskTableWindow: React.FC<ManagedWindowProps> = React.forwardRef(
  ({ ...otherProps }, ref) => {
    const classes = useStyles();
    const safeAsync = useAsync();
    const { buildingApi, tasksApi, dispensersApi, ingestorsApi } =
      React.useContext(RmfIngressContext) || {};
    const rxRmf = React.useContext(RxRmfContext);
    const { showErrorAlert, showSuccessAlert } = React.useContext(AppControllerContext);

    const [places, setPlaces] = React.useState<Place[] | null>(null);
    React.useEffect(() => {
      if (!buildingApi) return;
      (async () => {
        const buildingMap = (await safeAsync(buildingApi.getBuildingMapBuildingMapGet()))
          .data as RmfModels.BuildingMap;
        setPlaces(getPlaces(buildingMap));
      })();
    }, [safeAsync, buildingApi]);
    const placeNames = React.useMemo(() => (places ? places.map((p) => p.vertex.name) : null), [
      places,
    ]);

    const [fetchedTasks, setFetchedTasks] = React.useState<Task[]>([]);
    const [updatedSummaries, setUpdatedSummaries] = React.useState<Record<string, TaskSummary>>({});
    const [page, setPage] = React.useState(0);
    const [hasMore, setHasMore] = React.useState(true);
    const tasks = React.useMemo(
      () => fetchedTasks.map((t) => ({ ...t, summary: updatedSummaries[t.task_id] || t.summary })),
      [fetchedTasks, updatedSummaries],
    );
    const taskSummaries = React.useMemo<RmfModels.TaskSummary[]>(
      () => tasks.map((t) => t.summary as RmfModels.TaskSummary),
      [tasks],
    );

    const fetchTasks = React.useCallback(
      async (page: number) => {
        if (!tasksApi) {
          return [];
        }
        const resp = await tasksApi.getTasksTasksGet(
          undefined,
          undefined,
          undefined,
          undefined,
          undefined,
          undefined,
          undefined,
          undefined,
          undefined,
          11,
          page * 10,
          '-priority,-start_time',
        );
        const results = resp.data as Task[];
        setHasMore(results.length > 10);
        setFetchedTasks(results.slice(0, 10));
      },
      [tasksApi],
    );

    const refresh = React.useCallback(async () => {
      fetchTasks(page);
    }, [fetchTasks, page]);

    React.useEffect(() => {
      refresh();
    }, [refresh]);

    const [autoRefreshEnabled, setAutoRefreshEnabled] = React.useState(true);
    const autoRefreshTooltipPrefix = autoRefreshEnabled ? 'Disable' : 'Enable';
    React.useEffect(() => {
      if (!autoRefreshEnabled || !rxRmf) return;
      const subs = fetchedTasks.map((t) =>
        rxRmf
          .taskSummaries(t.task_id)
          .pipe(throttleTime(1000))
          .subscribe(
            (newSummary) =>
              newSummary &&
              setUpdatedSummaries((prev) => ({
                ...prev,
                [newSummary.task_id]: newSummary,
              })),
          ),
      );
      return () => {
        subs.forEach((s) => s.unsubscribe());
      };
    }, [autoRefreshEnabled, rxRmf, fetchedTasks]);

    const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
    const submitTasks = React.useCallback<Required<CreateTaskFormProps>['submitTasks']>(
      async (tasks) => {
        if (!tasksApi) {
          showErrorAlert('Failed to submit tasks: server not available');
          return;
        }
        await Promise.all(tasks.map((t) => tasksApi.submitTaskTasksSubmitTaskPost(t)));
        refresh();
      },
      [tasksApi, refresh, showErrorAlert],
    );

    const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
    /* istanbul ignore next */
    const tasksFromFile = (): Promise<SubmitTask[]> => {
      return new Promise((res) => {
        const fileInputEl = uploadFileInputRef.current;
        if (!fileInputEl) {
          return [];
        }
        let taskFiles: SubmitTask[];
        const listener = async () => {
          try {
            if (!fileInputEl.files || fileInputEl.files.length === 0) {
              return res([]);
            }
            try {
              taskFiles = parseTasksFile(await fileInputEl.files[0].text());
            } catch (err) {
              showErrorAlert(err.message);
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

    const [dispensers, setDispensers] = React.useState<Dispenser[]>([]);
    React.useEffect(() => {
      if (!dispensersApi) return;
      (async () => {
        setDispensers((await safeAsync(dispensersApi.getDispensersDispensersGet())).data);
      })();
    }, [safeAsync, dispensersApi]);
    const dispenserGuids = React.useMemo(() => dispensers.map((d) => d.guid), [dispensers]);

    const [ingestors, setIngestors] = React.useState<Ingestor[]>([]);
    React.useEffect(() => {
      if (!ingestorsApi) return;
      (async () => {
        setIngestors((await safeAsync(ingestorsApi.getIngestorsIngestorsGet())).data);
      })();
    }, [safeAsync, ingestorsApi]);
    const ingestorGuids = React.useMemo(() => ingestors.map((i) => i.guid), [ingestors]);

    return (
      placeNames && (
        <Window
          ref={ref}
          title="Tasks"
          toolbar={
            <Box>
              <Tooltip title={`${autoRefreshTooltipPrefix} auto refresh`}>
                <IconButton
                  className={autoRefreshEnabled ? classes.enabledToggleButton : undefined}
                  onClick={() => {
                    setAutoRefreshEnabled((prev) => !prev);
                  }}
                  aria-label={`${autoRefreshTooltipPrefix} auto refresh`}
                >
                  <AutorenewIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="Refresh">
                <IconButton onClick={() => refresh()} aria-label="Refresh">
                  <RefreshIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="Create task">
                <IconButton onClick={() => setOpenCreateTaskForm(true)} aria-label="Create Task">
                  <AddOutlinedIcon />
                </IconButton>
              </Tooltip>
            </Box>
          }
          {...otherProps}
        >
          <Box>
            <TableContainer>
              <TaskTable
                // className={classes.taskPanel}
                tasks={taskSummaries}
              />
            </TableContainer>
            <TablePagination
              component="div"
              page={page}
              count={hasMore ? -1 : page * 10 + tasks.length}
              rowsPerPage={10}
              rowsPerPageOptions={[10]}
              onChangePage={(_ev, newPage) => setPage(newPage)}
              style={{ flex: '0 0 auto' }}
            />
          </Box>
          {openCreateTaskForm && (
            <CreateTaskForm
              cleaningZones={placeNames}
              loopWaypoints={placeNames}
              deliveryWaypoints={placeNames}
              dispensers={dispenserGuids}
              ingestors={ingestorGuids}
              open={openCreateTaskForm}
              onClose={() => setOpenCreateTaskForm(false)}
              submitTasks={submitTasks}
              tasksFromFile={tasksFromFile}
              onSuccess={() => {
                showSuccessAlert('Successfully created task');
                setOpenCreateTaskForm(false);
              }}
              onFail={(e) => {
                showErrorAlert(`Failed to create task: ${e.message}`);
              }}
            />
          )}
        </Window>
      )
    );
  },
);

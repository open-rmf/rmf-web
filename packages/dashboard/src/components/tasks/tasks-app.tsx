import DownloadIcon from '@mui/icons-material/Download';
import RefreshIcon from '@mui/icons-material/Refresh';
import {
  Box,
  IconButton,
  Menu,
  MenuItem,
  styled,
  Tab,
  Tabs,
  TableContainer,
  Toolbar,
  Tooltip,
  useMediaQuery,
} from '@mui/material';
import {
  ApiServerModelsTortoiseModelsTasksTaskStateLeaf as TaskQueueEntry,
  TaskRequest,
  TaskState,
} from 'api-client';
import React from 'react';
import {
  FilterFields,
  MuiMouseEvent,
  SortFields,
  TaskDataGridTable,
  Tasks,
  Window,
} from 'react-components';
import { AppEvents } from '../app-events';
import { MicroAppProps } from '../micro-app';
import { RmfAppContext } from '../rmf-app';
import { TaskSchedule } from './task-schedule';
import { TaskSummary } from './task-summary';
import { downloadCsvFull, downloadCsvMinimal } from './utils';

const RefreshTaskQueueTableInterval = 30000;

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
  // Removing top padding for Schedule as there is too much whitespace after
  // the day-week-month view has been shifted to the right.
  return (
    <div
      role="tabpanel"
      hidden={selectedTabIndex !== index}
      id={tabPanelId(index)}
      aria-labelledby={tabId(index)}
      {...other}
    >
      {selectedTabIndex === index && (
        <Box
          component="div"
          sx={{ p: 3, pt: selectedTabIndex === TaskTablePanel.Schedule ? 0 : 3 }}
        >
          {children}
        </Box>
      )}
    </div>
  );
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
      const [selectedPanelIndex, setSelectedPanelIndex] = React.useState(TaskTablePanel.QueueTable);

      const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
      const [openTaskSummary, setOpenTaskSummary] = React.useState(false);
      const [selectedTask, setSelectedTask] = React.useState<TaskState | null>(null);
      const [taskQueueTableData, setTaskQueueTableData] = React.useState<Tasks>({
        isLoading: true,
        entries: [],
        requests: {},
        total: 0,
        page: 1,
        pageSize: 10,
      });
      const [filterFields, setFilterFields] = React.useState<FilterFields>({ model: undefined });
      const [sortFields, setSortFields] = React.useState<SortFields>({ model: undefined });

      const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
      const classes = {
        typography: 'MuiTypography-root',
        button: 'MuiButton-text',
      };
      const StyledDiv = styled('div')(({ theme }) => ({
        [`& .${classes.typography}`]: {
          fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
        },
        [`& .${classes.button}`]: {
          fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
        },
      }));

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

        if (selectedPanelIndex !== TaskTablePanel.QueueTable) {
          console.debug('Stop subscribing to task queue updates when viewing schedule tab');
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

        (async () => {
          const resp = await rmf.tasksApi.queryTaskQueueEntryTasksQueueEntryGet(
            filterColumn && filterColumn === 'id_' ? filterValue : undefined,
            filterColumn && filterColumn === 'category' ? filterValue : undefined,
            filterColumn && filterColumn === 'requester' ? filterValue : undefined,
            filterColumn && filterColumn === 'pickup' ? filterValue : undefined,
            filterColumn && filterColumn === 'destination' ? filterValue : undefined,
            filterColumn && filterColumn === 'assigned_to' ? filterValue : undefined,
            filterColumn && filterColumn === 'status' ? filterValue : undefined,
            filterColumn && filterColumn === 'unix_millis_request_time' ? filterValue : undefined,
            filterColumn && filterColumn === 'unix_millis_start_time' ? filterValue : undefined,
            filterColumn && filterColumn === 'unix_millis_finish_time' ? filterValue : undefined,
            GET_LIMIT,
            (taskQueueTableData.page - 1) * GET_LIMIT, // Datagrid component need to start in page 1. Otherwise works wrong
            orderBy,
            undefined,
          );
          const results = resp.data as TaskQueueEntry[];
          const newTasks = results.slice(0, GET_LIMIT);

          // NOTE(ac): we are not using getAllTaskRequests here to prevent
          // adding it into the dependency list.
          const taskIds: string[] = newTasks.map((task) => task.id_);
          const taskIdsQuery = taskIds.join(',');
          const taskRequests = (await rmf.tasksApi.queryTaskRequestsTasksRequestsGet(taskIdsQuery))
            .data;

          const taskRequestMap: Record<string, TaskRequest> = {};
          let requestIndex = 0;
          for (const id of taskIds) {
            if (requestIndex < taskRequests.length && taskRequests[requestIndex]) {
              taskRequestMap[id] = taskRequests[requestIndex];
            }
            ++requestIndex;
          }

          setTaskQueueTableData((old) => ({
            ...old,
            isLoading: false,
            entries: newTasks,
            requests: taskRequestMap,
            total:
              results.length === GET_LIMIT
                ? taskQueueTableData.page * GET_LIMIT + 1
                : taskQueueTableData.page * GET_LIMIT - 9,
          }));
        })();
      }, [
        rmf,
        refreshTaskAppCount,
        taskQueueTableData.page,
        filterFields.model,
        sortFields.model,
        selectedPanelIndex,
      ]);

      const getAllTasks = async (timestamp: Date) => {
        if (!rmf) {
          return [];
        }

        const resp = await rmf.tasksApi.queryTaskStatesTasksGet(
          undefined,
          undefined,
          undefined,
          undefined,
          undefined,
          undefined,
          undefined,
          `0,${timestamp.getTime()}`,
          undefined,
          undefined,
          undefined,
          undefined,
          '-unix_millis_start_time',
          undefined,
        );
        const allTasks = resp.data as TaskState[];
        return allTasks;
      };

      const getAllTaskRequests = async (tasks: TaskState[]) => {
        if (!rmf) {
          return {};
        }

        const taskIds: string[] = tasks.map((task) => task.booking.id);
        const taskIdsQuery = taskIds.join(',');
        const taskRequests = (await rmf.tasksApi.queryTaskRequestsTasksRequestsGet(taskIdsQuery))
          .data;

        const taskRequestMap: Record<string, TaskRequest> = {};
        let requestIndex = 0;
        for (const id of taskIds) {
          if (requestIndex < taskRequests.length && taskRequests[requestIndex]) {
            taskRequestMap[id] = taskRequests[requestIndex];
          }
          ++requestIndex;
        }
        return taskRequestMap;
      };

      const exportTasksToCsv = async (minimal: boolean) => {
        const now = new Date();
        const allTasks = await getAllTasks(now);
        const allTaskRequests = await getAllTaskRequests(allTasks);
        if (!allTasks || !allTasks.length) {
          return;
        }
        if (minimal) {
          downloadCsvMinimal(now, allTasks, allTaskRequests);
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

      const handlePanelChange = (_: React.SyntheticEvent, newSelectedTabIndex: number) => {
        setSelectedPanelIndex(newSelectedTabIndex);
        setAutoRefresh(newSelectedTabIndex === TaskTablePanel.QueueTable);
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
                    sx={{ marginBottom: isScreenHeightLessThan800 ? 1.8 : 0 }}
                    id="export-button"
                    aria-controls={openExportMenu ? 'export-menu' : undefined}
                    aria-haspopup="true"
                    aria-expanded={openExportMenu ? 'true' : undefined}
                    onClick={handleClickExportMenu}
                    color="inherit"
                  >
                    <DownloadIcon transform={`scale(${isScreenHeightLessThan800 ? 0.8 : 1})`} />
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
                  sx={{ marginBottom: isScreenHeightLessThan800 ? 1.8 : 0 }}
                  onClick={() => {
                    AppEvents.refreshTaskApp.next();
                  }}
                  aria-label="Refresh"
                >
                  <RefreshIcon transform={`scale(${isScreenHeightLessThan800 ? 0.8 : 1})`} />
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
              sx={{
                fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
              }}
            />
            <Tab
              label="Schedule"
              id={tabId(TaskTablePanel.Schedule)}
              aria-controls={tabPanelId(TaskTablePanel.Schedule)}
              sx={{
                fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
              }}
            />
          </Tabs>
          <TabPanel selectedTabIndex={selectedPanelIndex} index={TaskTablePanel.QueueTable}>
            <TableContainer>
              <TaskDataGridTable
                tasks={taskQueueTableData}
                onTaskClick={async (_: MuiMouseEvent, task: TaskQueueEntry) => {
                  if (!rmf) {
                    console.error('Task API not available');
                    return;
                  }
                  const state = (await rmf.tasksApi.getTaskStateTasksTaskIdStateGet(task.id_)).data;
                  setSelectedTask(state);
                  if (state.assigned_to) {
                    AppEvents.robotSelect.next([state.assigned_to.group, state.assigned_to.name]);
                  }
                  setOpenTaskSummary(true);
                }}
                setFilterFields={setFilterFields}
                setSortFields={setSortFields}
                onPageChange={(newPage: number) =>
                  setTaskQueueTableData((old: Tasks) => ({ ...old, page: newPage + 1 }))
                }
                onPageSizeChange={(newPageSize: number) =>
                  setTaskQueueTableData((old: Tasks) => ({ ...old, pageSize: newPageSize }))
                }
              />
            </TableContainer>
          </TabPanel>
          <TabPanel selectedTabIndex={selectedPanelIndex} index={TaskTablePanel.Schedule}>
            <StyledDiv>
              <TaskSchedule />
            </StyledDiv>
          </TabPanel>
          <input type="file" style={{ display: 'none' }} ref={uploadFileInputRef} />
          {openTaskSummary && (
            <TaskSummary
              onClose={() => setOpenTaskSummary(false)}
              task={selectedTask ?? undefined}
              request={
                selectedTask ? taskQueueTableData.requests[selectedTask.booking.id] : undefined
              }
            />
          )}
          {children}
        </Window>
      );
    },
  ),
);

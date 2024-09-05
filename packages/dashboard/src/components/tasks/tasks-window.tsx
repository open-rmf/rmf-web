import DownloadIcon from '@mui/icons-material/Download';
import RefreshIcon from '@mui/icons-material/Refresh';
import {
  Box,
  Button,
  Grid,
  Menu,
  MenuItem,
  styled,
  Tab,
  TableContainer,
  Tabs,
  Tooltip,
  useMediaQuery,
} from '@mui/material';
import { TaskStateInput as TaskState } from 'api-client';
import React from 'react';
import {
  FilterFields,
  MuiMouseEvent,
  SortFields,
  TaskDataGridTable,
  Tasks,
  Window,
} from 'react-components';

import { useAppController } from '../../hooks/use-app-controller';
import { useRmfApi } from '../../hooks/use-rmf-api';
import { AppEvents } from '../app-events';
import { MicroAppProps } from '../micro-app';
import { TaskSchedule } from './task-schedule';
import { TaskSummary } from './task-summary';
import { exportCsvFull, exportCsvMinimal } from './utils';

const RefreshTaskQueueTableInterval = 15000;
const QueryLimit = 100;

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
    index === selectedTabIndex && (
      <Box
        role="tabpanel"
        id={tabPanelId(index)}
        aria-labelledby={tabId(index)}
        flexGrow={1}
        display="flex"
        {...other}
      >
        {selectedTabIndex === index && (
          <Box sx={{ p: 3, pt: selectedTabIndex === TaskTablePanel.Schedule ? 0 : 3 }} flexGrow={1}>
            {children}
          </Box>
        )}
      </Box>
    )
  );
}

export const TasksWindow = React.memo(
  React.forwardRef(
    (
      { onClose, children, ...otherProps }: React.PropsWithChildren<MicroAppProps>,
      ref: React.Ref<HTMLDivElement>,
    ) => {
      const rmfApi = useRmfApi();
      const appController = useAppController();
      const [autoRefresh, setAutoRefresh] = React.useState(true);
      const [refreshTaskAppCount, setRefreshTaskAppCount] = React.useState(0);
      const [selectedPanelIndex, setSelectedPanelIndex] = React.useState(TaskTablePanel.QueueTable);

      const uploadFileInputRef = React.useRef<HTMLInputElement>(null);
      const [openTaskSummary, setOpenTaskSummary] = React.useState(false);
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

      const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
      const classes = {
        typography: 'MuiTypography-root',
        button: 'MuiButton-text',
      };
      const StyledDiv = styled('div')(() => ({
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
          let labelFilter: string | undefined = undefined;
          if (filterColumn && filterColumn.startsWith('label=')) {
            labelFilter = `${filterColumn.substring(6)}=${filterValue}`;
          }

          const resp = await rmfApi.tasksApi.queryTaskStatesTasksGet(
            filterColumn && filterColumn === 'id_' ? filterValue : undefined,
            filterColumn && filterColumn === 'category' ? filterValue : undefined,
            filterColumn && filterColumn === 'requester' ? filterValue : undefined,
            filterColumn && filterColumn === 'assigned_to' ? filterValue : undefined,
            filterColumn && filterColumn === 'status' ? filterValue : undefined,
            labelFilter,
            filterColumn && filterColumn === 'unix_millis_request_time' ? filterValue : undefined,
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
        })();
      }, [
        rmfApi,
        refreshTaskAppCount,
        tasksState.page,
        filterFields.model,
        sortFields.model,
        selectedPanelIndex,
      ]);

      const getPastMonthTasks = async (timestamp: Date) => {
        const currentMillis = timestamp.getTime();
        const oneMonthMillis = 31 * 24 * 60 * 60 * 1000;
        const allTasks: TaskState[] = [];
        let queries: TaskState[] = [];
        let queryIndex = 0;
        do {
          queries = (
            await rmfApi.tasksApi.queryTaskStatesTasksGet(
              undefined,
              undefined,
              undefined,
              undefined,
              undefined,
              undefined,
              undefined,
              `${currentMillis - oneMonthMillis},${currentMillis}`,
              undefined,
              QueryLimit,
              queryIndex * QueryLimit,
              '-unix_millis_start_time',
              undefined,
            )
          ).data;
          if (queries.length === 0) {
            break;
          }

          allTasks.push(...queries);
          queryIndex += 1;
        } while (queries.length !== 0);
        return allTasks;
      };

      const exportTasksToCsv = async (minimal: boolean) => {
        AppEvents.loadingBackdrop.next(true);
        const now = new Date();
        const pastMonthTasks = await getPastMonthTasks(now);

        if (!pastMonthTasks || !pastMonthTasks.length) {
          appController.showAlert('error', 'No tasks found over the past month.');
          AppEvents.loadingBackdrop.next(false);
          return;
        }
        if (minimal) {
          exportCsvMinimal(now, pastMonthTasks);
        } else {
          exportCsvFull(now, pastMonthTasks);
        }
        AppEvents.loadingBackdrop.next(false);
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
            <Box display="flex" gap={1} marginRight={1}>
              <Tooltip title="Export task history of the past 31 days" placement="top">
                <Button
                  sx={{
                    fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
                    paddingTop: isScreenHeightLessThan800 ? 0 : 'inherit',
                    paddingBottom: isScreenHeightLessThan800 ? 0 : 'inherit',
                    marginBottom: isScreenHeightLessThan800 ? 1.8 : 'inherit',
                  }}
                  id="export-button"
                  aria-controls={openExportMenu ? 'export-menu' : undefined}
                  aria-haspopup="true"
                  aria-expanded={openExportMenu ? 'true' : undefined}
                  variant="outlined"
                  onClick={handleClickExportMenu}
                  color="inherit"
                  startIcon={
                    <DownloadIcon transform={`scale(${isScreenHeightLessThan800 ? 0.8 : 1})`} />
                  }
                >
                  Export past 31 days
                </Button>
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
                    exportTasksToCsv(true);
                    handleCloseExportMenu();
                  }}
                  disableRipple
                >
                  Export Minimal
                </MenuItem>
                <MenuItem
                  onClick={() => {
                    exportTasksToCsv(false);
                    handleCloseExportMenu();
                  }}
                  disableRipple
                >
                  Export Full
                </MenuItem>
              </Menu>
              <Tooltip title="Refreshes the task queue table" color="inherit" placement="top">
                <Button
                  sx={{
                    fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
                    paddingTop: isScreenHeightLessThan800 ? 0 : 'inherit',
                    paddingBottom: isScreenHeightLessThan800 ? 0 : 'inherit',
                    marginBottom: isScreenHeightLessThan800 ? 1.8 : 'inherit',
                  }}
                  id="refresh-button"
                  variant="outlined"
                  onClick={() => {
                    AppEvents.refreshTaskApp.next();
                  }}
                  aria-label="Refresh"
                  color="inherit"
                  startIcon={
                    <RefreshIcon transform={`scale(${isScreenHeightLessThan800 ? 0.8 : 1})`} />
                  }
                >
                  Refresh Task Queue
                </Button>
              </Tooltip>
            </Box>
          }
          {...otherProps}
        >
          <Grid container direction="column" wrap="nowrap" height="100%">
            <Tabs
              value={selectedPanelIndex}
              onChange={handlePanelChange}
              aria-label="Task App Tabs"
            >
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
              <TableContainer sx={{ height: '100%' }}>
                <TaskDataGridTable
                  tasks={tasksState}
                  onTaskClick={(_: MuiMouseEvent, task: TaskState) => {
                    setSelectedTask(task);
                    if (task.assigned_to) {
                      AppEvents.robotSelect.next([task.assigned_to.group, task.assigned_to.name]);
                    }
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
              <StyledDiv>
                <TaskSchedule />
              </StyledDiv>
            </TabPanel>
            <input type="file" style={{ display: 'none' }} ref={uploadFileInputRef} />
            {openTaskSummary && (
              <TaskSummary
                onClose={() => setOpenTaskSummary(false)}
                task={selectedTask ?? undefined}
              />
            )}
          </Grid>
          {children}
        </Window>
      );
    },
  ),
);

export default TasksWindow;

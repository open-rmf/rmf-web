import {
  InsertInvitation as ScheduleIcon,
  LowPriority,
  Person as UserIcon,
} from '@mui/icons-material/';
import { Box, Stack, styled, Tooltip, Typography } from '@mui/material';
import {
  DataGrid,
  getGridDateOperators,
  getGridStringOperators,
  GridCellParams,
  GridColumns,
  GridEventListener,
  GridFilterModel,
  GridRowParams,
  GridSortModel,
  GridValueGetterParams,
  MuiEvent,
} from '@mui/x-data-grid';
import {
  ApiServerModelsRmfApiTaskStateStatus as Status,
  TaskStateOutput as TaskState,
} from 'api-client';
import * as React from 'react';

import { TaskBookingLabels } from './booking-label';
import { getTaskBookingLabelFromTaskState } from './task-booking-label-utils';
import { parseTaskPriority } from './utils';

const classes = {
  taskActiveCell: 'MuiDataGrid-cell-active-cell',
  taskCancelledCell: 'MuiDataGrid-cell-cancelled-cell',
  taskCompletedCell: 'MuiDataGrid-cell-completed-cell',
  taskFailedCell: 'MuiDataGrid-cell-failed-cell',
  taskPendingCell: 'MuiDataGrid-cell-pending-cell',
  taskQueuedCell: 'MuiDataGrid-cell-queued-cell',
  taskUnknownCell: 'MuiDataGrid-cell-unknown-cell',
  taskLateCell: 'MuiDataGrid-cell-late-cell',
};

const StyledDataGrid = styled(DataGrid)(({ theme }) => ({
  [`& .${classes.taskActiveCell}`]: {
    backgroundColor: theme.palette.success.light,
    color: theme.palette.getContrastText(theme.palette.success.light),
  },
  [`& .${classes.taskCancelledCell}`]: {
    backgroundColor: theme.palette.grey[500],
    color: theme.palette.getContrastText(theme.palette.grey[500]),
  },
  [`& .${classes.taskCompletedCell}`]: {
    backgroundColor: theme.palette.info.light,
    color: theme.palette.getContrastText(theme.palette.info.light),
  },
  [`& .${classes.taskFailedCell}`]: {
    backgroundColor: theme.palette.error.main,
    color: theme.palette.getContrastText(theme.palette.error.main),
  },
  [`& .${classes.taskQueuedCell}`]: {
    backgroundColor: theme.palette.grey[300],
    color: theme.palette.getContrastText(theme.palette.grey[300]),
  },
  [`& .${classes.taskUnknownCell}`]: {
    backgroundColor: theme.palette.warning.main,
    color: theme.palette.getContrastText(theme.palette.warning.main),
  },
  [`& .${classes.taskLateCell}`]: {
    backgroundColor: theme.palette.warning.main,
    color: theme.palette.getContrastText(theme.palette.warning.main),
  },
})) as typeof DataGrid;

function isTaskOutdated(taskState: TaskState): boolean {
  if (
    !taskState.unix_millis_finish_time ||
    !taskState.status ||
    (taskState.status !== Status.Underway && taskState.status !== Status.Queued)
  ) {
    return false;
  }

  const finishDateTime = new Date(taskState.unix_millis_finish_time);
  const nowDateTime = new Date();
  return nowDateTime > finishDateTime;
}

export interface Tasks {
  isLoading: boolean;
  data: TaskState[];
  total: number;
  page: number;
  pageSize: number;
}

interface TaskData {
  state: TaskState;
  requestLabel: TaskBookingLabels | null;
}

export interface FilterFields {
  model: GridFilterModel | undefined;
}

export interface SortFields {
  model: GridSortModel | undefined;
}

export type MuiMouseEvent = MuiEvent<React.MouseEvent<HTMLElement>>;

export interface TableDataGridState {
  tasks: Tasks;
  onTaskClick?(ev: MuiMouseEvent, task: TaskState): void;
  onPageChange: (newPage: number) => void;
  onPageSizeChange: (newPageSize: number) => void;
  setFilterFields: React.Dispatch<React.SetStateAction<FilterFields>>;
  setSortFields: React.Dispatch<React.SetStateAction<SortFields>>;
}

const TaskRequester = (
  requester: string | undefined | null,
  scheduled: boolean,
  prioritized: boolean,
): JSX.Element => {
  if (!requester) {
    return <Typography variant="body1">n/a</Typography>;
  }

  return (
    <Stack direction="row" alignItems="center" gap={1}>
      {prioritized ? (
        <Tooltip title="User prioritized">
          <LowPriority sx={{ transform: 'rotate(0.5turn)' }} />
        </Tooltip>
      ) : null}
      {scheduled ? (
        <Tooltip title="User scheduled">
          <ScheduleIcon />
        </Tooltip>
      ) : (
        <Tooltip title="User submitted">
          <UserIcon />
        </Tooltip>
      )}
      <Typography variant="body1">{requester}</Typography>
    </Stack>
  );
};

export function TaskDataGridTable({
  tasks,
  onTaskClick,
  onPageChange,
  onPageSizeChange,
  setFilterFields,
  setSortFields,
}: TableDataGridState): JSX.Element {
  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiMouseEvent,
  ) => {
    if (onTaskClick) {
      onTaskClick(event, params.row.state);
    }
  };

  const getMinimalStringFilterOperators = getGridStringOperators().filter(
    // TODO: implement contains on the server end as well
    (operator) => operator.value === 'equals',
  );

  const getMinimalDateOperators = getGridDateOperators(true).filter(
    (operator) => operator.value === 'onOrAfter' || operator.value === 'onOrBefore',
  );

  const columns: GridColumns<TaskData> = [
    {
      field: 'unix_millis_request_time',
      headerName: 'Date',
      width: 150,
      editable: false,
      renderCell: (cellValues) => {
        if (!cellValues.row.state.booking.unix_millis_request_time) {
          return 'n/a';
        }
        const date = new Date(cellValues.row.state.booking.unix_millis_request_time);
        const day = date.toLocaleDateString(undefined, { day: 'numeric' });
        const month = date.toLocaleDateString(undefined, { month: 'short' });
        const year = date.toLocaleDateString(undefined, { year: 'numeric' });
        return `${day} ${month} ${year}`;
      },
      flex: 1,
      filterOperators: getMinimalDateOperators,
      filterable: true,
    },
    {
      field: 'requester',
      headerName: 'Requester',
      width: 150,
      editable: false,
      renderCell: (cellValues) => {
        let prioritized = false;
        if (cellValues.row.state.booking.priority) {
          prioritized = parseTaskPriority(cellValues.row.state.booking.priority);
        }

        let scheduled = false;
        if (cellValues.row.requestLabel && 'scheduled' in cellValues.row.requestLabel) {
          scheduled = cellValues.row.requestLabel.scheduled === 'true';
        }

        return TaskRequester(cellValues.row.state.booking.requester, scheduled, prioritized);
      },
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'label=pickup',
      headerName: 'Pickup',
      width: 150,
      editable: false,
      valueGetter: (params) => {
        if (params.row.requestLabel && params.row.requestLabel.pickup) {
          return params.row.requestLabel.pickup;
        }
        return 'n/a';
      },
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'label=destination',
      headerName: 'Destination',
      width: 150,
      editable: false,
      valueGetter: (params) => {
        if (params.row.requestLabel && params.row.requestLabel.destination) {
          return params.row.requestLabel.destination;
        }
        return 'n/a';
      },
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'assigned_to',
      headerName: 'Robot',
      width: 100,
      editable: false,
      valueGetter: (params) =>
        params.row.state.assigned_to ? params.row.state.assigned_to.name : 'n/a',
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'unix_millis_start_time',
      headerName: 'Start Time',
      width: 150,
      editable: false,
      renderCell: (cellValues) => {
        const startDateTime = cellValues.row.state.unix_millis_start_time
          ? new Date(cellValues.row.state.unix_millis_start_time)
          : undefined;
        const startTimeString = startDateTime ? `${startDateTime.toLocaleTimeString()}` : 'n/a';
        return (
          <Tooltip
            title={
              <Typography variant="caption" noWrap>
                Start time: {startDateTime ? startDateTime.toLocaleString() : 'n/a'}
              </Typography>
            }
          >
            <Box component="div">{startTimeString}</Box>
          </Tooltip>
        );
      },
      flex: 1,
      filterOperators: getMinimalDateOperators,
      filterable: true,
    },
    {
      field: 'unix_millis_finish_time',
      headerName: 'End Time',
      width: 150,
      editable: false,
      renderCell: (cellValues) => {
        let warnDateTime: Date | undefined = undefined;
        if (cellValues.row.requestLabel && 'unix_millis_warn_time' in cellValues.row.requestLabel) {
          const warnMillisNum = parseInt(cellValues.row.requestLabel.unix_millis_warn_time);
          if (!Number.isNaN(warnMillisNum)) {
            warnDateTime = new Date(warnMillisNum);
          }
        }

        const finishDateTime = cellValues.row.state.unix_millis_finish_time
          ? new Date(cellValues.row.state.unix_millis_finish_time)
          : undefined;
        const finishTimeString = finishDateTime ? `${finishDateTime.toLocaleTimeString()}` : 'n/a';
        return (
          <Tooltip
            title={
              <React.Fragment>
                <Typography variant="caption" display="block" noWrap>
                  Warning time: {warnDateTime ? warnDateTime.toLocaleString() : 'n/a'}
                </Typography>
                <Typography variant="caption" noWrap>
                  Finish time: {finishDateTime ? finishDateTime.toLocaleString() : 'n/a'}
                </Typography>
              </React.Fragment>
            }
          >
            <Box component="div">{finishTimeString}</Box>
          </Tooltip>
        );
      },
      flex: 1,
      filterOperators: getMinimalDateOperators,
      filterable: true,
    },
    {
      field: 'status',
      headerName: 'State',
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.state.status ? params.row.state.status : 'unknown',
      renderCell: (cellValues) => {
        const statusString = cellValues.row.state.status ? cellValues.row.state.status : 'unknown';
        if (isTaskOutdated(cellValues.row.state)) {
          return (
            <Tooltip
              title={
                <React.Fragment>
                  <Typography>
                    Finish time is in the past, but task is still queued or underway.
                  </Typography>
                  <Typography>
                    The task may have been interrupted or stalled during the execution.
                  </Typography>
                </React.Fragment>
              }
            >
              <Box component="div">{`${statusString} (stale)`}</Box>
            </Tooltip>
          );
        }

        return <Box component="div">{statusString}</Box>;
      },
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
  ];

  const handleFilterModelChange = React.useCallback(
    (filterModel: GridFilterModel) => {
      setFilterFields({ model: filterModel });
    },
    [setFilterFields],
  );

  const handleSortModelChange = React.useCallback(
    (sortModel: GridSortModel) => {
      setSortFields({ model: sortModel });
    },
    [setSortFields],
  );

  const taskData: TaskData[] = tasks.data.map((state) => {
    return {
      state,
      requestLabel: getTaskBookingLabelFromTaskState(state),
    };
  });

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <StyledDataGrid
        getRowId={(r) => r.state.booking.id}
        rows={taskData}
        rowCount={tasks.total}
        loading={tasks.isLoading}
        pageSize={tasks.pageSize}
        rowsPerPageOptions={[10]}
        pagination
        paginationMode="server"
        filterMode="server"
        onFilterModelChange={handleFilterModelChange}
        sortingMode="server"
        onSortModelChange={handleSortModelChange}
        page={tasks.page - 1}
        onPageChange={onPageChange}
        onPageSizeChange={onPageSizeChange}
        columns={columns}
        onRowClick={handleEvent}
        getCellClassName={(params: GridCellParams<string>) => {
          if (params.field === 'status') {
            if (isTaskOutdated(params.row.state)) {
              return classes.taskUnknownCell;
            }

            switch (params.value) {
              case Status.Underway:
                return classes.taskActiveCell;
              case Status.Completed:
                return classes.taskCompletedCell;
              case Status.Canceled:
                return classes.taskCancelledCell;
              case Status.Failed:
                return classes.taskFailedCell;
              case Status.Queued:
                return classes.taskQueuedCell;
              default:
                return classes.taskUnknownCell;
            }
          } else if (params.field === 'unix_millis_finish_time') {
            if (!params.row.state.unix_millis_finish_time) {
              return classes.taskUnknownCell;
            }

            let warnDateTime: Date | undefined = undefined;
            if (params.row.requestLabel && 'unix_millis_warn_time' in params.row.requestLabel) {
              const warnMillisNum = parseInt(params.row.requestLabel.unix_millis_warn_time);
              if (!Number.isNaN(warnMillisNum)) {
                warnDateTime = new Date(warnMillisNum);
              }
            }

            const finishDateTime = params.row.state.unix_millis_finish_time
              ? new Date(params.row.state.unix_millis_finish_time)
              : undefined;

            if (warnDateTime && finishDateTime && finishDateTime > warnDateTime) {
              return classes.taskLateCell;
            }
          }
          return '';
        }}
        disableVirtualization={true}
      />
    </div>
  );
}

import {
  DataGrid,
  getGridDateOperators,
  getGridStringOperators,
  GridColDef,
  GridEventListener,
  GridValueGetterParams,
  MuiEvent,
  GridRowParams,
  GridCellParams,
  GridFilterModel,
  GridSortModel,
} from '@mui/x-data-grid';
import {
  Box,
  styled,
  Stack,
  Typography,
  Tooltip,
  useMediaQuery,
  SxProps,
  Theme,
} from '@mui/material';
import * as React from 'react';
import {
  TaskState,
  ApiServerModelsRmfApiTaskStateStatus as Status,
  TaskBookingLabel,
} from 'api-client';
import { InsertInvitation as ScheduleIcon, Person as UserIcon } from '@mui/icons-material/';
import { getTaskBookingLabelFromTaskState } from './task-booking-label-utils';

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
}));

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
  requestLabel: TaskBookingLabel | null;
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

const TaskRequester = (requester: string | null, sx: SxProps<Theme>): JSX.Element => {
  if (!requester) {
    return (
      <Typography variant="body1" sx={sx}>
        n/a
      </Typography>
    );
  }

  /** When a task is created as scheduled,
   we save the requester as USERNAME__scheduled.
   Therefore, we remove the __schedule because the different icon is enough indicator to know
   if the task was adhoc or scheduled.
  */
  return (
    <Stack direction="row" alignItems="center" gap={1}>
      {requester.includes('scheduled') ? (
        <>
          <Tooltip title="User scheduled" sx={sx}>
            <ScheduleIcon />
          </Tooltip>
          <Typography variant="body1" sx={sx}>
            {requester.split('__')[0]}
          </Typography>
        </>
      ) : (
        <>
          <Tooltip title="User submitted" sx={sx}>
            <UserIcon />
          </Tooltip>
          <Typography variant="body1" sx={sx}>
            {requester}
          </Typography>
        </>
      )}
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
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const sxProp: SxProps<Theme> = {
    fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
  };

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

  const columns: GridColDef[] = [
    {
      field: 'unix_millis_request_time',
      headerName: 'Date',
      width: 150,
      editable: false,
      renderCell: (cellValues) => {
        const date = new Date(cellValues.row.state.booking.unix_millis_request_time);
        const day = date.toLocaleDateString(undefined, { day: 'numeric' });
        const month = date.toLocaleDateString(undefined, { month: 'short' });
        const year = date.toLocaleDateString(undefined, { year: 'numeric' });
        return cellValues.row.state.booking.unix_millis_request_time
          ? `${day} ${month} ${year}`
          : 'n/a';
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
      renderCell: (cellValues) => TaskRequester(cellValues.row.state.booking.requester, sxProp),
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'pickup',
      headerName: 'Pickup',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => {
        if (params.row.requestLabel && params.row.requestLabel.description.pickup) {
          return params.row.requestLabel.description.pickup;
        }
        return 'n/a';
      },
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'destination',
      headerName: 'Destination',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => {
        if (params.row.requestLabel && params.row.requestLabel.description.destination) {
          return params.row.requestLabel.description.destination;
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
      valueGetter: (params: GridValueGetterParams) =>
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
        if (cellValues.row.requestLabel?.description.unix_millis_warn_time) {
          const warnMillisNum = parseInt(
            cellValues.row.requestLabel.description.unix_millis_warn_time as string,
          );
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
        autoHeight
        getRowId={(r) => r.state.booking.id}
        rows={taskData}
        rowCount={tasks.total}
        loading={tasks.isLoading}
        pageSize={tasks.pageSize}
        rowsPerPageOptions={[10]}
        sx={sxProp}
        autoPageSize={isScreenHeightLessThan800}
        density={isScreenHeightLessThan800 ? 'compact' : 'standard'}
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
            if (params.row.requestLabel?.description.unix_millis_warn_time) {
              const warnMillisNum = parseInt(
                params.row.requestLabel.description.unix_millis_warn_time as string,
              );
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
      />
    </div>
  );
}

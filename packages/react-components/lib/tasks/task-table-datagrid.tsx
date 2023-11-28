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
import { styled, TextField, Stack, Typography, Tooltip, useMediaQuery } from '@mui/material';
import * as React from 'react';
import { TaskState, TaskRequest, Status } from 'api-client';
import { InsertInvitation as ScheduleIcon, Person as UserIcon } from '@mui/icons-material/';

const classes = {
  taskActiveCell: 'MuiDataGrid-cell-active-cell',
  taskCancelledCell: 'MuiDataGrid-cell-cancelled-cell',
  taskCompletedCell: 'MuiDataGrid-cell-completed-cell',
  taskFailedCell: 'MuiDataGrid-cell-failed-cell',
  taskPendingCell: 'MuiDataGrid-cell-pending-cell',
  taskQueuedCell: 'MuiDataGrid-cell-queued-cell',
  taskUnknownCell: 'MuiDataGrid-cell-unknown-cell',
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
}));

export interface Tasks {
  isLoading: boolean;
  data: TaskState[];
  requests: Record<string, TaskRequest>;
  total: number;
  page: number;
  pageSize: number;
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

const TaskRequester = (requester: string | null): JSX.Element => {
  if (!requester) {
    return <Typography variant="body1">unknown</Typography>;
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
          <Tooltip title="User scheduled">
            <ScheduleIcon />
          </Tooltip>
          <Typography variant="body1">{requester.split('__')[0]}</Typography>
        </>
      ) : (
        <>
          <Tooltip title="User submitted">
            <UserIcon />
          </Tooltip>
          <Typography variant="body1">{requester}</Typography>
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
  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiMouseEvent,
  ) => {
    if (onTaskClick) {
      onTaskClick(event, params.row);
    }
  };

  const getMinimalStringFilterOperators = getGridStringOperators().filter(
    // TODO: implement contains on the server end as well
    (operator) => operator.value === 'equals',
  );

  const getPickup = (state: TaskState): string => {
    const request: TaskRequest | undefined = tasks.requests[state.booking.id];
    if (request === undefined || request.category === 'patrol') {
      return 'n/a';
    }

    // custom deliveries
    const supportedDeliveries = [
      'delivery_pickup',
      'delivery_sequential_lot_pickup',
      'delivery_area_pickup',
    ];
    if (
      !request.description['category'] ||
      !supportedDeliveries.includes(request.description['category'])
    ) {
      return 'n/a';
    }

    // TODO(ac): use schemas
    try {
      const deliveryType: string = request.description['category'];
      const perform_action_description =
        request.description['phases'][0]['activity']['description']['activities'][1]['description'][
          'description'
        ];

      switch (deliveryType) {
        case 'delivery_pickup':
          const pickup_lot: string = perform_action_description['pickup_lot'];
          return pickup_lot;
        case 'delivery_sequential_lot_pickup':
        case 'delivery_area_pickup':
          const pickup_zone: string = perform_action_description['pickup_zone'];
          return pickup_zone;
        default:
          return 'n/a';
      }
    } catch (e) {
      console.error(`Failed to parse pickup lot/zone from task request: ${(e as Error).message}`);
    }

    return 'n/a';
  };

  const getDestination = (state: TaskState): string => {
    const request: TaskRequest | undefined = tasks.requests[state.booking.id];
    if (request === undefined) {
      return 'n/a';
    }

    // patrol
    if (
      request.category === 'patrol' &&
      request.description['places'] !== undefined &&
      request.description['places'].length > 0
    ) {
      return request.description['places'].at(-1);
    }

    // custom deliveries
    const supportedDeliveries = [
      'delivery_pickup',
      'delivery_sequential_lot_pickup',
      'delivery_area_pickup',
    ];
    if (
      !request.description['category'] ||
      !supportedDeliveries.includes(request.description['category'])
    ) {
      return 'n/a';
    }

    // TODO(ac): use schemas
    try {
      const destination =
        request.description['phases'][0]['activity']['description']['activities'][2]['description'];
      return destination;
    } catch (e) {
      console.error(`Failed to parse destination from task request: ${(e as Error).message}`);
    }

    return 'n/a';
  };

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
        const date = new Date(cellValues.row.booking.unix_millis_request_time);
        const day = date.toLocaleDateString(undefined, { day: 'numeric' });
        const month = date.toLocaleDateString(undefined, { month: 'short' });
        const year = date.toLocaleDateString(undefined, { year: 'numeric' });
        return (
          <TextField
            variant="standard"
            value={
              cellValues.row.booking.unix_millis_request_time ? `${day} ${month} ${year}` : 'n/a'
            }
            InputProps={{ disableUnderline: true }}
            multiline
          />
        );
      },
      flex: 1,
      filterOperators: getMinimalDateOperators,
      filterable: true,
    },
    {
      field: 'requester',
      headerName: 'Requester',
      minWidth: 160,
      editable: false,
      renderCell: (cellValues) => TaskRequester(cellValues.row.booking.requester),
      flex: 2,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'pickup',
      headerName: 'Pickup',
      width: 90,
      valueGetter: (params: GridValueGetterParams) => getPickup(params.row),
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'destination',
      headerName: 'Destination',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => getDestination(params.row),
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'assigned_to',
      headerName: 'Robot',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.assigned_to ? params.row.assigned_to.name : 'unknown',
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
        return (
          <TextField
            variant="standard"
            value={
              cellValues.row.unix_millis_start_time
                ? `${new Date(
                    cellValues.row.unix_millis_start_time,
                  ).toLocaleDateString()} ${new Date(
                    cellValues.row.unix_millis_start_time,
                  ).toLocaleTimeString()}`
                : 'unknown'
            }
            InputProps={{ disableUnderline: true }}
            multiline
          />
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
        return (
          <TextField
            variant="standard"
            value={
              cellValues.row.unix_millis_finish_time
                ? `${new Date(
                    cellValues.row.unix_millis_finish_time,
                  ).toLocaleDateString()} ${new Date(
                    cellValues.row.unix_millis_finish_time,
                  ).toLocaleTimeString()}`
                : 'unknown'
            }
            InputProps={{ disableUnderline: true }}
            multiline
          />
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
        params.row.status ? params.row.status : 'unknown',
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

  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <StyledDataGrid
        autoHeight
        getRowId={(r) => r.booking.id}
        rows={tasks.data}
        rowCount={tasks.total}
        loading={tasks.isLoading}
        pageSize={tasks.pageSize}
        rowsPerPageOptions={[10]}
        sx={{
          fontSize: isScreenHeightLessThan800 ? '0.9rem' : 'inherit',
        }}
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
          }
          return '';
        }}
      />
    </div>
  );
}

import {
  DataGrid,
  GridColDef,
  GridEventListener,
  GridValueGetterParams,
  MuiEvent,
  GridRowParams,
  GridCellParams,
} from '@mui/x-data-grid';
import { styled } from '@mui/material';
import { TaskState, Status } from 'api-client';
import React from 'react';

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

export interface DefaultTableDataGridProps {
  isLoading: boolean;
  data: TaskState[];
  total: number;
  page: number;
  pageSize: number;
}

export interface TaskDataGridTableProps {
  tasks: DefaultTableDataGridProps;
  onTaskClick?(ev: MuiEvent<React.MouseEvent<HTMLElement>>, task: TaskState): void;
  onPageChange: (newPage: number) => void;
  onPageSizeChange: (newPageSize: number) => void;
}

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
    editable: false,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.assigned_to ? params.row.assigned_to.name : 'unknown',
  },
  {
    field: 'unix_millis_start_time',
    headerName: 'Start Time',
    width: 150,
    editable: false,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.unix_millis_start_time
        ? new Date(params.row.unix_millis_start_time).toLocaleDateString()
        : 'unknown',
  },
  {
    field: 'unix_millis_finish_time',
    headerName: 'End Time',
    width: 150,
    editable: false,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.unix_millis_finish_time
        ? new Date(params.row.unix_millis_finish_time).toLocaleTimeString()
        : '-',
  },
  {
    field: 'status',
    headerName: 'State',
    width: 150,
    editable: false,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.status ? params.row.status : 'unknown',
  },
];

export function TaskDataGridTable({
  tasks,
  onTaskClick,
  onPageChange,
  onPageSizeChange,
}: TaskDataGridTableProps): JSX.Element {
  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onTaskClick) {
      onTaskClick(event, params.row);
    }
  };

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <StyledDataGrid
        autoHeight
        getRowId={(r) => r.booking.id}
        rows={tasks.data}
        rowCount={tasks.total}
        loading={tasks.isLoading}
        rowsPerPageOptions={[10, 30, 50, 70, 100]}
        pagination
        page={tasks.page - 1}
        pageSize={tasks.pageSize}
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

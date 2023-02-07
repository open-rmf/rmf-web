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
  GridToolbarColumnsButton,
  GridToolbarContainer,
  GridToolbarContainerProps,
  GridToolbarFilterButton,
} from '@mui/x-data-grid';
import { styled, TextField } from '@mui/material';
import Button, { ButtonProps } from '@mui/material/Button';
import FileDownloadIcon from '@mui/icons-material/FileDownload';
import * as React from 'react';
import { TaskState, Status } from 'api-client';

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

export interface TableDataGridState {
  tasks: Tasks;
  onTaskClick?(ev: MuiEvent<React.MouseEvent<HTMLElement>>, task: TaskState): void;
  onPageChange: (newPage: number) => void;
  onPageSizeChange: (newPageSize: number) => void;
  setFilterFields: React.Dispatch<React.SetStateAction<FilterFields>>;
  setSortFields: React.Dispatch<React.SetStateAction<SortFields>>;
  exportTasksMinimal: () => Promise<void>;
  exportTasksFull: () => Promise<void>;
}

export function TaskDataGridTable({
  tasks,
  onTaskClick,
  onPageChange,
  onPageSizeChange,
  setFilterFields,
  setSortFields,
  exportTasksMinimal,
  exportTasksFull,
}: TableDataGridState): JSX.Element {
  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onTaskClick) {
      onTaskClick(event, params.row);
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
      field: 'id_',
      headerName: 'ID',
      width: 90,
      valueGetter: (params: GridValueGetterParams) => params.row.booking.id,
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'category',
      headerName: 'Category',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.category ? params.row.category : 'unknown',
      flex: 1,
      filterOperators: getMinimalStringFilterOperators,
      filterable: true,
    },
    {
      field: 'assigned_to',
      headerName: 'Assignee',
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

  const CustomToolbar = (props: GridToolbarContainerProps) => {
    const exportButtonProps: ButtonProps = {
      color: 'primary',
      size: 'small',
      startIcon: <FileDownloadIcon />,
    };

    return (
      <GridToolbarContainer {...props}>
        <GridToolbarColumnsButton />
        <GridToolbarFilterButton />
        <Button {...exportButtonProps} onClick={exportTasksFull}>
          Full Export
        </Button>
        <Button {...exportButtonProps} onClick={exportTasksMinimal}>
          Minimal Export
        </Button>
      </GridToolbarContainer>
    );
  };

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
        components={{ Toolbar: CustomToolbar }}
      />
    </div>
  );
}

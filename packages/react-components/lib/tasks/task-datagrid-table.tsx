import {
  DataGrid,
  GridColDef,
  GridEventListener,
  GridValueGetterParams,
  MuiEvent,
  GridRowParams,
  GridCellParams,
  GridFilterInputValueProps,
  GridFilterItem,
  GRID_DATE_COL_DEF,
  GridColTypeDef,
  useGridApiContext,
  GridRenderEditCellParams,
} from '@mui/x-data-grid';
import { FormControl, InputBase, NativeSelect, styled, TextField } from '@mui/material';
import locale from 'date-fns/locale/en-US';
import { TaskState, Status } from 'api-client';
import React from 'react';
import { DatePicker, DateTimePicker } from '@mui/x-date-pickers';
import { AdapterDateFns } from '@mui/x-date-pickers/AdapterDateFns';

/**
 * Custom filter operation
 * https://mui.com/x/react-data-grid/filtering/
 */
const buildApplyDateFilterFn = (
  filterItem: GridFilterItem,
  compareFn: (value1: number, value2: number) => boolean,
  showTime: boolean,
) => {
  if (!filterItem.value) {
    return null;
  }

  const filterValueMs = filterItem.value.getTime();

  return ({ value }: GridCellParams<Date>): boolean => {
    if (!value) {
      return false;
    }

    // Make a copy of the date to not reset the hours in the original object
    const dateCopy = new Date(value);
    dateCopy.setHours(showTime ? value.getHours() : 0, showTime ? value.getMinutes() : 0, 0, 0);
    const cellValueMs = dateCopy.getTime();

    return compareFn(cellValueMs, filterValueMs);
  };
};

const getDateFilterOperators = (showTime: boolean): GridColTypeDef['filterOperators'] => {
  return [
    {
      value: 'is',
      getApplyFilterFn: (filterItem) => {
        return buildApplyDateFilterFn(filterItem, (value1, value2) => value1 === value2, showTime);
      },
      InputComponent: GridFilterDateInput,
      InputComponentProps: { showTime },
    },
    {
      value: 'not',
      getApplyFilterFn: (filterItem) => {
        return buildApplyDateFilterFn(filterItem, (value1, value2) => value1 !== value2, showTime);
      },
      InputComponent: GridFilterDateInput,
      InputComponentProps: { showTime },
    },
    {
      value: 'after',
      getApplyFilterFn: (filterItem) => {
        return buildApplyDateFilterFn(filterItem, (value1, value2) => value1 > value2, showTime);
      },
      InputComponent: GridFilterDateInput,
      InputComponentProps: { showTime },
    },
    {
      value: 'onOrAfter',
      getApplyFilterFn: (filterItem) => {
        return buildApplyDateFilterFn(filterItem, (value1, value2) => value1 >= value2, showTime);
      },
      InputComponent: GridFilterDateInput,
      InputComponentProps: { showTime },
    },
    {
      value: 'before',
      getApplyFilterFn: (filterItem) => {
        return buildApplyDateFilterFn(filterItem, (value1, value2) => value1 < value2, showTime);
      },
      InputComponent: GridFilterDateInput,
      InputComponentProps: { showTime },
    },
    {
      value: 'onOrBefore',
      getApplyFilterFn: (filterItem) => {
        return buildApplyDateFilterFn(filterItem, (value1, value2) => value1 <= value2, showTime);
      },
      InputComponent: GridFilterDateInput,
      InputComponentProps: { showTime },
    },
    {
      value: 'isEmpty',
      getApplyFilterFn: () => {
        return ({ value }): boolean => {
          return value == null;
        };
      },
    },
    {
      value: 'isNotEmpty',
      getApplyFilterFn: () => {
        return ({ value }): boolean => {
          return value != null;
        };
      },
    },
  ];
};

const dateAdapter = new AdapterDateFns({ locale });

/**
 * `date` column
 */

const dateColumnType: GridColTypeDef<Date | string, string> = {
  ...GRID_DATE_COL_DEF,
  resizable: false,
  renderEditCell: (params) => {
    return <GridEditDateCell {...params} />;
  },
  filterOperators: getDateFilterOperators(false),
  valueFormatter: (params) => {
    if (typeof params.value === 'string') {
      return params.value;
    }
    if (params.value) {
      return dateAdapter.format(params.value, 'keyboardDate');
    }
    return '';
  },
};

const GridEditDateInput = styled(InputBase)({
  fontSize: 'inherit',
  padding: '0 9px',
});

const GridEditDateCell = ({
  id,
  field,
  value,
  colDef,
}: GridRenderEditCellParams<Date | string | null>) => {
  const apiRef = useGridApiContext();

  const Component = colDef.type === 'dateTime' ? DateTimePicker : DatePicker;

  const handleChange = (newValue: unknown) => {
    apiRef.current.setEditCellValue({ id, field, value: newValue });
  };

  return (
    <Component
      value={value}
      renderInput={({ inputRef, inputProps, InputProps, disabled, error }) => (
        <GridEditDateInput
          fullWidth
          autoFocus
          ref={inputRef}
          {...InputProps}
          disabled={disabled}
          error={error}
          inputProps={inputProps}
        />
      )}
      onChange={handleChange}
    />
  );
};

const GridFilterDateInput = (props: GridFilterInputValueProps & { showTime?: boolean }) => {
  const { item, showTime, applyValue, apiRef } = props;

  const Component = showTime ? DateTimePicker : DatePicker;

  const handleFilterChange = (newValue: unknown) => {
    applyValue({ ...item, value: newValue });
  };

  return (
    <Component
      value={item.value || null}
      renderInput={(params) => (
        <TextField
          {...params}
          variant="standard"
          label={apiRef.current.getLocaleText('filterPanelInputLabel')}
        />
      )}
      InputAdornmentProps={{
        sx: {
          '& .MuiButtonBase-root': {
            marginRight: -1,
          },
        },
      }}
      onChange={handleFilterChange}
    />
  );
};

const buildApplyStateFilterFn = (
  filterItem: GridFilterItem,
  compareFn: (value1: string, value2: string) => boolean,
) => {
  if (!filterItem.value) {
    return null;
  }

  const filterValue = filterItem.value;

  return ({ value }: GridCellParams<string>): boolean => {
    if (!value) {
      return false;
    }

    return compareFn(value, filterValue.target.value);
  };
};

const getSelectFilterOperators = (): GridColTypeDef['filterOperators'] => {
  return [
    {
      value: 'is',
      getApplyFilterFn: (filterItem) => {
        return buildApplyStateFilterFn(filterItem, (value1, value2) => value1 === value2);
      },
      InputComponent: GridFilterSelectInput,
      InputComponentProps: { type: 'string' },
    },
    {
      value: 'isEmpty',
      getApplyFilterFn: () => {
        return ({ value }): boolean => {
          return value == null;
        };
      },
    },
    {
      value: 'isNotEmpty',
      getApplyFilterFn: () => {
        return ({ value }): boolean => {
          return value != null;
        };
      },
    },
  ];
};

const GridFilterSelectInput = (props: GridFilterInputValueProps) => {
  const { item, applyValue } = props;
  const handleFilterChange = (newValue: unknown) => {
    applyValue({ ...item, value: newValue });
  };

  return (
    <FormControl fullWidth>
      <NativeSelect onChange={handleFilterChange} sx={{ mt: '16px' }}>
        <option value={''}></option>
        <option value={Status.Uninitialized}>{Status.Uninitialized}</option>
        <option value={Status.Blocked}>{Status.Blocked}</option>
        <option value={Status.Error}>{Status.Error}</option>
        <option value={Status.Failed}>{Status.Failed}</option>
        <option value={Status.Queued}>{Status.Queued}</option>
        <option value={Status.Standby}>{Status.Standby}</option>
        <option value={Status.Underway}>{Status.Underway}</option>
        <option value={Status.Delayed}>{Status.Delayed}</option>
        <option value={Status.Skipped}>{Status.Skipped}</option>
        <option value={Status.Canceled}>{Status.Canceled}</option>
        <option value={Status.Killed}>{Status.Killed}</option>
        <option value={Status.Completed}>{Status.Completed}</option>
      </NativeSelect>
    </FormControl>
  );
};

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
    flex: 1,
  },
  {
    field: 'category',
    headerName: 'Category',
    width: 150,
    editable: false,
    flex: 1,
  },
  {
    field: 'name',
    headerName: 'Assignee',
    width: 150,
    editable: false,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.assigned_to ? params.row.assigned_to.name : 'unknown',
    flex: 1,
  },
  {
    field: 'unix_millis_start_time',
    headerName: 'Start Time',
    width: 150,
    editable: false,
    ...dateColumnType,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.unix_millis_start_time
        ? new Date(params.row.unix_millis_start_time).toLocaleDateString()
        : 'unknown',
    flex: 1,
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
    flex: 1,
  },
  {
    field: 'status',
    headerName: 'State',
    editable: false,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.status ? params.row.status : 'unknown',
    flex: 1,
    filterOperators: getSelectFilterOperators(),
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

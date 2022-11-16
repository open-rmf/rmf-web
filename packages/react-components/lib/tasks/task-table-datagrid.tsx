import {
  DataGrid,
  GridColDef,
  GridEventListener,
  GridValueGetterParams,
  MuiEvent,
  GridRowParams,
  GridCellParams,
  GridColTypeDef,
  GridFilterInputValueProps,
  GridFilterItem,
  GridRenderEditCellParams,
  useGridApiContext,
  GRID_DATE_COL_DEF,
} from '@mui/x-data-grid';
import locale from 'date-fns/locale/en-US';
import { Box, InputBase, styled, TextField, TextFieldProps } from '@mui/material';
import * as React from 'react';
import { TaskState, Status } from 'api-client';
import SyncIcon from '@mui/icons-material/Sync';
import { DatePicker, TimePicker, DateTimePicker } from '@mui/x-date-pickers';
import { AdapterDateFns } from '@mui/x-date-pickers/AdapterDateFns';

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
  category: string | undefined;
  taskId: string | undefined;
  startTime: string | undefined;
  finisTime: string | undefined;
}

export interface TableDataGridState {
  tasks: Tasks;
  onTaskClick?(ev: MuiEvent<React.MouseEvent<HTMLElement>>, task: TaskState): void;
  onPageChange: (newPage: number) => void;
  onPageSizeChange: (newPageSize: number) => void;
  setFilterFields: React.Dispatch<React.SetStateAction<FilterFields>>;
}

export function TaskDataGridTable({
  tasks,
  onTaskClick,
  onPageChange,
  onPageSizeChange,
  setFilterFields,
}: TableDataGridState): JSX.Element {
  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onTaskClick) {
      onTaskClick(event, params.row);
    }
  };

  const [categoryFilter, setCategoryFilter] = React.useState('');
  /**
   * Custom filter operation
   * https://mui.com/x/react-data-grid/filtering/
   */
  const buildApplyDateFilterFn = (
    filterItem: GridFilterItem,
    compareFn: (value1: number, value2: number | string) => boolean,
    showTime: boolean,
  ) => {
    if (!filterItem.value) {
      return null;
    }

    const filterValueMs = filterItem.value.getTime();

    return ({ value }: GridCellParams): boolean => {
      if (!value || value === '-') {
        return false;
      }

      if (showTime) {
        const filterValueTime = new Date(filterValueMs).toLocaleTimeString();
        return compareFn(value, filterValueTime);
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
          return buildApplyDateFilterFn(
            filterItem,
            (value1, value2) => value1 === value2,
            showTime,
          );
        },
        InputComponent: GridFilterDateInput,
        InputComponentProps: { showTime },
      },
      {
        value: 'not',
        getApplyFilterFn: (filterItem) => {
          return buildApplyDateFilterFn(
            filterItem,
            (value1, value2) => value1 !== value2,
            showTime,
          );
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

    const Component = colDef.type === 'dateTime' ? TimePicker : DatePicker;

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
    const [filterValueState, setFilterValueState] = React.useState(item.value ?? '');
    const filterTimeout = React.useRef<ReturnType<typeof setTimeout>>();

    React.useEffect(() => {
      return () => {
        clearTimeout(filterTimeout.current);
      };
    }, []);

    React.useEffect(() => {
      const itemValue = item.value ?? '';
      setFilterValueState(itemValue);
    }, [item.value]);

    const handleFilterTimePickerChange = (newValue: unknown) => {
      const filterValue = new Date(dateAdapter.toISO(newValue as Date)).getTime() / 1000;

      clearTimeout(filterTimeout.current);

      setFilterValueState(newValue);

      filterTimeout.current = setTimeout(() => {
        setFilterFields((old) => ({
          ...old,
          finisTime: filterValue.toString(),
        }));
        applyValue({ ...item, value: newValue });
      }, 5000);
    };

    const handleFilterChange = (newValue: unknown) => {
      const localDate = new Date(newValue as Date);
      const filterValue = dateAdapter.toISO(localDate);

      setFilterFields((old) => ({
        ...old,
        startTime: filterValue === '' ? undefined : filterValue,
      }));

      applyValue({ ...item, value: newValue });
    };

    return showTime ? (
      <DateTimePicker
        value={filterValueState}
        InputAdornmentProps={{
          sx: {
            '& .MuiButtonBase-root': {
              marginRight: -1,
            },
          },
        }}
        onChange={handleFilterTimePickerChange}
        renderInput={(params) => (
          <TextField
            {...params}
            variant="standard"
            label={apiRef.current.getLocaleText('filterPanelInputLabel')}
          />
        )}
      />
    ) : (
      <DatePicker
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

  const buildApplyCategoryFilter = (
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

      return compareFn(value, filterValue);
    };
  };

  const getCategoryFilterOperators = (): GridColTypeDef['filterOperators'] => {
    return [
      {
        value: 'is',
        getApplyFilterFn: (filterItem) => {
          return buildApplyCategoryFilter(filterItem, (value1, value2) => value1 === value2);
        },
        InputComponent: GridFilterCategoryInput,
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

  const GridFilterCategoryInput = (props: GridFilterInputValueProps) => {
    const { item, applyValue, focusElementRef = null } = props;

    const [filterValueState, setFilterValueState] = React.useState(item.value ?? '');
    const [applying, setIsApplying] = React.useState(false);

    const filterTimeout = React.useRef<ReturnType<typeof setTimeout>>();
    React.useEffect(() => {
      return () => {
        clearTimeout(filterTimeout.current);
      };
    }, []);

    React.useEffect(() => {
      const itemValue = item.value ?? '';
      setFilterValueState(itemValue);

      // Setting the flag state to reset the category filter when closing the filter panel
      setCategoryFilter(itemValue);
    }, [item.value]);

    const updateFilterValue = (category: string) => {
      clearTimeout(filterTimeout.current);
      setFilterValueState(category);

      setIsApplying(true);
      filterTimeout.current = setTimeout(() => {
        setIsApplying(false);
        setFilterFields((old) => ({
          ...old,
          category: category === '' ? undefined : category,
        }));
        applyValue({ ...item, value: category });
      }, 500);
    };

    const handleLowerFilterChange: TextFieldProps['onChange'] = (event) => {
      updateFilterValue(event.target.value);
    };

    return (
      <Box
        sx={{
          display: 'inline-flex',
          flexDirection: 'row',
          alignItems: 'end',
          height: 48,
          pl: '20px',
        }}
      >
        <TextField
          name="filter"
          placeholder="Filter Value"
          label="Value"
          variant="standard"
          value={filterValueState}
          onChange={handleLowerFilterChange}
          inputRef={focusElementRef}
          sx={{ mr: 2 }}
          InputProps={applying ? { endAdornment: <SyncIcon /> } : {}}
        />
      </Box>
    );
  };

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
      filterOperators: getCategoryFilterOperators(),
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
      filterOperators: getDateFilterOperators(false),
    },
    {
      field: 'unix_millis_finish_time',
      headerName: 'End Time',
      width: 150,
      editable: false,
      type: 'dateTime',
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
      filterOperators: getDateFilterOperators(true),
    },
    {
      field: 'status',
      headerName: 'State',
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.status ? params.row.status : 'unknown',
      flex: 1,
    },
  ];

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
        onPreferencePanelClose={() => {
          setFilterFields((old) => ({
            ...old,
            category: categoryFilter === '' ? undefined : categoryFilter,
          }));
        }}
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

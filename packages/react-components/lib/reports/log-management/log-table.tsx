import React from 'react';
import { DataGrid, GridRenderCellParams } from '@mui/x-data-grid';
import { LogLevel } from '.';
import { Typography, styled } from '@mui/material';
import { format } from 'date-fns';
import { CustomLookupFilter } from './custom-lookup-filter';

export type ContainerType = {
  id: number;
  name: string;
};

export type LogRowsType = {
  level: string;
  message: string;
  created: string;
  container: ContainerType;
}[];

export interface LogTableProps {
  rows: LogRowsType | [];
  tableSize?: number; // units vh or rem
  addMoreRows?(): void;
}

const classes = {
  error: 'log-table-error',
  debug: 'log-table-debug',
  warn: 'log-table-warn',
  info: 'log-table-info',
  cellContent: 'log-table-cell-content',
};
const LogTableRoot = styled('div')(({ theme }) => ({
  [`& .${classes.error}`]: {
    color: theme.palette.error.main,
  },
  [`& .${classes.debug}`]: {
    color: theme.palette.secondary.dark,
  },
  [`& .${classes.warn}`]: {
    color: theme.palette.warning.light,
  },
  [`& .${classes.info}`]: {
    color: theme.palette.info.main,
  },
  [`& .${classes.cellContent}`]: {
    display: 'block',
    marginBlockStart: '1em',
    marginBlockEnd: '1em',
    marginInlineStart: '0px',
    marginInlineEnd: '0px',
  },
}));

export const LogTable = (props: LogTableProps): React.ReactElement => {
  const { rows, addMoreRows } = props;
  const [pageSize, setPageSize] = React.useState(100);
  const getLogLevelStyle = (level: string): string | undefined => {
    level = level.toLowerCase();
    switch (level) {
      case LogLevel.Error:
        return classes.error;
      case LogLevel.Warn:
        return classes.warn;
      case LogLevel.Fatal:
        return classes.error;
      case LogLevel.Debug:
        return classes.debug;
      case LogLevel.Info:
        return classes.info;
      default:
        return undefined;
    }
  };

  const [selectedFilter, setSelectedFilter] = React.useState<string[]>([]);
  // FIXME: we cannot copy the LogLevel Enum directly and remove the all attribute because it has a protected attribute.
  const logLevels = React.useMemo(() => {
    const logLevelCopy: Record<string, string> = {};
    Object.keys(LogLevel).forEach((element: string) => {
      logLevelCopy[element] = LogLevel[element as keyof typeof LogLevel];
    });
    delete logLevelCopy['All'];
    return logLevelCopy;
  }, []);

  return (
    <LogTableRoot style={{ height: '100%', width: '100%' }} id="log-table">
      <DataGrid
        getRowId={(r) => r.container.id}
        autoHeight={true}
        columns={[
          {
            headerName: 'Level',
            field: 'level',
            type: 'string',
            align: 'center',
            width: 100,
            sortable: false,
            renderCell: (rowData: GridRenderCellParams) => {
              return (
                <Typography
                  className={`${getLogLevelStyle(rowData.value as string)} ${classes.cellContent}`}
                >
                  {rowData.row.level}
                </Typography>
              );
            },
          },
          {
            headerName: 'Container',
            field: 'name',
            type: 'string',
            align: 'center',
            width: 120,
            sortable: false,
            renderCell: (rowData: GridRenderCellParams) => {
              return (
                <Typography className={classes.cellContent}>
                  {rowData.row.container.name ? rowData.row.container.name : 'Unknown'}
                </Typography>
              );
            },
          },
          {
            headerName: 'Message',
            field: 'message',
            type: 'string',
            width: 1200,
            sortable: false,
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography className={classes.cellContent}>{rowData.value}</Typography>;
            },
          },
          {
            headerName: 'Timestamp',
            field: 'created',
            type: 'datetime',
            filterable: false,
            sortable: false,
            align: 'center',
            renderCell: (rowData: GridRenderCellParams) => {
              return (
                <Typography className={classes.cellContent} data-testid={'log-table-date'}>
                  {format(new Date(rowData.value as number), 'MMM dd yyyy hh:mm aaa')}
                </Typography>
              );
            },
          },
        ]}
        components={{
          Toolbar: () => (
            <CustomLookupFilter
              lookup={logLevels}
              selectedFilter={selectedFilter}
              setSelectedFilter={setSelectedFilter}
            />
          ),
        }}
        rows={rows.filter((row) => {
          if (!selectedFilter.includes(row.level)) return row;
        })}
        pageSize={pageSize}
        rowsPerPageOptions={[50, 100]}
        onPageChange={() => {
          if (addMoreRows) {
            addMoreRows();
          }
        }}
        onPageSizeChange={(pageSize) => {
          setPageSize(pageSize);
        }}
        disableColumnMenu={true}
      />
    </LogTableRoot>
  );
};

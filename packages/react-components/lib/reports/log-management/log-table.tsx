import React from 'react';
// import MaterialTable, { Column } from 'material-table';
import { DataGrid } from '@mui/x-data-grid';
import { LogLevel } from '.';
import { Typography } from '@material-ui/core';
import { makeStyles } from '@material-ui/styles';
// import { materialTableIcons } from '../../material-table-icons';
import { format } from 'date-fns';

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
  tableSize?: string; // units vh or rem
  addMoreRows?(): void;
}

const useStyles = makeStyles((theme) => ({
  error: {
    color: theme.palette.error.main,
  },
  debug: {
    color: theme.palette.secondary.dark,
  },
  warn: {
    color: theme.palette.warning.light,
  },
  info: {
    color: theme.palette.info.main,
  },
  cellContent: {
    display: 'block',
    marginBlockStart: '1em',
    marginBlockEnd: '1em',
    marginInlineStart: '0px',
    marginInlineEnd: '0px',
  },
}));

export const LogTable = (props: LogTableProps): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;
  const classes = useStyles();
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
    <DataGrid
      columns={[
        {
          headerName: 'Level',
          field: 'level',
          type: 'string',
          align: 'center',
          // cellStyle: { padding: '0px', width: '2rem', maxWidth: '2rem' },
          // headerStyle: {
          //   width: '2rem',
          //   maxWidth: '2rem',
          // },
          // filterCellStyle: {
          //   maxHeight: '2px',
          // },
          // lookup: logLevels as Column<{
          //   level: string;
          //   message: string;
          //   created: string;
          // }>['lookup'],
          // filterComponent: (props) => <CustomLookupFilterParser {...props} />,
          valueFormatter: (rowData) => {
            return (
              <Typography
                className={`${getLogLevelStyle(rowData.row.level)} ${classes.cellContent}`}
              >
                {rowData.row.level}
              </Typography>
            );
          },
        },
        {
          headerName: 'Container',
          field: 'container',
          type: 'string',
          align: 'center',
          // cellStyle: { padding: '0px', width: '2rem', maxWidth: '2rem' },
          // headerStyle: {
          //   width: '2rem',
          //   maxWidth: '2rem',
          // },
          // filterCellStyle: {
          //   maxHeight: '2px',
          // },
          valueFormatter: (rowData) => {
            return (
              <Typography className={classes.cellContent}>
                {rowData.row.container ? rowData.row.container.name : 'Unknown'}
              </Typography>
            );
          },
        },
        {
          headerName: 'Message',
          field: 'message',
          type: 'string',
          // cellStyle: { padding: '0px', width: '75rem', minWidth: '75rem', whiteSpace: 'pre-wrap' },
          // headerStyle: {
          //   width: '75rem',
          //   minWidth: '75rem',
          // },
          valueFormatter: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.row.message}</Typography>;
          },
        },
        {
          headerName: 'Timestamp',
          field: 'created',
          type: 'datetime',
          filterable: false,
          align: 'center',
          // cellStyle: { padding: '0px' },
          valueFormatter: (rowData) => {
            return (
              <Typography className={classes.cellContent} data-testid={'log-table-date'}>
                {format(new Date(rowData.row.created), 'MMM dd yyyy hh:mm aaa')}
              </Typography>
            );
          },
        },
      ]}
      rows={rows}
      pageSize={100}
      rowsPerPageOptions={[50, 100, 200]}
      onPageChange={(page, pageSize) => {
        if (addMoreRows) {
          rows.length / pageSize - 1 === page && addMoreRows();
        }
      }}
    />
  );
};

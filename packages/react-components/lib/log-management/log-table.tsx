import React from 'react';
import MaterialTable from 'material-table';
import { LogLevel } from '.';
import { makeStyles, Typography } from '@material-ui/core';
import moment from 'moment';
import { materialTableIcons } from '../material-table-icons';

export type LogRowsType = { level: string; message: string; timestamp: string }[];

export interface LogTableProps {
  rows: LogRowsType | [];
  tableSize?: string; // units vh or rem
}

const useStyles = makeStyles((theme) => ({
  textColumn: {
    whiteSpace: 'pre-wrap',
    width: '65rem',
  },
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
}));

export const LogTable = (props: LogTableProps): React.ReactElement => {
  const { rows, tableSize } = props;
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

  return (
    <MaterialTable
      title="Log Result"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Level</Typography>,
          field: 'level',
          type: 'string',
          align: 'center',
          cellStyle: { padding: '0px' },
          lookup: LogLevel,
          render: (rowData) => {
            return (
              <p className={`${getLogLevelStyle(rowData.level)} `}>
                <Typography>{rowData.level}</Typography>
              </p>
            );
          },
        },
        {
          title: <Typography>Message</Typography>,
          field: 'message',
          type: 'string',
          cellStyle: { padding: '0px' },
          render: (rowData) => {
            return (
              <p className={classes.textColumn}>
                <Typography>{rowData.message}</Typography>
              </p>
            );
          },
        },
        {
          title: <Typography>Timestamp</Typography>,
          field: 'timestamp',
          type: 'datetime',
          filtering: false,
          align: 'center',
          cellStyle: { padding: '0px' },
          render: (rowData) => {
            return (
              <p data-testid={'log-table-date'}>
                <Typography>{moment(rowData.timestamp).format('lll')}</Typography>
              </p>
            );
          },
        },
      ]}
      data={rows}
      options={{
        filtering: true,
        search: false,
        pageSize: 100,
        pageSizeOptions: [50, 100, 200],
        maxBodyHeight: tableSize ? tableSize : '80vh',
      }}
    />
  );
};

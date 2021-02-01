import React from 'react';
import { makeStyles } from '@material-ui/core';
import { SearchLogForm } from './search-log-form';
import { LogRowsType, LogTable } from './log-table';

const useStyles = makeStyles(() => ({
  table: {
    overflowY: 'scroll',
    height: '60vh',
    paddingTop: '20px',
  },
}));

export interface LogQueryPayload {
  toLogDate: moment.Moment;
  fromLogDate: moment.Moment;
  searchText: string;
  sourceLog: string;
  logLevel: string;
  rowsCount: number;
}

export interface LogManagementProps {
  getLogs: (data: LogQueryPayload) => Promise<LogRowsType>;
  getLabels: () => Promise<{ label: string; value: string }[]>;
}

export const LogManagement = (props: LogManagementProps): React.ReactElement => {
  const { getLogs, getLabels } = props;
  const [logs, setLogs] = React.useState<LogRowsType>([]);
  const [logLabels, setLogLabels] = React.useState<{ label: string; value: string }[]>([]);

  const classes = useStyles();

  React.useEffect(() => {
    const getLogLabels = async () => {
      const labels = await getLabels();
      setLogLabels(labels);
    };
    getLogLabels();
  }, [getLabels]);

  const searchLogs = async (
    toLogDate: moment.Moment,
    fromLogDate: moment.Moment,
    searchText: string,
    sourceLog: string,
    logLevel: string,
    rowsCount: number,
  ) => {
    const payload = {
      toLogDate,
      fromLogDate,
      searchText,
      sourceLog,
      logLevel,
      rowsCount,
    };
    setLogs(await getLogs(payload));
  };

  return (
    <>
      <SearchLogForm logLabelValues={logLabels} search={searchLogs}></SearchLogForm>
      <div className={classes.table}>
        {logs.length !== 0 && <LogTable rows={logs} tableSize={'49vh'} />}
      </div>
    </>
  );
};

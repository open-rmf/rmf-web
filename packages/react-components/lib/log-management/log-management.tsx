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

export interface LogManagementProps {
  getLogs: () => Promise<LogRowsType>;
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
  }, []);

  const searchLogs = async (
    searchText: string,
    sourceLog: string,
    logLevel: string,
    rowsCount: number,
  ) => {
    const payload = {
      searchText: searchText,
      sourceLog: sourceLog,
      logLevel: logLevel,
      rowsCount: rowsCount,
    };
    setLogs(await getLogs());
  };

  return (
    <>
      <SearchLogForm logLabelValues={logLabels} search={searchLogs}></SearchLogForm>
      <div className={classes.table}>
        <LogTable rows={logs}></LogTable>
      </div>
    </>
  );
};

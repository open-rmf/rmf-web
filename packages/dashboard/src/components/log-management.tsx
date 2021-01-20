import React from 'react';
import { LogTable, SearchLogForm } from 'react-components';
import { makeStyles } from '@material-ui/core';

const useStyles = makeStyles(() => ({
  table: {
    overflowY: 'scroll',
    height: '60vh',
    paddingTop: '20px',
  },
}));

export const LogManagementApp = (): React.ReactElement => {
  const [logs] = React.useState([]);
  const [logSources] = React.useState([]);

  const classes = useStyles();

  React.useEffect(() => {
    // TODO: get the log labels from the backend on component will mount
  }, []);

  const searchLogs = (
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
    // TODO: send this payload to the backend endpoint to get the logs.
    // remove console.log
    console.log(payload);
  };

  return (
    <>
      <SearchLogForm logLabelValues={logSources} search={searchLogs}></SearchLogForm>
      <div className={classes.table}>
        <LogTable rows={logs}></LogTable>
      </div>
    </>
  );
};

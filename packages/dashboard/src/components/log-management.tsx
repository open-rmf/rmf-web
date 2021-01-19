import React from 'react';
import axios from 'axios';
import { LogRowsType, LogTable, SearchLogForm } from 'react-components';
import { url } from 'inspector';

const LogManagement = (): React.ReactElement => {
  const [logs, setLogs] = React.useState([]);
  const [logSources, setLogSources] = React.useState([] as string[]);

  React.useEffect(() => {
    // Get the log list
    // axios.get()
  }, []);

  const searchLogs = (
    searchText: string,
    sourceLog: string,
    logLevel: string,
    rowsCount: string,
  ) => {
    const payload = {
      searchText: searchText,
      sourceLog: sourceLog,
      logLevel: logLevel,
      rowsCount: rowsCount,
    };
    axios({ url: `localhost`, baseURL: `localhost`, method: 'get', data: payload })
      .then((response) => {
        console.log(response);
      })
      .catch((error) => {
        console.error(error);
      });
  };

  return (
    <>
      <SearchLogForm sourceLogValues={logSources} search={searchLogs}></SearchLogForm>
      <div>
        <LogTable rows={logs}></LogTable>
      </div>
    </>
  );
};

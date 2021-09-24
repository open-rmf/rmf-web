import React from 'react';
import { makeStyles } from '@material-ui/core';
import { SearchLogForm } from './search-log-form';
import { LogRowsType, LogTable } from './log-table';

const useStyles = makeStyles((theme) => ({
  table: {
    overflowY: 'scroll',
    paddingTop: '20px',
  },
  background: {
    backgroundColor: theme.palette.background.paper,
  },
}));

export interface LogQueryPayload {
  toLogDate?: Date | null;
  fromLogDate?: Date | null;
  logLabel?: string | null;
  logLevel?: string | null;
  offset?: number | null;
}

export interface LogManagementProps {
  getLogs: (data: LogQueryPayload) => Promise<LogRowsType>;
  getLabels: () => Promise<{ label: string; value: string }[]>;
}

export const LogManagement = (props: LogManagementProps): React.ReactElement => {
  const { getLogs, getLabels } = props;
  const [logs, setLogs] = React.useState<LogRowsType>([]);
  const [logLabels, setLogLabels] = React.useState<{ label: string; value: string }[]>([]);
  const [lastSearchParams, setLastSearchParams] = React.useState<LogQueryPayload>({});

  const classes = useStyles();

  React.useEffect(() => {
    const getLogLabels = async () => {
      const labels = await getLabels();
      setLogLabels(labels);
    };
    getLogLabels();
  }, [getLabels]);

  const searchLogs = async (payload: LogQueryPayload) => {
    setLastSearchParams(payload);
    setLogs(await getLogs(payload));
  };

  const getMoreLogs = async () => {
    setLogs(logs.concat(await getLogs({ ...lastSearchParams, offset: logs.length })));
  };

  return (
    <div className={classes.background}>
      <SearchLogForm logLabelValues={logLabels} search={searchLogs}></SearchLogForm>
      <div className={classes.table}>
        {logs.length !== 0 && <LogTable rows={logs} tableSize={'48vh'} addMoreRows={getMoreLogs} />}
      </div>
    </div>
  );
};

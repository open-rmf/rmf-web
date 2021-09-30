import React from 'react';
import { styled } from '@material-ui/core';
import { SearchLogForm } from './search-log-form';
import { LogRowsType, LogTable } from './log-table';

const classes = {
  table: 'log-management-table',
  background: 'log-management-background',
};
const LogManagementRoot = styled('div')(({ theme }) => ({
  [`&.${classes.background}`]: {
    backgroundColor: theme.palette.background.paper,
  },
  [`& .${classes.table}`]: {
    overflowY: 'scroll',
    paddingTop: '20px',
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
    <LogManagementRoot className={classes.background}>
      <SearchLogForm logLabelValues={logLabels} search={searchLogs}></SearchLogForm>
      <div className={classes.table}>
        {logs.length !== 0 && <LogTable rows={logs} addMoreRows={getMoreLogs} />}
      </div>
    </LogManagementRoot>
  );
};

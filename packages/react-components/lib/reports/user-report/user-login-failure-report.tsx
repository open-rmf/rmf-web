import React from 'react';
import { DefaultReportQueryPayload, defaultReportStyles } from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import {
  UserLoginFailureReportTable,
  UserLoginFailureRowsType,
} from './user-login-failure-report-table';

export interface UserLoginFailureReportProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<UserLoginFailureRowsType>;
}

export const UserLoginFailureReport = (props: UserLoginFailureReportProps): React.ReactElement => {
  const { getLogs } = props;
  const [logs, setLogs] = React.useState<UserLoginFailureRowsType>([]);
  const [lastSearchParams, setLastSearchParams] = React.useState<DefaultReportQueryPayload>({});

  const classes = defaultReportStyles();

  const searchLogs = async (payload: DefaultReportQueryPayload) => {
    setLastSearchParams(payload);
    setLogs(await getLogs(payload));
  };

  const getMoreLogs = async () => {
    setLogs(logs.concat(await getLogs({ ...lastSearchParams, offset: logs.length })));
  };

  return (
    <>
      <DefaultDatesForm search={searchLogs} />
      <div className={classes.table}>
        {logs.length !== 0 && (
          <UserLoginFailureReportTable rows={logs} tableSize={'48vh'} addMoreRows={getMoreLogs} />
        )}
      </div>
    </>
  );
};

export default UserLoginFailureReport;

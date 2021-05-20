import React from 'react';
import { DefaultReportQueryPayload, defaultReportStyles } from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { UserLoginReportTable, UserLoginRowsType } from './user-login-report-table';

export interface UserLoginReportProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<UserLoginRowsType>;
}

export const UserLoginReport = (props: UserLoginReportProps): React.ReactElement => {
  const { getLogs } = props;
  const [logs, setLogs] = React.useState<UserLoginRowsType>([]);
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
          <UserLoginReportTable rows={logs} tableSize={'48vh'} addMoreRows={getMoreLogs} />
        )}
      </div>
    </>
  );
};

export default UserLoginReport;

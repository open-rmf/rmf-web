import React from 'react';
import { DefaultReportQueryPayload, defaultReportStyles } from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { UserLogoutReportTable, UserLogoutRowsType } from './user-logout-report-table';

export interface UserLogoutReportProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<UserLogoutRowsType>;
}

export const UserLogoutReport = (props: UserLogoutReportProps): React.ReactElement => {
  const { getLogs } = props;
  const [logs, setLogs] = React.useState<UserLogoutRowsType>([]);
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
          <UserLogoutReportTable rows={logs} tableSize={'48vh'} addMoreRows={getMoreLogs} />
        )}
      </div>
    </>
  );
};

export default UserLogoutReport;

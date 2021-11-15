import React from 'react';
import {
  DefaultReportQueryPayload,
  defaultReportClasses,
  StyledDefaultReport,
} from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { UserLoginReportTable, UserLoginRowsType } from './user-login-report-table';

import { ReportConfigProps } from '../utils';
export interface UserLoginReportProps extends ReportConfigProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<UserLoginRowsType>;
}

export const UserLoginReport = (props: UserLoginReportProps): React.ReactElement => {
  const { getLogs, ...otherProps } = props;
  const [logs, setLogs] = React.useState<UserLoginRowsType>([]);
  const [lastSearchParams, setLastSearchParams] = React.useState<DefaultReportQueryPayload>({});

  const searchLogs = async (payload: DefaultReportQueryPayload) => {
    setLastSearchParams(payload);
    setLogs(await getLogs(payload));
  };

  const getMoreLogs = async () => {
    setLogs(logs.concat(await getLogs({ ...lastSearchParams, offset: logs.length })));
  };

  return (
    <StyledDefaultReport>
      <DefaultDatesForm search={searchLogs} {...otherProps} />
      <div className={defaultReportClasses.table}>
        {logs.length !== 0 && (
          <UserLoginReportTable rows={logs} tableSize={500} addMoreRows={getMoreLogs} />
        )}
      </div>
    </StyledDefaultReport>
  );
};

export default UserLoginReport;

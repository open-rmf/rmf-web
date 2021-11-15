import React from 'react';
import {
  DefaultReportQueryPayload,
  StyledDefaultReport,
  defaultReportClasses,
} from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import {
  UserLoginFailureReportTable,
  UserLoginFailureRowsType,
} from './user-login-failure-report-table';
import { ReportConfigProps } from '../utils';

export interface UserLoginFailureReportProps extends ReportConfigProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<UserLoginFailureRowsType>;
}

export const UserLoginFailureReport = (props: UserLoginFailureReportProps): React.ReactElement => {
  const { getLogs, ...otherProps } = props;
  const [logs, setLogs] = React.useState<UserLoginFailureRowsType>([]);
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
          <UserLoginFailureReportTable rows={logs} tableSize={500} addMoreRows={getMoreLogs} />
        )}
      </div>
    </StyledDefaultReport>
  );
};

export default UserLoginFailureReport;

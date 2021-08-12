import React from 'react';
import { DefaultReportQueryPayload, UserLoginReport, UserLoginRowsType } from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';
import { ReportConfigProps } from 'react-components';

const UserLoginReportConfig = (props: ReportConfigProps) => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<UserLoginRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/user/login/`,
      params,
      authenticator.token,
    )) as UserLoginRowsType;
  };

  return <UserLoginReport getLogs={getLogs} {...props} />;
};

export default UserLoginReportConfig;

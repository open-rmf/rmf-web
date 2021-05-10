import React from 'react';
import { DefaultReportQueryPayload, UserLoginReport, UserLoginRowsType } from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';

const UserLoginFailureReportConfig = () => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<UserLoginRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/user_login_failure/`,
      params,
      authenticator.token,
    )) as UserLoginRowsType;
  };

  return <UserLoginRowsType getLogs={getLogs} />;
};

export default UserLoginFailureReportConfig;

import React from 'react';
import { DefaultReportQueryPayload, UserLogoutReport, UserLogoutRowsType } from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';
import { ConfigProps } from 'react-components';

const UserLogoutReportConfig = (props: ConfigProps) => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<UserLogoutRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/user/logout/`,
      params,
      authenticator.token,
    )) as UserLogoutRowsType;
  };

  return <UserLogoutReport getLogs={getLogs} {...props} />;
};

export default UserLogoutReportConfig;

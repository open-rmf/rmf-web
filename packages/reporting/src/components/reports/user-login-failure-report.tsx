import React from 'react';
import {
  DefaultReportQueryPayload,
  UserLoginFailureReport,
  UserLoginFailureRowsType,
} from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';
import { ConfigProps } from 'react-components';

const UserLoginFailureReportConfig = (props: ConfigProps) => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<UserLoginFailureRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/user/loginfailure/`,
      params,
      authenticator.token,
    )) as UserLoginFailureRowsType;
  };

  return <UserLoginFailureReport getLogs={getLogs} {...props} />;
};

export default UserLoginFailureReportConfig;

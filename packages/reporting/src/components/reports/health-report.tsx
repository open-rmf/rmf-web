import React from 'react';
import { DefaultReportQueryPayload, HealthReport, HealthRowsType } from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';
import { ConfigProps } from 'react-components';

const HealthReportConfig = (props: ConfigProps) => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<HealthRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/health/`,
      params,
      authenticator.token,
    )) as HealthRowsType;
  };

  return <HealthReport getLogs={getLogs} {...props} />;
};

export default HealthReportConfig;

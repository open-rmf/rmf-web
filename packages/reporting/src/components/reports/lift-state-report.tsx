import React from 'react';
import { DefaultReportQueryPayload, LiftStateReport, LiftStateRowsType } from 'react-components';
import appConfig from '../../app-config';
import { getLogData } from './utils';
import { AuthenticatorContext } from '../auth-contexts';

const LiftStateReportConfig = () => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<LiftStateRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/lift_state/`,
      params,
      authenticator.token,
    )) as LiftStateRowsType;
  };

  return <LiftStateReport getLogs={getLogs} />;
};

export default LiftStateReportConfig;

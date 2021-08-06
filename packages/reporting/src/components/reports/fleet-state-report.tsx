import React from 'react';
import { DefaultReportQueryPayload, FleetStateReport, FleetStateRowsType } from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';
import { ReportConfigProps } from 'react-components';

const FleetStateReportConfig = (props: ReportConfigProps) => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<FleetStateRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/fleet_state/`,
      params,
      authenticator.token,
    )) as FleetStateRowsType;
  };

  return <FleetStateReport getLogs={getLogs} {...props} />;
};

export default FleetStateReportConfig;

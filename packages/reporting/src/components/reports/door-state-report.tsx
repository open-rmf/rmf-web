import React from 'react';
import { DefaultReportQueryPayload, DoorStateReport, DoorStateRowsType } from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';

const DoorStateReportConfig = () => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<DoorStateRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/door_state/`,
      params,
      authenticator.token,
    )) as DoorStateRowsType;
  };

  return <DoorStateReport getLogs={getLogs} />;
};

export default DoorStateReportConfig;

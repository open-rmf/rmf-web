import React from 'react';
import {
  DefaultReportQueryPayload,
  DispenserStateReport,
  DispenserStateRowsType,
} from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';

const DispenserStateReportConfig = () => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<DispenserStateRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/dispenser_state/`,
      params,
      authenticator.token,
    )) as DispenserStateRowsType;
  };

  return <DispenserStateReport getLogs={getLogs} />;
};

export default DispenserStateReportConfig;

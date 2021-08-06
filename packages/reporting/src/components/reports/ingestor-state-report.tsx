import React from 'react';
import {
  DefaultReportQueryPayload,
  IngestorStateReport,
  IngestorStateRowsType,
} from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';
import { getLogData } from './utils';
import { ReportConfigProps } from 'react-components';

const IngestorStateReportConfig = (props: ReportConfigProps) => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: DefaultReportQueryPayload): Promise<IngestorStateRowsType> => {
    return (await getLogData(
      `${appConfig.reportingServerUrl}/report/ingestor_state/`,
      params,
      authenticator.token,
    )) as IngestorStateRowsType;
  };

  return <IngestorStateReport getLogs={getLogs} {...props} />;
};

export default IngestorStateReportConfig;

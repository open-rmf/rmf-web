import React from 'react';
import axios from 'axios';
import { LogManagement, LogQueryPayload, LogRowsType } from 'react-components';
import appConfig from '../../app-config';
import { AuthenticatorContext } from '../auth-contexts';

const AllLogsReport = () => {
  const authenticator = React.useContext(AuthenticatorContext);
  const getLogs = async (params: LogQueryPayload): Promise<LogRowsType> => {
    try {
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/raw_logs/`, {
        params: {
          toLogDate: params.toLogDate ? params.toLogDate.format() : null,
          fromLogDate: params.fromLogDate ? params.fromLogDate.format() : null,
          logLabel: params.logLabel,
          logLevel: params.logLevel,
          offset: params.offset,
        },
        headers: {
          Authorization: 'Bearer ' + authenticator.token,
        },
      });
      return response.data as LogRowsType;
    } catch (error) {
      console.error(error);
      return [];
    }
  };

  const getLogServerLabels = async () => [
    { label: 'Web Server', value: 'web-server' },
    { label: 'RMF core', value: 'rmf-core' },
  ];
  return <LogManagement getLogs={getLogs} getLabels={getLogServerLabels} />;
};

export default AllLogsReport;

import React from 'react';
import axios from 'axios';
import { LogManagement, LogQueryPayload, LogRowsType } from 'react-components';
import appConfig from '../../app-config';

const AllLogsReport = () => {
  const getLogs = async (params: LogQueryPayload): Promise<LogRowsType> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/raw_logs/`, {
        params: {
          toLogDate: params.toLogDate ? params.toLogDate.format() : null,
          fromLogDate: params.toLogDate ? params.toLogDate.format() : null,
          logLabel: params.logLabel,
          logLevel: params.logLevel,
        },
      });
      console.log(response);
      return response.data as LogRowsType;
    } catch (error) {
      console.error(error);
      return [];
    }
  };

  // (data: LogQueryPayload) => Promise<LogRowsType>
  const getLogServerLabels = async () => [
    { label: 'Web Server', value: 'web-server' },
    { label: 'RMF core', value: 'rmf-core' },
  ];
  return <LogManagement getLogs={getLogs} getLabels={getLogServerLabels} />;
};

export default AllLogsReport;

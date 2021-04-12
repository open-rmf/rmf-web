import React from 'react';
import axios from 'axios';
import { LogManagement, LogQueryPayload, LogRowsType } from 'react-components';
import appConfig from '../../app-config';

const AllLogsReport = () => {
  // (data: LogQueryPayload) => Promise<LogRowsType>
  const getLogs = async (data: LogQueryPayload): Promise<LogRowsType> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/raw_logs/`);
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

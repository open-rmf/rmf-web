import React from 'react';
import axios from 'axios';
import { DefaultReportQueryPayload, HealthReport, HealthRowsType } from 'react-components';
import appConfig from '../../app-config';

const HealthReportConfig = () => {
  const getLogs = async (params: DefaultReportQueryPayload): Promise<HealthRowsType> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/health/`, {
        params: {
          toLogDate: params.toLogDate ? params.toLogDate.format() : null,
          fromLogDate: params.fromLogDate ? params.fromLogDate.format() : null,
          offset: params.offset,
          limit: params.limit,
        },
      });
      console.log(response);
      return response.data as HealthRowsType;
    } catch (error) {
      console.error(error);
      return [];
    }
  };

  return <HealthReport getLogs={getLogs} />;
};

export default HealthReportConfig;

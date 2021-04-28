import React from 'react';
import axios from 'axios';
import { DefaultReportQueryPayload, LiftStateReport, LiftStateRowsType } from 'react-components';
import appConfig from '../../app-config';

const LiftStateReportConfig = () => {
  const getLogs = async (params: DefaultReportQueryPayload): Promise<LiftStateRowsType> => {
    try {
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/lift_state/`, {
        params: {
          toLogDate: params.toLogDate ? params.toLogDate.format() : null,
          fromLogDate: params.fromLogDate ? params.fromLogDate.format() : null,
          offset: params.offset,
          limit: params.limit,
        },
      });
      return response.data as LiftStateRowsType;
    } catch (error) {
      console.error(error);
      return [];
    }
  };

  return <LiftStateReport getLogs={getLogs} />;
};

export default LiftStateReportConfig;

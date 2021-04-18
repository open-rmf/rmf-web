import React from 'react';
import axios from 'axios';
import {
  DefaultReportQueryPayload,
  DispenserStateReport,
  DispenserStateRowsType,
} from 'react-components';
import appConfig from '../../app-config';

const DispenserStateReportConfig = () => {
  const getLogs = async (params: DefaultReportQueryPayload): Promise<DispenserStateRowsType> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get(`${appConfig.reportingServerUrl}/report/dispenser_state/`, {
        params: {
          toLogDate: params.toLogDate ? params.toLogDate.format() : null,
          fromLogDate: params.fromLogDate ? params.fromLogDate.format() : null,
          offset: params.offset,
          limit: params.limit,
        },
      });
      return response.data as DispenserStateRowsType;
    } catch (error) {
      console.error(error);
      return [];
    }
  };

  return <DispenserStateReport getLogs={getLogs} />;
};

export default DispenserStateReportConfig;
